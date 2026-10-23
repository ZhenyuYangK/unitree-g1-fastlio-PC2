import asyncio
import json
import secrets
import time
import uuid
from collections import OrderedDict
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal, List, Dict
from urllib.parse import urlsplit

from fastapi import Depends, FastAPI, Header, Request, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
from fastapi.responses import JSONResponse, Response, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, UUID4

from .adapters.mock import MockAdapter
from .adapters.ros_readonly import RosReadOnlyAdapter
from .adapters.local import LocalAdapter
from .config import Settings
from .control import CommandCache, ControlLease, GatewayError
from .workspace import WorkspaceData
from .mapping import MappingManager
from .navigation import NavigationManager
from .journal import CommandJournal
from .adapters.ros_control import RosControlAdapter
from .compat import to_thread
from .cruise import CruiseManager


class Payload(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Login(Payload):
    token: str = Field(max_length=1024)


class LeaseRequest(Payload):
    owner: str = Field(min_length=1, max_length=40, pattern=r".*\S.*")


class Command(Payload):
    request_id: UUID4


class MissionStart(Command):
    mission_id: str = Field(min_length=1, max_length=80, pattern=r"^[a-z][a-z0-9_]*$")
    revision: str = Field(default='', max_length=64)


class MissionContinue(Command):
    run_id: str = Field(min_length=1, max_length=128)
    wait_id: str = Field(min_length=1, max_length=128)


class MissionCancel(Command):
    run_id: str = Field(min_length=1, max_length=128)


class Operation(Command):
    operation: Literal["mapping.start", "mapping.save", "mapping.finish", "mapping.stop", "navigation.start", "navigation.start-manual", "navigation.continue", "navigation.prepare", "navigation.stop"]
    name: str = Field(default="", max_length=48, pattern=r"^[a-zA-Z0-9_\u4e00-\u9fff-]*$")


class WaypointEdit(Payload):
    name: str = Field(min_length=1, max_length=160, pattern=r".*\S.*")
    x: float = Field(strict=True, allow_inf_nan=False)
    y: float = Field(strict=True, allow_inf_nan=False)
    yaw: float = Field(strict=True, allow_inf_nan=False)


class WaypointCreate(WaypointEdit):
    id: str = Field(min_length=1, max_length=80, pattern=r"^[a-z][a-z0-9_]*$")


class MissionEdit(Payload):
    name: str = Field(min_length=1, max_length=160, pattern=r".*\S.*")
    steps: List[Dict] = Field(min_length=1, max_length=1000)


class MissionCreate(MissionEdit):
    id: str = Field(min_length=1, max_length=80, pattern=r"^[a-z][a-z0-9_]*$")


class InitialPose(Command):
    x: float = Field(strict=True, allow_inf_nan=False)
    y: float = Field(strict=True, allow_inf_nan=False)
    yaw: float = Field(strict=True, allow_inf_nan=False, ge=-3.141593, le=3.141593)


class GoTo(Command):
    waypoint_id: str = Field(min_length=1, max_length=80, pattern=r"^[a-z][a-z0-9_]*$")
    revision: str = Field(default='', max_length=64)


class CruiseStart(Command):
    points: List[str] = Field(min_length=1, max_length=1000)
    laps: int = Field(default=1, strict=True, ge=1, le=10000)
    loop: bool = Field(default=False, strict=True)
    timeout_sec: float = Field(default=60, strict=True, gt=0, le=3600, allow_inf_nan=False)
    dwell_sec: float = Field(default=2, strict=True, ge=0, le=3600, allow_inf_nan=False)
    revision: str = Field(default='', max_length=64)


class CruiseControl(Command):
    run_id: str = Field(min_length=1, max_length=128)


def create_app(settings=None, adapter=None, mapping=None, navigation=None):
    settings = settings or Settings.from_env()
    robot = adapter or (MockAdapter(settings) if settings.mode == "mock" else LocalAdapter() if settings.mode == "local" else RosControlAdapter(settings) if settings.mode == 'ros_control' else RosReadOnlyAdapter(settings))
    workspace = None if settings.mode == "mock" else WorkspaceData(settings)
    mapper = mapping or MappingManager(settings)
    navigator = navigation or NavigationManager(settings, robot)
    cruise = CruiseManager(settings)
    journal = CommandJournal(Path(settings.workspace) / 'runtime/navigation/web/commands.sqlite3') if settings.mode == 'ros_control' else None
    lease, cache, command_lock = ControlLease(settings.lease_seconds), CommandCache(), None
    sessions = OrderedDict()
    instance_id = uuid.uuid4().hex
    sequence = 0

    @asynccontextmanager
    async def lifespan(app):
        nonlocal command_lock
        command_lock = asyncio.Lock()
        try:
            await robot.start()
            yield
        finally:
            await navigator.close()
            await mapper.close()
            await robot.close()

    app = FastAPI(title="G1 Web Gateway · 框架", version="0.1.0", lifespan=lifespan,
                  docs_url=None, redoc_url=None, openapi_url=None)
    app.state.robot, app.state.lease = robot, lease
    app.state.workspace = workspace
    app.state.mapping = mapper
    app.state.navigation = navigator
    app.state.cruise = cruise

    @app.exception_handler(GatewayError)
    async def gateway_error(request, exc):
        return JSONResponse(status_code=exc.status,
                            content={"error": {"code": exc.code, "message": exc.message}})

    def same_origin(connection):
        origin = connection.headers.get("origin")
        if origin:
            parsed = urlsplit(origin)
            if parsed.scheme not in {"http", "https"} or parsed.netloc != connection.headers.get("host"):
                raise GatewayError(403, "ORIGIN_DENIED", "只接受同源网页请求")

    def authenticated(connection):
        if not settings.token:
            return True
        cookie = connection.cookies.get("g1_session", "")
        return sessions.get(cookie, 0.0) > time.monotonic()

    async def auth(request: Request):
        if request.method not in {"GET", "HEAD"}:
            same_origin(request)
        if not authenticated(request):
            raise GatewayError(401, "AUTH_REQUIRED", "请输入访问口令")

    async def state():
        nonlocal sequence
        data = await robot.snapshot()
        mapping_state = await mapper.snapshot()
        data['mapping'] = mapping_state
        data['cruise'] = await cruise.snapshot()
        lifecycle = await navigator.snapshot()
        data.setdefault('navigation', {}).update(lifecycle=lifecycle)
        data['capabilities']['navigation_control'] = navigator.enabled
        data['capabilities']['mapping_control'] = mapping_state['available']
        if workspace:
            configuration, map_data = await to_thread(workspace.status)
            data["configuration"], data["map"] = configuration, map_data
            if mapping_state['active'] or mapping_state['busy']:
                configuration['write_blocker'] = '建图进行中，请结束建图后编辑导航配置'
            elif mapping_state['available'] and mapping_state['phase'] == 'UNKNOWN':
                configuration['write_blocker'] = '建图状态未知，请核对进程后编辑导航配置'
            elif lifecycle['busy']:
                configuration['write_blocker'] = '定位与导航准备中，请完成后编辑配置'
            data["capabilities"].update(waypoint_write=configuration["available"], mission_write=configuration["available"])
        sequence += 1
        return {"type": "state", "schema_version": 1, "instance_id": instance_id,
                "sequence": sequence, "server_time": time.time(),
                "data": dict(data, mode=settings.mode, control=lease.snapshot())}

    @app.get("/api/v1/health")
    async def health():
        return {"ok": True, "mode": settings.mode, "schema_version": 1}

    @app.get("/api/v1/session")
    async def session(request: Request):
        return {"authenticated": authenticated(request), "auth_required": bool(settings.token)}

    @app.post("/api/v1/session")
    async def login(payload: Login, request: Request):
        same_origin(request)
        if not secrets.compare_digest(payload.token.encode("utf-8"), settings.token.encode("utf-8")):
            raise GatewayError(401, "INVALID_TOKEN", "访问口令不正确")
        now = time.monotonic()
        for key in list(sessions):
            if sessions[key] <= now:
                sessions.pop(key)
        key = secrets.token_urlsafe(32)
        sessions[key] = now + 12 * 3600
        while len(sessions) > 128:
            sessions.popitem(last=False)
        response = JSONResponse({"authenticated": True})
        response.set_cookie("g1_session", key, httponly=True, samesite="strict",
                            secure=settings.cookie_secure, max_age=12 * 3600)
        return response

    @app.delete("/api/v1/session", dependencies=[Depends(auth)])
    async def logout(request: Request):
        sessions.pop(request.cookies.get("g1_session", ""), None)
        response = JSONResponse({"authenticated": False})
        response.delete_cookie("g1_session")
        return response

    @app.get("/api/v1/state", dependencies=[Depends(auth)])
    async def get_state():
        return await state()

    @app.get("/api/v1/catalog", dependencies=[Depends(auth)])
    async def catalog():
        if workspace:
            result = await to_thread(workspace.catalog)
            return JSONResponse(jsonable_encoder(result), headers={"ETag": '"' + result["revision"] + '"', "Cache-Control": "no-cache"})
        return await robot.catalog()

    @app.get("/api/v1/map", dependencies=[Depends(auth)])
    async def map_metadata():
        if not workspace:
            raise GatewayError(501, "DEMO_MAP", "模拟模式没有现场地图资源")
        return await to_thread(workspace.map)

    @app.get("/api/v1/map/image", dependencies=[Depends(auth)])
    async def map_image(version: str = "", if_none_match: str = Header(default="")):
        if not workspace:
            raise GatewayError(501, "DEMO_MAP", "模拟模式没有现场地图资源")
        content, revision = await to_thread(workspace.map_image, version)
        etag = '"' + revision + '"'
        headers = {"ETag": etag, "Cache-Control": "private, no-cache"}
        return Response(status_code=304, headers=headers) if if_none_match == etag else Response(content, media_type="image/png", headers=headers)

    @app.get("/api/v1/openapi.json", dependencies=[Depends(auth)])
    async def schema():
        return app.openapi()

    @app.post("/api/v1/control/acquire", dependencies=[Depends(auth)])
    async def acquire(payload: LeaseRequest):
        return lease.acquire(payload.owner)

    @app.post("/api/v1/control/renew", dependencies=[Depends(auth)])
    async def renew(x_control_lease: str = Header(default="")):
        return lease.renew(x_control_lease)

    @app.post("/api/v1/control/release", dependencies=[Depends(auth)])
    async def release(x_control_lease: str = Header(default="")):
        lease.release(x_control_lease)
        return {"released": True}

    async def command(name, payload, secret, action):
        async with command_lock:
            if settings.mode not in {"mock", 'ros_control'}:
                robot.unsupported()
            lease.require(secret)
            fingerprint = name + json.dumps(payload.model_dump(mode="json"), sort_keys=True)
            if journal:
                previous = await to_thread(journal.begin, str(payload.request_id), fingerprint)
                if previous is not None:
                    return previous
                try:
                    mapping_state = await mapper.snapshot()
                    if name not in ('cancel', 'close_gate', 'cruise_pause', 'cruise_stop'):
                        if mapping_state['active'] or mapping_state['busy'] or mapping_state['phase'] == 'UNKNOWN':
                            raise GatewayError(409, 'MAPPING_BUSY', '建图运行或状态未知，不能执行导航命令')
                        if name in ('start', 'go_to', 'cruise_start'):
                            blocker = await to_thread(navigator.blocker)
                            if blocker:
                                raise GatewayError(409, 'NAVIGATION_BLOCKED', blocker)
                        if name in ('start', 'go_to', 'cruise_start') and (await navigator.snapshot())['busy']:
                            raise GatewayError(409, 'NAVIGATION_BUSY', '导航准备操作尚未完成')
                        if name in ('start', 'go_to', 'cruise_start'):
                            if not payload.revision:
                                raise GatewayError(428, 'REVISION_REQUIRED', '请重新载入任务和导航点后预览')
                            current = await to_thread(workspace.catalog)
                            await to_thread(workspace.map)
                            if current['revision'] != payload.revision:
                                raise GatewayError(409, 'CONFIGURATION_CHANGED', '目标或任务配置已变化，请重新预览后开始')
                        if name in ('start', 'go_to', 'initial_pose') and (await cruise.snapshot()).get('active'):
                            raise GatewayError(409, 'CRUISE_BUSY', '连续巡航仍在运行或暂停，不能发送其他导航命令')
                    detail = await action()
                    result = dict(detail, accepted=True, simulated=False, request_id=str(payload.request_id), operation_id=str(payload.request_id))
                    await to_thread(journal.finish, str(payload.request_id), result)
                    return result
                except GatewayError as exc:
                    await to_thread(journal.finish, str(payload.request_id), error=(exc.status, exc.code, exc.message))
                    raise
            previous = cache.get(secret, str(payload.request_id), fingerprint)
            if previous is not None:
                return previous
            detail = await action()
            result = dict(detail, accepted=True, simulated=True, request_id=str(payload.request_id),
                          operation_id=uuid.uuid4().hex)
            cache.put(secret, str(payload.request_id), fingerprint, result)
            return result

    @app.post("/api/v1/missions/start", status_code=202, dependencies=[Depends(auth)])
    async def start(payload: MissionStart, x_control_lease: str = Header(default="")):
        return await command("start", payload, x_control_lease, lambda: robot.start_mission(payload.mission_id))

    @app.post("/api/v1/missions/continue", status_code=202, dependencies=[Depends(auth)])
    async def next_station(payload: MissionContinue, x_control_lease: str = Header(default="")):
        return await command("continue", payload, x_control_lease,
                             lambda: robot.continue_mission(payload.run_id, payload.wait_id))

    @app.post("/api/v1/missions/cancel", status_code=202, dependencies=[Depends(auth)])
    async def cancel(payload: MissionCancel, x_control_lease: str = Header(default="")):
        return await command("cancel", payload, x_control_lease, lambda: robot.cancel_mission(payload.run_id))

    @app.post('/api/v1/navigation/go-to', status_code=202, dependencies=[Depends(auth)])
    async def go_to(payload: GoTo, x_control_lease: str = Header(default='')):
        return await command('go_to', payload, x_control_lease, lambda: robot.go_to(payload.waypoint_id))

    @app.post('/api/v1/navigation/initial-pose', status_code=202, dependencies=[Depends(auth)])
    async def initial_pose(payload: InitialPose, x_control_lease: str = Header(default='')):
        return await command('initial_pose', payload, x_control_lease, lambda: robot.initial_pose(payload.x, payload.y, payload.yaw))

    @app.get('/api/v1/cruise', dependencies=[Depends(auth)])
    async def cruise_state():
        return await cruise.snapshot()

    @app.post('/api/v1/cruise/start', status_code=202, dependencies=[Depends(auth)])
    async def cruise_start(payload: CruiseStart, x_control_lease: str = Header(default='')):
        async def action():
            data = await robot.snapshot()
            if data['mission']['active'] or not data.get('navigation', {}).get('ready'):
                raise GatewayError(409, 'NAVIGATION_NOT_READY', '连续巡航需要新鲜定位、Nav2 和真实桥就绪且无活动任务')
            return await cruise.start(payload)
        return await command('cruise_start', payload, x_control_lease, action)

    @app.post('/api/v1/cruise/{action}', status_code=202, dependencies=[Depends(auth)])
    async def cruise_control(action: Literal['pause', 'resume', 'stop'], payload: CruiseControl,
                             x_control_lease: str = Header(default='')):
        async def apply():
            if action == 'resume' and not (await robot.snapshot()).get('localization', {}).get('valid'):
                raise GatewayError(409, 'LOCALIZATION_INVALID', '定位失效，不能恢复巡航')
            return await cruise.control(action, payload.run_id)
        return await command('cruise_' + action, payload, x_control_lease, apply)

    async def save_configuration(kind, operation, identifier, payload, secret, revision):
        if not workspace:
            raise GatewayError(501, "DEMO_DATA", "模拟目录不可写，请使用默认 local 模式编辑本地数据")
        async with command_lock:
            lease.require(secret)
            mapping_state = await mapper.snapshot()
            if (await navigator.snapshot())['busy']:
                raise GatewayError(409, 'NAVIGATION_BUSY', '导航准备中，不能修改导航配置')
            if mapping_state['active'] or mapping_state['busy']:
                raise GatewayError(409, 'MAPPING_BUSY', '建图进行中，不能修改导航配置')
            if mapping_state['available'] and mapping_state['phase'] == 'UNKNOWN':
                raise GatewayError(409, 'MAPPING_UNKNOWN', '建图状态未知，请核对进程后编辑导航配置')
            mission = (await robot.snapshot())["mission"]
            if mission["active"]:
                raise GatewayError(409, "MISSION_BUSY", "任务运行中，请结束后再修改配置")
            return await to_thread(workspace.save, kind, operation, identifier, payload, revision)

    @app.post("/api/v1/waypoints", dependencies=[Depends(auth)])
    async def create_waypoint(payload: WaypointCreate, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("waypoints", "create", payload.id, payload.model_dump(exclude={"id"}), x_control_lease, if_match)

    @app.put("/api/v1/waypoints/{waypoint_id}", dependencies=[Depends(auth)])
    async def update_waypoint(waypoint_id: str, payload: WaypointEdit, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("waypoints", "update", waypoint_id, payload.model_dump(), x_control_lease, if_match)

    @app.delete("/api/v1/waypoints/{waypoint_id}", dependencies=[Depends(auth)])
    async def delete_waypoint(waypoint_id: str, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("waypoints", "delete", waypoint_id, None, x_control_lease, if_match)

    @app.post("/api/v1/missions", dependencies=[Depends(auth)])
    async def create_mission(payload: MissionCreate, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("missions", "create", payload.id, payload.model_dump(exclude={"id"}), x_control_lease, if_match)

    @app.put("/api/v1/missions/{mission_id}", dependencies=[Depends(auth)])
    async def update_mission(mission_id: str, payload: MissionEdit, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("missions", "update", mission_id, payload.model_dump(), x_control_lease, if_match)

    @app.delete("/api/v1/missions/{mission_id}", dependencies=[Depends(auth)])
    async def delete_mission(mission_id: str, x_control_lease: str = Header(default=""), if_match: str = Header(default="")):
        return await save_configuration("missions", "delete", mission_id, None, x_control_lease, if_match)

    @app.post("/api/v1/operations", dependencies=[Depends(auth)])
    async def operations(payload: Operation, x_control_lease: str = Header(default="")):
        manager = mapper if payload.operation.startswith('mapping.') else navigator
        if not manager.enabled:
            raise GatewayError(501, "NOT_IMPLEMENTED", "该环境或进程操作尚未接入")
        async with command_lock:
            lease.require(x_control_lease)
            if (await robot.snapshot())['mission']['active']:
                raise GatewayError(409, 'MISSION_BUSY', '任务运行中，不能操作建图')
            if payload.operation == 'mapping.start' and (await robot.snapshot()).get('stack_present'):
                raise GatewayError(409, 'NAVIGATION_BUSY', '定位或导航节点仍在运行，请先结束导航')
            other = navigator if manager is mapper else mapper
            if (await other.snapshot())['busy']:
                raise GatewayError(409, 'STACK_BUSY', '建图或导航操作正在进行')
            result = await manager.submit(payload.request_id, payload.operation, payload.name)
            if (payload.operation == 'mapping.start' and hasattr(robot, 'mapping_preview')
                    and getattr(app.state, 'preview_operation', None) != result['operation_id']):
                robot.mapping_preview.reset()
                app.state.preview_operation = result['operation_id']
            return JSONResponse(result, status_code=202)

    @app.get('/api/v1/operations/{operation_id}', dependencies=[Depends(auth)])
    async def operation_state(operation_id: str):
        try:
            identifier = str(uuid.UUID(operation_id))
        except ValueError as exc:
            raise GatewayError(404, 'OPERATION_NOT_FOUND', '操作不存在') from exc
        if navigator.records.exists() and (navigator.records / (identifier + '.json')).exists():
            return await to_thread(navigator.operation, operation_id)
        return await to_thread(mapper.operation, operation_id)

    @app.get('/api/v1/navigation', dependencies=[Depends(auth)])
    async def navigation_state():
        return dict((await robot.snapshot()).get('navigation', {}), lifecycle=await navigator.snapshot())

    @app.get('/api/v1/navigation/logs', dependencies=[Depends(auth)])
    async def navigation_logs():
        return {'logs': await to_thread(navigator.logs)}

    @app.get('/api/v1/navigation/report', dependencies=[Depends(auth)])
    async def navigation_report(goal_id: str):
        if settings.mode != 'ros_control':
            robot.unsupported()
        try:
            identifier = str(uuid.UUID(goal_id))
        except ValueError as exc:
            raise GatewayError(404, 'REPORT_NOT_FOUND', '任务报告不存在') from exc
        from .mapping import read_json
        saved = await to_thread(read_json, Path(settings.workspace) / 'runtime/missions/actions' / (identifier + '.json'))
        report = Path(saved.get('report_path', '')).resolve() if saved else None
        root = (Path(settings.workspace) / 'runtime/missions').resolve()
        if not report or root not in report.parents or report.suffix != '.json' or not report.is_file():
            raise GatewayError(404, 'REPORT_NOT_FOUND', '尚未生成可下载的任务报告')
        return FileResponse(report, filename=report.name, media_type='application/json', headers={'Cache-Control':'private, no-store'})

    @app.get('/api/v1/mapping', dependencies=[Depends(auth)])
    async def mapping_state():
        return await mapper.snapshot()

    @app.get('/api/v1/mapping/logs', dependencies=[Depends(auth)])
    async def mapping_logs():
        return {'logs': await to_thread(mapper.logs)}

    @app.get('/api/v1/mapping/artifact', dependencies=[Depends(auth)])
    async def mapping_artifact(session_id: str):
        path = await to_thread(mapper.artifact, session_id)
        return FileResponse(path, filename=path.name, media_type='application/octet-stream', headers={'Cache-Control': 'private, no-store'})

    @app.websocket("/ws/state")
    async def stream(websocket: WebSocket):
        try:
            same_origin(websocket)
        except GatewayError:
            await websocket.close(code=4403)
            return
        if not authenticated(websocket):
            await websocket.close(code=4401)
            return
        await websocket.accept()
        try:
            while authenticated(websocket):
                await asyncio.wait_for(websocket.send_json(await state()), timeout=2.0)
                try:
                    await asyncio.wait_for(websocket.receive_text(), timeout=0.5)
                except asyncio.TimeoutError:
                    pass
            await websocket.close(code=4401)
        except (WebSocketDisconnect, asyncio.TimeoutError):
            pass

    frontend = Path(__file__).resolve().parents[2] / "frontend"
    app.mount("/", StaticFiles(directory=str(frontend), html=True), name="frontend")
    return app
