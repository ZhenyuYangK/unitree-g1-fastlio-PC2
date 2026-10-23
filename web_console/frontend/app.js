/* Same-origin HTTP commands and snapshot-only WebSocket. No browser ROS dependency. */
const $ = (id) => document.getElementById(id);
// randomUUID requires a secure context; getRandomValues also works for a LAN HTTP demo.
function randomId() {
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 15) | 64; bytes[8] = (bytes[8] & 63) | 128;
  const hex = [...bytes].map((value) => value.toString(16).padStart(2, "0")).join("");
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
}
const titles = { overview: "运行总览", waypoints: "导航点", missions: "巡航任务", mapping: "建图管理", integration: "系统接入" };
const phases = { IDLE: "待命", UNKNOWN: "未知", STARTING: "准备中", NAVIGATING: "前往站点", SPEAKING_ZH: "中文讲解", SPEAKING_EN: "英文讲解", WAITING_FOR_CONTINUE: "等待放行", CONTINUING: "准备继续", CANCELING: "正在取消", CANCELED: "已取消", COMPLETED: "已完成", FAILED: "失败" };
Object.assign(phases, { CHECKING: "就绪检查", ARRIVED: "已到站", DWELLING: "站点停留", CLEANING_UP: "收尾中", SUCCEEDED: "已完成", CHECKED: "校验完成", ABORTED: "已中止", STOP_UNCONFIRMED: "停止未确认" });
let snapshot = null, catalog = null, socket = null, lastReceived = 0, connected = false;
let pending = false, leaseId = sessionStorage.getItem("g1_control_lease") || "";
let haveLease = false, retryTimer = null, retryDelay = 1000, instanceId = null, sequence = -1;
let leaseVersion = 0, leaseValidUntil = 0, renewal = null, snapshotReceivedAt = 0;
let toastTimer = null, editor = null, mappingUI = null, navigationUI = null, cruiseUI = null, mapMetadata = null, mapView = null, mapPickingDraft = null;
let catalogLoading = null, mapLoading = false;
let catalogRetryAt = 0, mapRetryAt = 0;
const deviceName = sessionStorage.getItem("g1_device_name") || `浏览器 ${randomId().slice(0, 4)}`;
sessionStorage.setItem("g1_device_name", deviceName);

function renderTheme() {
  const dark = document.documentElement.dataset.theme === "dark";
  $("themeButton").textContent = dark ? "切换到浅色" : "切换到深色";
  $("themeButton").setAttribute("aria-pressed", String(dark));
}
$("themeButton").addEventListener("click", () => {
  const theme = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
  document.documentElement.dataset.theme = theme;
  document.querySelector('meta[name="theme-color"]').content = theme === "dark" ? "#0b1730" : "#eef1f5";
  try { localStorage.setItem("g1_theme", theme); } catch { /* Keep the theme for this page. */ }
  renderTheme();
});
renderTheme();

function toast(message, error = false) {
  $("toast").textContent = message;
  $("toast").classList.toggle("error", error);
  $("toast").hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $("toast").hidden = true; }, 4500);
}

async function api(path, { method = "GET", body, control = false, revision } = {}) {
  const headers = {};
  const requestLease = leaseId, requestVersion = leaseVersion;
  if (body !== undefined) headers["Content-Type"] = "application/json";
  if (control) headers["X-Control-Lease"] = leaseId;
  if (revision) headers["If-Match"] = `"${revision}"`;
  const response = await fetch(`/api/v1${path}`, {
    method, headers, credentials: "same-origin", signal: AbortSignal.timeout(8000),
    ...(body === undefined ? {} : { body: JSON.stringify(body) }),
  });
  const result = await response.json();
  if (!response.ok) {
    if (response.status === 401) showLogin();
    if (result.error?.code === "CONTROL_REQUIRED" && control && requestLease === leaseId && requestVersion === leaseVersion) clearLease();
    throw new Error(result.error?.message || result.detail?.[0]?.msg || `接口返回 ${response.status}`);
  }
  return result;
}

function view(name) {
  document.querySelectorAll("[data-page]").forEach((el) => { el.hidden = el.dataset.page !== name; });
  document.querySelectorAll("[data-view]").forEach((el) => {
    const active = el.dataset.view === name;
    el.classList.toggle("active", active);
    if (active) el.setAttribute("aria-current", "page"); else el.removeAttribute("aria-current");
  });
  $("pageTitle").textContent = titles[name];
}
document.querySelectorAll("[data-view]").forEach((el) => el.addEventListener("click", () => view(el.dataset.view)));

function clearLease() {
  leaseVersion += 1;
  leaseId = "";
  haveLease = false;
  leaseValidUntil = 0;
  sessionStorage.removeItem("g1_control_lease");
}

function hasControl() {
  return Boolean(leaseId && haveLease && Date.now() < leaseValidUntil);
}

function controlTimeText(fresh) {
  const control = snapshot?.data.control;
  if (!fresh) return "操作剩余时间：待同步";
  if (!control?.held) return "操作剩余时间：未取得操作权";
  if (leaseId && !hasControl()) return "操作剩余时间：核验中";
  const elapsed = Math.max(0, Date.now() - snapshotReceivedAt) / 1000;
  const remaining = Math.max(0, Math.ceil(control.remaining_sec - elapsed));
  return `操作剩余时间：${String(Math.floor(remaining / 60)).padStart(2, "0")}:${String(remaining % 60).padStart(2, "0")}`;
}

function showLogin() {
  connected = false;
  haveLease = false;
  clearTimeout(retryTimer);
  if (socket) socket.close();
  if (!$("loginDialog").open) $("loginDialog").showModal();
  render();
}
$("loginDialog").addEventListener("cancel", (event) => event.preventDefault());
$("loginForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  const button = event.currentTarget.querySelector("button");
  button.disabled = true;
  try {
    await api("/session", { method: "POST", body: { token: $("tokenInput").value } });
    $("tokenInput").value = "";
    $("loginError").textContent = "";
    $("loginDialog").close();
    await initialize();
  } catch (error) { $("loginError").textContent = error.message; }
  finally { button.disabled = false; }
});

function applyCatalog(result) {
  catalog = result;
  const previous = $("missionSelect").value;
  $("missionSelect").replaceChildren();
  for (const mission of catalog.missions) {
    const option = document.createElement("option");
    option.value = mission.id; option.textContent = mission.name;
    $("missionSelect").append(option);
  }
  if (catalog.missions.some((mission) => mission.id === previous)) $("missionSelect").value = previous;
  renderCatalog(); render();
}

async function loadCatalog() {
  if (catalogLoading) return catalogLoading;
  catalogLoading = (async () => {
    try { applyCatalog(await api("/catalog")); return true; }
    catch (error) { catalogRetryAt = performance.now() + 5000; toast(error.message, true); return false; }
    finally { catalogLoading = null; }
  })();
  return catalogLoading;
}

async function loadMap() {
  if (mapLoading || catalog?.simulated) return;
  mapLoading = true;
  try {
    const result = await api("/map");
    if (result.revision !== mapMetadata?.revision) {
      mapMetadata = result; mapView = [0, 0, result.width, result.height];
      $("liveMapImage").setAttribute("href", result.image_url);
      $("liveMapImage").setAttribute("width", result.width);
      $("liveMapImage").setAttribute("height", result.height);
    }
    render();
  } catch (error) { mapRetryAt = performance.now() + 5000; toast(error.message, true); }
  finally { mapLoading = false; }
}

function canEdit(requireSynced = true) {
  const data = snapshot?.data;
  return Boolean(connected && performance.now() - lastReceived < 2000 && hasControl() && data?.control.held
    && data.configuration?.available && catalog?.source === "workspace" && (!requireSynced || catalog.revision === data.configuration.revision)
    && !data.configuration.write_blocker && !data.mission.active
    && data.capabilities.waypoint_write && !pending);
}

function renderCatalog() {
  $("waypointCaption").textContent = `${catalog.simulated ? "模拟目录" : "现场目录"} · ${catalog.map_id} · yaw 单位为弧度`;
  $("waypointTable").replaceChildren();
  for (const [index, point] of catalog.waypoints.entries()) {
    const row = document.createElement("tr");
    row.dataset.pointId = point.id;
    const markerCell = document.createElement("td");
    const pointId = document.createElement("span"); pointId.className = "point-id"; pointId.textContent = point.id;
    markerCell.append(stopMarker(index + 1), pointId);
    row.append(markerCell);
    for (const [column, value] of [point.name, point.pose.x.toFixed(2), point.pose.y.toFixed(2), point.pose.yaw.toFixed(2)].entries()) {
      const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
      if (column === 0) cell.className = "stop-name";
    }
    const action = document.createElement("td");
    const edit = document.createElement("button"); edit.className = "button secondary compact data-edit";
    edit.textContent = "编辑"; edit.setAttribute("aria-label", `编辑${point.name}`);
    edit.addEventListener("click", () => editor.openWaypoint(point)); action.append(edit); row.append(action);
    $("waypointTable").append(row);
  }
  $("missionCatalog").replaceChildren();
  for (const mission of catalog.missions) {
    const card = document.createElement("article"); card.className = "catalog-card";
    const content = document.createElement("div");
    const title = document.createElement("h3"); title.textContent = mission.name;
    const route = document.createElement("div"); route.className = "catalog-route";
    mission.waypoints.forEach((id, index) => {
      if (index) { const arrow = document.createElement("span"); arrow.className = "route-arrow"; arrow.textContent = "→"; arrow.setAttribute("aria-hidden", "true"); route.append(arrow); }
      const marker = stopMarker(index + 1); marker.title = catalog.waypoints.find((point) => point.id === id)?.name || id;
      route.append(marker);
    });
    const caption = document.createElement("p"); caption.textContent = `${mission.waypoints.length} 个站点 · 查看路线后开始巡航`;
    content.append(title, route, caption);
    const button = document.createElement("button"); button.className = "button secondary"; button.textContent = "在控制台选择 →";
    button.addEventListener("click", () => {
      if (snapshot?.data.mission.active) { toast("任务运行中，请等待其结束后切换", true); return; }
      $("missionSelect").value = mission.id; view("overview"); render();
    });
    const actions = document.createElement("div"); actions.className = "catalog-actions";
    const edit = document.createElement("button"); edit.className = "button secondary data-edit"; edit.textContent = "编辑任务";
    edit.addEventListener("click", () => editor.openMission(mission));
    actions.append(button, edit); card.append(content, actions); $("missionCatalog").append(card);
  }
}

function connect() {
  clearTimeout(retryTimer);
  const previous = socket;
  const current = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws/state`);
  socket = current;
  connected = false;
  if (previous) previous.close();
  current.addEventListener("open", () => {
    if (socket !== current || !leaseId) return;
    haveLease = false;
    renew();
  });
  current.addEventListener("message", (event) => {
    if (socket !== current) return;
    let message;
    try { message = JSON.parse(event.data); } catch { return; }
    if (message.type !== "state" || message.schema_version !== 1) return;
    if (instanceId !== message.instance_id) {
      if (instanceId !== null) { clearLease(); loadCatalog(); }
      instanceId = message.instance_id; sequence = -1;
    }
    if (message.sequence <= sequence) return;
    sequence = message.sequence;
    snapshot = message; connected = true; lastReceived = performance.now(); snapshotReceivedAt = Date.now(); retryDelay = 1000;
    if (message.data.configuration?.available && message.data.configuration.revision !== catalog?.revision && performance.now() >= catalogRetryAt) loadCatalog();
    if (!message.data.simulated && message.data.map.available && message.data.map.revision !== mapMetadata?.revision && performance.now() >= mapRetryAt) loadMap();
    render();
  });
  current.addEventListener("close", (event) => {
    if (socket !== current) return;
    connected = false; render();
    if (event.code === 4401 || event.code === 4403) { showLogin(); return; }
    if (!$("loginDialog").open) {
      retryTimer = setTimeout(connect, retryDelay);
      retryDelay = Math.min(10000, retryDelay * 2);
    }
  });
  current.addEventListener("error", () => current.close());
}

function svgPoint(pose) {
  return mapMetadata && !snapshot?.data.simulated ? G1MapGeometry.worldToPixel(pose, mapMetadata) : [100 + pose.x * 55, 500 - pose.y * 43];
}
function stopMarker(number, complete = false, large = false) {
  const marker = document.createElement("span");
  marker.className = `stop-marker${complete ? " is-complete" : ""}${large ? " stop-marker-large" : ""}`;
  marker.setAttribute("aria-hidden", "true");
  const label = document.createElement("span"); label.textContent = String(number).padStart(2, "0");
  marker.append(label); return marker;
}

function routeView(mission) {
  const single = mission.mission_id?.startsWith('waypoint:');
  const selected = single && mission.active ? {id:mission.mission_id, name:mission.mission_name, waypoints:[mission.mission_id.slice(9)]}
    : catalog?.missions.find((item) => item.id === (mission.active ? mission.mission_id : $("missionSelect").value));
  const points = (selected?.waypoints || []).map((id) => catalog.waypoints.find((point) => point.id === id)).filter(Boolean);
  const matches = Boolean(mission.run_id && selected?.id === mission.mission_id);
  const step = matches ? Math.min(mission.current_step, points.length) : 0;
  const finished = matches && mission.state === "SUCCEEDED";
  const completed = finished ? points.length : Math.max(0, step - 1 + (matches && mission.state === "CONTINUING" ? 1 : 0));
  return { selected, points, matches, step, finished, completed, index: Math.max(0, step - 1) };
}

function renderWayfinding(data, mission, fresh) {
  $("bannerStopMarker").hidden = false;
  const route = routeView(mission), point = route.points[route.index];
  const emergency = data.emergency_stop?.active === true;
  $("signBanner").classList.toggle("emergency-active", emergency);
  const bannerMarkerKey = `${point ? route.index + 1 : "—"}:${route.finished}`;
  if ($("bannerStopMarker").dataset.key !== bannerMarkerKey) {
    $("bannerStopMarker").dataset.key = bannerMarkerKey;
    $("bannerStopMarker").replaceChildren(stopMarker(point ? route.index + 1 : "—", route.finished, true));
  }
  $("signKicker").textContent = !fresh || mission.stale ? "最近站点 · 状态过期" : route.finished ? "路线完成" : route.matches && mission.active ? mission.state === "NAVIGATING" ? "正在前往" : "当前站点" : route.step ? "最近任务站点" : "首站";
  $("signStopName").textContent = point?.name || "暂无巡航路线";
  $("signStatus").textContent = emergency ? "紧急停止已触发" : !fresh || mission.stale ? "等待重新连接" : route.matches ? phases[mission.state] || mission.state : "准备开始";
  $("signDetail").textContent = !fresh || mission.stale ? "状态已过期，请等待重新连接。" : route.matches && mission.message ? mission.message : route.selected ? `${route.selected.name} · ${route.points.length} 个站点` : "请先选择一条可用路线。";
  const next = route.points[route.index + 1];
  $("nextStopLabel").textContent = route.finished ? "巡航结束" : route.matches && !mission.active ? "下一步" : "下一站";
  $("signNextStop").textContent = route.finished ? "全部站点已完成" : route.matches && !mission.active ? "等待下一次任务" : next?.name || "完成本次巡航";
  $("stopListCount").textContent = `${route.points.length} 站`;
  const current = fresh && !mission.stale && route.matches && mission.active;
  const listKey = JSON.stringify([route.selected?.id, route.points, route.index, route.completed, current, mission.state]);
  if ($("stopList").dataset.key !== listKey) {
    $("stopList").dataset.key = listKey; $("stopList").replaceChildren();
    route.points.forEach((stop, index) => {
      const complete = index < route.completed, active = current && index === route.index && !complete;
      const row = document.createElement("li"); row.className = `stop-row${complete ? " is-complete" : ""}${active ? " is-current" : ""}`;
      row.dataset.pointId = stop.id;
      if (active) row.setAttribute("aria-current", "step");
      const copy = document.createElement("div"); copy.className = "stop-row-copy";
      const name = document.createElement("h3"); name.className = "stop-row-name"; name.textContent = stop.name;
      const meta = document.createElement("span"); meta.className = "stop-row-meta"; meta.textContent = stop.id;
      copy.append(name, meta);
      const status = document.createElement("span"); status.className = "stop-row-status";
      status.textContent = complete ? "已完成 ✓" : active ? phases[mission.state] || "当前站点" : route.matches && !fresh ? "状态过期" : route.matches && route.step > 0 && index === route.index ? phases[mission.state] || "任务结束" : "待前往";
      row.append(stopMarker(index + 1, complete), copy, status); $("stopList").append(row);
    });
  }
  document.querySelectorAll("#waypointTable tr").forEach((row) => {
    const index = route.points.findIndex((stop) => stop.id === row.dataset.pointId);
    const complete = index >= 0 && index < route.completed;
    row.classList.toggle("is-current", current && index === route.index && !complete);
    const marker = row.querySelector(".stop-marker"); marker.classList.toggle("is-complete", complete);
    const number = index >= 0 ? index + 1 : (catalog?.waypoints.findIndex((point) => point.id === row.dataset.pointId) ?? 0) + 1;
    marker.firstElementChild.textContent = String(number).padStart(2, "0");
  });
  return route;
}

function renderMap(data, mission, route) {
  const fresh = connected && performance.now() - lastReceived < 2000;
  const demo = data.simulated && catalog?.simulated;
  const available = Boolean(data.map.available && (demo || mapMetadata));
  $("map").toggleAttribute("hidden", !available); $("mapEmpty").hidden = available;
  $("demoMapLayer").toggleAttribute("hidden", !demo); $("demoMapScale").toggleAttribute("hidden", !demo);
  $("liveMapImage").toggleAttribute("hidden", demo || !available);
  $("mapMode").textContent = demo ? "示意地图" : available ? "后端地图" : "地图不可用";
  $("mapCaption").textContent = demo ? "示意地图 / 单位：米" : available ? `${mapMetadata.image_name} · ${mapMetadata.resolution} 米/像素` : "等待后端地图";
  $("mapEmpty").textContent = data.map.error || "正在加载后端地图…";
  $("poseValue").textContent = !fresh ? '位置数据已过期' : data.pose ? `X ${data.pose.x.toFixed(2)} / Y ${data.pose.y.toFixed(2)}` : "机器人位置未连接";
  for (const id of ["mapZoomIn", "mapZoomOut", "mapFit"]) $(id).disabled = !available || demo;
  $("map").classList.toggle("is-picking", Boolean(mapPickingDraft));
  $("mapPickHint").hidden = !mapPickingDraft;
  if (!available) return;
  if (navigationUI?.follow && data.pose && fresh && mapMetadata && !demo) {
    const [x,y] = svgPoint(data.pose); mapView = [x-mapView[2]/2,y-mapView[3]/2,mapView[2],mapView[3]];
  }
  const box = demo ? [0, 0, 860, 570] : mapView;
  $("map").setAttribute("viewBox", box.join(" "));
  $("map").setAttribute("aria-label", demo ? "模拟实验室地图" : "后端实验室地图，标注导航点和任务站点顺序");
  const scale = demo ? 1 : Math.max(box[2] / 860, box[3] / 570) * 0.7;
  const points = demo ? route.points : catalog?.waypoints || [];
  const editable = canEdit() && !editor?.saving;
  const markerKey = JSON.stringify([catalog?.revision, mapMetadata?.revision, points, route.selected?.id, route.completed, scale, editable]);
  if ($("waypointLayer").dataset.key !== markerKey) {
    $("waypointLayer").dataset.key = markerKey;
    $("routeLine").setAttribute("points", route.points.map((point) => svgPoint(point.pose).join(",")).join(" "));
    $("waypointLayer").replaceChildren();
    const ns = "http://www.w3.org/2000/svg";
    const element = (tag, attributes) => {
      const node = document.createElementNS(ns, tag);
      Object.entries(attributes).forEach(([key, value]) => node.setAttribute(key, value)); return node;
    };
    points.forEach((point, index) => {
      const routeIndex = route.points.findIndex((stop) => stop.id === point.id);
      const number = routeIndex < 0 ? index + 1 : routeIndex + 1;
      const [x, y] = svgPoint(point.pose);
      const group = element("g", { class: `stop-marker-svg${routeIndex >= 0 && routeIndex < route.completed ? " is-complete" : ""}`,
        transform: `translate(${x} ${y}) scale(${scale})`, "aria-label": `站点 ${number} ${point.name}`, "data-waypoint-id": point.id });
      const label = element("text", { class: "stop-number", x: 0, y: 0 }); label.textContent = String(number).padStart(2, "0");
      const title = element("title", {}); title.textContent = `${point.name} · ${point.id}`;
      // Keep the original hit area while halving only the waypoint's visible icon.
      group.append(title, element("rect", { class: "stop-hit-area", x: -24, y: -22, width: 48, height: 44, rx: 5 }), element("circle", { class: "stop-circle", r: 8 }), label);
      if (editable) {
        group.setAttribute("role", "button"); group.setAttribute("tabindex", "0");
        group.addEventListener("click", (event) => { if (!mapPickingDraft) { event.stopPropagation(); editor.openWaypoint(point); } });
        group.addEventListener("keydown", (event) => { if (["Enter", " "].includes(event.key)) { event.preventDefault(); editor.openWaypoint(point); } });
      }
      $("waypointLayer").append(group);
    });
  }
  $("robotMarker").toggleAttribute("hidden", !data.pose || !fresh);
  if (data.pose) {
    const [x, y] = svgPoint(data.pose), originYaw = demo ? 0 : mapMetadata.origin[2];
    $("robotMarker").style.transform = `translate(${x}px, ${y}px) rotate(${-(data.pose.yaw - originYaw) * 180 / Math.PI}deg) scale(${scale})`;
  }
  navigationUI?.draw(data, svgPoint, scale, fresh);
}

function render() {
  const fresh = connected && performance.now() - lastReceived < 2000;
  $("connection").textContent = fresh ? "实时连接" : "连接中 / 状态过期";
  $("connection").className = `connection ${fresh ? "connected" : "offline"}`;
  $("controlTime").textContent = controlTimeText(fresh);
  if (!snapshot) return;
  const data = snapshot.data, mission = data.mission, control = data.control;
  const ownControl = hasControl();
  const canControl = fresh && ownControl && control.held && data.capabilities.mission_control && !pending;
  const local = data.mode === "local";
  const canAcquire = fresh && (data.capabilities.mission_control || data.capabilities.waypoint_write || data.capabilities.mission_write || data.capabilities.mapping_control) && !pending;
  $("modeText").textContent = data.simulated ? "模拟演示 · 未连接真实机器人，地图与反馈均为示例" : local ? "本地数据 · 地图、导航点和任务与后端文件同步，机器人实时状态未连接" : data.mode === 'ros_control' ? 'ROS 控制 · 定位导航、单点导航与巡航任务' : "ROS 只读 · 实时位姿、规划路径、建图和配置编辑";
  document.querySelector(".mode-tag").textContent = data.simulated ? "模拟" : local ? "本地" : "ROS";
  const localizationStages = {WAITING_DATA:'等待雷达数据',COLLECTING:'采集定位点云',SEARCHING:'全局搜索',VERIFYING:'确认初始位姿',VERIFYING_CANDIDATES:'比较定位候选',RETRYING:'重新采集定位',AMBIGUOUS:'定位仍有歧义',FAILED:'定位失败'};
  const localizationStage = data.localization.status_stale === false && !data.localization.valid ? localizationStages[data.localization.state] : '';
  $("localizationValue").textContent = local ? "未连接机器人" : !fresh ? "状态过期" : localizationStage || (data.localization.stale ? "状态过期" : data.localization.valid ? "定位有效" : "未定位");
  $("localizationDetail").textContent = local ? "地图与配置编辑可用" : `${data.localization.state || "UNKNOWN"} · ${data.simulated ? "模拟数据" : "现场数据"}`;
  $("missionValue").textContent = local ? "路线预览" : !fresh || mission.stale ? "状态过期" : phases[mission.state] || mission.state;
  $("missionDetail").textContent = mission.mission_name || "当前无运行任务";
  $("stepValue").replaceChildren(document.createTextNode(String(mission.current_step)), Object.assign(document.createElement("em"), { textContent: ` / ${mission.total_steps}` }));
  $("stepDetail").textContent = mission.current_waypoint || "等待任务开始";
  $("controlValue").textContent = !fresh ? "状态过期" : leaseId && !ownControl ? "正在核验操作权" : ownControl && control.held ? "本设备控制" : control.held ? control.owner : "暂无控制设备";
  $("controlDetail").textContent = controlTimeText(fresh);
  $("controlMetricLabel").textContent = data.simulated ? "控制设备" : "操作设备";
  $("controlButton").textContent = leaseId && !ownControl ? "重新核验操作权" : data.simulated ? ownControl ? "释放控制权" : "取得控制权" : ownControl ? "释放操作权" : "取得操作权";
  $("controlButton").disabled = !canAcquire || (control.held && !leaseId);
  $("missionSelect").disabled = mission.active || !catalog?.missions.length;
  const selected = catalog?.missions.find((item) => item.id === $("missionSelect").value);
  $("routeSummary").textContent = selected ? `${selected.waypoints.length} 个导航点` : "无可用任务";
  $("advanceSummary").textContent = selected?.advance_mode === "manual" ? "到站人工放行" : "按任务配置推进";
  $("missionPhase").textContent = local ? "配置预览" : !fresh || mission.stale ? "状态过期" : phases[mission.state] || mission.state;
  $("progressTitle").textContent = mission.current_waypoint || "等待开始";
  $("progressCount").textContent = `${mission.current_step} / ${mission.total_steps}`;
  $("progressBar").style.width = `${mission.total_steps ? mission.current_step / mission.total_steps * 100 : 0}%`;
  $("progressBar").parentElement.setAttribute("aria-valuemax", String(mission.total_steps || selected?.waypoints.length || 1));
  $("progressBar").parentElement.setAttribute("aria-valuenow", String(mission.current_step));
  $("progressMessage").textContent = mission.message || (data.simulated ? "取得控制权后，可以开始模拟巡航。" : "等待现场任务状态。");
  $("runId").textContent = mission.run_id ? `${mission.run_id.slice(0, 12)}…` : "—";
  $("runId").title = mission.run_id;
  $("taskReport").hidden = !fresh || !mission.report_url || mission.active;
  if (mission.report_url) $("taskReport").href = mission.report_url;
  const requiresAudio = selected?.steps?.some((step) => step.speech);
  const navigationReady = data.simulated || (data.navigation?.ready && !data.navigation?.lifecycle?.busy && !data.mapping?.active && !data.mapping?.busy && !data.configuration?.write_blocker && (!requiresAudio || data.navigation.checks?.audio_bridge?.ready));
  $("startButton").disabled = !canControl || mission.active || data.cruise?.active || !selected || !navigationReady;
  $("startButton").textContent = data.simulated ? "开始模拟巡航 ↗" : data.capabilities.mission_control ? '开始巡航 ↗' : "真机控制未启用";
  $("startButton").hidden = mission.active;
  $("continueButton").hidden = !mission.active;
  $("continueButton").disabled = !canControl || mission.controllable === false || mission.stale || mission.state !== "WAITING_FOR_CONTINUE" || !mission.wait_id;
  $("continueButton").textContent = mission.current_step === mission.total_steps ? "放行并完成任务 →" : "继续下一站 →";
  $("cancelButton").disabled = !canControl || mission.controllable === false || !mission.active || mission.state === "CANCELING";
  $("controlNote").textContent = data.capabilities.mission_control ? '自主任务在页面断线后继续，人工等待阶段持续等待放行；取消后以收尾结果为准。' : "取得操作权后可修改导航点、路线和讲解。";
  $("snapshotTime").textContent = `快照 ${new Date(snapshot.server_time * 1000).toLocaleTimeString("zh-CN", { hour12: false })} · #${snapshot.sequence}`;
  const route = renderWayfinding(data, mission, fresh);
  if (local && fresh) {
    $("signKicker").textContent = "路线预览 · 首站";
    $("signStatus").textContent = data.configuration?.available ? "配置已同步" : "配置不可用";
    $("signDetail").textContent = route.selected ? `${route.selected.name} · ${route.points.length} 个站点` : "新建任务并选择导航点";
    $("progressMessage").textContent = "当前显示后端配置，机器人实时任务状态未连接。";
  }
  const config = data.configuration;
  $("dataSyncStatus").textContent = data.simulated ? "示例数据" : !fresh ? "同步连接已断开" : config?.error || config?.write_blocker
    || (catalog?.revision === config?.revision ? `后端配置已同步 · ${catalog?.waypoints.length || 0} 个导航点` : "正在同步后端配置…");
  $("dataSyncStatus").title = catalog?.files ? Object.values(catalog.files).join("\n") : "";
  editor?.refreshControls(canEdit());
  cruiseUI?.render(data, fresh && ownControl && control.held && !pending, fresh);
  navigationUI?.render(data, fresh && ownControl && control.held && !pending, fresh);
  renderMap(data, mission, route);
  mappingUI?.render(data, fresh && ownControl && control.held && !pending, fresh);
  const logKey = JSON.stringify(data.logs);
  if ($("logList").dataset.key !== logKey) {
    $("logList").dataset.key = logKey; $("logList").replaceChildren();
    [...data.logs].reverse().forEach((entry) => {
      const row = document.createElement("div"); row.className = "log-row";
      const time = document.createElement("time"); time.textContent = new Date(entry.time * 1000).toLocaleTimeString("zh-CN", { hour12: false });
      const message = document.createElement("span"); message.textContent = entry.message;
      row.append(time, message); $("logList").append(row);
    });
  }
}

async function withPending(action) {
  if (pending) return;
  pending = true; render();
  try { await action(); } catch (error) { toast(error.message, true); }
  finally { pending = false; render(); }
}

$("controlButton").addEventListener("click", () => withPending(async () => {
  if (leaseId && !hasControl()) { await renew(); return; }
  if (leaseId) { await api("/control/release", { method: "POST", control: true }); clearLease(); }
  else {
    const started = Date.now();
    const result = await api("/control/acquire", { method: "POST", body: { owner: deviceName } });
    leaseVersion += 1;
    leaseId = result.lease_id; haveLease = true; leaseValidUntil = started + result.expires_in * 1000;
    sessionStorage.setItem("g1_control_lease", leaseId);
  }
}));

async function sendCommand(path, body) {
  const result = await api(path, { method: "POST", control: true, body: { request_id: randomId(), ...body } });
  toast(result.message);
}
$("startButton").addEventListener("click", () => withPending(() => sendCommand("/missions/start", { mission_id: $("missionSelect").value, revision:catalog?.revision || '' })));
$("continueButton").addEventListener("click", () => withPending(() => sendCommand("/missions/continue", { run_id: snapshot.data.mission.run_id, wait_id: snapshot.data.mission.wait_id })));
$("cancelButton").addEventListener("click", () => withPending(() => sendCommand("/missions/cancel", { run_id: snapshot.data.mission.run_id || snapshot.data.mission.goal_id })));
$("missionSelect").addEventListener("change", render);

async function renew() {
  if (!leaseId || $("loginDialog").open) return;
  const secret = leaseId, version = leaseVersion;
  if (renewal?.secret === secret && renewal.version === version) return renewal.promise;
  const attempt = { secret, version };
  renewal = attempt;
  attempt.promise = (async () => {
    const started = Date.now();
    try {
      const result = await api("/control/renew", { method: "POST", control: true });
      if (leaseId !== secret || leaseVersion !== version) return;
      leaseValidUntil = started + result.remaining_sec * 1000;
      haveLease = !$("loginDialog").open;
    } catch (error) {
      if (leaseId !== secret || leaseVersion !== version) return;
      if (haveLease) toast("操作权暂未核验，连接恢复后自动重试", true);
      haveLease = false;
    } finally {
      if (renewal === attempt) renewal = null;
      render();
    }
  })();
  return attempt.promise;
}

function resumeControl() {
  if (document.hidden || $("loginDialog").open) return;
  if (leaseId) { haveLease = false; renew(); }
  if (!connected || performance.now() - lastReceived >= 2000) connect();
  render();
}
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { haveLease = false; render(); }
  else resumeControl();
});
window.addEventListener("online", resumeControl);
window.addEventListener("pageshow", resumeControl);

async function initialize() {
  const session = await api("/session");
  if (!session.authenticated) { showLogin(); return; }
  await loadCatalog();
  if (!catalog?.simulated) await loadMap();
  await renew();
  connect();
}
cruiseUI = new G1CruiseUI({api, randomId, notify:toast, pending:withPending, render, catalog:() => catalog});
mappingUI = new G1MappingUI({ api, randomId, notify: toast, pending: withPending });
navigationUI = new G1NavigationUI({api, randomId, notify:toast, pending:withPending, render, view,
  map:() => mapMetadata, catalog:() => catalog, cancelEditorPick:() => { mapPickingDraft = null; }});
editor = new G1DataEditor({
  catalog: () => catalog, api, render, canEdit,
  mapAvailable: () => Boolean(mapMetadata && snapshot?.data.map.available),
  reload: loadCatalog, notify: toast,
  saved: (result) => { applyCatalog(result.catalog); toast(result.message); },
  pick: (draft) => { if (!canEdit()) return; navigationUI.picking = false; navigationUI.preview = null; mapPickingDraft = draft; view("overview"); render(); },
});
$("refreshDataButton").addEventListener("click", async () => { await loadCatalog(); await loadMap(); });
$("openConfiguration").addEventListener("click", () => view("waypoints"));
$("cancelMapPick").addEventListener("click", () => { mapPickingDraft = null; render(); });
$("map").addEventListener("click", (event) => {
  if (!mapPickingDraft || !mapMetadata || !canEdit()) return;
  const point = $("map").createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
  const pixel = point.matrixTransform($("map").getScreenCTM().inverse());
  if (pixel.x < 0 || pixel.y < 0 || pixel.x > mapMetadata.width || pixel.y > mapMetadata.height) return;
  const draft = mapPickingDraft; mapPickingDraft = null;
  editor.pointPicked(draft, G1MapGeometry.pixelToWorld([pixel.x, pixel.y], mapMetadata)); render();
});
function zoomMap(factor) {
  if (!mapView || !mapMetadata) return;
  const [x, y, width, height] = mapView;
  const ratio = width * factor / mapMetadata.width;
  if (ratio < 0.1 || ratio > 2) return;
  mapView = [x + width * (1 - factor) / 2, y + height * (1 - factor) / 2, width * factor, height * factor]; render();
}
$("mapZoomIn").addEventListener("click", () => zoomMap(0.8));
$("mapZoomOut").addEventListener("click", () => zoomMap(1.25));
$("mapFit").addEventListener("click", () => { navigationUI.follow = false; if (mapMetadata) mapView = [0, 0, mapMetadata.width, mapMetadata.height]; render(); });
setInterval(renew, 5000);
setInterval(render, 500);
initialize().catch((error) => toast(`连接失败：${error.message}，请刷新后重试`, true));
