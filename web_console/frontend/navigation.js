/* Navigation preparation and explicit map previews. Commands are never retried. */
globalThis.G1NavigationUI = class {
  constructor(options) {
    this.o = options; this.$ = (id) => document.getElementById(id);
    this.picking = false; this.preview = null; this.follow = false;
    this.logsAt = 0; this.loadingLogs = false;
    this.actions = { navigationStart: 'navigation.start', navigationManual: 'navigation.start-manual', navigationContinue: 'navigation.continue', navigationPrepare: 'navigation.prepare', navigationStop: 'navigation.stop' };
    for (const [id, operation] of Object.entries(this.actions)) this.$(id).addEventListener('click', () => options.pending(async () => {
      const result = await options.api('/operations', {method:'POST', control:true, body:{request_id:options.randomId(), operation}});
      options.notify(result.message);
    }));
    this.$('navigationInitial').addEventListener('click', () => {
      const pose = this.data?.pose || {x:0, y:0, yaw:0};
      this.setPose(pose); this.$('initialPoseError').textContent = ''; this.$('initialPoseDialog').showModal();
    });
    this.$('closeInitialPose').addEventListener('click', () => this.closePose());
    this.$('initialPoseDialog').addEventListener('cancel', () => { this.preview = null; });
    for (const id of ['initialX','initialY','initialYaw']) this.$(id).addEventListener('input', () => {
      const pose = this.formPose(); if (pose) this.preview = pose; options.render();
    });
    this.$('pickInitialPose').addEventListener('click', () => {
      if (!options.map()) return;
      this.$('initialPoseDialog').close(); this.picking = true; this.follow = false;
      options.cancelEditorPick(); options.view('overview'); options.render();
      this.$('mapWrap').scrollIntoView({block:'center'});
    });
    this.$('cancelNavigationPick').addEventListener('click', () => { this.picking = false; this.preview = null; options.render(); });
    this.$('initialPoseForm').addEventListener('submit', (event) => {
      event.preventDefault(); options.pending(async () => {
        try {
          const pose = this.formPose(); if (!pose) throw new Error('请输入有效的位置和朝向');
          const result = await options.api('/navigation/initial-pose', {method:'POST',control:true,body:{request_id:options.randomId(),...pose}});
          this.closePose(); options.notify(result.message);
        } catch (error) { this.$('initialPoseError').textContent = error.message; }
      });
    });
    const map = this.$('map');
    map.addEventListener('pointerdown', (event) => {
      if (!this.picking || !this.initialAllowed) return;
      const pose = this.eventPose(event); if (!pose) return;
      event.preventDefault(); map.setPointerCapture(event.pointerId);
      this.preview = {...pose,yaw:0}; this.drag = event.pointerId; options.render();
    });
    map.addEventListener('pointermove', (event) => {
      if (this.drag !== event.pointerId || !this.preview) return;
      const pose = this.eventPose(event); if (!pose) return;
      const dx = pose.x-this.preview.x, dy = pose.y-this.preview.y;
      if (Math.hypot(dx,dy) > 0.03) this.preview.yaw = Math.atan2(dy,dx);
      options.render();
    });
    map.addEventListener('pointerup', (event) => {
      if (this.drag !== event.pointerId) return;
      this.drag = null; this.picking = false;
      if (this.initialAllowed && this.preview) { this.setPose(this.preview); this.$('initialPoseDialog').showModal(); }
      options.render();
    });
    map.addEventListener('pointercancel', () => { this.drag = null; this.picking = false; this.preview = null; options.render(); });
    this.$('targetWaypoint').addEventListener('change', () => options.render());
    this.$('goToButton').addEventListener('click', () => {
      const point = this.target(); if (!point) return;
      this.confirmedTarget = point.id; this.confirmedRevision = options.catalog()?.revision;
      this.$('navigationConfirmDetail').textContent = `${point.name} · X ${point.pose.x.toFixed(2)} m / Y ${point.pose.y.toFixed(2)} m · 朝向 ${(point.pose.yaw*180/Math.PI).toFixed(1)}°`;
      this.$('navigationConfirmError').textContent = ''; this.$('navigationConfirm').showModal();
    });
    this.$('closeNavigationConfirm').addEventListener('click', () => this.$('navigationConfirm').close());
    this.$('navigationConfirmForm').addEventListener('submit', (event) => {
      event.preventDefault(); options.pending(async () => {
        try {
          if (this.confirmedTarget !== this.target()?.id || this.confirmedRevision !== options.catalog()?.revision) throw new Error('目标配置已变化，请关闭窗口后重新预览');
          const result = await options.api('/navigation/go-to', {method:'POST',control:true,body:{request_id:options.randomId(),waypoint_id:this.confirmedTarget,revision:this.confirmedRevision || ''}});
          this.$('navigationConfirm').close(); options.notify(result.message);
        } catch (error) { this.$('navigationConfirmError').textContent = error.message; }
      });
    });
    this.$('mapFollow').addEventListener('click', () => { this.follow = !this.follow; options.render(); });
    this.$('navigationRefreshLogs').addEventListener('click', () => this.loadLogs());
  }
  formPose() {
    if (!['initialX','initialY','initialYaw'].every((id) => this.$(id).value.trim() && this.$(id).validity.valid)) return null;
    const x = Number(this.$('initialX').value), y = Number(this.$('initialY').value), yaw = Number(this.$('initialYaw').value)*Math.PI/180;
    return [x,y,yaw].every(Number.isFinite) ? {x,y,yaw} : null;
  }
  setPose(pose) {
    this.$('initialX').value = pose.x.toFixed(3); this.$('initialY').value = pose.y.toFixed(3);
    this.$('initialYaw').value = (pose.yaw*180/Math.PI).toFixed(2); this.preview = pose;
  }
  closePose() { this.$('initialPoseDialog').close(); this.preview = null; this.o.render(); }
  eventPose(event) {
    const metadata = this.o.map(); if (!metadata) return null;
    const point = this.$('map').createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
    const pixel = point.matrixTransform(this.$('map').getScreenCTM().inverse());
    if (pixel.x < 0 || pixel.y < 0 || pixel.x > metadata.width || pixel.y > metadata.height) return null;
    return G1MapGeometry.pixelToWorld([pixel.x,pixel.y],metadata);
  }
  target() { return this.o.catalog()?.waypoints.find((p) => p.id === this.$('targetWaypoint').value); }
  render(data, permitted, fresh) {
    this.data = data;
    const nav = data.navigation || {}, lifecycle = nav.lifecycle || {}, mission = data.mission;
    const busy = data.cruise?.active || mission.active || nav.active_nav_goal || lifecycle.busy || data.mapping?.active || data.mapping?.busy;
    const labels = {controller_server:'路径跟踪',planner_server:'路径规划',recoveries_server:'恢复行为',bt_navigator:'导航执行',motion_bridge:'运动桥',audio_bridge:'语音桥',task_manager:'任务执行器'};
    this.$('navigationChecks').replaceChildren();
    for (const [key,label] of Object.entries(labels)) {
      const check = nav.checks?.[key]; const row = document.createElement('li');
      row.className = fresh && check?.ready ? 'is-ready' : '';
      row.textContent = `${label} · ${!fresh || check?.stale ? '状态过期' : check?.ready ? '已就绪' : '未就绪'}`;
      const speed = check?.parameters?.max_linear_x;
      if (key === 'motion_bridge' && fresh && check?.ready && Number.isFinite(speed)) row.textContent += ` · 上限 ${speed.toFixed(2)} m/s`;
      this.$('navigationChecks').append(row);
    }
    const localization = data.localization || {};
    const stages = {WAITING_DATA:'等待雷达数据',COLLECTING:'采集定位点云',SEARCHING:'全局搜索',VERIFYING:'确认初始位姿',VERIFYING_CANDIDATES:'比较定位候选',RETRYING:'重新采集定位',AMBIGUOUS:'定位仍有歧义',FAILED:'定位失败'};
    const stage = localization.status_stale === false && !localization.valid ? stages[localization.state] : '';
    this.$('navigationPhase').textContent = !fresh ? '状态过期' : stage || (lifecycle.busy ? '准备中' : nav.ready ? '导航就绪' : '等待准备');
    for (const [id,operation] of Object.entries(this.actions)) {
      const permission = operation === 'navigation.start-manual' ? 'can_start' : 'can_' + operation.split('.')[1];
      this.$(id).disabled = !permitted || !lifecycle[permission] || mission.active || data.cruise?.active || nav.active_nav_goal;
    }
    const initialOperation = lifecycle.operation?.action;
    this.initialAllowed = Boolean(permitted && data.capabilities.initial_pose && !mission.active && !data.cruise?.active && !nav.active_nav_goal && !data.mapping?.active && !data.mapping?.busy
      && (!lifecycle.busy || ['navigation.start','navigation.start-manual'].includes(initialOperation)));
    this.$('navigationInitial').disabled = !this.initialAllowed;
    this.$('submitInitialPose').disabled = !this.initialAllowed;
    this.$('pickInitialPose').disabled = !this.initialAllowed || !this.o.map();
    if (!this.initialAllowed && this.picking) { this.picking = false; this.drag = null; this.preview = null; }
    const key = JSON.stringify([this.o.catalog()?.revision,this.o.catalog()?.waypoints]);
    if (this.targetKey !== key) {
      this.targetKey = key; const previous = this.$('targetWaypoint').value; this.$('targetWaypoint').replaceChildren();
      for (const point of this.o.catalog()?.waypoints || []) { const option = document.createElement('option'); option.value = point.id; option.textContent = point.name; this.$('targetWaypoint').append(option); }
      if (this.o.catalog()?.waypoints.some((p) => p.id === previous)) this.$('targetWaypoint').value = previous;
    }
    this.$('targetWaypoint').disabled = Boolean(mission.active);
    const target = this.target();
    this.$('targetDetail').textContent = target ? `X ${target.pose.x.toFixed(2)} m / Y ${target.pose.y.toFixed(2)} m · 朝向 ${(target.pose.yaw*180/Math.PI).toFixed(1)}°` : '请先添加导航点';
    this.$('goToButton').disabled = !permitted || !data.capabilities.single_goal || busy || !nav.ready || !target;
    this.$('confirmGoTo').disabled = this.$('goToButton').disabled;
    const selected = this.o.catalog()?.missions.find((m) => m.id === this.$('missionSelect').value);
    const audioReason = !data.simulated && selected?.steps?.some((step) => step.speech) && !nav.checks?.audio_bridge?.ready ? '所选巡航任务需要真实语音桥' : '';
    const reason = !fresh ? '状态过期，等待重新连接' : lifecycle.blocker || nav.blockers?.join('；') || audioReason || (!permitted ? '请先取得操作权' : busy ? '正在执行任务或导航准备' : '已就绪，可以选择目标或巡航任务');
    this.$('navigationBlocker').textContent = reason;
    this.$('navigationNote').textContent = !lifecycle.available ? '当前模式仅显示状态。启用导航操作需要 ros_control 模式。' : reason;
    this.$('stopConfirmation').textContent = mission.state === 'STOP_UNCONFIRMED' ? '停止未确认，请用遥控器确认机器人状态，并按恢复流程处理。' : mission.gate_closed === true && mission.goals_terminal === true && !mission.active ? '本次任务停止收尾已确认' : mission.active ? '执行中；取消后等待停止收尾确认' : '';
    const op = lifecycle.operation;
    const states = {QUEUED:'已接受',RUNNING:'执行中',SUCCEEDED:'已完成',FAILED:'失败',INTERRUPTED:'结果未确认'};
    this.$('navigationOperation').textContent = op ? `${states[op.state] || op.state}` : '暂无操作';
    const attempt = Number.isInteger(localization.attempt) && localization.attempt > 0
      ? ` · 第 ${localization.attempt}/${localization.max_attempts} 次` : '';
    const candidates = Array.isArray(localization.candidates) ? localization.candidates : [];
    const scores = candidates.filter((item) => Number.isFinite(item.score))
      .map((item) => `候选 ${item.candidate} ${(item.score*100).toFixed(1)} 分${item.stable ? '（稳定）' : ''}`).join('，');
    const detail = stage ? `${stage}${attempt}${scores ? ' · ' + scores : ''}${['AMBIGUOUS','FAILED'].includes(localization.state) ? '；可设置初始位姿后继续启动' : ''}` : '';
    this.$('navigationOperationMessage').textContent = fresh && detail
      ? `${detail}${op?.state === 'FAILED' ? ' · ' + op.message : ''}`
      : op?.message || '从自动定位开始；定位失败后可设置初始位姿并继续。';
    this.$('navigationProcesses').replaceChildren();
    for (const process of lifecycle.processes || []) { const p = document.createElement('p'); p.textContent = `${process.name} · ${process.state === 'running' ? '运行中' : process.state === 'stopped' ? '已退出' : '归属未确认'}`; this.$('navigationProcesses').append(p); }
    if (!document.querySelector('[data-page="integration"]').hidden && performance.now()-this.logsAt > 3000) this.loadLogs();
  }
  draw(data, point, scale, fresh) {
    const path = data.path;
    const available = fresh && path?.available && !path.stale;
    this.$('planLine').toggleAttribute('hidden', !available);
    if (available) this.$('planLine').setAttribute('points', path.points.map(([x,y]) => point({x,y}).join(',')).join(' '));
    this.$('pathStatus').textContent = available ? '规划路径（实线）' : path?.available ? '规划路径已过期' : '暂无规划路径';
    const yaw = this.o.map()?.origin[2] || 0;
    const currentTarget = data.mission.active ? this.o.catalog()?.waypoints.find((p) => p.id === (data.mission.current_waypoint || data.mission.next_waypoint)) : this.target();
    for (const [id,pose] of [['targetMarker',currentTarget?.pose],['initialPoseMarker',this.preview]]) {
      this.$(id).toggleAttribute('hidden', !pose);
      if (pose) { const [x,y] = point(pose); this.$(id).setAttribute('transform', `translate(${x} ${y}) rotate(${-(pose.yaw-yaw)*180/Math.PI}) scale(${scale})`); }
    }
    this.$('navigationPickHint').hidden = !this.picking;
    this.$('map').classList.toggle('is-setting-pose',this.picking);
    this.$('mapFollow').disabled = !fresh || !data.pose || !this.o.map() || this.picking;
    this.$('mapFollow').setAttribute('aria-pressed',String(this.follow));
  }
  async loadLogs() {
    if (this.loadingLogs) return; this.loadingLogs = true; this.logsAt = performance.now();
    try {
      const result = await this.o.api('/navigation/logs'); this.$('navigationLogs').replaceChildren();
      for (const log of result.logs) { const section = document.createElement('section'), h = document.createElement('h3'), pre = document.createElement('pre'); h.textContent = log.name; pre.textContent = log.text; section.append(h,pre); this.$('navigationLogs').append(section); }
      if (!result.logs.length) this.$('navigationLogs').textContent = '暂无导航日志';
    } catch (error) { this.$('navigationLogs').textContent = error.message; }
    finally { this.loadingLogs = false; }
  }
};
