/* P5 continuous cruise: pause/resume are independent of arrival continue. */
globalThis.G1CruiseUI = class {
  constructor(o) {
    this.o = o; this.$ = (id) => document.getElementById(id);
    this.$('cruiseForm').addEventListener('submit', (event) => {
      event.preventDefault(); o.pending(async () => {
        const result = await o.api('/cruise/start', {method:'POST', control:true, body:{
          request_id:o.randomId(), points:this.points(), laps:Number(this.$('cruiseLaps').value),
          loop:this.$('cruiseLoop').checked, timeout_sec:Number(this.$('cruiseTimeout').value),
          dwell_sec:Number(this.$('cruiseDwell').value), revision:o.catalog()?.revision || ''}});
        o.notify(result.message);
      });
    });
    for (const action of ['pause','resume','stop']) this.$('cruise'+action[0].toUpperCase()+action.slice(1)).addEventListener('click', () => o.pending(async () => {
      const result = await o.api('/cruise/'+action, {method:'POST', control:true,
        body:{request_id:o.randomId(), run_id:this.state?.run_id || ''}});
      o.notify(result.message);
    }));
    this.$('cruisePoints').addEventListener('input', () => o.render());
  }
  points() {
    const entered = this.$('cruisePoints').value.trim();
    return entered ? entered.split(/[,，\s]+/).filter(Boolean)
      : this.o.catalog()?.missions.find((m) => m.id === this.$('missionSelect').value)?.waypoints || [];
  }
  render(data, permitted, fresh) {
    const s = this.state = data.cruise || {}; const active = s.active || false;
    this.$('cruiseRoute').textContent = '路线：' + (this.points().join(' → ') || '请选择任务或填写导航点');
    this.$('cruisePhase').textContent = s.state || 'NOT_RUNNING';
    this.$('cruiseStatus').textContent = !s.available ? '连续巡航控制需要 ros_control 模式。'
      : !fresh ? '状态过期，等待连接恢复' : `${s.state || 'NOT_RUNNING'} · 第 ${s.lap || 0} / ${s.laps == null ? '循环' : s.laps} 圈 · ${s.current_waypoint || '等待导航'} · 剩余停留 ${Number(s.dwell_remaining || 0).toFixed(1)} 秒${s.message ? ' · '+s.message : ''}`;
    const control = permitted && s.available && s.controllable !== false;
    this.$('cruiseStart').disabled = !control || active || s.state === 'STOP_UNCONFIRMED' || !this.points().length
      || data.mission.active || !data.navigation?.ready || data.navigation?.lifecycle?.busy || !!data.configuration?.write_blocker || data.mapping?.active || data.mapping?.busy;
    this.$('cruisePause').disabled = !control || !active || ['PAUSING','PAUSED','STOPPING'].includes(s.state);
    this.$('cruiseResume').disabled = !control || !s.live || s.state !== 'PAUSED' || !data.localization.valid;
    this.$('cruiseStop').disabled = !control || !s.live || !active || s.state === 'STOPPING';
    for (const id of ['cruisePoints','cruiseLaps','cruiseLoop','cruiseTimeout','cruiseDwell']) this.$(id).disabled = active;
  }
};
