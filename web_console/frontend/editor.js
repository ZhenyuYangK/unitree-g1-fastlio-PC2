/* Configuration forms write through versioned HTTP endpoints; no local-only saves. */
globalThis.G1DataEditor = class {
  constructor(options) {
    this.options = options;
    this.$ = (id) => document.getElementById(id);
    this.saving = false;
    document.querySelectorAll('[data-close]').forEach((button) => button.addEventListener('click', () => this.$(button.dataset.close).close()));
    this.$('newWaypointButton').addEventListener('click', () => this.openWaypoint());
    this.$('newMissionButton').addEventListener('click', () => this.openMission());
    this.$('editSelectedMission').addEventListener('click', () => this.openMission(this.options.catalog()?.missions.find((m) => m.id === this.$('missionSelect').value)));
    this.$('waypointForm').addEventListener('submit', (event) => { event.preventDefault(); this.saveWaypoint(); });
    this.$('missionForm').addEventListener('submit', (event) => { event.preventDefault(); this.saveMission(); });
    this.$('deleteWaypointButton').addEventListener('click', () => this.perform('waypoint', 'DELETE', '/waypoints/' + encodeURIComponent(this.pointId), null, this.pointRevision));
    this.$('deleteMissionButton').addEventListener('click', () => this.perform('mission', 'DELETE', '/missions/' + encodeURIComponent(this.missionId), null, this.missionRevision));
    this.$('reloadWaypointButton').addEventListener('click', async () => { await options.reload(); this.openWaypoint(options.catalog()?.waypoints.find((p) => p.id === this.pointId)); });
    this.$('reloadMissionButton').addEventListener('click', async () => { await options.reload(); this.openMission(options.catalog()?.missions.find((m) => m.id === this.missionId)); });
    this.$('pickWaypointPosition').addEventListener('click', () => {
      const draft = { values: Object.fromEntries(new FormData(this.$('waypointForm'))), id: this.pointId, revision: this.pointRevision };
      this.$('waypointDialog').close(); options.pick(draft);
    });
    this.$('mapNewPoint').addEventListener('click', () => options.pick({ values: { id: this.newId('point', 'waypoints'), name: '新导航点', x: 0, y: 0, yaw: 0 }, id: null, revision: options.catalog().revision }));
    this.$('addMissionStep').addEventListener('click', () => this.addStep());
  }

  newId(prefix, collection) {
    let index = 1;
    while (this.options.catalog()?.[collection].some((item) => item.id === `${prefix}_${String(index).padStart(2, '0')}`)) index++;
    return `${prefix}_${String(index).padStart(2, '0')}`;
  }

  refreshControls(enabled) {
    document.querySelectorAll('.data-edit').forEach((button) => { button.disabled = !enabled || this.saving; });
    this.$('editSelectedMission').disabled ||= !this.$('missionSelect').value;
    this.$('mapNewPoint').disabled ||= !this.options.mapAvailable();
    this.$('pickWaypointPosition').disabled = !enabled || !this.options.mapAvailable();
  }

  openWaypoint(point, draft = null) {
    if (!this.options.canEdit(false)) return this.options.notify('请先取得编辑权，任务运行时不可编辑', true);
    this.pointId = draft ? draft.id : point?.id || null;
    this.pointRevision = draft?.revision || this.options.catalog().revision;
    const values = draft?.values || (point ? { id: point.id, name: point.name, ...point.pose } : { id: this.newId('point', 'waypoints'), name: '新导航点', x: 0, y: 0, yaw: 0 });
    const form = this.$('waypointForm');
    for (const [key, value] of Object.entries(values)) form.elements.namedItem(key).value = value;
    form.elements.namedItem('id').readOnly = Boolean(this.pointId);
    this.$('waypointEditorTitle').textContent = this.pointId ? '编辑导航点' : '新增导航点';
    this.$('deleteWaypointButton').hidden = !this.pointId;
    this.$('waypointError').textContent = '';
    if (!this.$('waypointDialog').open) this.$('waypointDialog').showModal();
  }

  pointPicked(draft, position) {
    draft.values.x = Number(position.x.toFixed(5)); draft.values.y = Number(position.y.toFixed(5));
    this.openWaypoint(null, draft);
  }

  saveWaypoint() {
    const values = Object.fromEntries(new FormData(this.$('waypointForm')));
    const id = values.id; delete values.id;
    for (const key of ['x', 'y', 'yaw']) values[key] = Number(values[key]);
    this.perform('waypoint', this.pointId ? 'PUT' : 'POST', '/waypoints' + (this.pointId ? '/' + encodeURIComponent(this.pointId) : ''), this.pointId ? values : { id, ...values }, this.pointRevision);
  }

  openMission(mission) {
    if (!this.options.canEdit(false)) return this.options.notify('请先取得编辑权，任务运行时不可编辑', true);
    this.missionId = mission?.id || null;
    this.missionRevision = this.options.catalog().revision;
    const form = this.$('missionForm');
    form.elements.namedItem('id').value = this.missionId || this.newId('mission', 'missions');
    form.elements.namedItem('id').readOnly = Boolean(this.missionId);
    form.elements.namedItem('name').value = mission?.name || '新巡航任务';
    this.$('missionEditorTitle').textContent = this.missionId ? '编辑巡航任务' : '新建巡航任务';
    this.$('deleteMissionButton').hidden = !this.missionId;
    this.$('missionError').textContent = '';
    this.$('missionSteps').replaceChildren();
    for (const step of mission?.steps || [null]) this.addStep(step);
    if (!this.$('missionDialog').open) this.$('missionDialog').showModal();
  }

  renumber() {
    [...this.$('missionSteps').children].forEach((row, index) => { row.querySelector('[data-step-title]').textContent = `站点 ${String(index + 1).padStart(2, '0')}`; });
  }

  addStep(step = null) {
    const existing = Boolean(step);
    step ||= {};
    const row = this.$('missionStepTemplate').content.firstElementChild.cloneNode(true);
    row.originalContent = structuredClone(step.speech?.content || {});
    const field = (name) => row.querySelector(`[data-field="${name}"]`);
    for (const point of this.options.catalog().waypoints) {
      const option = document.createElement('option'); option.value = point.id; option.textContent = `${point.name} · ${point.id}`; field('waypoint').append(option);
    }
    if (step.waypoint) field('waypoint').value = step.waypoint;
    for (const [name, fallback] of [['advance_mode', existing ? 'auto' : 'manual'], ['timeout_sec', existing ? 60 : 180], ['dwell_sec', existing ? 2 : 0]]) field(name).value = step[name] ?? fallback;
    field('language').value = step.speech?.language || 'none';
    for (const language of ['zh', 'en']) {
      const content = step.speech?.content?.[language];
      field(`${language}_text`).value = content?.text || '';
      field(`${language}_wait`).value = content?.wait_sec ?? 5;
      field(`${language}_speaker`).value = content?.speaker_id ?? (language === 'zh' ? 0 : 1);
    }
    const speechVisibility = () => {
      for (const language of ['zh', 'en']) {
        const show = ['both', language].includes(field('language').value);
        row.querySelector(`[data-speech="${language}"]`).hidden = !show;
        row.querySelectorAll(`[data-speech="${language}"] input`).forEach((input) => { input.disabled = !show; });
      }
    };
    field('language').addEventListener('change', speechVisibility); speechVisibility();
    row.querySelectorAll('[data-step-action]').forEach((button) => button.addEventListener('click', () => {
      if (button.dataset.stepAction === 'remove') row.remove();
      if (button.dataset.stepAction === 'up' && row.previousElementSibling) row.parentElement.insertBefore(row, row.previousElementSibling);
      if (button.dataset.stepAction === 'down' && row.nextElementSibling) row.parentElement.insertBefore(row.nextElementSibling, row);
      this.renumber();
    }));
    this.$('missionSteps').append(row); this.renumber();
  }

  saveMission() {
    try {
      const form = this.$('missionForm'), steps = [];
      for (const row of this.$('missionSteps').children) {
        const value = (name) => row.querySelector(`[data-field="${name}"]`).value;
        const step = { waypoint: value('waypoint'), advance_mode: value('advance_mode'), timeout_sec: Number(value('timeout_sec')), dwell_sec: Number(value('dwell_sec')) };
        const language = value('language');
        if (language !== 'none') {
          const content = structuredClone(row.originalContent);
          for (const lang of language === 'both' ? ['zh', 'en'] : [language]) {
            if (!value(`${lang}_text`).trim()) throw new Error('选定语言的讲解内容不能为空');
            content[lang] = { text: value(`${lang}_text`), wait_sec: Number(value(`${lang}_wait`)), speaker_id: Number(value(`${lang}_speaker`)) };
          }
          step.speech = { mode: 'tts', language, content, failure_policy: 'abort' };
        }
        steps.push(step);
      }
      if (!steps.length) throw new Error('任务至少需要一个站点');
      const payload = { name: form.elements.namedItem('name').value, steps };
      const id = form.elements.namedItem('id').value;
      this.perform('mission', this.missionId ? 'PUT' : 'POST', '/missions' + (this.missionId ? '/' + encodeURIComponent(this.missionId) : ''), this.missionId ? payload : { id, ...payload }, this.missionRevision);
    } catch (error) { this.$('missionError').textContent = error.message; }
  }

  async perform(kind, method, path, body, revision) {
    if (this.saving || !this.options.canEdit()) return;
    this.saving = true; this.refreshControls(false);
    try {
      const result = await this.options.api(path, { method, body: body ?? undefined, control: true, revision });
      this.options.saved(result);
      this.$(`${kind}Dialog`).close();
    } catch (error) { this.$(`${kind}Error`).textContent = error.message; }
    finally { this.saving = false; this.options.render(); }
  }
};
