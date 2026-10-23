/* Mapping commands are accepted asynchronously; server snapshots report their outcome. */
globalThis.G1MappingUI = class {
  constructor(options) {
    this.options = options;
    this.$ = (id) => document.getElementById(id);
    this.lastLogAt = 0; this.loadingLogs = false; this.terminalOperation = '';
    for (const [id, operation] of [['mappingStart', 'start'], ['mappingSave', 'save'], ['mappingFinish', 'finish'], ['mappingStop', 'stop']]) {
      this.$(id).addEventListener('click', () => options.pending(async () => {
        const name = operation === 'start' ? this.$('mappingName').value.trim() : '';
        if (name && !/^[a-zA-Z0-9_\u4e00-\u9fff-]{1,48}$/.test(name)) throw new Error('地图名称请使用汉字、字母、数字、下划线或短横线');
        const result = await options.api('/operations', { method: 'POST', control: true,
          body: { request_id: options.randomId(), operation: 'mapping.' + operation, name } });
        options.notify(result.message); this.loadLogs();
      }));
    }
    this.$('mappingRefreshLogs').addEventListener('click', () => this.loadLogs());
    this.resize = new ResizeObserver(() => { this.previewKey = ''; if (this.lastPreview) this.draw(this.lastPreview); });
    this.resize.observe(this.$('mappingCanvas'));
  }

  render(data, permitted, fresh) {
    const state = data.mapping;
    if (!state) return;
    const labels = { STOPPED: '未运行', RUNNING: '采集中', DEGRADED: '部分进程退出', UNKNOWN: '状态未知' };
    const actions = { 'mapping.start': '启动建图', 'mapping.save': '保存地图', 'mapping.finish': '保存并结束', 'mapping.stop': '停止建图' };
    const statuses = { QUEUED: '已接受', RUNNING: '执行中', SUCCEEDED: '已完成', FAILED: '未成功', INTERRUPTED: '结果未确认' };
    this.$('mappingPhase').textContent = !fresh ? '状态过期' : state.busy ? '正在处理' : labels[state.phase] || state.phase;
    for (const [id, action] of [['mappingStart', 'start'], ['mappingSave', 'save'], ['mappingFinish', 'finish'], ['mappingStop', 'stop']]) this.$(id).disabled = !permitted || !state['can_' + action] || data.mission.active;
    this.$('mappingName').disabled = !permitted || state.active || state.busy;
    this.$('mappingStart').hidden = state.active;
    this.$('mappingFinish').hidden = !state.active;
    this.$('mappingAccessNote').textContent = !fresh ? '连接已断开，等待重新同步。' : !state.available ? '当前模式不提供建图操作，请使用本机 local 或 ROS 模式。' : data.mission.active ? '任务运行中，不能操作建图。' : state.error || state.blocker || (state.busy ? '操作在后端执行，关闭页面后仍会继续。' : !permitted ? '请先取得操作权。' : '可管理建图，保存结果以操作完成状态为准。');
    const op = state.operation;
    this.$('mappingOperation').textContent = op ? `${actions[op.action]} · ${statuses[op.state] || op.state}` : '暂无操作';
    this.$('mappingOperationMessage').textContent = op?.message || '开始建图后可查看进度和结果。';
    this.$('mappingOperationTime').textContent = op ? `开始于 ${new Date(op.started_at * 1000).toLocaleTimeString('zh-CN', { hour12: false })}` : '';
    if (!document.querySelector('[data-page="mapping"]').hidden && !data.emergency_stop?.active) {
      this.$('bannerStopMarker').hidden = true;
      this.$('signKicker').textContent = '建图工作区';
      this.$('signStopName').textContent = state.busy ? actions[op?.action] || '正在处理' : state.active ? '采集环境' : '准备建图';
      this.$('signStatus').textContent = !fresh ? '状态过期' : labels[state.phase] || state.phase;
      this.$('signDetail').textContent = state.session?.name || '启动建图后，用遥控器带机器人采集环境。';
      this.$('nextStopLabel').textContent = '下一步';
      this.$('signNextStop').textContent = state.active ? '保存地图' : '开始采集';
    }
    if (op && ['SUCCEEDED', 'FAILED', 'INTERRUPTED'].includes(op.state) && this.terminalOperation !== op.operation_id) {
      this.terminalOperation = op.operation_id;
      if (this.knownOperation === op.operation_id) this.options.notify(op.message, op.state !== 'SUCCEEDED');
    }
    this.knownOperation = op?.operation_id;
    const output = state.session;
    this.$('mappingOutputName').textContent = output?.name || '尚未创建会话';
    this.$('mappingOutputDetail').textContent = output?.saved ? `${output.points.toLocaleString()} 个点 · ${(output.bytes / 1024 / 1024).toFixed(2)} MB · ${new Date(output.saved_at * 1000).toLocaleTimeString('zh-CN', { hour12: false })} 保存` : '尚未生成有效 PCD，保存后可以下载。';
    this.$('mappingDownload').hidden = !fresh || !output?.saved || state.busy;
    if (output?.download_url) this.$('mappingDownload').href = output.download_url;
    const key = JSON.stringify(state.processes);
    if (this.$('mappingProcesses').dataset.key !== key) {
      this.$('mappingProcesses').dataset.key = key;
      this.$('mappingProcesses').replaceChildren();
      for (const process of state.processes) {
        const row = document.createElement('p');
        row.textContent = `${process.name} · ${process.state === 'running' ? '运行中' : process.state === 'stopped' ? '已退出' : '进程归属未确认'}`;
        this.$('mappingProcesses').append(row);
      }
    }
    const preview = data.mapping_preview;
    this.lastPreview = preview;
    const stale = !fresh || !preview || preview.stale;
    this.$('mappingPreviewStatus').textContent = !preview?.available ? '等待点云' : stale ? '最近点云 · 已过期' : !state.active && !state.busy ? '最近点云 · 已停止' : '实时点云';
    this.$('mappingPreviewEmpty').hidden = Boolean(preview?.available);
    this.$('mappingPreviewEmpty').textContent = data.mode === 'local' ? '使用 ROS 模式可查看实时点云；建图操作仍可使用。' : '启动建图并接入 ROS 后显示实时点云';
    this.$('mappingPreviewMeta').textContent = preview?.available ? `${preview.frame_id} · ${preview.scans} 次采样 · 最近一帧 ${preview.last_scan_points.toLocaleString()} 点` : '尚未接收点云';
    if (preview?.available && !document.querySelector('[data-page="mapping"]').hidden) this.draw(preview);
    if (!document.querySelector('[data-page="mapping"]').hidden && performance.now() - this.lastLogAt > 2500) this.loadLogs();
  }

  async loadLogs() {
    if (this.loadingLogs) return;
    this.loadingLogs = true; this.lastLogAt = performance.now();
    try {
      const result = await this.options.api('/mapping/logs');
      const fragment = document.createDocumentFragment();
      for (const log of result.logs) {
        const section = document.createElement('section');
        const title = document.createElement('h3'); title.textContent = log.name;
        const text = document.createElement('pre'); text.textContent = log.text || '等待日志输出…';
        section.append(title, text); fragment.append(section);
      }
      if (!result.logs.length) { const p = document.createElement('p'); p.textContent = '暂无建图日志'; fragment.append(p); }
      this.$('mappingLogs').replaceChildren(fragment);
    } catch (error) { this.$('mappingLogs').textContent = error.message; }
    finally { this.loadingLogs = false; }
  }

  draw(preview) {
    const canvas = this.$('mappingCanvas'), width = canvas.clientWidth, height = canvas.clientHeight;
    if (!width || !height) return;
    const key = `${preview.revision}:${width}:${height}:${document.documentElement.dataset.theme}`;
    if (this.previewKey === key) return;
    this.previewKey = key;
    const ratio = Math.min(devicePixelRatio || 1, 2);
    canvas.width = width * ratio; canvas.height = height * ratio;
    const context = canvas.getContext('2d'); context.scale(ratio, ratio);
    const style = getComputedStyle(document.documentElement);
    context.fillStyle = style.getPropertyValue('--map-ground'); context.fillRect(0, 0, width, height);
    const points = preview.points;
    if (!points.length) return;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const [x, y] of points) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y); }
    const scale = Math.min((width - 56) / Math.max(1, maxX - minX), (height - 56) / Math.max(1, maxY - minY));
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    context.fillStyle = style.getPropertyValue('--route');
    for (const [x, y] of points) context.fillRect(width / 2 + (x - cx) * scale, height / 2 - (y - cy) * scale, 1.7, 1.7);
    context.fillStyle = style.getPropertyValue('--secondary'); context.font = '12px Barlow, sans-serif';
    context.fillText(`X ${(maxX - minX).toFixed(1)} m / Y ${(maxY - minY).toFixed(1)} m`, 16, height - 12);
  }
};
