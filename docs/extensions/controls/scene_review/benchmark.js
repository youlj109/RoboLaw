"use strict";
(() => {
  let key = "", epoch = 0;
  window.renderBenchmarkSummary = async scenes => {
    const link = scenes.flatMap(s => s.execution_artifacts || []).find(a =>
      a.label.startsWith("严格安全门禁汇总：") && a.url.startsWith("/execution-reports/") && a.url.endsWith("/report.json"));
    const next = JSON.stringify([link?.url, scenes.map(s => [s.id, s.visual_config])]);
    if (key === next) return;
    key = next;
    const token = ++epoch;
    const progress = document.getElementById("benchmark-progress");
    const counts = document.getElementById("benchmark-counts");
    const reportLink = document.getElementById("benchmark-report");
    progress.textContent = "严格门禁：等待报告";
    counts.textContent = "不以任务成功替代安全验收";
    reportLink.hidden = true;
    if (!link) return;
    try {
      const response = await fetch(link.url);
      if (!response.ok) throw Error("HTTP " + response.status);
      const report = await response.json();
      if (token !== epoch) return;
      const ids = new Set(report.scenes?.map(s => s.scene));
      if (report.total !== 9 || report.scenes?.length !== 9 || ids.size !== 9 ||
          !scenes.every(s => ids.has(s.id))) throw Error("报告场景覆盖不完整");
      const passed = report.scenes.filter(r => r.accepted === true &&
        scenes.some(s => s.id === r.scene && s.visual_config === r.current_visual_config)).length;
      progress.textContent = `严格门禁快照 ${passed} / 9`;
      counts.textContent = `${passed === 9 && report.accepted === true ? "全部通过（仅报告声明范围）" : "尚未全部通过"} · ${report.created_utc || "报告时间缺失"} · 后续运行不会自动补成通过`;
      reportLink.href = link.url;
      reportLink.hidden = false;
    } catch (error) {
      if (token !== epoch) return;
      key = "";
      progress.textContent = "严格门禁：报告不可用";
      counts.textContent = error.message + "；不得视为验收通过";
    }
  };
})();
