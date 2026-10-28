"use strict";
(() => {
  let epoch = 0, key = "";
  const el = id => document.getElementById(id);
  const labels = {retain_visual:"视觉上可保留", revised_visual_passed:"新摆位视觉通过",
    retain_as_controlled_fixture:"受控实验夹具可保留；家居语义一般"};
  window.renderVisualProposal = async scene => {
    const link = (scene.planning_artifacts || []).find(a =>
      a.label.startsWith("本轮逐场景视觉判断") && a.url.startsWith("/plans/") && a.url.endsWith("/round_history.json"));
    const next = JSON.stringify([scene.id, scene.visual_config, link?.url]);
    if (next === key) return;
    key = next;
    const token = ++epoch, host = el("visual-proposal"), button = el("visual-proposal-append");
    host.hidden = true;
    button.disabled = true;
    button.onclick = null;
    if (!link) return;
    try {
      const response = await fetch(link.url);
      if (!response.ok) throw Error("HTTP " + response.status);
      const report = await response.json();
      if (token !== epoch) return;
      const entries = (report.scene_reviews || []).filter(r => r.scene === scene.id);
      if (entries.length !== 1 || entries[0].visual_config !== scene.visual_config)
        throw Error("本轮建议与当前场景版本不匹配");
      const item = entries[0];
      if (typeof item.adjustment !== "string" || !item.adjustment || typeof item.observations !== "string")
        throw Error("视觉建议内容缺失");
      el("visual-proposal-verdict").textContent = labels[item.verdict] || "视觉结论未分类";
      el("visual-proposal-observations").textContent = "画面依据：" + item.observations;
      el("visual-proposal-adjustment").textContent = "调整建议：" + item.adjustment;
      el("visual-proposal-scope").textContent = "仅图像判断；接近轨迹、真实任务与安全验收分开。下方可填写你的决定并自动保存。";
      button.disabled = false;
      button.onclick = () => {
        const plan = el("plan"), addition = "本轮视觉建议：" + item.adjustment;
        if (!plan.value.includes(addition)) {
          plan.value += (plan.value ? "\n\n" : "") + addition;
          plan.dispatchEvent(new Event("input", {bubbles:true}));
        }
        plan.focus();
        plan.scrollIntoView({block:"center", behavior:"smooth"});
      };
      host.hidden = false;
    } catch (error) {
      if (token !== epoch) return;
      key = "";
      el("visual-proposal-verdict").textContent = "视觉摘要暂不可用：" + error.message;
      el("visual-proposal-observations").textContent = "";
      el("visual-proposal-adjustment").textContent = "";
      el("visual-proposal-scope").textContent = "保留你的原标注；可查看逐轮日志和原始报告。";
      host.hidden = false;
    }
  };
})();
