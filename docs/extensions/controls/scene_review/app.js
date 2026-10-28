"use strict";
const $ = id => document.getElementById(id);
const fields = ["status", "observations", "plan", "verification", "notes", "planning_review", "planning_notes", "execution_notes"];
const labels = {pending:"尚未初评", good:"初评较优", change:"需要修改", uncertain:"信息不足"};
const visualLabels = {pending:"待视觉复审",reviewing:"正在看图",adjusting:"正在调整并重渲染",passed:"视觉通过",blocked:"视觉问题未解决"};
const planningLabels = {pending:"待生成",running:"正在生成",generated:"已返回轨迹 · 未验收",failed:"生成失败 · 已跳过",skipped:"已跳过"};
const executionLabels = {pending:"待完整执行",baseline:"匹配基线采集中",running:"正在物理执行",recorded:"已记录 · 待完整验证",passed:"任务与数据验证通过",failed:"执行或验证失败"};
let scenes = [], selected = null, filter = "all", online = false;
const drafts = new Map(), busy = new Set(), conflicts = new Set(), errors = new Map(), timers = new Map();
const draftKey = "robolaw-scene-review-drafts-v1";
try { for (const [id, draft] of JSON.parse(localStorage.getItem(draftKey) || "[]")) drafts.set(id, draft); } catch (_) {}
function persist() {
  try { localStorage.setItem(draftKey, JSON.stringify([...drafts])); }
  catch (_) { $("connection").textContent = "浏览器草稿备份不可用；请确认服务端保存成功"; }
}
function visible() {
  return scenes.filter(s => filter === 'all' ||
    (filter === 'execution_pending' ? s.execution_status !== 'passed' :
     filter === 'execution_failed' ? s.execution_status === 'failed' :
     filter === 'execution_passed' ? s.execution_status === 'passed' : s.status === filter));
}
function summary() {
  const complete = scenes.filter(s => s.status !== "pending").length;
  $("progress").textContent = `${complete} / ${scenes.length}`;
  $("bar").style.width = `${scenes.length ? complete / scenes.length * 100 : 0}%`;
  $("counts").replaceChildren(...Object.entries(labels).map(([status, label]) => {
    const el = document.createElement("span"); el.className = status;
    el.textContent = `${label} ${scenes.filter(s => s.status === status).length}`; return el;
  }));
  $("visual-progress").textContent = `视觉修订 ${scenes.filter(s => s.visual_status === "passed").length} / ${scenes.length}`;
  $("visual-counts").textContent = Object.entries(visualLabels).map(([k,v]) => `${v} ${scenes.filter(s => s.visual_status===k).length}`).join(" · ");
  const finished = scenes.filter(s => ['generated','failed','skipped'].includes(s.planning_status)).length;
  $("planning-progress").textContent = `轨迹生成 ${finished} / ${scenes.length}`;
  const auxiliary = scenes.filter(s => {
    if (s.planning_status !== 'generated') return false;
    try { return JSON.parse(s.planning_log).selected_endpoint_policy?.auxiliary_pregrasp_height_m > 0; }
    catch (_) { return false; }
  }).length;
  $("planning-counts").textContent = `返回轨迹 ${scenes.filter(s=>s.planning_status==='generated').length} · 其中辅助预抓取候选 ${auxiliary}（原终点未达） · 失败跳过 ${scenes.filter(s=>['failed','skipped'].includes(s.planning_status)).length} · 运行中 ${scenes.filter(s=>s.planning_status==='running').length} · 全部未做安全验收`;
  $("execution-progress").textContent = `完整执行 ${scenes.filter(s=>s.execution_status==='passed').length} / ${scenes.length}`;
  $("execution-counts").textContent = `待验证 ${scenes.filter(s=>s.execution_status==='recorded').length} · 失败 ${scenes.filter(s=>s.execution_status==='failed').length} · 进行中 ${scenes.filter(s=>['running','baseline'].includes(s.execution_status)).length} · 不继承历史布局的验收结论`;
  window.renderBenchmarkSummary?.(scenes);
}
function showImage(url, label) {
  $("hero").src = url; $("original").href = url; $("filename").textContent = label;
}
function visualReview(s, force=false) {
  window.renderPlanning?.(s, force);
  $("execution-state").textContent = executionLabels[s.execution_status] || executionLabels.pending;
  $("execution-state").className = s.execution_status === 'passed' ? 'good' : 'change';
  $("execution-hint").textContent = s.execution_hint || '接近路径不代表完整任务；真实按压/抓持/旋转与数据证据在此单独记录。';
  $("execution-log").textContent = s.execution_log || '尚无本轮物理执行结果。';
  $("execution-files").replaceChildren(...(s.execution_artifacts || []).map(a=>{
    const link=document.createElement('a');link.href=a.url;link.textContent=a.label;
    link.target='_blank';link.rel='noopener';return link;
  }));
  const videoFile=(s.execution_artifacts || []).find(a=>a.url.endsWith('/all_cameras.mp4') && a.label.startsWith('本轮实采'));
  const video=$("execution-video"), videoLabel=$("execution-video-label");
  video.hidden=videoLabel.hidden=!videoFile;
  if (videoFile) {
    videoLabel.textContent=(s.execution_status==='passed' ? '已通过的实采视频：' : '历史/最近实采视频，当前修订版尚未验收：')+
      videoFile.url+'。请结合任务谓词与接触记录查看，不代表有效避障验收。';
    if (video.getAttribute('src')!==videoFile.url) video.src=videoFile.url;
  } else if (video.hasAttribute('src')) {
    video.removeAttribute('src');video.load();
  }
  const comparisonFile=(s.execution_artifacts || []).find(a=>a.label.startsWith('同视角对照') && a.url.endsWith('.mp4'));
  const comparison=$("comparison-video"), comparisonLabel=$("comparison-video-label");
  comparison.hidden=comparisonLabel.hidden=!comparisonFile;
  if (comparisonFile) {
    comparisonLabel.textContent=(s.execution_status==='passed' ? '原速同视角对照：' : '历史版本对照，不代表当前修订版验收：')+
      '左为匹配基线，右为所记录版本实采，均保持15帧/秒。较短片段结束后停留末帧，并非额外实采；不是避障必要性证明。';
    if (comparison.getAttribute('src')!==comparisonFile.url) comparison.src=comparisonFile.url;
  } else if (comparison.hasAttribute('src')) {
    comparison.removeAttribute('src');comparison.load();
  }
  $("visual-status").textContent = visualLabels[s.visual_status] || "待视觉复审";
  $("visual-status").className = s.visual_status === "passed" ? "good" : "change";
  $("visual-log").textContent = s.visual_log || "逐场景审核尚未开始。结论只来自图像，不运行碰撞、距离或轨迹验收。";
  $("visual-config").textContent = s.visual_config ? "本轮独立配置：" + s.visual_config + "（未做物理/安全验收）" : "未修改仿真布局";
  const entries = [{url:s.image,label:"修改前 / 原始截图"},...(s.visual_images || [])];
  const key = JSON.stringify(entries);
  if (force || $("visual-gallery").dataset.entries !== key) {
    $("visual-gallery").dataset.entries = key;
    $("visual-gallery").replaceChildren(...entries.map(entry => {
      const button = document.createElement("button"), img = document.createElement("img"), label = document.createElement("span");
      img.src = entry.url; img.alt = entry.label; label.textContent = entry.label;
      button.append(img,label); button.onclick = () => showImage(entry.url,entry.label); return button;
    }));
    const latest = entries[entries.length-1]; showImage(latest.url,latest.label);
  }
}
function renderList() {
  $("scenes").replaceChildren(...visible().map(s => {
    const b = document.createElement("button"); b.className = "scene" + (s.id === selected ? " selected" : "");
    b.onclick = () => choose(s.id);
    const img = document.createElement("img"); img.src = s.image; img.alt = s.title; img.loading = "lazy";
    const text = document.createElement("span"), title = document.createElement("b"), status = document.createElement("small");
    const executionActive=s.execution_status && s.execution_status !== 'pending';
    title.textContent = s.title; status.textContent = (executionActive ? executionLabels[s.execution_status] : s.planning_status && s.planning_status !== 'pending' ? planningLabels[s.planning_status] : s.visual_status && s.visual_status !== 'pending' ? visualLabels[s.visual_status] : labels[s.status]) + (drafts.has(s.id) ? " · 有未保存草稿" : "");
    status.className=executionActive ? s.execution_status==='passed' ? 'good' : s.execution_status==='failed' ? 'change' : 'pending' : s.planning_status === 'failed' ? 'change' : s.visual_status === 'passed' ? 'good' : s.status;
    text.append(title, status); b.append(img, text); return b;
  }));
  const list = visible(), index = list.findIndex(s => s.id === selected);
  $("prev").disabled = index <= 0; $("next").disabled = index < 0 || index >= list.length - 1;
}
function saveState() {
  if (!selected) return;
  const s = scenes.find(s => s.id === selected);
  $("save-state").textContent = conflicts.has(selected) ? "保存冲突 · 草稿保留" : busy.has(selected) ? "正在保存…" : errors.has(selected) ? errors.get(selected) : drafts.has(selected) ? "待保存 · 草稿已备份" : "已保存到本地磁盘";
  $("save-state").style.color = conflicts.has(selected) || errors.has(selected) ? "#b45309" : "#08785c";
  $("conflict").hidden = !conflicts.has(selected);
  $("stamp").textContent = s.updated_at ? `更新于 ${new Date(s.updated_at).toLocaleString()} · v${s.revision}` : "等待初评";
}
function choose(id) {
  selected = id;
  const s = scenes.find(s => s.id === id);
  $("detail").hidden = !s; $("empty").hidden = !!s;
  if (!s) return;
  $("title").textContent = s.title; $("scene-number").textContent = `SCENE ${s.id.slice(0,2)} / 09`;
  $("hero").src = s.image; $("hero").alt = s.title; $("original").href = s.image;
  $("filename").textContent = s.id + ".jpg";
  visualReview(s, true);
  for (const key of fields) $(key).value = drafts.get(id)?.changes[key] ?? s[key];
  renderList(); saveState();
  location.hash = id;
}
async function refresh() {
  try {
    const response = await fetch("/api/scenes"); if (!response.ok) throw new Error("HTTP " + response.status);
    const data = await response.json(); scenes = data.scenes; online = true;
    $("connection").textContent = "本机服务已连接 · 每3秒同步进度";
    summary();
    if (!selected) choose(scenes.some(s => s.id === location.hash.slice(1)) ? location.hash.slice(1) : visible()[0]?.id);
    else {
      renderList(); saveState();
      const s = scenes.find(s => s.id === selected);
      visualReview(s);
      for (const key of fields) if (!(key in (drafts.get(selected)?.changes || {})) && document.activeElement !== $(key)) $(key).value = s[key];
    }
    for (const id of drafts.keys()) if (!busy.has(id) && !conflicts.has(id) && !timers.has(id)) schedule(id, 100);
  } catch (_) { online = false; $("connection").textContent = "连接中断 · 草稿保留，正在重试"; }
}
function schedule(id, delay=700) {
  clearTimeout(timers.get(id));
  timers.set(id, setTimeout(() => { timers.delete(id); save(id); }, delay));
}
function edit(key, value) {
  const s = scenes.find(s => s.id === selected);
  const d = drafts.get(selected) || {base:{}, changes:{}};
  if (!(key in d.base)) d.base[key] = s[key];
  d.changes[key] = value; drafts.set(selected, d);
  errors.delete(selected); persist(); saveState(); renderList(); schedule(selected);
}
async function save(id) {
  if (busy.has(id) || conflicts.has(id) || !drafts.has(id)) return;
  const sent = JSON.parse(JSON.stringify(drafts.get(id)));
  busy.add(id); saveState();
  try {
    const response = await fetch(`/api/scenes/${id}`, {method:"PATCH", headers:{"Content-Type":"application/json"}, body:JSON.stringify({...sent, actor:"browser"})});
    const result = await response.json();
    if (response.status === 409) { conflicts.add(id); throw new Error(result.error); }
    if (!response.ok) throw new Error(result.error || "保存失败");
    const d = drafts.get(id);
    for (const key of Object.keys(sent.changes)) {
      if (d.changes[key] === sent.changes[key]) { delete d.changes[key]; delete d.base[key]; }
      else d.base[key] = result[key];
    }
    if (!Object.keys(d.changes).length) drafts.delete(id);
    scenes = scenes.map(s => s.id === id ? result : s); errors.delete(id); persist(); summary(); renderList();
  } catch (e) { errors.set(id, "未保存到磁盘 · " + e.message); }
  finally { busy.delete(id); saveState(); if (drafts.has(id) && !conflicts.has(id) && !errors.has(id)) schedule(id,100); }
}
for (const key of fields) $(key).addEventListener(["status","planning_review"].includes(key) ? "change" : "input", e => edit(key,e.target.value));
$("filter").onchange = e => { filter = e.target.value; const list = visible(); choose(list.some(s => s.id === selected) ? selected : list[0]?.id); renderList(); };
for (const [button, delta] of [["prev",-1],["next",1]]) $(button).onclick = () => { const list = visible(); const i = list.findIndex(s => s.id === selected); if (list[i+delta]) choose(list[i+delta].id); };
$("image-open").onclick = () => { $("full-image").src = $("hero").src; $("lightbox").showModal(); };
$("close-image").onclick = () => $("lightbox").close();
$("use-remote").onclick = () => { if (!confirm("放弃当前场景本页未保存草稿？已保存的历史记录不受影响。")) return; drafts.delete(selected); conflicts.delete(selected); errors.delete(selected); persist(); choose(selected); };
$("retry-local").onclick = async () => { if (!confirm("确认用本页草稿覆盖这些字段的最新内容？历史版本仍会保留。")) return; await refresh(); if (!online) return; const s = scenes.find(s => s.id === selected), d = drafts.get(selected); if (!d) return; for (const key of Object.keys(d.changes)) d.base[key] = s[key]; conflicts.delete(selected); errors.delete(selected); persist(); save(selected); };
window.addEventListener("beforeunload", e => { if (drafts.size) { persist(); e.preventDefault(); e.returnValue = ""; } });
window.addEventListener("hashchange", () => { const id = location.hash.slice(1); if (id !== selected && scenes.some(s => s.id === id)) choose(id); });
refresh(); setInterval(refresh,3000);
