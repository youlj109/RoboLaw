"use strict";
(() => {
  const el = id => document.getElementById(id);
  const canvas = el("trajectory-canvas"), ctx = canvas.getContext("2d");
  let data = null, key = "", epoch = 0, frame = 0, yaw = -.65, pitch = .55, zoom = 1;
  let center = [0,0,0], span = 1, playing = false, lastTime = 0, lastFrame = 0, drag = null, raf = null;
  const labels = {pending:"待生成",running:"正在生成",generated:"已返回轨迹 · 未验收",failed:"失败 · 已跳过",skipped:"已跳过",annotated:"已标注 · 未改规划"};
  let historyEpoch = 0;
  async function showRoundHistory(scene) {
    const token=++historyEpoch, host=el('planning-round-history');
    const file=(scene.planning_artifacts || []).find(a=>a.url.endsWith('/round_history.json'));
    if(!file){host.hidden=true;host.replaceChildren();return;}
    try {
      const response=await fetch(file.url);if(!response.ok)throw Error('HTTP '+response.status);
      const history=await response.json();if(token!==historyEpoch)return;
      const title=document.createElement('h4');title.textContent='连续轮次：一轮一次改动，失败留到下一轮';
      const table=document.createElement('table'),head=document.createElement('tr');
      for(const label of ['轮次','本轮唯一改动','结果','路径查询次数']){const cell=document.createElement('th');cell.textContent=label;head.append(cell);}
      table.append(head);
      for(const round of history.rounds || []) for(const item of round.cases || []) {
        if(item.scene!==scene.id)continue;
        const row=document.createElement('tr');
        for(const value of [round.round,item.action,labels[item.status] || item.status,item.motiongen_calls ?? 0]){
          const cell=document.createElement('td');cell.textContent=String(value);row.append(cell);
        }
        table.append(row);
      }
      host.replaceChildren(title,table);host.hidden=false;
    }catch(e){if(token===historyEpoch){host.textContent='轮次记录加载失败：'+e.message;host.hidden=false;}}
  }
  function stop() { playing = false; if(raf!==null)cancelAnimationFrame(raf);raf=null;el("trajectory-play").textContent = "播放"; }
  function fit() {
    const points = data.frames.flat().concat([data.target],data.boxes.filter(b=>b.hazard).flatMap(b=>[b.low,b.high]));
    const low = [0,1,2].map(i=>Math.min(...points.map(p=>p[i]))), high = [0,1,2].map(i=>Math.max(...points.map(p=>p[i])));
    center = low.map((v,i)=>(v+high[i])/2); span = Math.max(...high.map((v,i)=>v-low[i]),.6);
    yaw = -.65; pitch = .55; zoom = 1;
  }
  function draw() {
    if (!data || el("trajectory-wrap").hidden) return;
    const width = Math.max(300,canvas.clientWidth), height = canvas.clientHeight;
    const ratio = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(width*ratio) || canvas.height !== Math.round(height*ratio)) {
      canvas.width = Math.round(width*ratio); canvas.height = Math.round(height*ratio);
    }
    ctx.setTransform(ratio,0,0,ratio,0,0); ctx.clearRect(0,0,width,height);
    const scale = Math.min(width,height)*.72/span*zoom;
    const project = p => {
      const [x,y,z] = p.map((v,i)=>v-center[i]);
      const u = x*Math.cos(yaw)-y*Math.sin(yaw), d=x*Math.sin(yaw)+y*Math.cos(yaw);
      return [width/2+u*scale,height/2-(z*Math.cos(pitch)-d*Math.sin(pitch))*scale];
    };
    function line(points,color,thickness=1) {
      if (!points.length) return;
      ctx.beginPath(); points.forEach((p,i)=>{const [x,y]=project(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
      ctx.strokeStyle=color;ctx.lineWidth=thickness;ctx.stroke();
    }
    const edges=[[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
    for (const b of data.boxes) {
      const vertices = Array.from({length:8},(_,i)=>[0,1,2].map(j=>(i&(1<<j))?b.high[j]:b.low[j]));
      for (const [a,c] of edges) line([vertices[a],vertices[c]],b.hazard?"#dc8418aa":"#99aaba33",b.hazard?1.5:.6);
    }
    line(data.tcp,"#0eaaa8",2);
    const body = data.frames[frame];
    line(body,"#2f65b2",5);
    for (const p of body) { const [x,y]=project(p);ctx.beginPath();ctx.arc(x,y,3,0,2*Math.PI);ctx.fillStyle="#264e81";ctx.fill(); }
    const [tx,ty]=project(data.target);ctx.strokeStyle="#9a47bb";ctx.lineWidth=2;ctx.strokeRect(tx-5,ty-5,10,10);
    const [sx,sy]=project(data.tcp[0]);ctx.fillStyle="#107c5e";ctx.beginPath();ctx.arc(sx,sy,4,0,Math.PI*2);ctx.fill();
    ctx.font="12px sans-serif";ctx.fillStyle="#52677d";ctx.fillText(data.status==='generated'?"规划轨迹 · 非物理执行 / 非安全验证":"生成失败：仅显示起点和接近目标",12,23);
    el("trajectory-frame").value=frame;
    el("trajectory-time").textContent=`${(frame*data.dt).toFixed(2)} s · ${frame+1}/${data.frames.length}`;
    el("trajectory-q").textContent="q [rad]: " + data.q[frame].map(v=>v.toFixed(3)).join("  ");
  }
  function tick(time) {
    if (!playing || !data) return;
    frame=Math.min(data.frames.length-1,lastFrame+Math.floor((time-lastTime)/1000/data.dt));draw();
    if (frame===data.frames.length-1) stop(); else raf=requestAnimationFrame(tick);
  }
  window.renderPlanning = async (scene, force=false) => {
    showRoundHistory(scene);
    let auxiliary = false;
    try { auxiliary = JSON.parse(scene.planning_log).selected_endpoint_policy?.auxiliary_pregrasp_height_m > 0; }
    catch (_) { /* Old logs may be plain text. */ }
    el("planning-state").textContent=labels[scene.planning_status] || labels.pending;
    if (scene.planning_status === "generated" && auxiliary)
      el("planning-state").textContent="已返回辅助预抓取轨迹 · 原终点未达";
    el("planning-state").className=scene.planning_status==="generated" && !auxiliary?"good":"change";
    el("planning-hint").textContent=scene.planning_hint || "本轮每场景只尝试一次；失败自动跳过，等待人工标注，不自动修改场景。";
    el("planning-log").textContent=scene.planning_log || "暂无规划结果";
    const artifacts=scene.planning_artifacts || [];
    const fileLinks=artifacts.map(a=>{
      const link=document.createElement("a");link.href=a.url;link.textContent=a.label;link.target="_blank";link.rel="noopener";
      if (a.url.endsWith(".csv") || a.url.endsWith(".npy")) link.download=a.url.split("/").pop();return link;
    });
    const visibleLinks=fileLinks.slice(0,7);
    if(fileLinks.length>7){
      const archive=document.createElement('details'),summary=document.createElement('summary');
      summary.textContent=`历史记录文件（${fileLinks.length-7}）`;archive.append(summary,...fileLinks.slice(7));visibleLinks.push(archive);
    }
    el("planning-files").replaceChildren(...visibleLinks);
    const preview=artifacts.find(a=>a.url.endsWith("/preview.json"));
    const nextKey=scene.id+":"+(preview?.url || "none")+":"+scene.planning_status;
    if (!force && key===nextKey) return;
    key=nextKey;const token=++epoch;stop();data=null;el("trajectory-wrap").hidden=true;
    if (!preview) return;
    try {
      const response=await fetch(preview.url);if(!response.ok)throw Error("HTTP "+response.status);
      const result=await response.json();if(token!==epoch)return;
      data=result;frame=0;fit();el("trajectory-frame").max=data.frames.length-1;
      el("trajectory-play").disabled=data.frames.length<2;el("trajectory-wrap").hidden=false;draw();
    } catch (e) { if(token===epoch)el("planning-hint").textContent += " 预览加载失败："+e.message; }
  };
  el("trajectory-frame").oninput=e=>{stop();frame=Number(e.target.value);draw();};
  el("trajectory-play").onclick=()=>{
    if(playing)return stop();if(!data || data.frames.length<2)return;
    if(frame===data.frames.length-1)frame=0;playing=true;lastTime=performance.now();lastFrame=frame;
    el("trajectory-play").textContent="暂停";raf=requestAnimationFrame(tick);
  };
  el("trajectory-reset").onclick=()=>{if(data){fit();draw();}};
  canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw();};
  canvas.onpointerup=()=>{drag=null;};canvas.onpointercancel=()=>{drag=null;};
  canvas.addEventListener("wheel",e=>{e.preventDefault();zoom=Math.max(.3,Math.min(5,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
  new ResizeObserver(draw).observe(canvas);
})();
