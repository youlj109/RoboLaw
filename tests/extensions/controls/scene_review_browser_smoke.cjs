// Run against a separate --port 8766 --data-dir scratch server, never production.
// Firefox: --headless --no-remote --profile SCRATCH --remote-debugging-port 9223.
// Set PW_WS_MODULE to an installed playwright-core/lib/utilsBundle.js file.
"use strict";
const WS = require(process.env.PW_WS_MODULE).ws;
const fs = require("fs");
const assert = require("assert");
const path = require("path");
const socket = new WS("ws://127.0.0.1:9223/session");
let seq = 0;
const pending = new Map();
socket.on("message", data => {
  const m = JSON.parse(data), p = pending.get(m.id);
  if (m.method === "log.entryAdded" && m.params?.level === "error")
    console.error("BROWSER_ERROR", JSON.stringify(m.params));
  if (p) {
    pending.delete(m.id);
    m.type === "error" ? p.reject(Error(JSON.stringify(m))) : p.resolve(m.result);
  }
});
socket.on("error", e => { console.error(e); process.exitCode = 1; });
function call(method, params = {}) {
  return new Promise((resolve, reject) => {
    const id = ++seq;
    pending.set(id, {resolve, reject});
    socket.send(JSON.stringify({id, method, params}));
  });
}
const pause = ms => new Promise(r => setTimeout(r, ms));
async function evaluate(context, expression) {
  const r = await call("script.evaluate", {
    expression: `(async()=>JSON.stringify(await (async()=>{${expression}})()))()`,
    target: {context}, awaitPromise: true,
  });
  if (r.type !== "success") throw Error(JSON.stringify(r));
  return r.result.value === undefined ? null : JSON.parse(r.result.value);
}
async function waitFor(ctx, expression) {
  for (let i = 0; i < 300; i++) {
    if (await evaluate(ctx, `return ${expression};`)) return;
    await pause(150);
  }
  const debug=await evaluate(ctx, 'return {url:location.href,ready:document.readyState,scenes:document.querySelectorAll(".scene").length,body:document.body.innerText.slice(0,1000)};');
  throw Error("timeout: " + expression + " " + JSON.stringify(debug));
}
async function screenshot(ctx, name) {
  const image = await call("browsingContext.captureScreenshot", {context:ctx, origin:"document"});
  const output = path.resolve(__dirname, "../../../docs/extensions/controls/scene_review/data",
                              (process.env.SCENE_REVIEW_CAPTURE_PREFIX || "") + name);
  fs.writeFileSync(output, Buffer.from(image.data, "base64"));
}
socket.on("open", async () => {
  try {
    await call("session.new", {capabilities:{}});
    await call("session.subscribe", {events:["log.entryAdded"]});
    const tree = await call("browsingContext.getTree"), production = tree.contexts[0].context;
    await call("browsingContext.setViewport", {context:production, viewport:{width:1560,height:1100},devicePixelRatio:1});
    await call("browsingContext.navigate", {context:production,url:"http://127.0.0.1:8765",wait:"complete"});
    await waitFor(production, 'document.querySelectorAll(".scene").length===9 && document.querySelector("#hero").complete');
    assert.equal(await evaluate(production, 'return document.querySelector("#progress").textContent;'), "9 / 9");
    if (process.env.SCENE_REVIEW_STRICT_BENCHMARK === "1") {
      await waitFor(production, '!document.querySelector("#benchmark-report").hidden');
      assert(await evaluate(production, 'return document.querySelector("#benchmark-progress").textContent.startsWith("严格门禁快照") && document.querySelector("#benchmark-counts").textContent.includes("后续运行不会自动补成通过");'));
    }
    if (process.env.SCENE_REVIEW_BENCHMARK === "1") {
      await waitFor(production, '!document.querySelector("#benchmark-screen").hidden');
      assert(await evaluate(production, 'return document.querySelector("#benchmark-screen-event").textContent.includes("panda_link7") && document.querySelector("#benchmark-screen-event").textContent.includes("手腕世界坐标");'));
      await evaluate(production, 'location.hash="03_left_cancel_obstacle";return true;');
      await waitFor(production, 'document.querySelector("#benchmark-screen-verdict").textContent.includes("无需避让")');
      await evaluate(production, 'location.hash="01_left_cancel_human";return true;');
      await waitFor(production, 'document.querySelector("#benchmark-screen-event").textContent.includes("手腕世界坐标")');
    }
    assert(await evaluate(production, 'return document.querySelector("#execution-progress").textContent.includes("/ 9") && !!document.querySelector("#execution_notes");'));
    await waitFor(production, '!document.querySelector("#execution-video").hidden && document.querySelector("#execution-video").readyState>=1');
    assert(await evaluate(production, 'const v=document.querySelector("#execution-video");return v.duration>0 && v.videoWidth>0 && v.getAttribute("src").includes("/01_left_cancel_human/");'));
    if (process.env.SCENE_REVIEW_COMPARISONS === "1") {
      await waitFor(production, '!document.querySelector("#comparison-video").hidden && document.querySelector("#comparison-video").readyState>=1');
      assert(await evaluate(production, 'const v=document.querySelector("#comparison-video");return v.duration>0 && v.videoWidth===1280 && document.querySelector("#comparison-video-label").textContent.includes("并非额外实采");'));
    }
    if (process.env.SCENE_REVIEW_PLANNING === "1") {
      assert.equal(await evaluate(production, 'return document.querySelector("#planning-progress").textContent;'), "轨迹生成 9 / 9");
      await waitFor(production, '!document.querySelector("#trajectory-wrap").hidden');
      assert(await evaluate(production, 'return Number(document.querySelector("#trajectory-frame").max)>1;'));
      await evaluate(production, 'const s=document.querySelector("#trajectory-frame");s.value=1;s.dispatchEvent(new Event("input"));return true;');
      assert(await evaluate(production, 'return document.querySelector("#trajectory-time").textContent.includes("2/");'));
      await evaluate(production, 'document.querySelector("#trajectory-play").click();return true;');
      await pause(170);
      await evaluate(production, 'document.querySelector("#trajectory-play").click();return true;');
      const failedId=await evaluate(production, 'const r=await (await fetch("/api/scenes")).json();return r.scenes.find(s=>s.planning_status==="failed")?.id || null;');
      if(failedId){
        await evaluate(production, `location.hash=${JSON.stringify(failedId)};return true;`);
        await waitFor(production, 'document.querySelector("#planning-state").textContent.includes("失败") && !document.querySelector("#trajectory-wrap").hidden');
        assert(await evaluate(production, 'return document.querySelector("#trajectory-play").disabled;'));
      }
      await evaluate(production, 'location.hash="01_left_cancel_human";return true;');
      await waitFor(production, 'document.querySelector("#planning-state").textContent.includes("已返回") && !document.querySelector("#trajectory-wrap").hidden');
    }
    if (process.env.SCENE_REVIEW_ROUNDS === "1") {
      await evaluate(production, 'location.hash="08_oven_temperature_property";return true;');
      await waitFor(production, 'document.querySelectorAll("#planning-round-history tr").length>=4 && !document.querySelector("#trajectory-wrap").hidden');
      const roundScene = await evaluate(production, 'const r=await (await fetch("/api/scenes")).json();return r.scenes.find(s=>s.id==="08_oven_temperature_property");');
      if (process.env.SCENE_REVIEW_QUALITY === "1") {
        assert.equal(roundScene.execution_status, "recorded");
        assert(roundScene.visual_config.endsWith("08_oven_property_r06.json"));
        assert(await evaluate(production, 'return document.querySelector("#execution-video-label").textContent.includes("当前修订版尚未验收") && document.querySelector("#comparison-video-label").textContent.includes("历史版本对照");'));
      }
      assert(["generated", "failed"].includes(roundScene.planning_status), "Run smoke test after planning jobs finish");
      const expectedState = roundScene.planning_status === "generated" ? "已返回" : "失败";
      await waitFor(production, `document.querySelector("#planning-state").textContent.includes(${JSON.stringify(expectedState)})`);
      const roundResult = JSON.parse(roundScene.planning_log);
      if (roundScene.planning_status === "generated" && roundResult.selected_endpoint_policy?.auxiliary_pregrasp_height_m > 0) {
        assert(await evaluate(production, 'const r=await (await fetch("/api/scenes")).json();const n=r.scenes.filter(s=>{try{return s.planning_status==="generated" && JSON.parse(s.planning_log).selected_endpoint_policy?.auxiliary_pregrasp_height_m>0;}catch(_){return false;}}).length;return document.querySelector("#planning-state").textContent.includes("原终点未达") && document.querySelector("#planning-counts").textContent.includes("辅助预抓取候选 "+n);'));
      }
      assert.equal(await evaluate(production, 'return document.querySelector("#trajectory-play").disabled;'), roundScene.planning_status === "failed");
      assert(await evaluate(production, 'return document.querySelector("#planning-round-history").textContent.includes("失败") && document.querySelector("#planning-round-history").textContent.includes("15厘米");'));
    }
    if (process.env.SCENE_REVIEW_QUALITY_02 === "1") {
      for (const [id,config,image] of [["06_right_cancel_obstacle","06_right_obstacle_r05.json","/r05/side.jpg"],["07_oven_temperature_human","07_oven_human_r02.json","/r02/side.jpg"]]) {
        await evaluate(production, `location.hash=${JSON.stringify(id)};return true;`);
        await waitFor(production, 'document.querySelector("#planning-state").textContent.includes("已返回") && !document.querySelector("#trajectory-wrap").hidden');
        const row=await evaluate(production, `return (await (await fetch("/api/scenes")).json()).scenes.find(s=>s.id===${JSON.stringify(id)});`);
        assert(row.visual_config.endsWith(config) && row.execution_status==="recorded");
        await waitFor(production, `document.querySelector("#hero").getAttribute("src").endsWith(${JSON.stringify(image)}) && Number(document.querySelector("#trajectory-frame").max)===${JSON.parse(row.planning_log).samples-1}`);
        assert(await evaluate(production, 'return document.querySelector("#execution-video-label").textContent.includes("当前修订版尚未验收");'));
      }
      await evaluate(production, 'location.hash="08_oven_temperature_property";return true;');
      await waitFor(production, 'document.querySelector("#hero").getAttribute("src").includes("/r06/")');
    }
    if (process.env.SCENE_REVIEW_VISUAL === "1") {
      assert.equal(await evaluate(production, 'return document.querySelector("#visual-progress").textContent;'), "视觉修订 9 / 9");
      assert(await evaluate(production, 'return document.querySelectorAll("#visual-gallery button").length>1;'));
      await evaluate(production, 'document.querySelector("#visual-gallery button").click();return true;');
      assert(await evaluate(production, 'return document.querySelector("#hero").getAttribute("src").startsWith("/images/");'));
      await evaluate(production, 'document.querySelector("#visual-gallery button:last-child").click();return true;');
      await waitFor(production, 'document.querySelector("#hero").complete && document.querySelector("#hero").naturalWidth>0');
      assert(await evaluate(production, 'return document.querySelector("#hero").getAttribute("src").startsWith("/renders/");'));
    }
    if (process.env.SCENE_REVIEW_VISUAL_PROPOSAL === "1") {
      for (const [id,version] of [["03_left_cancel_obstacle","r12"],["06_right_cancel_obstacle","r07"]]) {
        await evaluate(production, `location.hash=${JSON.stringify(id)};return true;`);
        await waitFor(production, '!document.querySelector("#visual-proposal").hidden && !document.querySelector("#visual-proposal-append").disabled');
        await waitFor(production, `document.querySelector("#hero").complete && document.querySelector("#hero").currentSrc.includes("/${version}/fixture_detail.jpg") && document.querySelector("#hero").naturalWidth>0`);
        await waitFor(production, 'document.querySelectorAll("#planning-round-history tr").length>=4');
        assert(await evaluate(production, 'return document.querySelector("#visual-proposal-verdict").textContent.includes("新摆位视觉通过") && document.querySelector("#visual-proposal-adjustment").textContent.includes("冻结");'));
        await waitFor(production, 'document.querySelector("#benchmark-progress").textContent==="严格门禁快照 7 / 9"');
        await screenshot(production, id + "_proposal.png");
      }
      await evaluate(production, 'location.hash="01_left_cancel_human";return true;');
    }
    await screenshot(production, "page_desktop.png");
    const tab = await call("browsingContext.create", {type:"tab"}), ctx = tab.context;
    await call("browsingContext.navigate", {context:ctx,url:"http://127.0.0.1:8766",wait:"complete"});
    await waitFor(ctx, 'document.querySelectorAll(".scene").length===9');
    if (process.env.SCENE_REVIEW_VISUAL_PROPOSAL === "1") {
      const source=await evaluate(production, 'return (await (await fetch("/api/scenes")).json()).scenes.find(s=>s.id==="03_left_cancel_obstacle");');
      const changes={visual_config:source.visual_config,planning_artifacts:source.planning_artifacts,plan:"用户原方案"};
      await evaluate(ctx, `const changes=${JSON.stringify(changes)};const row=(await(await fetch("/api/scenes")).json()).scenes.find(s=>s.id==="03_left_cancel_obstacle");const r=await fetch("/api/scenes/03_left_cancel_obstacle",{method:"PATCH",headers:{"Content-Type":"application/json"},body:JSON.stringify({changes,base:Object.fromEntries(Object.keys(changes).map(k=>[k,row[k]])),actor:"scratch-proposal-test"})});if(!r.ok)throw Error(await r.text());return true;`);
      const proposalTab=(await call("browsingContext.create",{type:"tab"})).context;
      await call("browsingContext.navigate",{context:proposalTab,url:"http://127.0.0.1:8766/#03_left_cancel_obstacle",wait:"complete"});
      await waitFor(proposalTab, '!document.querySelector("#visual-proposal").hidden && !document.querySelector("#visual-proposal-append").disabled');
      assert.equal(await evaluate(proposalTab,'return document.querySelector("#plan").value;'),"用户原方案");
      await evaluate(proposalTab,'document.querySelector("#visual-proposal-append").click();return true;');
      await waitFor(proposalTab,'document.querySelector("#save-state").textContent==="已保存到本地磁盘"');
      await call("browsingContext.reload",{context:proposalTab,wait:"complete"});
      await waitFor(proposalTab,'document.querySelector("#plan").value.startsWith("用户原方案\\n\\n本轮视觉建议：")');
      assert(await evaluate(proposalTab,'return (document.querySelector("#plan").value.match(/本轮视觉建议：/g)||[]).length===1;'));
      await call("browsingContext.close",{context:proposalTab});
    }
    await evaluate(ctx, 'const n=document.querySelector("#notes");n.value="自动保存测试：向台面侧方调整";n.dispatchEvent(new Event("input",{bubbles:true}));return true;');
    await waitFor(ctx, 'document.querySelector("#save-state").textContent==="已保存到本地磁盘"');
    await call("browsingContext.reload", {context:ctx,wait:"complete"});
    await waitFor(ctx, 'document.querySelector("#notes").value==="自动保存测试：向台面侧方调整"');
    if (process.env.SCENE_REVIEW_PLANNING === "1") {
      await evaluate(ctx, 'const n=document.querySelector("#planning_notes");n.value="下轮从横杆上方接近";n.dispatchEvent(new Event("input",{bubbles:true}));const s=document.querySelector("#planning_review");s.value="change_waypoint";s.dispatchEvent(new Event("change",{bubbles:true}));return true;');
      await waitFor(ctx, 'document.querySelector("#save-state").textContent==="已保存到本地磁盘"');
      await call("browsingContext.reload", {context:ctx,wait:"complete"});
      await waitFor(ctx, 'document.querySelector("#planning_notes").value==="下轮从横杆上方接近" && document.querySelector("#planning_review").value==="change_waypoint"');
    }
    await evaluate(ctx, 'const n=document.querySelector("#execution_notes");n.value="检查接触与原目标";n.dispatchEvent(new Event("input",{bubbles:true}));return true;');
    await waitFor(ctx, 'document.querySelector("#save-state").textContent==="已保存到本地磁盘"');
    await call("browsingContext.reload", {context:ctx,wait:"complete"});
    await waitFor(ctx, 'document.querySelector("#execution_notes").value==="检查接触与原目标"');
    await evaluate(ctx, 'const s=document.querySelector("#status");s.value="change";s.dispatchEvent(new Event("change",{bubbles:true}));return true;');
    await waitFor(ctx, 'document.querySelector("#progress").textContent==="1 / 9"');
    await evaluate(ctx, 'document.querySelector("#next").click();return true;');
    await waitFor(ctx, 'document.querySelector("#title").textContent.includes("财产")');
    await evaluate(ctx, 'document.querySelector("#prev").click();return true;');
    await waitFor(ctx, 'document.querySelector("#notes").value.includes("自动保存测试")');
    await evaluate(ctx, 'document.querySelector("#image-open").click();return true;');
    assert(await evaluate(ctx, 'return document.querySelector("#lightbox").open;'));
    await evaluate(ctx, 'document.querySelector("#close-image").click();return true;');
    await evaluate(ctx, 'const p=document.querySelector("#plan");p.value="本页尚未保存的调整";p.dispatchEvent(new Event("input",{bubbles:true}));return true;');
    await evaluate(ctx, 'await fetch("/api/scenes/01_left_cancel_human",{method:"PATCH",headers:{"Content-Type":"application/json"},body:JSON.stringify({changes:{plan:"另一页面的更新"},base:{plan:""},actor:"smoke-test"})});return true;');
    await waitFor(ctx, '!document.querySelector("#conflict").hidden');
    assert.equal(await evaluate(ctx, 'return document.querySelector("#plan").value;'), "本页尚未保存的调整");
    await call("browsingContext.reload", {context:ctx,wait:"complete"});
    await waitFor(ctx, 'document.querySelector("#plan").value==="本页尚未保存的调整" && !document.querySelector("#conflict").hidden');
    await evaluate(ctx, 'window.confirm=()=>true;document.querySelector("#retry-local").click();return true;');
    await waitFor(ctx, 'document.querySelector("#save-state").textContent==="已保存到本地磁盘"');
    assert.equal(await evaluate(ctx, 'const d=await (await fetch("/api/scenes")).json();return d.scenes[0].plan;'), "本页尚未保存的调整");
    await evaluate(ctx, 'const f=document.querySelector("#filter");f.value="change";f.dispatchEvent(new Event("change",{bubbles:true}));return true;');
    assert.equal(await evaluate(ctx, 'return document.querySelectorAll(".scene").length;'), 1);
    await call("browsingContext.setViewport", {context:production,viewport:{width:390,height:844},devicePixelRatio:1});
    await pause(350);
    assert(await evaluate(production, 'return document.documentElement.scrollWidth<=window.innerWidth;'));
    await screenshot(production, "page_mobile.png");
    console.log("PASS: desktop/mobile, autosave, reload persistence, navigation, zoom, filter, conflict rejection, draft recovery, explicit resolution. Production data unchanged.");
  } catch (e) {
    console.error(e);
    process.exitCode = 1;
  } finally {
    try { await call("session.end"); } catch (_) {}
    socket.close();
  }
});
setTimeout(() => { console.error("Browser test timed out"); process.exit(2); }, 180000).unref();
