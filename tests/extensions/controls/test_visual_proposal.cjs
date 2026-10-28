"use strict";
const assert = require("node:assert/strict"), fs = require("node:fs"), vm = require("node:vm");
const path = require("node:path");
const code = fs.readFileSync(path.resolve(__dirname, "../../../docs/extensions/controls/scene_review/visual-proposal.js"), "utf8");
function setup(report, ok=true) {
  const elements = new Map();
  const el = id => {
    if (!elements.has(id)) elements.set(id, {value:"用户原方案", events:0,
      dispatchEvent(){this.events++;}, focus(){}, scrollIntoView(){}});
    return elements.get(id);
  };
  const context = {window:{}, document:{getElementById:el}, Event:class {},
    fetch:async()=>({ok,status:503,json:async()=>report})};
  vm.runInNewContext(code, context);
  return {render:context.window.renderVisualProposal, el};
}
const scene = {id:"03_left_cancel_obstacle",visual_config:"new.json",planning_artifacts:[{
  label:"本轮逐场景视觉判断",url:"/plans/current/round_history.json"}]};
const item = {scene:scene.id,visual_config:scene.visual_config,verdict:"revised_visual_passed",
  observations:"<img src=x onerror=alert(1)>",adjustment:"只改冗余IK，不移动布局"};
(async()=>{
  let t = setup({scene_reviews:[item]});
  await t.render(scene);
  assert.equal(t.el("visual-proposal").hidden,false);
  assert.equal(t.el("plan").value,"用户原方案");
  assert(t.el("visual-proposal-observations").textContent.includes(item.observations));
  t.el("visual-proposal-append").onclick();
  assert(t.el("plan").value.startsWith("用户原方案\n\n"));
  assert.equal(t.el("plan").events,1);
  t.el("visual-proposal-append").onclick();
  assert.equal(t.el("plan").events,1);
  for (const report of [{scene_reviews:[{...item,visual_config:"old"}]},
                         {scene_reviews:[item,item]}, {scene_reviews:[]}]) {
    t=setup(report);await t.render(scene);
    assert.equal(t.el("visual-proposal-append").disabled,true);
    assert.equal(t.el("plan").value,"用户原方案");
  }
  t=setup({},false);await t.render(scene);
  assert.equal(t.el("visual-proposal-append").disabled,true);
  await t.render({...scene,planning_artifacts:[]});
  assert.equal(t.el("visual-proposal").hidden,true);
  console.log("PASS: version-bound proposals, append-only autosave event, duplicate suppression, safe text, stale/duplicate/HTTP rejection");
})().catch(e=>{console.error(e);process.exitCode=1;});
