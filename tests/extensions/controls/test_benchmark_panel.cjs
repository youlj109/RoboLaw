"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");
const code = fs.readFileSync(path.resolve(__dirname, "../../../docs/extensions/controls/scene_review/benchmark.js"), "utf8");
async function check(report, expected, unavailable=false) {
  const elements = new Map();
  const scenes = Array.from({length:9}, (_,i)=>({id:String(i), visual_config:`cfg${i}`, execution_artifacts:[{
    label:"严格安全门禁汇总：7/9", url:"/execution-reports/benchmark_summary150/report.json"}]}));
  const context = {window:{}, document:{getElementById:id=>{
    if(!elements.has(id)) elements.set(id, {});
    return elements.get(id);
  }}, fetch:async()=>({ok:!unavailable, status:503, json:async()=>report})};
  vm.runInNewContext(code, context);
  await context.window.renderBenchmarkSummary(scenes);
  assert.equal(elements.get("benchmark-progress").textContent, expected);
}
(async()=>{
  const report = {total:9, accepted:false, created_utc:"2026-10-28", scenes:Array.from({length:9},(_,i)=>({
    scene:String(i), current_visual_config:`cfg${i}`, accepted:i<7}))};
  await check(report, "严格门禁快照 7 / 9");
  const stale = structuredClone(report);stale.scenes[0].current_visual_config="old";
  await check(stale, "严格门禁快照 6 / 9");
  const duplicate = structuredClone(report);duplicate.scenes[8].scene="0";
  await check(duplicate, "严格门禁：报告不可用");
  await check(report, "严格门禁：报告不可用", true);
  console.log("PASS: strict count, stale-layout rejection, duplicate coverage rejection, HTTP failure");
})().catch(error=>{console.error(error);process.exitCode=1;});
