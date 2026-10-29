// Run with: node --test tests/extensions/replicator/test_physical_retry_dashboard_ui.js
// Exercise the real render function with a minimal DOM, without a browser/server.
const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {resolve} = require('node:path');
const {test} = require('node:test');
const vm = require('node:vm');

const html = readFileSync(resolve(__dirname, '../../../docs/extensions/replicator/dashboard/index.html'), 'utf8');
const renderSource = html.match(/    function renderPhysical\(\)\{[\s\S]*?\n    \}\n    byId\('physicalSearch'\)/)[0]
  .replace(/\n    byId\('physicalSearch'\)$/, '');

function harness(data = null, retryRound = 2) {
  const nodes = new Map();
  const node = () => ({append() {}, replaceChildren() {}});
  const context = {
    state: {data}, physicalState: {retryRound, retryFilter: '', filter: '', query: '', signature: ''},
    byId: id => { if (!nodes.has(id)) nodes.set(id, node()); return nodes.get(id); },
    arr: value => Array.isArray(value) ? value : [], number: value => Number(value) || 0,
    str: (value, fallback = '') => value ?? fallback, clock: String, el: node,
    physicalRetryRow: () => null, physicalLabel: () => '', renderPhysicalDetail() {},
  };
  vm.createContext(context); vm.runInContext(renderSource, context);
  return {context, render: () => vm.runInContext('renderPhysical()', context)};
}

test('selection of round 2 during first fetch survives render and the arriving response', () => {
  const {context, render} = harness(null, 1);
  context.physicalState.retryRound = 2; // The selector change handler before fetch resolves.
  render();
  assert.equal(context.physicalState.retryRound, 2);
  assert.equal(context.byId('physicalRetryRound').value, '2');
  context.state.data = {physical_execution: {failure_retry_round2: {manifest_present: true, planning_present: true}}};
  render();
  assert.equal(context.physicalState.retryRound, 2);
  assert.equal(context.byId('physicalRetryRound').value, '2');
});

test('an actual response with no round 2 still falls back to round 1', () => {
  const {context, render} = harness({physical_execution: {failure_retry_round2: {manifest_present: false, planning_present: false}}});
  render();
  assert.equal(context.physicalState.retryRound, 1);
  assert.equal(context.byId('physicalRetryRound').value, '1');
});

test('a manual round 1 choice is retained when round 2 is available', () => {
  const {context, render} = harness({physical_execution: {failure_retry_round2: {manifest_present: true, planning_present: true}}}, 1);
  render(); render();
  assert.equal(context.physicalState.retryRound, 1);
});

test('planning-only and execution-only round 2 responses both retain round 2', () => {
  for (const presence of [{planning_present: true}, {manifest_present: true}]) {
    const {context, render} = harness({physical_execution: {failure_retry_round2: presence}});
    render();
    assert.equal(context.physicalState.retryRound, 2);
  }
});

test('round 3 remains selected while loading and when its planning list arrives', () => {
  const {context, render} = harness(null, 3);
  render();
  assert.equal(context.physicalState.retryRound, 3);
  context.state.data = {physical_execution: {failure_retry_round3: {planning_present: true}}};
  render();
  assert.equal(context.byId('physicalRetryRound').value, '3');
});

test('round 3 falls back to an available round 2, not to round 1', () => {
  const {context, render} = harness({physical_execution: {failure_retry_round2: {manifest_present: true}}}, 3);
  render();
  assert.equal(context.physicalState.retryRound, 2);
});

test('round 4 stays selected during loading and with planning-only or execution-only data', () => {
  for (const presence of [{planning_present: true}, {manifest_present: true}]) {
    const {context, render} = harness(null, 4);
    render();
    assert.equal(context.physicalState.retryRound, 4);
    context.state.data = {physical_execution: {failure_retry_round4: presence}};
    render();
    assert.equal(context.byId('physicalRetryRound').value, '4');
  }
});

test('round 4 falls back through 3 and 2 according to the first available round', () => {
  for (const [available, expected] of [
    [{failure_retry_round3: {planning_present: true}, failure_retry_round2: {manifest_present: true}}, 3],
    [{failure_retry_round2: {manifest_present: true}}, 2],
    [{}, 1],
  ]) {
    const {context, render} = harness({physical_execution: available}, 4);
    render();
    assert.equal(context.physicalState.retryRound, expected);
    assert.equal(context.byId('physicalRetryRound').value, String(expected));
  }
});

test('manual historical selection remains unchanged when round 4 is available', () => {
  for (const selected of [1, 2, 3]) {
    const {context, render} = harness({physical_execution: {
      failure_retry_round4: {manifest_present: true}, failure_retry_round3: {manifest_present: true},
      failure_retry_round2: {manifest_present: true},
    }}, selected);
    render(); render();
    assert.equal(context.physicalState.retryRound, selected);
  }
});


test('round 5 loading and planning/execution presence retain selection', () => {
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,5);render();assert.equal(context.physicalState.retryRound,5);
    context.state.data={physical_execution:{failure_retry_round5:presence}};render();
    assert.equal(context.byId('physicalRetryRound').value,'5');
  }
});
test('round 5 defaults back through all four historical rounds', () => {
  for (const expected of [4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},5);render();
    assert.equal(context.physicalState.retryRound,expected);
  }
});
test('round 5 presence never replaces a manual historical choice', () => {
  for (const selected of [1,2,3,4]) {
    const available={failure_retry_round5:{manifest_present:true},failure_retry_round2:{manifest_present:true},
      failure_retry_round3:{manifest_present:true},failure_retry_round4:{manifest_present:true}};
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
  }
});

test('round 6 loading and planning/execution presence retain selection', () => {
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,6);render();assert.equal(context.physicalState.retryRound,6);
    context.state.data={physical_execution:{failure_retry_round6:presence}};render();
    assert.equal(context.byId('physicalRetryRound').value,'6');
  }
});
test('round 6 defaults back through all five historical rounds', () => {
  for (const expected of [5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},6);render();
    assert.equal(context.physicalState.retryRound,expected);
  }
});
test('round 6 presence never replaces a manual historical choice', () => {
  for (const selected of [1,2,3,4,5]) {
    const available={failure_retry_round6:{manifest_present:true},failure_retry_round2:{manifest_present:true},
      failure_retry_round3:{manifest_present:true},failure_retry_round4:{manifest_present:true},failure_retry_round5:{manifest_present:true}};
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
  }
});

test('round 7 selection survives initial loading and available data', () => {
  const initial = 7;
  assert.match(html, /<option value="7">第七轮 · 场景修订 r7（3 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();
    assert.equal(context.byId('physicalRetryRound').value,'7');
    context.state.data={physical_execution:{failure_retry_round7:presence}};render();
    assert.equal(context.physicalState.retryRound,7);
    assert.equal(context.byId('physicalRetryRound').value,'7');
  }
});

test('round 7 falls back only after a response and chooses the newest available history', () => {
  for (const expected of [6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},7);render();
    assert.equal(context.physicalState.retryRound,expected);
    assert.equal(context.byId('physicalRetryRound').value,String(expected));
  }
});

test('round 7 availability preserves all six manual historical selections', () => {
  const available=Object.fromEntries([2,3,4,5,6,7].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6]) {
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
    assert.equal(context.byId('physicalRetryRound').value,String(selected));
  }
});

test('round 8 selection survives initial loading and available data', () => {
  const initial = 8;
  assert.match(html, /<option value="8">第八轮 · 场景修订 r8（2 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();
    assert.equal(context.byId('physicalRetryRound').value,'8');
    context.state.data={physical_execution:{failure_retry_round8:presence}};render();
    assert.equal(context.physicalState.retryRound,8);
    assert.equal(context.byId('physicalRetryRound').value,'8');
  }
});

test('round 8 falls back only after a response and chooses the newest available history', () => {
  for (const expected of [7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},8);render();
    assert.equal(context.physicalState.retryRound,expected);
    assert.equal(context.byId('physicalRetryRound').value,String(expected));
  }
});

test('round 8 availability preserves all seven manual historical selections', () => {
  const available=Object.fromEntries([2,3,4,5,6,7,8].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7]) {
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
    assert.equal(context.byId('physicalRetryRound').value,String(selected));
  }
});

test('round 9 selection survives initial loading and available data', () => {
  const initial = 9;
  assert.match(html, /<option value="9">第九轮 · 沿用 r8 布局（2 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();
    assert.equal(context.byId('physicalRetryRound').value,'9');
    context.state.data={physical_execution:{failure_retry_round9:presence}};render();
    assert.equal(context.physicalState.retryRound,9);
    assert.equal(context.byId('physicalRetryRound').value,'9');
  }
});

test('round 9 falls back only after a response and chooses the newest available history', () => {
  for (const expected of [8,7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},9);render();
    assert.equal(context.physicalState.retryRound,expected);
    assert.equal(context.byId('physicalRetryRound').value,String(expected));
  }
});

test('round 9 availability preserves all eight manual historical selections', () => {
  const available=Object.fromEntries([2,3,4,5,6,7,8,9].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7,8]) {
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
    assert.equal(context.byId('physicalRetryRound').value,String(selected));
  }
});

test('round 10 selection survives initial loading and available data', () => {
  const initial = 10;
  assert.match(html, /<option value="10">第十轮 · 固定关节目标（2 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();
    assert.equal(context.byId('physicalRetryRound').value,'10');
    context.state.data={physical_execution:{failure_retry_round10:presence}};render();
    assert.equal(context.physicalState.retryRound,10);
    assert.equal(context.byId('physicalRetryRound').value,'10');
  }
});


test('round 10 falls back through all available history and preserves manual selections', () => {
  for (const expected of [9,8,7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},10);render();
    assert.equal(context.physicalState.retryRound,expected);
  }
  const available=Object.fromEntries([2,3,4,5,6,7,8,9,10].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7,8,9]) {
    const {context,render}=harness({physical_execution:available},selected);render();render();
    assert.equal(context.physicalState.retryRound,selected);
  }
});

test('round 11 survives initial loading and available data', () => {
  const initial = 11;
  assert.match(html, /<option value="11">第十一轮 · 求解策略修复（2 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();
    assert.equal(context.byId('physicalRetryRound').value,'11');
    context.state.data={physical_execution:{failure_retry_round11:presence}};render();
    assert.equal(context.physicalState.retryRound,11);
    assert.equal(context.byId('physicalRetryRound').value,'11');
  }
});



test('round 11 falls back through all history and preserves manual choices', () => {
  for (const expected of [10,9,8,7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},11);render();assert.equal(context.physicalState.retryRound,expected);
  }
  const available=Object.fromEntries([2,3,4,5,6,7,8,9,10,11].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7,8,9,10]) {
    const {context,render}=harness({physical_execution:available},selected);render();assert.equal(context.physicalState.retryRound,selected);
  }
});


test('round 12 retains the successful round 11 history', () => {
  const initial = 12;
  assert.match(html, /<option value="12">第十二轮 · 携带路径修复（1 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();assert.equal(context.physicalState.retryRound,12);
    context.state.data={physical_execution:{failure_retry_round12:presence}};render();assert.equal(context.physicalState.retryRound,12);
  }
  for (const expected of [11,10,9,8,7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},12);render();assert.equal(context.physicalState.retryRound,expected);
  }
});

test('round 12 availability does not change manually selected history', () => {
  const available=Object.fromEntries([2,3,4,5,6,7,8,9,10,11,12].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7,8,9,10,11]) {
    const {context,render}=harness({physical_execution:available},selected);render();assert.equal(context.physicalState.retryRound,selected);
  }
});


test('round 13 is the actual UI default, supports one remaining scene and all history', () => {
  const initial=Number(html.match(/const physicalState=\{[^\n]*?retryRound:(\d+)/)[1]);assert.equal(initial,13);
  assert.match(html, /<option value="13">第十三轮 · 提前释放（1 景）<\/option>/);
  for (const presence of [{planning_present:true},{manifest_present:true}]) {
    const {context,render}=harness(null,initial);render();assert.equal(context.physicalState.retryRound,13);
    context.state.data={physical_execution:{failure_retry_round13:presence}};render();assert.equal(context.physicalState.retryRound,13);
  }
  for (const expected of [12,11,10,9,8,7,6,5,4,3,2,1]) {
    const available=expected>1?{['failure_retry_round'+expected]:{manifest_present:true}}:{};
    const {context,render}=harness({physical_execution:available},13);render();assert.equal(context.physicalState.retryRound,expected);
  }
});

test('round 13 availability preserves earlier selected outcomes and annotations', () => {
  const available=Object.fromEntries([2,3,4,5,6,7,8,9,10,11,12,13].map(round=>['failure_retry_round'+round,{manifest_present:true}]));
  for (const selected of [1,2,3,4,5,6,7,8,9,10,11,12]) {
    const {context,render}=harness({physical_execution:available},selected);render();assert.equal(context.physicalState.retryRound,selected);
  }
});
