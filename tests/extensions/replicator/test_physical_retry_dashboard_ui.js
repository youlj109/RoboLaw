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
