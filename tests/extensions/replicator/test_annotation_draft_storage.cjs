// Targeted regression for the actual inline dashboard draft functions. No browser,
// network, production annotations, or filesystem storage is used by these tests.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');

const html=fs.readFileSync(path.resolve(__dirname,'../../../docs/extensions/replicator/dashboard/index.html'),'utf8');
function section(start,end){const a=html.indexOf(start),b=html.indexOf(end,a);assert(a>=0&&b>a,'Dashboard function boundaries exist');return html.slice(a,b);}
const source=section('    const ANNO_STATUS=','    function annotationValue(')
  +section('    function annotationPersist(','    function annotationFiltered(')
  +section('    function annotationAcknowledge(','    async function saveAnnotation(')
  +'\nglobalThis.api={record:annotationRecord,persist:annotationPersist,ack:annotationAcknowledge};';
const sharedKey='robolaw.annotations.draft.v1:scene_one';
const tabKey='robolaw.annotations.tab-draft.v1:scene_one';
function storage(map){return {getItem:key=>map.has(key)?map.get(key):null,setItem:(key,value)=>map.set(key,String(value)),removeItem:key=>map.delete(key)};}
function tab(shared,session=new Map()){
  const context=vm.createContext({localStorage:storage(shared),sessionStorage:storage(session)});
  vm.runInContext(source,context);
  return {api:context.api,session};
}
function draft(note,seq=1){return {scene_id:'scene_one',note,status:'adjust',expected_revision:0,seq,config_sha256:'config',views:[],edited_at:'2026-10-25T00:00:00Z'};}
function write(current,value){const record=current.api.record('scene_one');record.draft=value;record.seq=value.seq;current.api.persist('scene_one');}
function acknowledge(current,snapshot){current.api.ack('scene_one',{...snapshot,revision:1},snapshot);}

test('A acknowledgement preserves B draft and B refresh restores it',()=>{
  const shared=new Map(),a=tab(shared),b=tab(shared),first=draft('A 的方案'),second=draft('B 尚未同步的方案');
  write(a,first);write(b,second);acknowledge(a,first);
  assert.equal(JSON.parse(shared.get(sharedKey)).note,second.note);
  assert.equal(JSON.parse(a.session.get(tabKey)).draft,null);
  assert.equal(JSON.parse(b.session.get(tabKey)).draft.note,second.note);
  const refreshedB=tab(shared,b.session),refreshedA=tab(shared,a.session);
  assert.equal(refreshedB.api.record('scene_one').draft.note,second.note);
  assert.equal(refreshedA.api.record('scene_one').draft,null,'Cleared tab does not import B backup');
});

test('duplicated tab session copies diverge safely after subsequent edits',()=>{
  const shared=new Map(),a=tab(shared),original=draft('复制前的草稿');write(a,original);
  const b=tab(shared,new Map(a.session));assert.equal(b.api.record('scene_one').draft.note,original.note);
  write(b,draft('复制标签页的新方案',2));acknowledge(a,original);
  assert.equal(tab(shared,b.session).api.record('scene_one').draft.note,'复制标签页的新方案');
  assert.equal(tab(shared,a.session).api.record('scene_one').draft,null);
});

test('legacy shared draft is migrated before another tab clears the shared backup',()=>{
  const legacy=draft('旧版留下的方案'),shared=new Map([[sharedKey,JSON.stringify(legacy)]]),a=tab(shared),b=tab(shared);
  assert.equal(b.api.record('scene_one').draft.note,legacy.note);
  assert.equal(JSON.parse(b.session.get(tabKey)).draft.note,legacy.note);
  assert.equal(a.api.record('scene_one').draft.note,legacy.note);acknowledge(a,legacy);
  assert.equal(shared.has(sharedKey),false);
  assert.equal(tab(shared,b.session).api.record('scene_one').draft.note,legacy.note);
});

test('late acknowledgement keeps newer input and updates only its expected revision',()=>{
  const shared=new Map(),a=tab(shared),b=tab(shared),old=draft('A 已发送的旧方案');write(a,old);
  write(a,draft('A 保存途中继续输入',2));write(b,draft('B 独立方案',2));acknowledge(a,old);
  const restoredA=tab(shared,a.session).api.record('scene_one').draft;
  assert.equal(restoredA.note,'A 保存途中继续输入');assert.equal(restoredA.expected_revision,1);
  assert.equal(tab(shared,b.session).api.record('scene_one').draft.note,'B 独立方案');
});

test('clean tab pins an empty state instead of importing another tab on refresh',()=>{
  const shared=new Map(),a=tab(shared),b=tab(shared);assert.equal(a.api.record('scene_one').draft,null);
  write(b,draft('其他标签页后来写入的草稿'));
  assert.equal(tab(shared,a.session).api.record('scene_one').draft,null);
});
