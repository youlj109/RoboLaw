// Exercise the production inline trajectory editor without touching real annotations.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const html=fs.readFileSync(path.resolve(__dirname,'../../../docs/extensions/replicator/dashboard/index.html'),'utf8');
const start=html.indexOf('    const TRAJECTORY_STATUS='),end=html.indexOf('    function render(){',start);
assert(start>=0&&end>start);
const source=html.slice(html.indexOf('    function safeURL('),html.indexOf('    function artifactLink('))+html.slice(start,end)+'\nglobalThis.api={state:trajectory,record:trajectoryRecord,persist:trajectoryPersist,ack:trajectoryAcknowledge,same:trajectorySame,binding:trajectoryBinding,save:saveTrajectory,views:trajectoryViews,suggestion:trajectorySuggestion,render:renderTrajectories,filter:trajectoryFiltered};';
function storage(map){return {getItem:key=>map.has(key)?map.get(key):null,setItem:(key,value)=>map.set(key,String(value)),removeItem:key=>map.delete(key)};}
class Node {
  constructor(){this.children=[];this.style={};this.dataset={};this.value='';this.src='';this.scrollTop=0;this.classList={toggle(){},contains(){return false;}};}
  append(...nodes){this.children.push(...nodes);}
  replaceChildren(...nodes){this.children=nodes;}
  setAttribute(key,value){this[key]=value;}
  removeAttribute(key){delete this[key];}
  addEventListener(){}
  get childNodes(){return this.children;}
}
function harness({shared=new Map(),session=new Map(),fetchImpl=async()=>({ok:true,status:200,json:async()=>({entries:[]})})}={}){
  const nodes=new Map();const byId=id=>{if(!nodes.has(id))nodes.set(id,new Node());return nodes.get(id);};
  const context=vm.createContext({localStorage:storage(shared),sessionStorage:storage(session),Map,Set,JSON,Object,Array,Date,String,Number,AbortController,URL,Blob,
    setTimeout(){return 1;},clearTimeout(){},setInterval(){},fetch:fetchImpl,
    document:{querySelectorAll(){return [];},activeElement:null},window:{location:{origin:'http://localhost:8765'}},
    state:{data:{tasks:[{id:'task',label:'抓放',layout:'U 型'}]}},byId,
    el:(tag,cls,text)=>{const node=new Node();node.tag=tag;node.className=cls;node.textContent=text;return node;},
    str:(v,f='')=>typeof v==='string'||typeof v==='number'?String(v):f,arr:v=>Array.isArray(v)?v:[],number:v=>Number(v)||0,
    taskName:t=>t.label||t.id||'',clock:v=>v,fullDate:v=>v,textLine:v=>typeof v==='string'?v:'',
    safeURL:v=>typeof v==='string'&&v.startsWith('/artifact/')?'http://localhost:8765'+v:null,
    artifactLink:item=>{if(!item.url?.startsWith('/artifact/'))return null;const node=new Node();node.href=item.url;return node;}
  });
  vm.runInContext(source,context);return {api:context.api,context,shared,session,nodes};
}
const draft=(note,seq=1,result='result-one')=>({scene_id:'scene',status:'adjust',note,seq,expected_revision:0,config_sha256:'config',result_sha256:result});
function write(h,value){const r=h.api.record('scene');r.draft=value;r.seq=value.seq;h.api.persist('scene');return r;}
function ready(h,result='result-one') {h.api.state.loaded=true;h.api.state.byScene.set('scene',{scene_id:'scene',config_sha256:'config',result_sha256:result});const r=h.api.record('scene');r.loaded=true;return r;}

test('trajectory drafts remain separate from scene annotations and survive refresh',()=>{
  const shared=new Map([['robolaw.annotations.draft.v1:scene',JSON.stringify({note:'场景摆位'})]]),h=harness({shared});write(h,draft('轨迹方向'));
  assert.equal(JSON.parse(shared.get('robolaw.annotations.draft.v1:scene')).note,'场景摆位');
  assert.equal(harness({shared,session:h.session}).api.record('scene').draft.note,'轨迹方向');
});
test('acknowledging one tab preserves the other tab trajectory draft',()=>{
  const shared=new Map(),a=harness({shared}),b=harness({shared}),sent=draft('A');write(a,sent);write(b,draft('B'));
  a.api.ack('scene',{...sent,revision:1},sent);
  assert.equal(harness({shared,session:a.session}).api.record('scene').draft,null);
  assert.equal(harness({shared,session:b.session}).api.record('scene').draft.note,'B');
  assert.equal(JSON.parse(shared.get('robolaw.trajectories.draft.v1:scene')).note,'B');
});
test('result hash participates in version equality and binding',()=>{
  const h=harness();assert.equal(h.api.same(draft('same'),draft('same',1,'result-two')),false);
  assert.equal(h.api.binding({config_sha256:'config'}).result_sha256,null);
  assert.equal(h.api.binding({config_sha256:'config',result_sha256:'result'}).result_sha256,'result');
});
test('changed planning result blocks an old draft before POST',async()=>{
  let calls=0;const h=harness({fetchImpl:async()=>{calls++;throw Error('should not send');}});ready(h,'result-two');const r=write(h,draft('旧结果方案'));
  await h.api.save('scene');assert.equal(calls,0);assert(r.conflict);assert.equal(r.draft.note,'旧结果方案');
});
test('autosave uses the independent endpoint and both current source hashes',async()=>{
  let request;const h=harness({fetchImpl:async(url,init)=>{request={url,body:JSON.parse(init.body)};return {ok:true,status:200,json:async()=>({entry:{...request.body,revision:1,updated_at:'now'}})};}});ready(h);const r=write(h,draft('从左侧接近'));
  await h.api.save('scene');assert.equal(request.url,'/api/trajectory-annotations');assert.equal(request.body.result_sha256,'result-one');assert.equal(request.body.config_sha256,'config');assert.equal(request.body.expected_revision,0);assert.equal(r.draft,null);assert.equal(r.server.note,'从左侧接近');
});
test('a concurrent edit remains a draft after the older POST is acknowledged',async()=>{
  let resolveResponse;const h=harness({fetchImpl:()=>new Promise(resolve=>{resolveResponse=resolve;})});ready(h);write(h,draft('first'));const pending=h.api.save('scene');write(h,draft('newer input',2));
  resolveResponse({ok:true,status:200,json:async()=>({entry:{...draft('first'),revision:1}})});await pending;
  const r=h.api.record('scene');assert.equal(r.draft.note,'newer input');assert.equal(r.draft.expected_revision,1);
});
test('a server conflict retains the local plan and the server alternative',async()=>{
  const h=harness({fetchImpl:async()=>({ok:false,status:409,json:async()=>({entry:{...draft('remote'),revision:3}})})});ready(h);const r=write(h,draft('local'));await h.api.save('scene');
  assert.equal(r.draft.note,'local');assert.equal(r.conflict.entry.note,'remote');assert.equal(r.server.revision,3);
});
test('offline save retains the draft without altering generated progress',async()=>{
  const h=harness({fetchImpl:async()=>{throw Error('offline');}});ready(h);const r=write(h,draft('offline note'));await h.api.save('scene');assert.equal(r.error,'offline');assert.equal(r.draft.note,'offline note');
});
test('mixed statuses render with failed/skipped filtering and source views',()=>{
  const h=harness();h.context.state.data.trajectory_generation={counts:{total:5,pending:1,running:1,generated:1,failed:1,skipped:1},entries:['pending','running','generated','failed','skipped'].map((status,i)=>({scene_id:'scene'+i,task_id:'task',variant_id:'variant'+i,variant_name:'变体'+i,status,config_sha256:'config',result_sha256:status==='pending'?null:'hash'+i,views:[{view_id:'overview',kind:'overview',image:{url:'/artifact/scene.png'}}],preview_url:status==='generated'?'/artifact/trajectory.png':null}))};
  h.api.render();assert.equal(h.nodes.get('trajProgressText').textContent,'本轮已处理 3 / 5');assert.equal(h.nodes.get('trajSceneList').children.length,5);
  h.api.state.filters.generation='attention';assert.equal(h.api.filter().length,2);
  const views=h.api.views(h.api.state.entries[2]);assert.equal(views[0].kind,'trajectory');assert.equal(views[1].kind,'overview');
});
test('failure hints are proposals, and generated/pending entries have no invented diagnosis',()=>{
  const h=harness();assert.match(h.api.suggestion({status:'failed',stage:'grasp_ik'}),/可尝试/);assert.equal(h.api.suggestion({status:'generated'}),'');assert.equal(h.api.suggestion({status:'pending'}),'');
});
test('retry progress separates retained successes and preserves historical notes for review',()=>{
  const h=harness();h.context.state.data.trajectory_generation={retry_round:1,carried_generated:1,
    counts:{total:3,generated:2,failed:1},round_counts:{total:2,generated:1,failed:1},
    entries:[{scene_id:'retained',status:'generated',carry_forward:true,result_sha256:'original'},
      {scene_id:'retry',status:'generated',retry_round:1,result_sha256:'new',previous_status:'failed',previous_stage:'lift',change_description:'碗沿抓点旋转30度'},
      {scene_id:'failed',status:'failed',retry_round:1,result_sha256:'failed-new'}].map(e=>({...e,task_id:'task',variant_id:'variant',variant_name:e.scene_id,config_sha256:'config',views:[]}))};
  const r=h.api.record('retry');r.server={scene_id:'retry',status:'adjust',note:'首轮失败建议',revision:1,config_sha256:'config',result_sha256:'old'};
  h.api.state.selected='retry';h.api.render();
  assert.equal(h.nodes.get('trajProgressText').textContent,'修订重试已处理 2 / 2 · 本轮新增 1 · 本轮失败 1 · 历史保留 1 · 累计已生成 2');
  assert.equal(h.nodes.get('trajProgress')['aria-valuemax'],'2');
  assert.match(h.nodes.get('trajVisualRound').textContent,/每景修改或标注 1 次，修改后规划 1 次/);
  assert.match(h.nodes.get('trajSummary').textContent,/当前结果标注 0 \/ 3.*1 条历史方案待复核/);
  assert(h.nodes.get('trajResult').children.some(n=>n.textContent==='本次修改：碗沿抓点旋转30度'));
  assert(h.nodes.get('trajResult').children.some(n=>n.textContent.includes('上一轮：生成失败 / lift')));
  assert.equal(h.nodes.get('trajNote').value,'首轮失败建议');
  assert.equal(r.server.result_sha256,'old');assert.equal(r.draft,null);
});

test('annotation-only results show zero fresh solving and identify inherited partial paths',()=>{
  const h=harness();h.context.state.data.trajectory_generation={retry_round:3,carried_generated:1,
    counts:{total:2,generated:1,failed:1},round_counts:{total:1,failed:1},round_actions:{planned:0,annotated:1},
    entries:[{scene_id:'scene',task_id:'task',variant_id:'variant',variant_name:'人体',status:'failed',
      retry_round:3,annotation_only:true,config_sha256:'config',result_sha256:'hash',views:[]}]};
  h.api.state.selected='scene';h.api.render();
  assert.match(h.nodes.get('trajProgressText').textContent,/仅标注 1（未求解）/);
  assert(h.nodes.get('trajResult').children.some(n=>n.textContent.includes('未重新运行规划')));
});
