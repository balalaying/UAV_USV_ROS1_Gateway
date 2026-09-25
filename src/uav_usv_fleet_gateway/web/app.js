"use strict";

const state={
  socket:null,manualClose:false,reconnectTimer:null,
  vehicles:new Map(),targets:new Map(),sensors:new Map(),
  cameras:new Map(),clouds:new Map(),fusion:new Map(),
  gateway:{},messages:0,lastMessage:0,messageTimes:[],typeStats:new Map(),
  selectedVehicle:"usv_01",selectedCamera:"",scale:7.5,panX:0,panY:0,
  pitch:0.72,yaw:-0.52,drag:null,cloudTimes:new Map(),cameraTimes:new Map()
};
const el=id=>document.getElementById(id);
const safe=value=>String(value??"不可用").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#039;"}[c]));
const fmt=(value,digits=2)=>Number.isFinite(Number(value))?Number(value).toFixed(digits):"--";
const websocketScheme=window.location.protocol==="https:"?"wss":"ws";
const query=new URLSearchParams(window.location.search);
el("wsUrl").value=query.get("ws")||`${websocketScheme}://${window.location.hostname}:9765/ws`;

function setConnection(text,kind){
  el("relayBadge").textContent=text;
  el("relayBadge").className=`status ${kind}`;
}
function connect(){
  disconnect(false);
  state.manualClose=false;
  const url=el("wsUrl").value.trim();
  if(!url)return;
  localStorage.setItem("fleetRelayUrl",url);
  setConnection("正在连接远端中继","connecting");
  try{state.socket=new WebSocket(url)}catch(_){setConnection("中继连接失败","offline");scheduleReconnect();return}
  state.socket.onopen=()=>{setConnection("远端中继在线","online");send("request_snapshot")};
  state.socket.onmessage=event=>handleMessage(event.data);
  state.socket.onerror=()=>setConnection("中继链路异常","offline");
  state.socket.onclose=()=>{state.socket=null;setConnection("中继已断开","offline");scheduleReconnect()};
}
function disconnect(manual=true){
  state.manualClose=manual;
  clearTimeout(state.reconnectTimer);
  if(state.socket){state.socket.onclose=null;state.socket.close();state.socket=null}
  if(manual)setConnection("中继未连接","offline");
}
function scheduleReconnect(){
  clearTimeout(state.reconnectTimer);
  if(!state.manualClose&&el("autoReconnect").checked)state.reconnectTimer=setTimeout(connect,1800);
}
function send(command){
  if(state.socket?.readyState===WebSocket.OPEN)state.socket.send(JSON.stringify({command}));
}
function upsert(map,key,value){if(key)map.set(key,value)}
function recordRate(map,key){
  const now=performance.now(),times=map.get(key)||[];
  times.push(now);
  while(times.length&&now-times[0]>3000)times.shift();
  map.set(key,times);
  return times.length/3;
}

function handleMessage(raw){
  let message;
  try{message=JSON.parse(raw)}catch(_){return}
  const now=Date.now(),type=message.message_type||"unknown",data=message.data||{};
  state.messages++;state.lastMessage=now;
  state.messageTimes.push(now);
  state.messageTimes=state.messageTimes.filter(value=>now-value<5000);
  const stats=state.typeStats.get(type)||{count:0,last:0,times:[]};
  stats.count++;stats.last=now;stats.times.push(now);
  stats.times=stats.times.filter(value=>now-value<5000);
  state.typeStats.set(type,stats);
  el("rawJson").textContent=type==="camera_frame"
    ?JSON.stringify({...message,data:{...data,data_base64:`<JPEG ${data.data_base64?.length||0} chars>`}},null,2)
    :JSON.stringify(message,null,2);

  switch(type){
    case"fleet_snapshot":applySnapshot(data);break;
    case"vehicle_state":upsert(state.vehicles,data.id,data);renderFleet();break;
    case"perception_targets":replaceTargets(data.targets||[]);break;
    case"sensor_status":upsert(state.sensors,`${data.vehicle_id}/${data.sensor_id}`,data);break;
    case"gateway_diagnostics":state.gateway=data;break;
    case"camera_frame":applyCamera(data);break;
    case"pointcloud_frame":applyCloud(data);break;
    case"fusion_debug":applyFusion(data);break;
  }
  updateSummary();
}
function applySnapshot(data){
  state.vehicles.clear();(data.vehicles||[]).forEach(v=>upsert(state.vehicles,v.id,v));
  state.sensors.clear();(data.sensors||[]).forEach(s=>upsert(state.sensors,`${s.vehicle_id}/${s.sensor_id}`,s));
  replaceTargets(data.targets||[]);
  state.gateway=data.gateway||{};
  renderFleet();
}
function replaceTargets(values){
  state.targets.clear();values.forEach(target=>upsert(state.targets,target.track_id,target));
}
function applyCamera(data){
  data.receivedAt=Date.now();
  data.fps=recordRate(state.cameraTimes,data.stream_id);
  state.cameras.set(data.stream_id,data);
  rebuildCameraOptions();
  if(!state.selectedCamera||state.selectedCamera===data.stream_id)renderCamera(data);
}
function applyCloud(data){
  data.receivedAt=Date.now();
  data.fps=recordRate(state.cloudTimes,data.stream_id);
  state.clouds.set(data.vehicle_id,data);
}
function applyFusion(data){
  data.receivedAt=Date.now();
  state.fusion.set(data.vehicle_id,data);
}
function rebuildCameraOptions(){
  const select=el("cameraStream"),previous=state.selectedCamera;
  const streams=[...state.cameras.values()].sort((a,b)=>a.stream_id.localeCompare(b.stream_id));
  select.innerHTML=streams.map(stream=>`<option value="${safe(stream.stream_id)}">${cameraLabel(stream)}</option>`).join("");
  const preferred=streams.find(item=>item.vehicle_id===state.selectedVehicle);
  const next=streams.some(item=>item.stream_id===previous)?previous:(preferred?.stream_id||streams[0]?.stream_id||"");
  state.selectedCamera=next;select.value=next;
  if(next&&state.cameras.has(next))renderCamera(state.cameras.get(next));
}
function cameraLabel(stream){
  const type=stream.stream_id.includes("down")?"下视相机":"船首感知相机";
  return `${displayVehicle(stream.vehicle_id)} · ${type}`;
}
function renderCamera(data){
  state.selectedCamera=data.stream_id;
  const image=el("cameraImage");
  image.src=`data:${data.encoding};base64,${data.data_base64}`;
  image.parentElement.classList.add("ready");
  el("cameraTitle").textContent=cameraLabel(data);
  el("cameraMeta").textContent=`${data.width} × ${data.height} · ${data.frame_id||"--"}`;
  el("cameraFps").textContent=`${fmt(data.fps,1)} FPS`;
}
function displayVehicle(id){
  const names={uav_01:"我方无人机一号",uav_02:"我方无人机二号",uav_03:"我方无人机三号",uav_04:"我方无人机四号",usv_01:"我方船一号",usv_02:"我方船二号",usv_03:"我方船三号",friendly_ship:"保护船",enemy_ship:"敌方目标船"};
  return names[id]||id;
}

function renderFleet(){
  const root=el("vehicleTable"),values=[...state.vehicles.values()].sort((a,b)=>a.id.localeCompare(b.id));
  el("vehicleSummary").textContent=`${values.length} 艘/架`;
  root.className=values.length?"vehicle-table":"vehicle-table empty";
  if(!values.length){root.textContent="等待载具状态";return}
  const header=["载具","类型","在线","模式","map 位置","速度","更新时间"];
  root.innerHTML=`<div class="vehicle-row header">${header.map(x=>`<span>${x}</span>`).join("")}</div>`+values.map(v=>{
    const p=v.position||{};
    return `<div class="vehicle-row"><span class="vehicle-id">${safe(displayVehicle(v.id))}</span><span>${safe(v.type)}</span><span class="${v.online&&!v.stale?"online-dot":"offline-dot"}">${v.online&&!v.stale?"在线":"离线"}</span><span>${safe(v.mode)}</span><span>${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}</span><span>${fmt(v.speed)} m/s</span><span>${v.last_update?new Date(v.last_update*1000).toLocaleTimeString():"--"}</span></div>`;
  }).join("");
}
function updateSummary(){
  const now=Date.now();
  state.messageTimes=state.messageTimes.filter(value=>now-value<5000);
  el("receiveRate").textContent=fmt(state.messageTimes.length/5,1);
  el("onlineCount").textContent=[...state.vehicles.values()].filter(v=>v.online&&!v.stale).length;
  el("targetCount").textContent=state.targets.size;
  const cloud=state.clouds.get(state.selectedVehicle);
  const fusion=state.fusion.get(state.selectedVehicle);
  el("pointCount").textContent=`${cloud?.point_count||0} 点`;
  el("cloudFps").textContent=`${fmt(cloud?.fps,1)} Hz`;
  el("boxCount").textContent=String(fusion?.box_count||0);
  el("fusionCount").textContent=String([...state.targets.values()].filter(t=>(t.source_mask||0)&8).length);
  el("frameId").textContent=cloud?.frame_id||fusion?.frame_id||"map";
  el("fixedFrame").textContent=`Fixed frame: ${cloud?.frame_id||"map"}`;
  el("cloudStatus").textContent=cloud?`${cloud.point_count} 点 · ${fmt(cloud.fps,1)} Hz`:"等待点云";
  el("dataAge").textContent=cloud?`${fmt((now-cloud.receivedAt)/1000,1)} s`:"--";
  renderNetworkStats(now);
}
function renderNetworkStats(now){
  const values=[
    ["消息总数",state.messages],["最近消息",state.lastMessage?new Date(state.lastMessage).toLocaleTimeString():"--"],
    ["远端客户端",state.gateway.connected_clients??"--"],["丢弃消息",state.gateway.dropped_messages??0],
    ["点云流",state.clouds.size],["相机流",state.cameras.size]
  ];
  el("networkStats").innerHTML=values.map(([k,v])=>`<div><dt>${safe(k)}</dt><dd>${safe(v)}</dd></div>`).join("");
  el("messageTypes").innerHTML=[...state.typeStats.entries()].sort().map(([type,stats])=>{
    stats.times=stats.times.filter(value=>now-value<5000);
    return `<div><dt>${safe(type)}</dt><dd>${fmt(stats.times.length/5,1)} Hz</dd></div>`;
  }).join("");
}

const canvas=el("perceptionCanvas"),ctx=canvas.getContext("2d");
function resize(){
  const rect=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
  const width=Math.max(1,Math.round(rect.width*dpr)),height=Math.max(1,Math.round(rect.height*dpr));
  if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}
  ctx.setTransform(dpr,0,0,dpr,0,0);
}
function selectedCenter(){
  const vehicle=state.vehicles.get(state.selectedVehicle),p=vehicle?.position||{};
  return [Number(p.x)||0,Number(p.y)||0,Number(p.z)||0];
}
function projection(x,y,z){
  const rect=canvas.getBoundingClientRect(),center=selectedCenter();
  const dx=x-center[0],dy=y-center[1],dz=z-center[2];
  const c=Math.cos(state.yaw),s=Math.sin(state.yaw);
  const rx=c*dx-s*dy,ry=s*dx+c*dy;
  const vertical=Math.cos(state.pitch)*ry-Math.sin(state.pitch)*dz;
  return [rect.width/2+state.panX+rx*state.scale,rect.height/2+state.panY-vertical*state.scale];
}
function draw(){
  resize();
  const rect=canvas.getBoundingClientRect();
  ctx.clearRect(0,0,rect.width,rect.height);
  ctx.fillStyle="#081117";ctx.fillRect(0,0,rect.width,rect.height);
  if(el("showGrid").checked)drawGrid();
  if(el("showCloud").checked)drawCloud();
  if(el("showBoxes").checked)drawBoxes();
  if(el("showTargets").checked)drawTargets();
  if(el("showVehicles").checked)drawVehicles();
  requestAnimationFrame(draw);
}
function drawGrid(){
  const center=selectedCenter(),extent=100,step=10;
  ctx.strokeStyle="#24343e";ctx.lineWidth=1;ctx.beginPath();
  for(let i=-extent;i<=extent;i+=step){
    line3(center[0]+i,center[1]-extent,0,center[0]+i,center[1]+extent,0);
    line3(center[0]-extent,center[1]+i,0,center[0]+extent,center[1]+i,0);
  }
  ctx.stroke();
}
function line3(x1,y1,z1,x2,y2,z2){
  const a=projection(x1,y1,z1),b=projection(x2,y2,z2);
  ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);
}
function drawCloud(){
  const cloud=state.clouds.get(state.selectedVehicle);
  if(!cloud?.xyz)return;
  ctx.fillStyle="rgba(235,247,255,.82)";
  const pointSize=Math.max(1,Math.min(2.2,state.scale/5));
  for(let i=0;i<cloud.xyz.length;i+=3){
    const p=projection(Number(cloud.xyz[i]),Number(cloud.xyz[i+1]),Number(cloud.xyz[i+2]));
    ctx.fillRect(p[0],p[1],pointSize,pointSize);
  }
}
function quaternionMatrix(q){
  let x=Number(q?.x)||0,y=Number(q?.y)||0,z=Number(q?.z)||0,w=Number(q?.w);
  if(!Number.isFinite(w))w=1;
  const n=Math.hypot(x,y,z,w)||1;x/=n;y/=n;z/=n;w/=n;
  return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];
}
function transformPoint(point,box){
  const r=quaternionMatrix(box.pose?.orientation),o=box.pose?.position||{};
  return [r[0][0]*point[0]+r[0][1]*point[1]+r[0][2]*point[2]+(Number(o.x)||0),r[1][0]*point[0]+r[1][1]*point[1]+r[1][2]*point[2]+(Number(o.y)||0),r[2][0]*point[0]+r[2][1]*point[1]+r[2][2]*point[2]+(Number(o.z)||0)];
}
function boxSegments(box){
  if(box.points?.length>=2){
    const transformed=box.points.map(p=>transformPoint(p,box)),segments=[];
    if(box.type===5){for(let i=0;i+1<transformed.length;i+=2)segments.push([transformed[i],transformed[i+1]])}
    else{for(let i=0;i+1<transformed.length;i++)segments.push([transformed[i],transformed[i+1]])}
    return segments;
  }
  const s=box.scale||{},hx=(Number(s.x)||2)/2,hy=(Number(s.y)||1)/2,hz=(Number(s.z)||1)/2;
  const local=[[-hx,-hy,-hz],[hx,-hy,-hz],[hx,hy,-hz],[-hx,hy,-hz],[-hx,-hy,hz],[hx,-hy,hz],[hx,hy,hz],[-hx,hy,hz]];
  const points=local.map(p=>transformPoint(p,box)),edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
  return edges.map(([a,b])=>[points[a],points[b]]);
}
function drawBoxes(){
  const payload=state.fusion.get(state.selectedVehicle);
  if(!payload?.boxes)return;
  ctx.strokeStyle="#35e889";ctx.lineWidth=2;ctx.beginPath();
  payload.boxes.forEach(box=>boxSegments(box).forEach(([a,b])=>line3(...a,...b)));
  ctx.stroke();
  payload.boxes.forEach(box=>{
    const p=box.pose?.position||{},screen=projection(Number(p.x)||0,Number(p.y)||0,(Number(p.z)||0)+(Number(box.scale?.z)||1)/2);
    label(`FUSION ${box.id}`,screen[0]+6,screen[1]-5,"#62f2a1");
  });
}
function drawVehicles(){
  for(const vehicle of state.vehicles.values()){
    const p=vehicle.position||{},screen=projection(Number(p.x)||0,Number(p.y)||0,Number(p.z)||0);
    const uav=vehicle.type==="UAV";
    ctx.fillStyle=uav?"#4da7ff":"#ffd052";ctx.strokeStyle="#061017";ctx.lineWidth=1.5;ctx.beginPath();
    if(uav){ctx.moveTo(screen[0]+8,screen[1]);ctx.lineTo(screen[0]-6,screen[1]-5);ctx.lineTo(screen[0]-3,screen[1]);ctx.lineTo(screen[0]-6,screen[1]+5)}
    else{ctx.rect(screen[0]-8,screen[1]-5,16,10);ctx.moveTo(screen[0]+8,screen[1]-5);ctx.lineTo(screen[0]+13,screen[1]);ctx.lineTo(screen[0]+8,screen[1]+5)}
    ctx.closePath();ctx.fill();ctx.stroke();label(displayVehicle(vehicle.id),screen[0]+9,screen[1]-8,"#dceaf0");
  }
}
function drawTargets(){
  for(const target of state.targets.values()){
    const p=target.position||{},screen=projection(Number(p.x)||0,Number(p.y)||0,Number(p.z)||0);
    ctx.strokeStyle=(Number(target.source_mask)||0)&8?"#35e889":"#ff5b64";ctx.lineWidth=2;
    ctx.strokeRect(screen[0]-7,screen[1]-7,14,14);
    label(target.track_id,screen[0]+9,screen[1]-8,ctx.strokeStyle);
    const v=target.velocity||{},tip=projection((Number(p.x)||0)+(Number(v.x)||0)*2,(Number(p.y)||0)+(Number(v.y)||0)*2,Number(p.z)||0);
    ctx.beginPath();ctx.moveTo(screen[0],screen[1]);ctx.lineTo(tip[0],tip[1]);ctx.stroke();
  }
}
function label(text,x,y,color){ctx.font="11px monospace";ctx.fillStyle=color;ctx.fillText(text,x,y)}

canvas.addEventListener("wheel",event=>{event.preventDefault();state.scale=Math.max(1.5,Math.min(35,state.scale*(event.deltaY<0?1.12:.89)))},{passive:false});
canvas.addEventListener("pointerdown",event=>{canvas.setPointerCapture(event.pointerId);state.drag={x:event.clientX,y:event.clientY,panX:state.panX,panY:state.panY}});
canvas.addEventListener("pointermove",event=>{if(state.drag){state.panX=state.drag.panX+event.clientX-state.drag.x;state.panY=state.drag.panY+event.clientY-state.drag.y}});
canvas.addEventListener("pointerup",()=>state.drag=null);
canvas.addEventListener("dblclick",resetView);
function resetView(){state.scale=7.5;state.panX=0;state.panY=0}

document.querySelectorAll(".tabs button").forEach(button=>button.onclick=()=>{
  document.querySelectorAll(".tabs button").forEach(item=>item.classList.toggle("active",item===button));
  document.querySelectorAll(".page").forEach(page=>page.classList.remove("active"));
  el(`${button.dataset.page}Page`).classList.add("active");
});
el("sensorSource").onchange=event=>{state.selectedVehicle=event.target.value;rebuildCameraOptions();resetView()};
el("cameraStream").onchange=event=>{state.selectedCamera=event.target.value;const camera=state.cameras.get(state.selectedCamera);if(camera)renderCamera(camera)};
el("topView").onclick=()=>{state.pitch=0;el("topView").classList.add("active");el("obliqueView").classList.remove("active")};
el("obliqueView").onclick=()=>{state.pitch=.72;el("obliqueView").classList.add("active");el("topView").classList.remove("active")};
el("resetView").onclick=resetView;
el("connectButton").onclick=connect;
el("disconnectButton").onclick=()=>disconnect(true);
el("snapshotButton").onclick=()=>send("request_snapshot");
window.addEventListener("beforeunload",()=>disconnect(true));
setInterval(updateSummary,500);
const saved=localStorage.getItem("fleetRelayUrl");if(saved&&!query.get("ws"))el("wsUrl").value=saved;
connect();
requestAnimationFrame(draw);
