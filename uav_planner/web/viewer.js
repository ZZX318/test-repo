/* Native Canvas 2D viewer. The planner stays in Python; this file only draws it. */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const canvas = $("view");
  const ctx = canvas.getContext("2d");
  let data = null;
  let dirty = true;
  let dragging = false;
  let dragMode = "orbit";
  let last = [0, 0];
  let playback = 1;
  let timer = null;
  let mode = "flight";
  let stepData = null;
  let cmpData = null;
  let stepFrame = 0;
  let stepTimer = null;
  const openMap = new Map();
  const closedArr = [];
  const pkey = (p) => p.map((v) => Math.round(v * 10)).join(",");
  const cam = { yaw: 0.70, pitch: 0.55, distance: 62, target: [16, 16, 7],
    fov: 48 * Math.PI / 180, near: 0.5, ortho: false };

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const sub = (a, b) => a.map((v, i) => v - b[i]);
  const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const norm = (a) => { const n = Math.hypot(...a) || 1; return a.map(v => v / n); };
  function resize() {
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, canvas.clientWidth * dpr);
    canvas.height = Math.max(1, canvas.clientHeight * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    dirty = true;
  }
  function fitTo(bounds) {
    const o = bounds.origin, m = bounds.maximum;
    cam.target = [(o[0]+m[0])/2, (o[1]+m[1])/2, o[2] + (m[2]-o[2])*.52];
    const span = Math.max(m[0]-o[0], m[1]-o[1], m[2]-o[2]);
    cam.distance = span * 2.25;
    cam.yaw = .70; cam.pitch = .55; cam.ortho = false; dirty = true;
  }
  function fit() {
    if (data) fitTo(data.bounds);
  }
  function basis() {
    const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    const eye = [cam.target[0] + cam.distance*cp*Math.sin(cam.yaw),
      cam.target[1] - cam.distance*cp*Math.cos(cam.yaw), cam.target[2] + cam.distance*sp];
    const f = norm(sub(cam.target, eye));
    const r = norm(cross(f, [0, 0, 1]));
    const u = cross(r, f);
    return { eye, f, r, u };
  }
  function projectPoint(p) {
    if (!canvas.clientHeight) return null;
    const b = basis(), d = sub(p, b.eye), zv = dot(d, b.f);
    if (zv < cam.near) return null;
    const focal = (canvas.clientHeight/2) / Math.tan(cam.fov/2);
    const scale = cam.ortho ? Math.min(canvas.clientWidth, canvas.clientHeight) / cam.distance : focal / zv;
    return [canvas.clientWidth/2 + dot(d,b.r)*scale, canvas.clientHeight/2 - dot(d,b.u)*scale, zv];
  }
  window.UAVViewerMath = { createCamera: () => ({...cam, target: cam.target.slice()}), projectPoint };
  window.UAVViewerState = () => ({yaw: cam.yaw, pitch: cam.pitch, distance: cam.distance, target: cam.target.slice()});
  function addLine(prims, a, b, color, width=1, alpha=.5) {
    const pa = projectPoint(a), pb = projectPoint(b);
    if (!pa || !pb) return;
    prims.push({ depth: (pa[2]+pb[2])/2, draw() {
      ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width;
      ctx.beginPath(); ctx.moveTo(pa[0],pa[1]); ctx.lineTo(pb[0],pb[1]); ctx.stroke(); ctx.globalAlpha = 1;
    }});
  }
  function addPoly(prims, points, fill, stroke, alpha=.85) {
    const q = points.map(projectPoint); if (q.some(v => !v)) return;
    prims.push({ depth: q.reduce((s,v)=>s+v[2],0)/q.length, draw() {
      ctx.globalAlpha = alpha; ctx.fillStyle = fill; ctx.beginPath(); ctx.moveTo(q[0][0],q[0][1]);
      q.slice(1).forEach(v=>ctx.lineTo(v[0],v[1])); ctx.closePath(); ctx.fill();
      if (stroke) { ctx.strokeStyle=stroke; ctx.lineWidth=.45; ctx.stroke(); } ctx.globalAlpha=1;
    }});
  }
  function addBox(prims, lo, hi) {
    const v = [[lo[0],lo[1],lo[2]],[hi[0],lo[1],lo[2]],[hi[0],hi[1],lo[2]],[lo[0],hi[1],lo[2]],
      [lo[0],lo[1],hi[2]],[hi[0],lo[1],hi[2]],[hi[0],hi[1],hi[2]],[lo[0],hi[1],hi[2]]];
    [[0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]]
      .forEach((f,i)=>addPoly(prims,f.map(j=>v[j]),i===1?"#e6a04f":"#a8612d","#f4c16c",.83));
  }
  function addSphere(prims, c, radius) {
    const q=projectPoint(c); if(!q) return;
    const edge=projectPoint([c[0]+radius,c[1],c[2]]); const r=edge?Math.max(3,Math.abs(edge[0]-q[0])):4;
    prims.push({depth:q[2],draw(){const g=ctx.createRadialGradient(q[0]-r*.3,q[1]-r*.35,1,q[0],q[1],r);
      g.addColorStop(0,"#ffd58c");g.addColorStop(.55,"#d98232");g.addColorStop(1,"#6f351e");
      ctx.fillStyle=g;ctx.globalAlpha=.9;ctx.beginPath();ctx.arc(q[0],q[1],r,0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;}});
  }
  function addCylinder(prims, ob) {
    const n=12, top=[], bot=[]; for(let i=0;i<n;i++){const a=i*Math.PI*2/n;
      top.push([ob.center_xy[0]+Math.cos(a)*ob.radius,ob.center_xy[1]+Math.sin(a)*ob.radius,ob.z_max]);
      bot.push([ob.center_xy[0]+Math.cos(a)*ob.radius,ob.center_xy[1]+Math.sin(a)*ob.radius,ob.z_min]);}
    for(let i=0;i<n;i++)addPoly(prims,[bot[i],bot[(i+1)%n],top[(i+1)%n],top[i]],"#9b5428","#e5a257",.78);
    addPoly(prims,top,"#d48738","#f4c16c",.8);
  }
  function addNodeMarker(prims, p, color, size, alpha){
    const q=projectPoint(p); if(!q) return;
    prims.push({depth:q[2], draw(){
      ctx.globalAlpha=alpha; ctx.fillStyle=color;
      ctx.fillRect(q[0]-size/2,q[1]-size/2,size,size);
      ctx.globalAlpha=1;
    }});
  }
  function sampleNodes(arr, limit){
    if (arr.length<=limit) return arr;
    const out=[], k=arr.length/limit;
    for(let i=0;i<limit;i++) out.push(arr[Math.floor(i*k)]);
    return out;
  }
  function clearStepState(){ openMap.clear(); closedArr.length=0; }
  function applyFrame(f){
    for(const o of f.open){
      const pos=[o[0],o[1],o[2]];
      openMap.set(pkey(pos), {pos, f:o[3]});
    }
    for(const c of f.close){
      openMap.delete(pkey(c));
      closedArr.push(c);
    }
  }
  function stopStepTimer(){
    if(stepTimer){clearInterval(stepTimer);stepTimer=null;const b=$("stepPlay");if(b)b.textContent="播放";}
  }
  function seekStep(target){
    if(!stepData) return;
    target=clamp(target,0,stepData.frames.length);
    if(target<stepFrame){
      clearStepState();
      openMap.set(pkey(stepData.start),{pos:stepData.start,f:null});
      for(let i=0;i<target;i++) applyFrame(stepData.frames[i]);
    } else {
      if(stepFrame===0 && openMap.size===0) openMap.set(pkey(stepData.start),{pos:stepData.start,f:null});
      for(let i=stepFrame;i<target;i++) applyFrame(stepData.frames[i]);
    }
    stepFrame=target; updateStepUI(); dirty=true;
  }
  function addPath(prims, path, color, width, alpha, limit=path.length) {
    for(let i=1;i<Math.min(path.length,limit);i++) addLine(prims,path[i-1],path[i],color,width,alpha);
  }
  function drawSceneBase(prims, d){
    const o=d.bounds.origin, m=d.bounds.maximum;
    for(let i=0;i<=8;i++){const x=o[0]+(m[0]-o[0])*i/8,y=o[1]+(m[1]-o[1])*i/8;
      addLine(prims,[x,o[1],o[2]],[x,m[1],o[2]],"#3c6077",1,.28);
      addLine(prims,[o[0],y,o[2]],[m[0],y,o[2]],"#3c6077",1,.28);}
    for(const ob of (d.obstacles_geometry||[])){if(ob.type==="box")addBox(prims,ob.minimum,ob.maximum);
      else if(ob.type==="sphere")addSphere(prims,ob.center,ob.radius);else if(ob.type==="cylinder")addCylinder(prims,ob);}
    if(!(d.obstacles_geometry||[]).length){const s=d.bounds.resolution||1;for(const p of d.obstacles||[])addBox(prims,p.map((v)=>v-s*.46),p.map((v)=>v+s*.46));}
  }
  function drawStep(){
    if(!stepData) return;
    dirty=false; ctx.clearRect(0,0,canvas.clientWidth,canvas.clientHeight);
    const prims=[];
    drawSceneBase(prims, stepData);
    for(const p of sampleNodes(closedArr,1500)) addNodeMarker(prims,p,"#4a90d9",4,.5);
    for(const v of openMap.values()) addNodeMarker(prims,v.pos,"#ffb347",4.5,.9);
    if(stepFrame>0){const f=stepData.frames[stepFrame-1];if(f&&f.cur)addNodeMarker(prims,f.cur,"#ffffff",9,1);}
    const qs=projectPoint(stepData.start);
    if(qs)prims.push({depth:qs[2],draw(){ctx.fillStyle="#49e4ff";ctx.shadowColor="#49e4ff";ctx.shadowBlur=12;ctx.beginPath();ctx.arc(qs[0],qs[1],6,0,7);ctx.fill();ctx.shadowBlur=0;}});
    const qg=projectPoint(stepData.goal);
    if(qg)prims.push({depth:qg[2],draw(){ctx.strokeStyle="#ff7181";ctx.lineWidth=2;ctx.strokeRect(qg[0]-6,qg[1]-6,12,12);}});
    if(stepFrame===stepData.frames.length && (stepData.path||[]).length){
      addPath(prims,stepData.path,"#61efff",7,.13);addPath(prims,stepData.path,"#a0f8ff",2.2,.95);}
    prims.sort((a,b)=>b.depth-a.depth); prims.forEach(p=>p.draw());
  }
  function drawCompare(){
    if(!cmpData) return;
    dirty=false; ctx.clearRect(0,0,canvas.clientWidth,canvas.clientHeight);
    const prims=[];
    drawSceneBase(prims, cmpData);
    for(const it of cmpData.items) addPath(prims,it.path,it.color,6,.12);
    for(const it of cmpData.items) addPath(prims,it.path,it.color,2.2,.9);
    const qs=projectPoint(cmpData.start);
    if(qs)prims.push({depth:qs[2],draw(){ctx.fillStyle="#49e4ff";ctx.shadowColor="#49e4ff";ctx.shadowBlur=12;ctx.beginPath();ctx.arc(qs[0],qs[1],6,0,7);ctx.fill();ctx.shadowBlur=0;}});
    const qg=projectPoint(cmpData.goal);
    if(qg)prims.push({depth:qg[2],draw(){ctx.strokeStyle="#ff7181";ctx.lineWidth=2;ctx.strokeRect(qg[0]-6,qg[1]-6,12,12);}});
    prims.sort((a,b)=>b.depth-a.depth); prims.forEach(p=>p.draw());
  }
  function draw() {
    if (mode==="step"){drawStep();return;}
    if (mode==="compare"){drawCompare();return;}
    if (!data) return;
    dirty=false; ctx.clearRect(0,0,canvas.clientWidth,canvas.clientHeight);
    const prims=[];
    drawSceneBase(prims, data);
    (data.paths||[]).forEach(p=>addPath(prims,p,"#61efff",7,.13)); (data.paths||[]).forEach(p=>addPath(prims,p,"#a0f8ff",2.2,.95));
    const lim=(data.trajectories||[]).length?Math.max(2,Math.floor((data.trajectories[0].length-1)*playback)+1):0;
    (data.trajectories||[]).forEach(p=>addPath(prims,p,"#78ffb5",4,.2,lim)); (data.trajectories||[]).forEach(p=>addPath(prims,p,"#b2ffd0",1.4,.85,lim));
    for(const p of data.starts||[]){const q=projectPoint(p);if(q){prims.push({depth:q[2],draw(){ctx.fillStyle="#49e4ff";ctx.shadowColor="#49e4ff";ctx.shadowBlur=13;ctx.beginPath();ctx.arc(q[0],q[1],6,0,7);ctx.fill();ctx.shadowBlur=0;}});}}
    for(const p of data.goals||[]){const q=projectPoint(p);if(q){prims.push({depth:q[2],draw(){ctx.strokeStyle="#ff7181";ctx.lineWidth=2;ctx.strokeRect(q[0]-6,q[1]-6,12,12);}});}}
    prims.sort((a,b)=>b.depth-a.depth); prims.forEach(p=>p.draw());
  }
  function schedule(){requestAnimationFrame(()=>{if(dirty)draw();schedule();});}
  function reset(){fit();playback=1;const bar=$("progress");if(bar)bar.value=100;}
  function installToolbar(){
    const bar=document.querySelector(".stagebar"); if(!bar)return;
    // Camera view buttons are shared by every mode.
    const views=document.createElement("div"); views.className="viewer-tools"; views.id="viewTools";
    views.innerHTML='<button data-view=\"iso\">等轴测</button><button data-view=\"top\">俯视</button><button data-view=\"front\">正视</button><button data-view=\"side\">侧视</button><button id=\"resetView\">重置视角</button>';
    bar.appendChild(views);
    views.querySelectorAll("[data-view]").forEach(b=>b.onclick=()=>{if(b.dataset.view==="top")cam.pitch=1.49,cam.yaw=.7;if(b.dataset.view==="front")cam.pitch=.18,cam.yaw=0;if(b.dataset.view==="side")cam.pitch=.18,cam.yaw=Math.PI/2;if(b.dataset.view==="iso")cam.pitch=.55,cam.yaw=.7;dirty=true;});
    $("resetView").onclick=()=>{fit();dirty=true;};
    // Trajectory playback is flight-mode only.
    const flight=document.createElement("div"); flight.className="viewer-tools"; flight.id="flightTools";
    flight.innerHTML='<button id=\"play\">播放轨迹</button><input id=\"progress\" type=\"range\" min=\"0\" max=\"100\" value=\"100\" title=\"回放进度\">';
    bar.appendChild(flight);
    $("progress").oninput=e=>{playback=Number(e.target.value)/100;dirty=true;};
    $("play").onclick=()=>{if(timer){clearInterval(timer);timer=null;$("play").textContent="播放轨迹";return;}playback=0;timer=setInterval(()=>{playback+=.012;if(playback>=1){playback=1;clearInterval(timer);timer=null;$("play").textContent="播放轨迹";}$("progress").value=playback*100;dirty=true;},40);$("play").textContent="暂停";};
  }
  canvas.style.touchAction="none";
  canvas.onpointerdown=e=>{dragging=true;dragMode=(e.button!==0||e.shiftKey)?"pan":"orbit";last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointerup=()=>{dragging=false;}; canvas.onpointercancel=()=>{dragging=false;};
  canvas.onpointermove=e=>{if(!dragging)return;const dx=e.clientX-last[0],dy=e.clientY-last[1];last=[e.clientX,e.clientY];
    if(dragMode==="orbit"){cam.yaw+=dx*.008;cam.pitch=clamp(cam.pitch+dy*.008,.02,1.50);}
    else{const b=basis(),k=cam.distance*.0025;cam.target=cam.target.map((v,i)=>v-b.r[i]*dx*k+b.u[i]*dy*k);}dirty=true;};
  canvas.onmousedown=e=>{dragging=true;dragMode=(e.button!==0||e.shiftKey)?"pan":"orbit";last=[e.clientX,e.clientY];};
  window.addEventListener("mousemove",e=>{if(!dragging)return;const dx=e.clientX-last[0],dy=e.clientY-last[1];last=[e.clientX,e.clientY];
    if(dragMode==="orbit"){cam.yaw+=dx*.008;cam.pitch=clamp(cam.pitch+dy*.008,.02,1.50);}
    else{const b=basis(),k=cam.distance*.0025;cam.target=cam.target.map((v,i)=>v-b.r[i]*dx*k+b.u[i]*dy*k);}dirty=true;});
  window.addEventListener("mouseup",()=>{dragging=false;});
  canvas.onwheel=e=>{e.preventDefault();cam.distance=clamp(cam.distance*Math.exp(e.deltaY*.0012),.35,400);dirty=true;};
  canvas.ondblclick=reset; window.onresize=resize; window.onkeydown=e=>{if(e.key.toLowerCase()==="r")reset();if(e.key==="ArrowLeft")cam.yaw-=.08;if(e.key==="ArrowRight")cam.yaw+=.08;if(e.key==="ArrowUp")cam.pitch=clamp(cam.pitch-.06,.02,1.5);if(e.key==="ArrowDown")cam.pitch=clamp(cam.pitch+.06,.02,1.5);if(e.key==="+")cam.distance*=.9;if(e.key==="-")cam.distance*=1.1;dirty=true;};
  function updateStepUI(){
    if(!stepData) return;
    const f = stepFrame>0 ? stepData.frames[stepFrame-1] : null;
    if(f && f.curinfo){
      $("stG").textContent=f.curinfo.g;
      $("stH").textContent=f.curinfo.h;
      $("stF").textContent=f.curinfo.f;
      $("stExp").textContent=f.exp.toLocaleString();
      $("stOpen").textContent=f.osize.toLocaleString();
    } else {
      $("stG").textContent="—";$("stH").textContent="—";$("stF").textContent="—";
      $("stExp").textContent="0";$("stOpen").textContent="1";
    }
    $("stTotal").textContent=stepData.total_expanded.toLocaleString();
    const pb=$("stepProgress");
    if(pb && document.activeElement!==pb) pb.value=stepFrame/stepData.frames.length*100;
  }
  async function loadStep(hmode){
    hmode = hmode || "euclidean";
    const q=new URLSearchParams({layout:$("layout").value,planner:"astar",shape:$("shape").value,
      heuristic:hmode,diagonal:$("diagonal").checked?"1":"0",smooth:"1",
      seed:$("seed").value,uavs:"1",obstacles:$("obstacles").value,hard:$("hard").value});
    $("status").textContent="正在生成算法搜索过程…";
    try{
      const r=await fetch("/api/step?"+q),d=await r.json();
      if(!r.ok)throw Error(d.error||"步进生成失败");
      stopStepTimer();
      stepData=d; stepFrame=0; clearStepState();
      fitTo(d.bounds);
      seekStep(0);
      const sa=$("stepAlgo"); if(sa) sa.value=hmode;
      $("status").textContent=(hmode==="zero"?"Dijkstra":"A*")+" 搜索过程 · 共 "+d.total_expanded.toLocaleString()+" 个扩展";
      dirty=true;
    } catch(e){ $("status").textContent="失败："+e.message; }
  }
  async function loadCompare(){
    const q=new URLSearchParams({layout:$("layout").value,planner:"astar",shape:$("shape").value,
      heuristic:"euclidean",diagonal:$("diagonal").checked?"1":"0",smooth:"1",
      seed:$("seed").value,uavs:"1",obstacles:$("obstacles").value,hard:$("hard").value});
    $("status").textContent="正在对四种算法做同地图对比…";
    try{
      const r=await fetch("/api/compare?"+q),d=await r.json();
      if(!r.ok)throw Error(d.error||"对比失败");
      cmpData=d; fitTo(d.bounds); renderCompareTable(d);
      $("status").textContent="算法对比完成 · "+new Date().toLocaleTimeString();
      dirty=true;
    }catch(e){$("status").textContent="失败："+e.message;}
  }
  function renderCompareTable(d){
    const tbl=$("cmpTable");
    tbl.innerHTML='<tr><td>算法</td><td>路径长</td><td>扩展数</td><td>耗时</td></tr>';
    const minExp=Math.min(...d.items.map(i=>i.expanded));
    const minLen=Math.min(...d.items.map(i=>i.length));
    for(const it of d.items){
      const tr=document.createElement("tr");
      const expMark=(it.expanded===minExp)?' <b style="color:var(--green)">最少</b>':"";
      const lenMark=(it.length===minLen)?' <b style="color:var(--green)">最短</b>':"";
      tr.innerHTML='<td><span class="dot" style="background:'+it.color+'"></span>'+it.name+'</td>'+
        '<td>'+it.length+lenMark+'</td>'+
        '<td>'+it.expanded.toLocaleString()+expMark+'</td>'+
        '<td>'+it.time+' s</td>';
      tbl.appendChild(tr);
    }
  }
  function installStepToolbar(){
    const bar=document.querySelector(".stagebar"); if(!bar)return;
    const t=document.createElement("div"); t.className="stepbar"; t.id="stepTools";
    t.style.display="none";
    t.innerHTML='<select id="stepAlgo"><option value="euclidean">A* · 有启发</option><option value="zero">Dijkstra · 无启发</option></select>'+
      '<button id="stepReset">重置</button><button id="stepBack">◀单步</button><button id="stepPlay">播放</button><button id="stepFwd">单步▶</button>'+
      '<input id="stepSpeed" class="stepspeed" type="range" min="1" max="20" value="4" title="播放速度（每次前进帧数）">'+
      '<input id="stepProgress" type="range" min="0" max="100" value="0" style="width:150px;accent-color:var(--amber)" title="搜索进度">';
    bar.appendChild(t);
    $("stepAlgo").onchange=function(){loadStep(this.value);};
    $("stepReset").onclick=()=>{stopStepTimer();seekStep(0);};
    $("stepBack").onclick=()=>{stopStepTimer();seekStep(stepFrame-1);};
    $("stepFwd").onclick=()=>{stopStepTimer();seekStep(stepFrame+1);};
    $("stepProgress").oninput=function(){stopStepTimer();seekStep(Math.floor(Number(this.value)/100*stepData.frames.length));};
    $("stepPlay").onclick=()=>{
      if(stepTimer){stopStepTimer();return;}
      if(stepFrame>=stepData.frames.length) seekStep(0);
      $("stepPlay").textContent="暂停";
      stepTimer=setInterval(()=>{
        if(stepFrame>=stepData.frames.length){stopStepTimer();return;}
        seekStep(stepFrame+Number($("stepSpeed").value));
      },50);
    };
  }
  function setMode(m){
    mode=m;
    $("modeStep").classList.toggle("active",m==="step");
    $("modeFlight").classList.toggle("active",m==="flight");
    $("modeCmp").classList.toggle("active",m==="compare");
    const st=$("stepTools"); if(st)st.style.display=m==="step"?"flex":"none";
    const ft=$("flightTools"); if(ft)ft.style.display=m==="flight"?"flex":"none";
    const vt=$("viewTools"); if(vt)vt.style.display="flex";
    $("legendStep").style.display=m==="step"?"block":"none";
    $("legendFlight").style.display=m==="flight"?"block":"none";
    $("legendCmp").style.display=m==="compare"?"block":"none";
    $("stepPanel").style.display=m==="step"?"block":"none";
    $("cmpPanel").style.display=m==="compare"?"block":"none";
    stopStepTimer();
    if(m==="step"){ loadStep($("stepAlgo")?$("stepAlgo").value:"euclidean"); }
    else if(m==="compare"){ loadCompare(); }
    else { if(data)dirty=true; }
  }
  async function run(){
    const btn=$("run");btn.disabled=true;btn.textContent="计算中…";$("status").textContent="正在生成场景、规划路径并执行安全检查";
    const q=new URLSearchParams({layout:$("layout").value,planner:$("planner").value,shape:$("shape").value,heuristic:$("heuristic").value,diagonal:$("diagonal").checked?"1":"0",smooth:$("smooth").checked?"1":"0",seed:$("seed").value,uavs:$("uavs").value,obstacles:$("obstacles").value,hard:$("hard").value});
    try{const r=await fetch("/api/plan?"+q),d=await r.json();if(!r.ok)throw Error(d.error||"规划失败");data=d;fit();renderMetrics(d);$("status").textContent="已完成 · seed "+d.controls.seed+" · "+new Date().toLocaleTimeString();dirty=true;}
    catch(e){$("status").textContent="失败："+e.message;}finally{btn.disabled=false;btn.textContent="运行实验 · 生成路径";}
  }
  function renderMetrics(d){const p=d.metrics.plan||{},e=d.metrics.execution||{},ok=!p.clearance_violation&&!e.clearance_violation&&p.success_rate>=.99;
    $("quality").textContent=ok?"SAFE · READY":"CHECK REQUIRED";$("quality").style.color=ok?"var(--green)":"var(--amber)";$("qualityText").textContent=ok?"规划与执行均满足硬净空约束":"存在约束或到达质量需要复核";
    const fmt=(v,n)=>v==null?"—":Number(v).toFixed(n==null?2:n),pct=v=>v==null?"—":(Number(v)*100).toFixed(1)+"%";
    $("length").textContent=fmt(p.mean_path_length)+" m";$("clearance").textContent=fmt(p.min_clearance)+" m";$("expanded").textContent=(d.results||[]).reduce((a,r)=>a+(r.expanded||0),0).toLocaleString();$("time").textContent=fmt((d.timings||{}).planning,3)+" s";$("ps").textContent=pct(p.success_rate);$("es").textContent=pct(e.success_rate);$("ec").textContent=fmt(e.min_clearance)+" m";$("sep").textContent=fmt(e.min_separation)+" m";$("smooth").textContent=fmt(e.mean_smoothness,3);
    $("explain").textContent=d.scenario.layout+" 场景中，"+d.controls.planner+" 为 "+d.controls.uavs+" 架 UAV 生成了 "+d.scenario.obstacles+" 个障碍约束。规划层先保证可行，执行层再用限速、限加速度和安全截断验证“飞得出来”。";
  }
  const valueLabels={seed:"seedv",uavs:"uavv",obstacles:"obsv",hard:"hardv"};
  ["seed","uavs","obstacles","hard"].forEach(id=>$(id).oninput=()=>{const label=$(valueLabels[id]);if(label)label.textContent=id==="hard"?Number($(id).value).toFixed(2)+" m":$(id).value;});
  function onControlChange(){
    if(mode==="step"){ stopStepTimer(); loadStep($("stepAlgo")?$("stepAlgo").value:"euclidean"); }
    else if(mode==="compare"){ loadCompare(); }
    else run();
  }
  $("run").onclick=onControlChange;
  ["layout","planner","shape","heuristic","diagonal","smooth","seed","uavs","obstacles","hard"]
    .forEach(id=>$(id).addEventListener("change",onControlChange));
  $("modeStep").onclick=()=>setMode("step");
  $("modeFlight").onclick=()=>setMode("flight");
  $("modeCmp").onclick=()=>setMode("compare");
  installToolbar(); installStepToolbar(); resize(); fit(); run(); schedule();
})();
