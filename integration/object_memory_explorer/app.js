"use strict";
const $=selector=>document.querySelector(selector);
const esc=value=>String(value).replace(/[&<>"']/g,char=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[char]);
const video=$("#video"),rawOverlay=$("#raw-overlay"),fusedCanvas=$("#fused"),mapCanvas=$("#map");
const slider=$("#slider"),seqInput=$("#seq"),objectSelect=$("#object");
// ponytail: one 4k-frame dataset fits; bound caches if longer sessions become common.
const cache=new Map(),objectCache=new Map(),mapBase=document.createElement("canvas");
const graphBases=[document.createElement("canvas"),document.createElement("canvas")];
let meta,current=null,currentSeq=-1,renderToken=0,mapHits=[],mapView=null;
let selectedGraph=null,graphViews=[],videoCallback=0,rafCallback=0,lastPresented=-1,lastPlaybackSeq=-1;

function showError(error){const box=$("#error");box.textContent=String(error);box.style.display="block";}
async function getJSON(url){const response=await fetch(url);if(!response.ok)throw new Error((await response.json()).error||response.statusText);return response.json();}
function frame(seq){if(!cache.has(seq))cache.set(seq,getJSON(`/api/frame?seq=${seq}`).catch(error=>{cache.delete(seq);throw error;}));return cache.get(seq);}
function matrixXYZ(matrix){return matrix&&[matrix[0][3],matrix[1][3],matrix[2][3]];}
function fit(canvas,fallbackHeight){const ratio=devicePixelRatio||1,width=canvas.clientWidth,height=canvas.clientHeight||fallbackHeight;canvas.width=Math.max(1,Math.round(width*ratio));canvas.height=Math.max(1,Math.round(height*ratio));const ctx=canvas.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);return [ctx,width,height,ratio];}

function project(point){const K=meta.explorer.camera.K,z=point[2];if(z<=.01)return null;return [K[0]*point[0]/z+K[2],K[4]*point[1]/z+K[5]];}
function drawPoseAxes(ctx,pose){
  const t=pose.t_mm.map(v=>v/1000),R=pose.R,origin=project(t);if(!origin)return null;
  [["#ff6375",0],["#55dda0",1],["#58a7ff",2]].forEach(([color,column])=>{const end=project(t.map((v,i)=>v+R[i][column]*.08));if(!end)return;ctx.beginPath();ctx.moveTo(...origin);ctx.lineTo(...end);ctx.strokeStyle=color;ctx.stroke();});return origin;
}
function drawDetections(ctx,row,corrected,predicted){
  ctx.lineWidth=2;ctx.font="12px system-ui";
  for(const detection of row.detections||[]){
    const [x1,y1,x2,y2]=detection.bbox||[0,0,0,0],memory=detection.object_memory;
    ctx.strokeStyle=memory?.object_id?"#55d8ff":"#ffc65a";ctx.strokeRect(x1,y1,x2-x1,y2-y1);
    if(predicted?.has(memory?.object_id))continue;
    const label=`${detection.object_name}${memory?.object_id?` · #${memory.object_id}`:""}`;
    ctx.fillStyle="#071018cc";ctx.fillRect(x1,Math.max(0,y1-19),ctx.measureText(label).width+10,19);ctx.fillStyle="#e9f4fb";ctx.fillText(label,x1+5,Math.max(13,y1-5));
    const pose=corrected?detection.corrected_pose:detection;if(pose)drawPoseAxes(ctx,pose);
  }
}
function drawPredictions(ctx,row){
  const detected=new Set((row.detections||[]).map(d=>d.object_memory?.object_id).filter(Boolean));ctx.font="12px system-ui";
  for(const pose of row.predictions||[]){const observed=detected.has(pose.object_id),origin=drawPoseAxes(ctx,pose);if(!origin)continue;const prefix=`#${pose.object_id} ${pose.object_name} · `,label=prefix+(observed?"관측 중":"맵 유지"),width=Math.max(ctx.measureText(prefix+"관측 중").width,ctx.measureText(prefix+"맵 유지").width);ctx.strokeStyle=observed?"#55d8ff":"#b68cff";ctx.beginPath();ctx.arc(...origin,5,0,Math.PI*2);ctx.stroke();ctx.fillStyle="#071018cc";ctx.fillRect(origin[0]+7,origin[1]-15,width+8,17);ctx.fillStyle=observed?"#dff8ff":"#e4ccff";ctx.fillText(label,origin[0]+11,origin[1]-3);}
}
function drawVideos(row){
  const width=meta.explorer.camera.width,height=meta.explorer.camera.height,raw=rawOverlay.getContext("2d"),fused=fusedCanvas.getContext("2d");
  rawOverlay.width=fusedCanvas.width=width;rawOverlay.height=fusedCanvas.height=height;
  raw.clearRect(0,0,width,height);drawDetections(raw,row,false);
  fused.clearRect(0,0,width,height);if(video.readyState>=2)fused.drawImage(video,0,0,width,height);const predicted=new Set((row.predictions||[]).map(p=>p.object_id));drawDetections(fused,row,true,predicted);drawPredictions(fused,row);
}

function buildMapBase(){
  const [,w,h,ratio]=fit(mapCanvas,390),bounds=meta.explorer.map_bounds_xz,pad=28;
  mapBase.width=mapCanvas.width;mapBase.height=mapCanvas.height;const ctx=mapBase.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);
  const dx=Math.max(.1,bounds.max[0]-bounds.min[0]),dz=Math.max(.1,bounds.max[1]-bounds.min[1]);
  const scale=Math.min((w-2*pad)/dx,(h-2*pad)/dz),point=xyz=>[pad+(xyz[0]-bounds.min[0])*scale,h-pad-(xyz[2]-bounds.min[1])*scale];
  ctx.clearRect(0,0,w,h);ctx.fillStyle="#6e879788";
  for(const xz of meta.explorer.dense_map.points_xz){const p=point([xz[0],0,xz[1]]);ctx.fillRect(p[0],p[1],1.2,1.2);}
  ctx.fillStyle="#8fa6b5";ctx.font="10px system-ui";ctx.fillText(`x ${bounds.min[0].toFixed(2)}…${bounds.max[0].toFixed(2)} m`,8,h-8);ctx.fillText(`z ${bounds.min[1].toFixed(2)}…${bounds.max[1].toFixed(2)} m`,8,13);
  mapView={w,h,ratio,point};
}
function drawMap(row){
  if(!mapView)buildMapBase();const {ratio,point}=mapView,ctx=mapCanvas.getContext("2d");
  ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,mapCanvas.width,mapCanvas.height);ctx.drawImage(mapBase,0,0);ctx.setTransform(ratio,0,0,ratio,0,0);ctx.lineWidth=1;
  if(row.camera){const p=point(row.camera.xyz),q=point(row.camera.xyz.map((v,i)=>v+row.camera.forward[i]*.25));ctx.strokeStyle="#fff";ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();ctx.fillStyle="#fff";ctx.beginPath();ctx.arc(...p,4,0,Math.PI*2);ctx.fill();}
  mapHits=[];
  for(const item of row.objects||[]){const xyz=matrixXYZ(item.fused_T_map_obj);if(!xyz)continue;const p=point(xyz),color={active:"#55dda0",tentative:"#ffc65a",lost:"#ff8b64",remembered:"#b68cff"}[item.status]||"#8fa6b5";ctx.fillStyle=color;ctx.beginPath();ctx.arc(...p,6,0,Math.PI*2);ctx.fill();ctx.fillStyle="#e9f4fb";ctx.font="11px system-ui";ctx.fillText(`#${item.object_id} ${item.object_name}`,p[0]+9,p[1]-8);mapHits.push({x:p[0],y:p[1],id:item.object_id});}
  for(const anchor of row.anchors||[]){const p=point(anchor.xyz);ctx.strokeStyle="#fff";ctx.lineWidth=2;for(const radius of [7,11]){ctx.beginPath();ctx.arc(...p,radius,0,Math.PI*2);ctx.stroke();}ctx.fillStyle="#ffc65a";ctx.fillText(`anchor ${anchor.object_name}`,p[0]+13,p[1]+13);}
}

function buildPlot(canvas,base,samples,series,labels){
  const [,w,h,ratio]=fit(canvas,180),pad=28;base.width=canvas.width;base.height=canvas.height;const ctx=base.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);ctx.strokeStyle="#294154";ctx.strokeRect(pad,10,w-pad-10,h-pad);
  const values=series.flatMap(key=>samples.map(s=>key(s)).filter(Number.isFinite)),last=Math.max(1,meta.explorer.frame_count-1);
  if(!samples.length||!values.length){ctx.fillStyle="#8fa6b5";ctx.fillText("관측 데이터 없음",pad+8,32);return {canvas,base,w,h,ratio,pad,last,drawable:false};}
  const min=Math.min(...values),max=Math.max(...values),span=Math.max(1e-9,max-min);
  series.forEach((key,i)=>{ctx.strokeStyle=["#55d8ff","#ff6375","#55dda0"][i];ctx.lineWidth=1.7;ctx.beginPath();let started=false;samples.forEach(sample=>{const value=key(sample);if(!Number.isFinite(value))return;const x=pad+sample.frame_seq/last*(w-pad-10),y=10+(max-value)/span*(h-pad-10);started?ctx.lineTo(x,y):ctx.moveTo(x,y);started=true;});ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText(labels[i],pad+i*110,h-7);});
  ctx.fillStyle="#8fa6b5";ctx.fillText(`${min.toFixed(2)} … ${max.toFixed(2)}`,w-120,h-7);return {canvas,base,w,h,ratio,pad,last,drawable:true};
}
function drawGraphMarkers(){
  for(const view of graphViews){const {canvas,base,w,h,ratio,pad,last,drawable}=view,ctx=canvas.getContext("2d");ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(base,0,0);if(!drawable)continue;ctx.setTransform(ratio,0,0,ratio,0,0);const marker=pad+currentSeq/last*(w-pad-10);ctx.strokeStyle="#ffffff99";ctx.beginPath();ctx.moveTo(marker,10);ctx.lineTo(marker,h-pad);ctx.stroke();}
}
function buildGraphs(){
  if(!selectedGraph)return;const samples=selectedGraph.samples;
  graphViews=[buildPlot($("#residual"),graphBases[0],samples,[s=>s.residual_translation_mm,s=>s.residual_rotation_deg],["translation mm","rotation deg"]),buildPlot($("#convergence"),graphBases[1],samples,[s=>s.fused_xyz?.[0],s=>s.fused_xyz?.[1],s=>s.fused_xyz?.[2]],["x m","y m","z m"])];drawGraphMarkers();
}
async function selectGraphs(){
  const id=Number(objectSelect.value);if(!id)return;try{if(!objectCache.has(id))objectCache.set(id,getJSON(`/api/object?id=${id}`).catch(error=>{objectCache.delete(id);throw error;}));const data=await objectCache.get(id);if(Number(objectSelect.value)!==id)return;selectedGraph=data;$("#object-summary").innerHTML=`#${id} ${esc(data.object_name)} · <span class="status-${esc(data.final_status)}">${esc(data.final_status)}</span> · ${data.samples.length} updates`;buildGraphs();}catch(error){showError(error);}
}

function seekVideo(time){if(!video.duration||Math.abs(video.currentTime-time)<=.02)return Promise.resolve();return new Promise(resolve=>{video.addEventListener("seeked",resolve,{once:true});video.currentTime=time;});}
async function render(seq,seek=false){
  seq=Math.trunc(Math.max(0,Math.min(meta.explorer.frame_count-1,Number(seq)||0)));const token=++renderToken;
  try{const row=await frame(seq);if(token!==renderToken)return;if(seek)await seekVideo((seq+.5)/meta.explorer.preview.fps);if(token!==renderToken)return;current=row;currentSeq=seq;slider.value=seq;seqInput.value=seq;$("#clock").textContent=`${row.timestamp_s.toFixed(3)} s`;$("#frame-state").textContent=row.sam_inferred?"SAM-6D inference":"SAM-6D skipped · map carry-forward";
    const messages=(row.transitions||[]).map(event=>`<b>#${event.object_id} ${esc(event.event)}</b>${esc(event.reason)}`);for(const detection of row.detections||[]){if(detection.anchor)messages.push(`<b>${esc(detection.object_name)} anchor</b>${esc(detection.anchor.event||detection.anchor.state)}`);}$("#events").innerHTML=messages.length?messages.join(" · "):"전이 없음";
    drawVideos(row);drawMap(row);drawGraphMarkers();
  }catch(error){showError(error);}
}
function step(delta){video.pause();render(currentSeq+delta,true);}
function stopPlayback(){if(videoCallback&&video.cancelVideoFrameCallback)video.cancelVideoFrameCallback(videoCallback);if(rafCallback)cancelAnimationFrame(rafCallback);videoCallback=rafCallback=0;lastPresented=lastPlaybackSeq=-1;}
function playbackFrame(seq){if(seq===lastPlaybackSeq)return;lastPlaybackSeq=seq;frame(Math.min(seq+1,meta.explorer.frame_count-1)).catch(showError);render(seq);}
function schedulePlayback(){
  if(video.paused)return;
  if(video.requestVideoFrameCallback)videoCallback=video.requestVideoFrameCallback((_,info)=>{videoCallback=0;if(video.paused)return;if(info.presentedFrames!==lastPresented){lastPresented=info.presentedFrames;playbackFrame(Math.round(info.mediaTime*meta.explorer.preview.fps));}schedulePlayback();});
  else rafCallback=requestAnimationFrame(()=>{rafCallback=0;if(video.paused)return;playbackFrame(Math.floor(video.currentTime*meta.explorer.preview.fps+1e-6));schedulePlayback();});
}

async function start(){
  meta=await getJSON("/api/meta");const info=meta.explorer,sam=meta.sam_inference;$("#meta").innerHTML=`<span>${esc(meta.dataset)}</span><span>${info.frame_count} frames</span>${sam?`<span>SAM ${sam.inferred_frames}/${sam.input_frames}</span>`:""}<span>${info.preview.fps.toFixed(3)} fps</span><span>x-z map</span>`;
  slider.max=seqInput.max=info.frame_count-1;video.src="/preview";objectSelect.innerHTML=meta.objects.map(item=>`<option value="${item.object_id}">#${item.object_id} ${esc(item.object_name)} · ${esc(item.status)}</option>`).join("");buildMapBase();await selectGraphs();
  slider.oninput=event=>{video.pause();render(event.target.value,true);};seqInput.onchange=event=>{video.pause();render(event.target.value,true);};$("#prev").onclick=()=>step(-1);$("#next").onclick=()=>step(1);video.onloadedmetadata=()=>render(Math.max(0,currentSeq),true);
  $("#play").onclick=()=>video.paused?video.play():video.pause();video.onplay=()=>{$("#play").textContent="❚❚";stopPlayback();schedulePlayback();};video.onpause=()=>{$("#play").textContent="▶";stopPlayback();};video.onseeking=()=>{lastPresented=lastPlaybackSeq=-1;};objectSelect.onchange=selectGraphs;
  mapCanvas.onclick=event=>{const rect=mapCanvas.getBoundingClientRect(),x=event.clientX-rect.left,y=event.clientY-rect.top,hit=mapHits.find(item=>Math.hypot(item.x-x,item.y-y)<14);if(hit){objectSelect.value=hit.id;selectGraphs();}};
  window.onresize=()=>{buildMapBase();if(current)drawMap(current);buildGraphs();};await render(0,true);
}
start().catch(showError);
