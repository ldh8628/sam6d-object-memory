(() => {
  "use strict";
  const data = window.PEM_EXPLORER_DATA;
  if (!data || data.schema_version !== 1) {
    document.body.innerHTML = "<p class='notice'>지원하지 않는 report schema입니다.</p>";
    return;
  }
  const $ = (q) => document.querySelector(q);
  const title = $("#title"), slider = $("#frame-slider"), search = $("#frame-search");
  const image = $("#frame-image"), canvas = $("#pose-canvas"), grid = $("#object-grid"), details = $("#details");
  const initialFrame = Math.trunc(Number(new URLSearchParams(location.search).get("frame")));
  let index = Number.isFinite(initialFrame) ? Math.max(0, Math.min(data.frames.length - 1, initialFrame)) : 0;
  let selectedObject = null, selectedCandidate = 0, topN = 10;
  title.textContent = data.title;
  $("#dataset-source").textContent = `${data.dataset.id || "dataset"} · ${data.dataset.camera_source || "source unspecified"}`;
  $("#frame-count").textContent = `${data.frames.length.toLocaleString()} frames`;
  $("#mode-badge").textContent = data.dataset.partial ? "PARTIAL DIAGNOSTIC" : "FULL DIAGNOSTIC";
  $("#mode-badge").className = data.dataset.partial ? "warning" : "";
  $("#gt-notice").textContent = `${data.ground_truth.notice}  판정: 회전 ≤ ${data.ground_truth.rotation_threshold_deg}°, 이동 ≤ ${data.ground_truth.translation_threshold_mm} mm. 축: ${data.axes.source}`;
  slider.max = Math.max(0, data.frames.length - 1); search.max = slider.max;

  const stateText = {detected:"PEM 결과",pem_pose_invalid:"PEM 최종 pose 무효",not_detected:"검출 없음",pem_input_rejected:"PEM 입력 탈락"};
  const fmt = (v, n=3) => v == null ? "—" : Number(v).toFixed(n);
  const esc = (value) => String(value).replace(/[&<>"']/g, ch => ({
    "&":"&amp;", "<":"&lt;", ">":"&gt;", "\"":"&quot;", "'":"&#39;"
  })[ch]);
  const ox = (entry) => {
    if (entry?.status === "extrinsic_missing") return `<span class="ox missing">GT 없음(extrinsic_missing)</span>`;
    if (entry?.status === "gt_missing") return `<span class="ox missing">GT 없음</span>`;
    if (entry?.status === "slam_missing") return `<span class="ox missing">SLAM 자세 없음</span>`;
    if (!entry || entry.status !== "available" || entry.present == null) return `<span class="ox missing">미수집</span>`;
    return `<span class="ox ${entry.present ? "ok" : "bad"}">${entry.present ? "O" : "X"}</span>`;
  };
  const selectedOx = (entry) => {
    if (entry?.status === "extrinsic_missing") return `<span class="ox missing">GT 없음(extrinsic_missing)</span>`;
    if (entry?.status === "slam_missing") return `<span class="ox missing">SLAM 자세 없음</span>`;
    if (entry?.status === "gt_missing") return `<span class="ox missing">GT 없음</span>`;
    if (!entry || entry.status !== "available" || entry.ok == null) return `<span class="ox missing">GT 없음</span>`;
    return `<span class="ox ${entry.ok ? "ok" : "bad"}">${entry.ok ? "O" : "X"}</span>`;
  };
  const verifyBadge = (status) => {
    const key=String(status||"unavailable").toLowerCase();
    const text={pass:"PASS",fail:"FAIL",diagnostic:"DIAGNOSTIC",unavailable:"UNAVAILABLE"}[key]||String(status||"—");
    return `<span class="verify-badge ${esc(key)}">${esc(text)}</span>`;
  };

  function drawArrow(ctx, line, color, label, sx, sy) {
    if (!line) return;
    const [a,b] = line, x0=a[0]*sx,y0=a[1]*sy,x1=b[0]*sx,y1=b[1]*sy,ang=Math.atan2(y1-y0,x1-x0);
    ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke();
    ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x1-10*Math.cos(ang-.45),y1-10*Math.sin(ang-.45));ctx.lineTo(x1-10*Math.cos(ang+.45),y1-10*Math.sin(ang+.45));ctx.fill();
    ctx.font="bold 11px sans-serif";ctx.fillText(label,x1+4,y1-4);
  }
  function drawAxes(frame) {
    const rect=canvas.getBoundingClientRect(), shape=data.camera.image_shape;
    canvas.width=Math.round(rect.width*devicePixelRatio);canvas.height=Math.round(rect.height*devicePixelRatio);
    const ctx=canvas.getContext("2d");ctx.scale(devicePixelRatio,devicePixelRatio);
    const sx=rect.width/shape[0],sy=rect.height/shape[1];
    frame.slots.forEach(slot=>{if(!slot.detection)return;drawArrow(ctx,slot.detection.front_line,"#ff6b7a",slot.object+" F",sx,sy);drawArrow(ctx,slot.detection.up_line,"#58d9ff","U",sx,sy);});
  }
  function renderCards(frame) {
    grid.innerHTML="";
    frame.slots.forEach(slot=>{
      const node=$("#slot-template").content.firstElementChild.cloneNode(true);
      node.querySelector(".object-name").textContent=slot.object;
      const st=node.querySelector(".state");st.textContent=stateText[slot.state]||slot.state;st.classList.add(slot.state);
      node.querySelector(".score").textContent=slot.detection?`score ${fmt(slot.detection.score)}`:(slot.rejection?.input_rejection||"");
      if(slot.object===selectedObject)node.classList.add("selected");
      node.disabled=!slot.detection;node.onclick=()=>{selectedObject=slot.object;selectedCandidate=0;render();};grid.appendChild(node);
    });
  }
  function evidenceHtml(candidate) {
    const ev=candidate?.evidence||{mode:"uncollected"};
    if(ev.mode==="uncollected")return `<p class="warning">이 실행에서는 점 단위 기하/텍스처 근거가 수집되지 않았습니다.</p>`;
    const cards=[];
    if(ev.pose)cards.push(`<div class="evidence-card"><img src="${ev.pose}"><span>후보 CAD 투영</span></div>`);
    if(ev.geometry)cards.push(`<div class="evidence-card"><img src="${ev.geometry}"><span>표본 기하 NN 거리 · PEM crop 좌표</span></div>`);
    if(ev.texture)cards.push(`<div class="evidence-card"><img src="${ev.texture}"><span>표본 PEM 특징 cosine · PEM crop 좌표</span></div>`);
    if(ev.shape_input)cards.push(`<div class="evidence-card"><img src="${ev.shape_input}"><span>입력 depth/radius mask</span></div>`);
    if(ev.shape_render)cards.push(`<div class="evidence-card"><img src="${ev.shape_render}"><span>후보 CAD point-splat mask</span></div>`);
    if(ev.shape_overlap)cards.push(`<div class="evidence-card"><img src="${ev.shape_overlap}"><span>교집합(초록) · 입력(빨강) · 렌더(노랑)</span></div>`);
    const g=ev.metrics?.geometry,t=ev.metrics?.texture;
    const overlap = g?.overlap_ratio == null ? "—" : `${fmt(g.overlap_ratio*100,1)}%`;
    const provenance = ev.provenance ? `<p class="provenance">측정값: ${esc(ev.provenance.point_values)} · 겹침: ${esc(ev.provenance.overlap)} · heatmap: ${esc(ev.provenance.heatmap)} · 자세 영상: ${esc(ev.provenance.pose)}</p>` : "";
    const s=ev.shape_metrics;
    const shapeMetrics=s?`<div class="metric-grid"><div class="metric">입력/렌더 면적비 ${fmt(s.size_ratio)}</div><div class="metric">2D point-mask IoU ${fmt(s.mask_iou)}</div><div class="metric">입력 coverage ${fmt(s.coverage)}</div><div class="metric">크기 검증 ${s.size_verified?"PASS":"FAIL"}</div></div><p class="provenance">${esc(ev.shape_note||"")}</p>`:"";
    return `<div class="evidence-grid">${cards.join("")}</div>${g?`<div class="metric-grid"><div class="metric">3D NN 겹침 ${g.overlap_count}/${g.point_count} (${overlap})</div><div class="metric">표본 NN 평균 ${fmt(g.mean_distance_mm,1)} mm</div><div class="metric">표본 texture ${fmt(t?.feature_mean)}</div><div class="metric">표본 color ${fmt(t?.color_mean)}</div></div>${provenance}`:"<p class='warning'>3D 점 근거는 미수집이며 자세/shape 이미지는 별도 proxy입니다.</p>"}${shapeMetrics}`;
  }
  function renderDetails(slot) {
    if(!slot?.detection){details.className="details empty";details.innerHTML="<p>검출된 객체 카드를 선택하면 후보를 자세히 볼 수 있습니다.</p>";return;}
    details.className="details";const d=slot.detection,cands=d.candidates||[],cand=cands[selectedCandidate],v=d.verify||{};
    const rows=cands.slice(0,topN).map((c,i)=>`<tr class="${i===selectedCandidate?"selected":""}" data-i="${i}"><td>${c.geometry_selected?"✓":""}</td><td>#${c.rank_geo}</td><td>${fmt(c.geo)}</td><td>#${c.rank_texture??"—"}</td><td>${fmt(c.texture_score)}</td><td>${fmt(c.color_score)}</td><td>${fmt(c.shape?.size_ratio)}</td><td>${fmt(c.shape?.mask_iou)}</td><td>${fmt(c.shape?.coverage)}</td><td>${selectedOx(c.gt)}</td></tr>`).join("");
    const poseWarning=slot.state==="pem_pose_invalid"?`<p class="pose-invalid-warning">최종 PEM pose를 투영할 수 없습니다 (${esc(d.pose_status?.reason||"invalid")}). 검출 정보와 후보 상세은 그대로 확인할 수 있습니다.</p>`:"";
    const selectedResult=slot.state==="pem_pose_invalid"?`<span class="ox missing">최종 pose 무효</span>`:selectedOx(d.gt);
    details.innerHTML=`<div class="detail-head"><div><p class="eyebrow">selected object</p><h2>${esc(slot.object)}</h2></div>${selectedResult}</div>
      ${poseWarning}<div class="section-title">기하-only 초기 포즈 → 최종 PEM pose</div><div class="matrix">R ${JSON.stringify(d.R)}\nt_mm ${JSON.stringify(d.t_mm)}</div>
      <div class="verification-grid"><div class="verification-card"><b>기하 선택</b>${verifyBadge(v.selection_method==="geometry_only"?"pass":"unavailable")}<small>300개 중 기하 1위 · proposal ${v.selected_proposal6000_index??"—"}</small></div><div class="verification-card"><b>텍스처 검증</b>${verifyBadge(v.texture?.status)}<small>score ${fmt(v.texture?.score)} · 텍스처 순위 #${v.texture?.rank??"—"}/${v.texture?.candidate_count??"—"}</small></div><div class="verification-card"><b>투영 크기</b>${verifyBadge(v.shape?.status)}<small>입력/렌더 ${fmt(v.shape?.size_ratio)} · 기준 ≤ ${fmt(v.shape?.size_ratio_max)}</small></div><div class="verification-card"><b>2D IoU</b>${verifyBadge(v.shape?.iou_status)}<small>IoU ${fmt(v.shape?.mask_iou)} · coverage ${fmt(v.shape?.coverage)}</small></div></div>
      <div class="stage-row"><div class="stage-box"><b>6,000 후보</b><br>${ox(d.stage6000)}<br><small>${d.stage6000.match_count??"—"} matches</small></div><div class="stage-box"><b>상위 300</b><br>${ox(d.stage300)}<br><small>${d.stage300.match_count??"—"} matches</small></div></div>
      <div class="section-title">후보 점수</div><div class="candidate-controls"><label>상위 N <select id="topn">${[5,10,20,50,100].map(n=>`<option ${n===topN?"selected":""}>${n}</option>`).join("")}</select></label><span>${cands.length} candidates</span></div>
      ${cands.length?`<table class="candidate-table"><thead><tr><th>선택</th><th>기하순위</th><th>geo</th><th>텍스처순위</th><th>texture</th><th>color</th><th>크기비</th><th>IoU</th><th>coverage</th><th>정답</th></tr></thead><tbody>${rows}</tbody></table>`:"<p class='warning'>후보 진단 미수집</p>"}
      <div class="section-title">기하 후보 #${cand?.rank_geo??"—"} 근거</div>${evidenceHtml(cand)}
      ${cand?`<div class="matrix">proposal6000 ${cand.proposal6000_index??"legacy/untracked"}\nR ${JSON.stringify(cand.R)}\nt_mm ${JSON.stringify(cand.t_mm)}</div>`:""}`;
    const select=$("#topn");if(select)select.onchange=e=>{topN=Number(e.target.value);selectedCandidate=Math.min(selectedCandidate,topN-1);renderDetails(slot);};
    details.querySelectorAll("tbody tr").forEach(tr=>tr.onclick=()=>{selectedCandidate=Number(tr.dataset.i);renderDetails(slot);});
  }
  function render(){
    const frame=data.frames[index];slider.value=index;search.value=index;$("#frame-label").textContent=`#${index} · source ${frame.source_index??"—"} · ${frame.stamp_ns}`;
    image.onload=()=>drawAxes(frame);image.src=frame.image;renderCards(frame);
    let slot=frame.slots.find(s=>s.object===selectedObject);if(!slot?.detection){slot=frame.slots.find(s=>s.detection);selectedObject=slot?.object||null;selectedCandidate=0;}renderDetails(slot);
  }
  slider.oninput=e=>{index=Number(e.target.value);selectedCandidate=0;render();};search.onchange=e=>{index=Math.max(0,Math.min(data.frames.length-1,Number(e.target.value)||0));selectedCandidate=0;render();};
  $("#prev").onclick=()=>{index=Math.max(0,index-1);selectedCandidate=0;render();};$("#next").onclick=()=>{index=Math.min(data.frames.length-1,index+1);selectedCandidate=0;render();};window.onresize=()=>drawAxes(data.frames[index]);render();
})();
