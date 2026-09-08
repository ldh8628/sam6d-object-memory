(() => {
  "use strict";
  const data = window.PEM_EXPLORER_DATA;
  if (!data || data.schema_version !== 1) {
    document.body.innerHTML = "<p class='notice'>지원하지 않는 report schema입니다.</p>";
    return;
  }
  const $ = (q) => document.querySelector(q);
  const title = $("#title"), slider = $("#frame-slider"), search = $("#frame-search");
  const image = $("#frame-image"), canvas = $("#pose-canvas"), grid = $("#object-grid"), details = $("#details"), viewFilter = $("#view-filter");
  const query = new URLSearchParams(location.search);
  const finiteNumber = value => typeof value === "number" && Number.isFinite(value);
  const rawPolicy = window.PEM_VERIFICATION_POLICY || data.verification_filter || {enabled:false};
  const hasAuthoritativeDecisions=rawPolicy?.decisions&&typeof rawPolicy.decisions==="object"&&!Array.isArray(rawPolicy.decisions);
  const fingerprintValid=!hasAuthoritativeDecisions||rawPolicy.report_fingerprint===data.dataset.report_fingerprint;
  const fallbackRaw=rawPolicy?.fallback_shadow;
  const fallbackPolicyValid=!fallbackRaw?.enabled||(fallbackRaw.mode==="shadow_only"&&fallbackRaw.selection_method==="original_geometry_rank_after_pose_cluster"&&fallbackRaw.actual_pem_pose==="unchanged"&&[fallbackRaw.selected_thresholds?.rotation_threshold_deg,fallbackRaw.selected_thresholds?.translation_threshold_mm].every(v=>finiteNumber(v)&&v>0));
  const policyValid = rawPolicy?.schema_version===1&&(!rawPolicy.dataset_id||rawPolicy.dataset_id===data.dataset.id)&&fingerprintValid&&fallbackPolicyValid&&(!rawPolicy.enabled||[rawPolicy.depth_valid_frac_min,rawPolicy.texture_score_min,rawPolicy.mask_iou_min].every(v=>finiteNumber(v)&&v>=0&&v<=1));
  const policy = policyValid?rawPolicy:{schema_version:1,enabled:false,mode:"shadow_only",reason:"policy_schema_or_dataset_mismatch"};
  const viewModes = new Set(["all","holdout_withheld","holdout_accepted","true_reject","false_reject","all_withheld"]);
  const initialFrame = Math.trunc(Number(query.get("frame")));
  const initialRank = Math.trunc(Number(query.get("rank_geo")));
  let index = Number.isFinite(initialFrame) ? Math.max(0, Math.min(data.frames.length - 1, initialFrame)) : 0;
  let selectedObject = query.get("object") || null;
  let selectedCandidateRank = Number.isFinite(initialRank) && initialRank >= 0 ? initialRank : 0;
  let topN = selectedCandidateRank >= 50 ? 100 : selectedCandidateRank >= 20 ? 50 : selectedCandidateRank >= 10 ? 20 : 10;
  let viewMode = viewModes.has(query.get("filter")) ? query.get("filter") : "all";
  let visibleFrames = [];
  title.textContent = data.title;
  $("#dataset-source").textContent = `${data.dataset.id || "dataset"} · ${data.dataset.camera_source || "source unspecified"}`;
  $("#frame-count").textContent = `${data.frames.length.toLocaleString()} frames`;
  $("#mode-badge").textContent = data.dataset.partial ? "PARTIAL DIAGNOSTIC" : "FULL DIAGNOSTIC";
  $("#mode-badge").className = data.dataset.partial ? "warning" : "";
  $("#gt-notice").textContent = `${data.ground_truth.notice}  판정: 회전 ≤ ${data.ground_truth.rotation_threshold_deg}°, 이동 ≤ ${data.ground_truth.translation_threshold_mm} mm. 축: ${data.axes.source}`;
  search.max = Math.max(0, data.frames.length - 1);

  const stateText = {detected:"PEM 결과",pem_pose_invalid:"PEM 최종 pose 무효",not_detected:"검출 없음",pem_input_rejected:"PEM 입력 탈락"};
  const referenceText = {trusted_reference_member:"GT 구성 프레임",untrusted_cluster_member:"비신뢰 GT 후보 프레임",evaluation_frame:"비참조 평가 프레임"};
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
  const finite = finiteNumber;
  const trustedObjects = new Set(data.ground_truth.trusted_objects || []);
  const evaluationFrames = data.frames.filter(frame => frame.slots.some(slot => slot.detection?.reference_role === "evaluation_frame")).map(frame => frame.index);
  const holdoutFrames = new Set(evaluationFrames.slice(Math.floor(evaluationFrames.length / 2)));

  function validAuthoritativeDecision(decision, expectedSplit, expectedEligibility) {
    if(!decision||typeof decision!=="object"||Array.isArray(decision))return false;
    const scores=[decision.depth,decision.texture,decision.iou];
    if(!scores.every(value=>finite(value)&&value>=0&&value<=1))return false;
    if(decision.evaluationEligible!==expectedEligibility||decision.split!==expectedSplit||typeof decision.gtOk!=="boolean")return false;
    const textureFailed=decision.texture<policy.texture_score_min,iouFailed=decision.iou<policy.mask_iou_min;
    const expectedStatus=textureFailed&&iouFailed?"withheld":"accepted";
    if(!(decision.depth+1e-8>=policy.depth_valid_frac_min&&decision.status===expectedStatus&&decision.textureFailed===textureFailed&&decision.iouFailed===iouFailed))return false;
    if(expectedStatus!=="withheld"||!policy.fallback_shadow?.enabled)return true;
    const fallback=decision.fallbackShadow;
    if(!fallback||typeof fallback!=="object")return false;
    if(fallback.status==="unavailable")return fallback.selected===null&&fallback.excluded_cluster_size===0&&fallback.excluded_similar_candidate_count===0&&typeof fallback.reason==="string";
    if(!Number.isInteger(fallback.excluded_cluster_size)||fallback.excluded_cluster_size<1||!Number.isInteger(fallback.excluded_similar_candidate_count)||fallback.excluded_similar_candidate_count!==fallback.excluded_cluster_size-1)return false;
    if(fallback.status==="none")return fallback.selected===null&&typeof fallback.reason==="string";
    if(fallback.status!=="selected"||!fallback.selected)return false;
    const selected=fallback.selected,thresholds=policy.fallback_shadow.selected_thresholds;
    return Number.isInteger(selected.rank_geo)&&selected.rank_geo>0&&finite(selected.texture_score)&&selected.texture_score>=policy.texture_score_min&&finite(selected.mask_iou)&&selected.mask_iou>=policy.mask_iou_min&&finite(selected.rotation_distance_deg)&&selected.rotation_distance_deg>=0&&finite(selected.translation_distance_mm)&&selected.translation_distance_mm>=0&&(selected.rotation_distance_deg>thresholds.rotation_threshold_deg||selected.translation_distance_mm>thresholds.translation_threshold_mm)&&typeof selected.gtOk==="boolean";
  }

  function verificationDecision(frame, slot) {
    const d=slot.detection,v=d?.verify||{};
    if(!d||!policy.enabled)return {status:"unavailable",reason:policy.reason||"policy_disabled"};
    const decisionKey=`${frame.stamp_ns}:${frame.source_index}:${slot.object}`;
    const evaluationEligible=d.reference_role==="evaluation_frame"&&trustedObjects.has(slot.object)&&d.gt?.status==="available";
    const split=holdoutFrames.has(frame.index)?"holdout":"tune";
    const authoritative=policy.decisions&&typeof policy.decisions==="object";
    const override=policy.decisions?.[decisionKey];
    if(override)return validAuthoritativeDecision(override,split,evaluationEligible)?override:{status:"unavailable",reason:"invalid_authoritative_decision"};
    if(authoritative)return {status:"unavailable",reason:"authoritative_decision_missing"};
    const depth=d.pem?.depth_valid_frac,texture=v.texture?.score,iou=v.shape?.mask_iou;
    if(!finite(depth)||!finite(texture)||!finite(iou))return {status:"unavailable",reason:"required_score_missing",depth,texture,iou};
    if(!evaluationEligible)return {status:"unavailable",reason:"outside_evaluation_population",depth,texture,iou,evaluationEligible,split};
    if(depth<policy.depth_valid_frac_min)return {status:"low_depth",reason:"depth_below_scope",depth,texture,iou,evaluationEligible,split};
    const textureFailed=texture<policy.texture_score_min,iouFailed=iou<policy.mask_iou_min;
    return {status:textureFailed&&iouFailed?"withheld":"accepted",reason:textureFailed&&iouFailed?"texture_and_iou_below_threshold":"independent_support_present",depth,texture,iou,textureFailed,iouFailed,evaluationEligible,split};
  }
  data.frames.forEach(frame=>frame.slots.forEach(slot=>{if(slot.detection)slot.detection._verification_filter=verificationDecision(frame,slot);}));

  const matchesSlot = (slot, mode) => {
    const d=slot.detection,v=d?._verification_filter;
    if(!v)return false;
    if(mode==="all_withheld")return v.evaluationEligible&&v.status==="withheld";
    if(!v.evaluationEligible||v.split!=="holdout")return false;
    if(mode==="holdout_withheld")return v.status==="withheld";
    if(mode==="holdout_accepted")return v.status==="accepted";
    const gtOk=v.gtOk??d.gt?.ok;
    if(mode==="true_reject")return v.status==="withheld"&&gtOk===false;
    if(mode==="false_reject")return v.status==="withheld"&&gtOk===true;
    return true;
  };
  const matchesFrame = (frame, mode) => mode==="all" || frame.slots.some(slot=>matchesSlot(slot,mode));
  function refreshVisibleFrames() {
    const matched=data.frames.filter(frame=>matchesFrame(frame,viewMode)).map(frame=>frame.index);
    visibleFrames=matched;
    if(visibleFrames.length&&!visibleFrames.includes(index))index=visibleFrames[0];
    slider.max=Math.max(0,visibleFrames.length-1);
    viewFilter.value=viewMode;
  }
  const holdoutEvaluated=[];
  data.frames.forEach(frame=>frame.slots.forEach(slot=>{const v=slot.detection?._verification_filter;if(v?.evaluationEligible&&v.split==="holdout"&&["accepted","withheld"].includes(v.status))holdoutEvaluated.push(slot.detection);}));
  const accepted=holdoutEvaluated.filter(d=>d._verification_filter.status==="accepted"),withheld=holdoutEvaluated.filter(d=>d._verification_filter.status==="withheld");
  const acceptedCorrect=accepted.filter(d=>(d._verification_filter.gtOk??d.gt?.ok)===true).length;
  const fallbackSummary=policy.fallback_shadow?.summary;
  const fallbackSummaryText=policy.fallback_shadow?.enabled&&fallbackSummary?` · 대체 Tune ${fallbackSummary.tune.replacement_correct}/${fallbackSummary.tune.replacement_count}, Holdout ${fallbackSummary.holdout.replacement_correct}/${fallbackSummary.holdout.replacement_count} · Holdout coverage ${fmt(fallbackSummary.holdout.coverage*100,1)}%`:"";
  $("#filter-summary").textContent=policy.enabled?`SHADOW 보류 예정 ${withheld.length}/${holdoutEvaluated.length} · 수용 예정 정확도 ${acceptedCorrect}/${accepted.length} (${accepted.length?fmt(acceptedCorrect/accepted.length*100,1):"—"}%)${fallbackSummaryText}`:`검증 필터 OFF (${policy.reason||"unavailable"})`;
  refreshVisibleFrames();

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
      const decision=slot.detection?._verification_filter;
      if(decision?.status==="withheld")node.classList.add("withheld");
      if(decision?.status==="accepted")node.classList.add("accepted");
      const st=node.querySelector(".state");st.textContent=(decision?.status==="withheld"?"보류 예정 · ":decision?.status==="accepted"?"수용 예정 · ":"")+(stateText[slot.state]||slot.state);st.classList.add(slot.state,decision?.status||"unavailable");
      node.querySelector(".score").textContent=slot.detection?`score ${fmt(slot.detection.score)} · depth ${fmt(decision?.depth)} · tex ${fmt(decision?.texture)} · IoU ${fmt(decision?.iou)}`:(slot.rejection?.input_rejection||"");
      if(slot.object===selectedObject)node.classList.add("selected");
      node.disabled=!slot.detection;node.onclick=()=>{selectedObject=slot.object;selectedCandidateRank=0;render();};grid.appendChild(node);
    });
  }
  function evidenceHtml(candidate) {
    const ev=candidate?.evidence||{mode:"uncollected"};
    if(ev.mode==="uncollected")return `<p class="warning">이 실행에서는 점 단위 기하/텍스처 근거가 수집되지 않았습니다.</p>`;
    const unavailable=ev.projection_status==="unavailable"?`<p class="projection-unavailable">투영 불가: ${esc(ev.projection_reason||"reason unavailable")}</p>`:"";
    const cards=[];
    if(ev.pose)cards.push(`<div class="evidence-card"><img src="${ev.pose}"><span>후보 CAD 투영</span></div>`);
    if(ev.geometry)cards.push(`<div class="evidence-card"><img src="${ev.geometry}"><span>표본 기하 NN 거리 · PEM crop 좌표</span></div>`);
    if(ev.texture)cards.push(`<div class="evidence-card"><img src="${ev.texture}"><span>표본 PEM 특징 cosine · PEM crop 좌표</span></div>`);
    if(ev.shape_input)cards.push(`<div class="evidence-card"><img src="${ev.shape_input}"><span>입력 depth/radius mask</span></div>`);
    if(ev.shape_render)cards.push(`<div class="evidence-card"><img src="${ev.shape_render}"><span>후보 CAD point-splat mask</span></div>`);
    if(ev.shape_overlap)cards.push(`<div class="evidence-card"><img src="${ev.shape_overlap}"><span>교집합(초록) · 입력(파랑) · 렌더(주황)</span></div>`);
    const g=ev.metrics?.geometry,t=ev.metrics?.texture;
    const overlap = g?.overlap_ratio == null ? "—" : `${fmt(g.overlap_ratio*100,1)}%`;
    const provenance = ev.mode==="sampled_pointwise" ? `<p class="provenance">측정값: exact sampled PEM values · 겹침: NN threshold is 5% CAD radius clipped to 2–20 mm · heatmap: PEM 224×224 crop coordinates · 자세 영상: CAD projection proxy</p>` : "";
    const s=ev.shape_metrics||candidate?.shape;
    const shapeMetrics=s?`<div class="metric-grid"><div class="metric">입력/렌더 면적비 ${fmt(s.size_ratio)}</div><div class="metric">2D point-mask IoU ${fmt(s.mask_iou)}</div><div class="metric">입력 coverage ${fmt(s.coverage)}</div><div class="metric">크기 검증 ${s.size_verified?"PASS":"FAIL"}</div></div><p class="provenance">입력 mask는 detection 내 후보가 공유; CAD는 point-splat 시각화 proxy</p>`:"";
    const shapeUnavailable=ev.shape_unavailable_reason?`<p class="projection-unavailable">rendered mask 불가: ${esc(ev.shape_unavailable_reason)}</p>`:"";
    return `${unavailable}<div class="evidence-grid">${cards.join("")}</div>${g?`<div class="metric-grid"><div class="metric">3D NN 겹침 ${g.overlap_count}/${g.point_count} (${overlap})</div><div class="metric">표본 NN 평균 ${fmt(g.mean_distance_mm,1)} mm</div><div class="metric">표본 texture ${fmt(t?.feature_mean)}</div><div class="metric">표본 color ${fmt(t?.color_mean)}</div></div>${provenance}`:"<p class='warning'>3D 점 근거는 미수집이며 자세/shape 이미지는 별도 proxy입니다.</p>"}${shapeMetrics}${shapeUnavailable}`;
  }
  function renderDetails(slot) {
    if(!slot?.detection){details.className="details empty";details.innerHTML="<p>검출된 객체 카드를 선택하면 후보를 자세히 볼 수 있습니다.</p>";return;}
    details.className="details";const d=slot.detection,cands=d.candidates||[],cand=cands.find(c=>Number(c.rank_geo)===selectedCandidateRank)||cands[0],v=d.verify||{},decision=d._verification_filter||{};
    if(cand)selectedCandidateRank=Number(cand.rank_geo);
    const fallback=decision.fallbackShadow;
    const fallbackRank=fallback?.selected?.rank_geo;
    const visibleCandidates=cands.slice(0,topN);
    const fallbackCandidate=cands.find(c=>Number(c.rank_geo)===fallbackRank);
    if(fallbackCandidate&&!visibleCandidates.includes(fallbackCandidate))visibleCandidates.push(fallbackCandidate);
    const rows=visibleCandidates.map(c=>`<tr class="${Number(c.rank_geo)===selectedCandidateRank?"selected ":""}${Number(c.rank_geo)===fallbackRank?"shadow-fallback":""}" data-rank="${c.rank_geo}"><td>${c.geometry_selected?"✓":""}</td><td>#${c.rank_geo}</td><td>${fmt(c.geo)}</td><td>#${c.rank_texture??"—"}</td><td>${fmt(c.texture_score)}</td><td>${fmt(c.color_score)}</td><td>${fmt(c.shape?.size_ratio)}</td><td>${fmt(c.shape?.mask_iou)}</td><td>${fmt(c.shape?.coverage)}</td><td>${selectedOx(c.gt)}</td></tr>`).join("");
    const poseWarning=slot.state==="pem_pose_invalid"?`<p class="pose-invalid-warning">최종 PEM pose를 투영할 수 없습니다 (${esc(d.pose_status?.reason||"invalid")}). 검출 정보와 후보 상세은 그대로 확인할 수 있습니다.</p>`:"";
    const selectedResult=slot.state==="pem_pose_invalid"?`<span class="ox missing">최종 pose 무효</span>`:selectedOx(d.gt);
    const gtInfo=data.ground_truth.objects?.[slot.object]||{};
    const gtQuality=gtInfo.quality_gate||{};
    const referenceBadge=`<p class="reference-role ${esc(d.reference_role||"evaluation_frame")}">${esc(referenceText[d.reference_role]||"참조 역할 미상")} · GT ${gtInfo.trusted?"trusted":"untrusted"}${gtInfo.unavailable_reason?` (${esc(gtInfo.unavailable_reason)})`:""} · 품질 ${gtQuality.selected_count??"—"}개 / 군집 ${gtInfo.cluster_size??"—"}개 (${gtInfo.cluster_share==null?"—":fmt(gtInfo.cluster_share*100,1)+"%"})</p>`;
    const filterStatus={withheld:"보류 예정",accepted:"수용 예정",low_depth:"범위 밖(depth)",unavailable:"판정 불가"}[decision.status]||decision.status||"판정 불가";
    const filterClass=decision.status==="withheld"?"withheld":decision.status==="accepted"?"accepted":"unavailable";
    const filterPanel=policy.enabled?`<div class="filter-verdict ${filterClass}"><div><b>SHADOW 검증: ${esc(filterStatus)}</b><small>기하 Top-1 pose는 변경하지 않음</small></div><div class="filter-metrics"><span>depth ${fmt(decision.depth,6)} / ≥ ${fmt(policy.depth_valid_frac_min,6)}</span><span class="${decision.textureFailed?"failed":""}">texture ${fmt(decision.texture,6)} / ≥ ${fmt(policy.texture_score_min,6)}</span><span class="${decision.iouFailed?"failed":""}">Mask IoU ${fmt(decision.iou,6)} / ≥ ${fmt(policy.mask_iou_min,6)}</span><span>${esc(decision.split||"—")} · ${decision.evaluationEligible?"GT 평가 대상":"정확도 집계 제외"}</span></div></div>`:"<p class='warning'>검증 보류 정책이 비활성화되어 있습니다.</p>";
    const fallbackThresholds=policy.fallback_shadow?.selected_thresholds;
    const fallbackReason={top1_pose_unavailable:"Top-1 pose 불가",candidate_pose_unavailable:"군집 밖 후보 pose 불가",no_candidate_pose_outside_cluster:"유사 pose 군집 밖 후보 없음",no_candidate_passes_texture_and_iou:"군집 밖에서 texture와 IoU를 모두 통과한 후보 없음"}[fallback?.reason]||fallback?.reason||"대체 후보 없음";
    const fallbackPanel=decision.status==="withheld"&&policy.fallback_shadow?.enabled?`<div class="shadow-panel"><div class="shadow-step withheld-top1"><b>보류 Top-1</b><span>texture ${fmt(decision.texture,6)} · IoU ${fmt(decision.iou,6)} · GT ${decision.gtOk===true?"O":decision.gtOk===false?"X":"—"}</span></div><div class="shadow-arrow">→</div><div class="shadow-step"><b>제외된 유사 후보</b><span>${fallback?.excluded_similar_candidate_count??"—"}개 (Top-1 포함 군집 ${fallback?.excluded_cluster_size??"—"}) · ≤ ${fmt(fallbackThresholds?.rotation_threshold_deg,0)}° / ≤ ${fmt(fallbackThresholds?.translation_threshold_mm,0)}mm</span></div><div class="shadow-arrow">→</div><div class="shadow-step ${fallback?.selected?"fallback-selected":"fallback-none"}"><b>Shadow 대체 후보</b>${fallback?.selected?`<span>기하 #${fallback.selected.rank_geo} · proposal ${fallback.selected.proposal6000_index??"—"}</span><span>texture ${fmt(fallback.selected.texture_score,6)} · IoU ${fmt(fallback.selected.mask_iou,6)} · GT ${fallback.selected.gtOk===true?"O":fallback.selected.gtOk===false?"X":"—"}</span><button id="go-fallback" data-rank="${fallback.selected.rank_geo}">대체 후보 행으로 이동</button>`:`<span>${esc(fallbackReason)}</span>`}</div></div>`:"";
    details.innerHTML=`<div class="detail-head"><div><p class="eyebrow">selected object</p><h2>${esc(slot.object)}</h2></div>${selectedResult}</div>
      ${filterPanel}${fallbackPanel}${referenceBadge}${poseWarning}<div class="section-title">기하-only 초기 포즈 → 최종 PEM pose</div><div class="matrix">R ${JSON.stringify(d.R)}\nt_mm ${JSON.stringify(d.t_mm)}</div>
      <div class="verification-grid"><div class="verification-card"><b>기하 선택</b>${verifyBadge(v.selection_method==="geometry_only"?"pass":"unavailable")}<small>300개 중 기하 1위 · proposal ${v.selected_proposal6000_index??"—"}</small></div><div class="verification-card"><b>텍스처 검증</b>${verifyBadge(v.texture?.status)}<small>score ${fmt(v.texture?.score)} · 텍스처 순위 #${v.texture?.rank??"—"}/${v.texture?.candidate_count??"—"}</small></div><div class="verification-card"><b>투영 크기</b>${verifyBadge(v.shape?.status)}<small>입력/렌더 ${fmt(v.shape?.size_ratio)} · 기준 ≤ ${fmt(v.shape?.size_ratio_max)}</small></div><div class="verification-card"><b>2D IoU</b>${verifyBadge(v.shape?.iou_status)}<small>IoU ${fmt(v.shape?.mask_iou)} · coverage ${fmt(v.shape?.coverage)}</small></div></div>
      <div class="stage-row"><div class="stage-box"><b>6,000 후보</b><br>${ox(d.stage6000)}<br><small>${d.stage6000.match_count??"—"} matches</small></div><div class="stage-box"><b>상위 300</b><br>${ox(d.stage300)}<br><small>${d.stage300.match_count??"—"} matches</small></div></div>
      <div class="section-title">후보 점수</div><div class="candidate-controls"><label>상위 N <select id="topn">${[5,10,20,50,100].map(n=>`<option ${n===topN?"selected":""}>${n}</option>`).join("")}</select></label><span>${cands.length} candidates</span></div>
      ${cands.length?`<table class="candidate-table"><thead><tr><th>선택</th><th>기하순위</th><th>geo</th><th>텍스처순위</th><th>texture</th><th>color</th><th>크기비</th><th>IoU</th><th>coverage</th><th>정답</th></tr></thead><tbody>${rows}</tbody></table>`:"<p class='warning'>후보 진단 미수집</p>"}
      <div class="section-title">기하 후보 #${cand?.rank_geo??"—"} 근거</div>${evidenceHtml(cand)}
      ${cand?`<div class="matrix">proposal6000 ${cand.proposal6000_index??"legacy/untracked"}\nR ${JSON.stringify(cand.R)}\nt_mm ${JSON.stringify(cand.t_mm)}</div>`:""}`;
    const select=$("#topn");if(select)select.onchange=e=>{topN=Number(e.target.value);if(!cands.slice(0,topN).some(c=>Number(c.rank_geo)===selectedCandidateRank))selectedCandidateRank=Number(cands[0]?.rank_geo||0);renderDetails(slot);};
    details.querySelectorAll("tbody tr").forEach(tr=>tr.onclick=()=>{selectedCandidateRank=Number(tr.dataset.rank);renderDetails(slot);});
    const goFallback=$("#go-fallback");if(goFallback)goFallback.onclick=()=>{const rank=Number(goFallback.dataset.rank);selectedCandidateRank=rank;topN=[5,10,20,50,100].find(n=>cands.slice(0,n).some(c=>Number(c.rank_geo)===rank))||100;renderDetails(slot);};
  }
  function render(){
    if(!visibleFrames.length){slider.value=0;slider.max=0;slider.disabled=true;$("#frame-label").textContent="조건에 맞는 결과 없음";image.removeAttribute("src");canvas.getContext("2d").clearRect(0,0,canvas.width,canvas.height);grid.innerHTML="<p class='warning'>선택한 검증 조건에 맞는 객체가 없습니다.</p>";details.className="details empty";details.innerHTML="<p>다른 검증 보기를 선택하세요.</p>";return;}
    slider.disabled=false;
    const frame=data.frames[index],position=Math.max(0,visibleFrames.indexOf(index));slider.value=position;search.value=index;$("#frame-label").textContent=`#${index} · 목록 ${position+1}/${visibleFrames.length} · source ${frame.source_index??"—"} · ${frame.stamp_ns}`;
    image.onload=()=>drawAxes(frame);image.src=frame.image;renderCards(frame);
    let slot=frame.slots.find(s=>s.object===selectedObject);
    if(!slot?.detection||(viewMode!=="all"&&!matchesSlot(slot,viewMode))){slot=viewMode==="all"?frame.slots.find(s=>s.detection):frame.slots.find(s=>matchesSlot(s,viewMode));selectedObject=slot?.object||null;selectedCandidateRank=0;}renderDetails(slot);
  }
  slider.oninput=e=>{if(!visibleFrames.length)return;index=visibleFrames[Number(e.target.value)]??index;selectedCandidateRank=0;render();};
  search.onchange=e=>{index=Math.max(0,Math.min(data.frames.length-1,Number(e.target.value)||0));if(!visibleFrames.includes(index)){viewMode="all";refreshVisibleFrames();}selectedCandidateRank=0;render();};
  viewFilter.onchange=e=>{viewMode=viewModes.has(e.target.value)?e.target.value:"all";refreshVisibleFrames();selectedObject=null;selectedCandidateRank=0;render();};
  $("#prev").onclick=()=>{if(!visibleFrames.length)return;const p=Math.max(0,visibleFrames.indexOf(index)-1);index=visibleFrames[p];selectedCandidateRank=0;render();};
  $("#next").onclick=()=>{if(!visibleFrames.length)return;const p=Math.min(visibleFrames.length-1,visibleFrames.indexOf(index)+1);index=visibleFrames[p];selectedCandidateRank=0;render();};window.onresize=()=>{if(visibleFrames.length)drawAxes(data.frames[index]);};render();
})();
