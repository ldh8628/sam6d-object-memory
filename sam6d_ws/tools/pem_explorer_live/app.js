(() => {
  "use strict";
  const $ = selector => document.querySelector(selector);
  const query = new URLSearchParams(location.search);
  const run = query.get("run");
  const worker = new Worker("worker.js");
  const labels = {
    unprocessed_realtime_drop: "실시간 드롭·미처리",
    unprocessed_capture_gap: "수집 누락",
    not_detected: "검출 없음",
    pem_passed: "PEM 통과",
    pem_rejected: "PEM 탈락",
    processed_no_detection: "처리 완료·검출 없음",
  };
  let report;
  let index = 0;
  let selectedObject = null;
  let selectedCandidate = null;
  let epoch = 0;
  let analysisEpoch = 0;
  let previewVideo = null;

  const esc = value => String(value ?? "").replace(/[&<>"']/g, char => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;",
  })[char]);
  const fmt = (value, digits = 3) => (
    value == null || !Number.isFinite(Number(value)) ? "미측정" : Number(value).toFixed(digits)
  );
  async function json(url) {
    const response = await fetch(url);
    const payload = await response.json();
    if (!response.ok) throw Error(payload.error || response.statusText);
    return payload;
  }
  const frameUrl = stamp => (
    `/api/run/${encodeURIComponent(run)}/frame?stamp_ns=${stamp}`
  );

  async function preparePreview() {
    if (!report.preview_video) {
      $("#rgb-source").textContent = "RGB는 bag에서 메모리 JPEG 인코딩";
      return;
    }
    const response = await fetch(`/api/run/${encodeURIComponent(run)}/preview`);
    if (!response.ok) throw Error(`preview load failed: ${response.statusText}`);
    const video = document.createElement("video");
    video.muted = true;
    video.preload = "auto";
    video.src = URL.createObjectURL(await response.blob());
    await new Promise((resolve, reject) => {
      video.onloadedmetadata = resolve;
      video.onerror = () => reject(Error("preview decode failed"));
    });
    previewVideo = video;
    $("#rgb-source").textContent = "RGB는 All-I H.264 preview";
  }

  async function loadFrameImage(frameIndex) {
    if (previewVideo) {
      const target = (frameIndex + 0.5) / Number(report.preview_video.fps);
      if (Math.abs(previewVideo.currentTime - target) > 1e-6) {
        await new Promise((resolve, reject) => {
          const done = () => {
            previewVideo.removeEventListener("seeked", done);
            previewVideo.removeEventListener("error", failed);
            resolve();
          };
          const failed = () => {
            previewVideo.removeEventListener("seeked", done);
            previewVideo.removeEventListener("error", failed);
            reject(Error("preview seek failed"));
          };
          previewVideo.addEventListener("seeked", done);
          previewVideo.addEventListener("error", failed);
          previewVideo.currentTime = target;
        });
      }
      return previewVideo;
    }
    const image = new Image();
    await new Promise((resolve, reject) => {
      image.onload = resolve;
      image.onerror = () => reject(Error("bag RGB load failed"));
      image.src = frameUrl(report.frames[frameIndex].stamp_ns);
    });
    return image;
  }

  function drawFrameImage(source) {
    const canvas = $("#frame-image");
    const width = source.videoWidth || source.naturalWidth;
    const height = source.videoHeight || source.naturalHeight;
    canvas.width = width; canvas.height = height;
    canvas.getContext("2d").drawImage(source, 0, 0, width, height);
  }

  function clearPose() {
    const canvas = $("#pose-canvas");
    canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height);
  }

  function drawAcceptedAxes(frame) {
    const canvas = $("#pose-canvas");
    if (!frame?.K || !frame?.image_size || !canvas) return;
    const [height, width] = frame.image_size;
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height;
    }
    const K = frame.K;
    const ctx = canvas.getContext("2d");
    const project = point => {
      if (!point.every(Number.isFinite) || point[2] <= 1e-6) return null;
      return [K[0] * point[0] / point[2] + K[2],
              K[4] * point[1] / point[2] + K[5]];
    };
    const colors = ["#ff4b55", "#4ee58a", "#4aa8ff"];
    frame.slots.forEach(slot => {
      const pose = slot.accepted_pose;
      if (!pose?.R || !pose?.t_mm) return;
      const origin = pose.t_mm.map(Number);
      const start = project(origin);
      if (!start) return;
      const length = 50;
      for (let axis = 0; axis < 3; axis += 1) {
        const endpoint = origin.map((value, row) => value + length * Number(pose.R[row][axis]));
        const end = project(endpoint);
        if (!end) continue;
        ctx.beginPath(); ctx.moveTo(...start); ctx.lineTo(...end);
        ctx.strokeStyle = colors[axis]; ctx.lineWidth = 3; ctx.stroke();
      }
      ctx.fillStyle = "#ffffff"; ctx.font = "11px sans-serif";
      ctx.fillText(slot.object, start[0] + 4, start[1] - 4);
    });
  }

  function renderSlotStatus(slot) {
    $("#details").className = "details";
    $("#details").innerHTML = `
      <div class="detail-head"><div><p class="eyebrow">selected object</p>
        <h2>${esc(slot.object)}</h2></div><b>${esc(labels[slot.status] || slot.status)}</b>
      </div>
      <p class="policy"><b>${slot.attempt_id == null ? "저장된 PEM 시도 없음" : "저장된 PEM 결과"}</b><br>
        <small>${slot.attempt_id != null
          ? "객체 카드를 클릭하면 후보 상세를 불러옵니다."
          : slot.status === "unprocessed_realtime_drop"
          ? "latest-frame 추론에서 이 bag frame을 처리하지 않았습니다. 검출 없음과 구분됩니다."
          : "이 처리 frame에는 해당 객체의 PEM 후보 또는 생산 pose가 없습니다."}</small></p>`;
  }

  function openSlotStatus(slot) {
    epoch += 1;
    analysisEpoch += 1;
    renderSlotStatus(slot);
  }

  function renderCards(frame) {
    const grid = $("#object-grid");
    grid.innerHTML = "";
    frame.slots.forEach(slot => {
      const node = $("#slot-template").content.firstElementChild.cloneNode(true);
      node.classList.add(slot.status);
      if (slot.object === selectedObject) node.classList.add("selected");
      node.querySelector(".object-name").textContent = slot.object;
      node.querySelector(".state").textContent = labels[slot.status] || slot.status;
      node.querySelector(".score").textContent = slot.rejection_reason || "";
      node.onclick = () => {
        selectedObject = slot.object;
        selectedCandidate = null;
        renderCards(frame);
        if (slot.attempt_id == null) openSlotStatus(slot);
        else openAttempt(slot);
      };
      grid.appendChild(node);
    });
  }

  async function renderFrame(targetIndex) {
    const myEpoch = ++epoch;
    analysisEpoch += 1;
    const frame = report.frames[targetIndex];
    const slot = (frame.slots.find(item => (
      item.object === selectedObject && item.attempt_id != null
    )) || frame.slots.find(item => item.attempt_id != null));
    try {
      const image = await loadFrameImage(targetIndex);
      if (myEpoch !== epoch) return;
      index = targetIndex;
      if (slot) selectedObject = slot.object;
      drawFrameImage(image);
      $("#frame-slider").value = index;
      $("#frame-search").value = index;
      $("#frame-label").textContent = (
        `#${index} · source ${frame.source_index} · ${frame.stamp_ns} · ` +
        `${labels[frame.status] || frame.status}`
      );
      clearPose();
      drawAcceptedAxes(frame);
      renderCards(frame);
      if (slot) {
        renderSlotStatus(slot);
      } else {
        $("#details").className = "details empty";
        $("#details").innerHTML = frame.processed
          ? "<p>처리 완료 · 검출 또는 PEM 시도 없음</p>"
          : "<p>latest-frame 추론에서 처리되지 않은 bag frame</p>";
      }
    } catch (error) {
      if (myEpoch === epoch) $("#notice").textContent = error.message;
    }
  }

  function stage(entry) {
    if (!entry) return "미측정";
    if (entry.status !== "available") return esc(entry.status || "미측정");
    return `${entry.present ? "O" : "X"} · ${entry.match_count ?? 0} matches`;
  }

  function renderAttempt(slot, data) {
      const hasCandidates = data.candidates.length > 0;
      selectedCandidate = hasCandidates ? (
        selectedCandidate ?? data.candidates.find(
          candidate => candidate.selected_actual)?.index300 ??
          data.candidates.find(candidate => candidate.geometry_top1)?.index300 ??
          data.candidates[0].index300
      ) : null;
      const attempt = data.attempt;
      const shadow = data.legacy_shadow;
      const actual = data.actual;
      const rows = data.candidates.map(candidate => `
        <tr data-index="${candidate.index300}"
            class="${candidate.index300 === selectedCandidate ? "selected " : ""}${candidate.selected_actual ? "actual " : ""}${candidate.geometry_top1 ? "geo" : ""}">
          <td>${candidate.selected_actual ? "실제 " : ""}${candidate.geometry_top1 ? "Geo#1" : ""}</td>
          <td>#${candidate.rank_geo}</td><td>${fmt(candidate.geometry, 5)}</td>
          <td>${candidate.texture_measured ? `#${candidate.rank_texture}` : "—"}</td>
          <td>${fmt(candidate.texture, 5)}</td><td>${fmt(candidate.mask_iou)}</td>
          <td>${fmt(candidate.coverage)}</td>
          <td>${candidate.gt?.status === "available" ? (candidate.gt.correct ? "O" : "X") : "—"}</td>
        </tr>`).join("") || `<tr><td colspan="8">PEM 입력 단계에서 탈락해 저장 후보가 없습니다.</td></tr>`;
      const fine = actual.final_pose || {};
      const withheld = shadow?.status === "withheld";
      $("#details").className = "details";
      $("#details").innerHTML = `
        <div class="detail-head"><div><p class="eyebrow">selected object</p>
          <h2>${esc(attempt.object)}</h2></div>
          <b class="${attempt.accepted ? "pass" : "fail"}">${attempt.accepted ? "PEM PASS" : "PEM REJECT"}</b>
        </div>
        <div class="summary-grid"><div class="summary-card"><b>실제 생산 선택</b>
          <small>index ${attempt.selected_index300 ?? "없음"} · survivors ${attempt.texture_survivors}/${attempt.mask_survivors} · cluster ${attempt.cluster_size} (${fmt(attempt.cluster_occupancy)})</small>
        </div><div class="summary-card"><b>Geometry Top-1</b>
          <small>index ${attempt.geometry_top1_index300 ?? data.candidates.find(item => item.geometry_top1)?.index300 ?? "—"} · 실제와 독립 비교</small>
        </div></div>
        <p class="policy ${withheld ? "withheld" : ""}"><b>Legacy shadow: ${esc(shadow?.status || "미측정")}</b><br>
          <small>depth ≥0.8 · texture &lt;0.449562 AND IoU &lt;0.420998 · 유사 pose ≤20°/25mm · fallback ${shadow?.fallback ? `geo #${shadow.fallback.rank_geo}` : "없음"} · ${esc(data.reference_role)}</small></p>
        <div class="stage-row"><div class="stage-box"><b>6,000 후보</b><small>${stage(attempt.stage_summary?.stage6000)}</small></div>
          <div class="stage-box"><b>상위 300</b><small>${stage(attempt.stage_summary?.stage300)}</small></div></div>
        <div class="section-title">실제 fine pose</div>
        <div class="matrix">valid ${fine.valid ?? "legacy/untracked"}\nR ${JSON.stringify(fine.R ?? null)}\nt_mm ${JSON.stringify(fine.t_mm ?? null)}\nreason ${attempt.rejection_reason ?? "none"}</div>
        <div class="section-title">${data.candidates.length} 후보</div>
        <table class="candidate-table"><thead><tr><th>역할</th><th>기하</th><th>geo score</th><th>tex rank</th><th>texture</th><th>IoU</th><th>coverage</th><th>pseudo-GT</th></tr></thead><tbody>${rows}</tbody></table>
        ${hasCandidates ? `<div class="section-title">선택 후보 근거</div><div class="evidence-grid">
          <div class="evidence-card"><canvas id="geometry"></canvas><span>196개 geometry FPS 거리 heatmap</span></div>
          <div class="evidence-card"><canvas id="texture"></canvas><span>2,048개 texture cosine heatmap</span></div>
          <div class="evidence-card"><canvas id="mask"></canvas><span>관측 Mask / 8,192 CAD 투영 / overlap</span></div>
          <div class="evidence-card"><pre id="scores">후보를 클릭하면 RAM에서 재계산합니다.</pre><span>저장값·재계산값·measurement provenance</span></div>
        </div>` : `<p class="warning">${esc(attempt.rejection_reason || "PEM input rejected")} · 후보/texture 근거 미측정</p>`}`;
      document.querySelectorAll("tbody tr").forEach(row => {
        row.onclick = () => analyze(slot.attempt_id, Number(row.dataset.index));
      });
  }

  async function openAttempt(slot) {
    const myEpoch = ++epoch;
    try {
      const data = await json(
        `/api/run/${encodeURIComponent(run)}/candidates?attempt=${slot.attempt_id}`,
      );
      if (myEpoch !== epoch) return;
      renderAttempt(slot, data);
    } catch (error) {
      if (myEpoch === epoch) {
        $("#details").className = "details";
        $("#details").innerHTML = `<p class="warning">${esc(error.message)}</p>`;
      }
    }
  }

  async function analyze(attempt, candidate) {
    const myAnalysis = ++analysisEpoch;
    selectedCandidate = candidate;
    document.querySelectorAll("tbody tr").forEach(row => {
      row.classList.toggle("selected", Number(row.dataset.index) === candidate);
    });
    const myEpoch = epoch;
    const scores = $("#scores");
    if (scores) scores.textContent = "계산 중…";
    try {
      const data = await json(
        `/api/run/${encodeURIComponent(run)}/analysis?attempt=${attempt}&candidate=${candidate}`,
      );
      if (myEpoch !== epoch || myAnalysis !== analysisEpoch) return;
      if (scores) scores.textContent = JSON.stringify({
        stored: data.stored,
        recomputed: data.recomputed,
        absolute_error: data.absolute_error,
        texture_measured_at_capture: data.texture_measured_at_capture,
        provenance_mismatch: data.provenance_mismatch,
        provenance_mismatch_assets: data.provenance_mismatch_assets,
      }, null, 2);
      worker.postMessage({ ...data, ui_epoch: myEpoch, ui_request: myAnalysis });
    } catch (error) {
      if (myEpoch === epoch && scores) {
        scores.textContent = `동적 분석 비활성/실패: ${error.message}`;
      }
    }
  }

  worker.onmessage = event => {
    const { ui_epoch, ui_request, ...layers } = event.data;
    if (ui_epoch !== epoch || ui_request !== analysisEpoch) return;
    Object.entries(layers).forEach(([id, pixels]) => {
      const canvas = $(`#${id}`);
      if (!canvas) return;
      canvas.width = pixels.w;
      canvas.height = pixels.h;
      canvas.getContext("2d").putImageData(
        new ImageData(pixels.rgba, pixels.w, pixels.h), 0, 0,
      );
    });
    const pixels = event.data.pose;
    const canvas = $("#pose-canvas");
    if (pixels && canvas) {
      canvas.width = pixels.w;
      canvas.height = pixels.h;
      canvas.getContext("2d").putImageData(
        new ImageData(pixels.rgba, pixels.w, pixels.h), 0, 0,
      );
      drawAcceptedAxes(report.frames[index]);
    }
  };

  function liveDepthIndex(stamp) {
    const rows = report.depth_frames;
    if (!rows.length) return -1;
    const target = BigInt(stamp);
    let lo = 0; let hi = rows.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (BigInt(rows[mid].stamp_ns) <= target) lo = mid + 1;
      else hi = mid;
    }
    const after = Math.min(rows.length - 1, lo);
    const before = Math.max(0, lo - 1);
    return target - BigInt(rows[before].stamp_ns) <= BigInt(rows[after].stamp_ns) - target
      ? before : after;
  }

  function renderLiveTimeline(current) {
    const canvas = $("#live-timeline");
    const width = Math.min(1200, Math.max(1, report.frames.length));
    canvas.width = width; canvas.height = 28;
    const ctx = canvas.getContext("2d");
    const colors = {pem_passed: "#36c98d", pem_rejected: "#e55768",
      processed_no_detection: "#68839a", unprocessed_realtime_drop: "#24394b"};
    for (let x = 0; x < width; x += 1) {
      const frame = report.frames[Math.min(
        report.frames.length - 1, Math.floor(x * report.frames.length / width),
      )];
      ctx.fillStyle = colors[frame.status] || "#24394b";
      ctx.fillRect(x, 0, 1, canvas.height);
    }
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(Math.min(width - 1, Math.floor(current * width / report.frames.length)), 0, 2, 28);
  }

  function renderLiveCards(frame) {
    const grid = $("#object-grid");
    const raw = [
      ...frame.detections.map(item => ({...item, status: "pem_passed"})),
      ...frame.rejections.map(item => ({...item, status: "pem_rejected"})),
      ...frame.pem_candidates.filter(item => (
        item.rejection_reason || item.input_rejection
      )).map(item => ({...item, status: "pem_rejected"})),
    ];
    const rows = [...new Map(raw.map(item => [
      `${item.object}:${item.status}:${item.rejection_reason || item.input_rejection || ""}`, item,
    ])).values()];
    grid.innerHTML = rows.map(item => `<div class="object-card ${item.status}">
      <span class="object-name">${esc(item.object)}</span>
      <span class="state">${esc(labels[item.status])}</span>
      <span class="score">${esc(item.rejection_reason || item.input_rejection || `pose ${fmt(item.score)}`)}</span>
    </div>`).join("");
  }

  function renderLiveDetails(frame, heldPoses) {
    const current = frame.detections[0];
    const rejection = frame.rejections[0] || frame.pem_candidates.find(
      item => item.rejection_reason || item.input_rejection,
    );
    const evidence = current || rejection || {};
    const fine = evidence.verify?.fine || {};
    const heldText = heldPoses.length ? heldPoses.map(pose => {
      const held = Number(BigInt(frame.stamp_ns) - BigInt(pose.stamp_ns)) / 1e9;
      return `${esc(pose.object)} · score ${fmt(pose.score)} · held ${held.toFixed(2)} s`;
    }).join("<br>") : "아직 없음";
    const depthIndex = liveDepthIndex(frame.stamp_ns);
    $("#details").className = "details";
    $("#details").innerHTML = `<div class="detail-head"><div><p class="eyebrow">live replay</p>
      <h2>${esc(labels[frame.status] || frame.status)}</h2></div>
      <b class="${report.completed ? "pass" : "fail"}">${report.completed ? "COMPLETE" : "INCOMPLETE"}</b></div>
      <div class="summary-grid"><div class="summary-card"><b>마지막 accepted pose</b>
        <small>${heldText}</small>
      </div><div class="summary-card"><b>현재 프레임</b><small>source ${frame.source_seq ?? "—"} · ${esc(frame.stamp_ns)}</small></div></div>
      <div class="section-title">현재 프레임 검증 수치</div>
      <div class="matrix">pose score ${fmt(evidence.pem?.pose_score ?? evidence.score)}\nMask IoU ${fmt(evidence.mask_iou ?? fine.mask_iou)}\nTexture ${fmt(evidence.texture_score ?? fine.texture_score)}\ncluster occupancy ${fmt(evidence.cluster_occupancy)}\nrejection ${rejection?.rejection_reason ?? rejection?.input_rejection ?? "none"}</div>
      <div class="section-title">저장 Depth</div>
      ${depthIndex < 0 ? "<p>저장된 Depth가 없습니다.</p>" : `<button id="show-depth">가장 가까운 Depth #${depthIndex} 보기</button><p><img id="depth-image" class="depth-preview" alt="선택 Depth 컬러맵"></p>`}`;
    const button = $("#show-depth");
    if (button) button.onclick = () => {
      $("#depth-image").src = `/api/run/${encodeURIComponent(run)}/depth?frame=${depthIndex}`;
    };
  }

  function startLiveReplay() {
    const frames = report.frames;
    if (!frames.length) throw Error("recorded RGB frames are unavailable");
    const fps = Number(report.manifest.recording?.fps || 30);
    const byObject = new Map();
    report.detections.forEach(pose => {
      if (!byObject.has(pose.object)) byObject.set(pose.object, []);
      byObject.get(pose.object).push(pose);
    });
    byObject.forEach(events => events.sort((a, b) => Number(
      BigInt(a.stamp_ns) - BigInt(b.stamp_ns),
    )));
    const before = (pose, frame) => (
      pose.source_seq != null && frame.source_seq != null
        ? Number(pose.source_seq) <= Number(frame.source_seq)
        : BigInt(pose.stamp_ns) <= BigInt(frame.stamp_ns)
    );
    const heldAt = frame => [...byObject.values()].flatMap(events => {
      let lo = 0; let hi = events.length;
      while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (before(events[mid], frame)) lo = mid + 1;
        else hi = mid;
      }
      return lo ? [events[lo - 1]] : [];
    });
    const video = $("#live-video");
    $("#frame-image").hidden = true;
    video.hidden = false;
    video.src = `/api/run/${encodeURIComponent(run)}/rgb`;
    $("#live-timeline").hidden = false;
    $("#rgb-source").textContent = "30 Hz H.264/NVENC 원본 RGB";
    $("#title").textContent = `${run} · Live RGB-D Replay`;
    $("#dataset-source").textContent = report.dataset.camera.frame_id || "RealSense color";
    $("#frame-count").textContent = `${frames.length.toLocaleString()} RGB · 처리 ${report.dataset.processed_frame_count.toLocaleString()}`;
    $("#profile-badge").textContent = `${report.capture_profile} depth`;
    $("#notice").textContent = report.completed
      ? "accepted/rejected/unprocessed는 저장 timestamp로 결합됩니다. pose held 시간을 함께 확인하세요."
      : "불완전 실행입니다. 종료 전까지 저장된 프레임만 표시합니다.";
    const slider = $("#frame-slider"); const search = $("#frame-search");
    slider.max = search.max = frames.length - 1;
    const show = frameIndex => {
      index = Math.max(0, Math.min(frames.length - 1, frameIndex));
      const frame = frames[index];
      const heldPoses = heldAt(frame);
      slider.value = search.value = index;
      $("#frame-label").textContent = `#${index} · ${frame.stamp_ns} · ${labels[frame.status]}`;
      clearPose();
      if (heldPoses.length) drawAcceptedAxes({...frame, slots: heldPoses.map(
        accepted_pose => ({object: accepted_pose.object, accepted_pose}),
      )});
      renderLiveTimeline(index); renderLiveCards(frame); renderLiveDetails(frame, heldPoses);
    };
    const seek = frameIndex => { video.currentTime = (frameIndex + 0.5) / fps; show(frameIndex); };
    video.ontimeupdate = () => show(Math.floor(video.currentTime * fps));
    video.onseeked = () => show(Math.floor(video.currentTime * fps));
    slider.oninput = event => seek(Number(event.target.value));
    search.onchange = event => seek(Number(event.target.value) || 0);
    $("#prev").onclick = () => seek(index - 1);
    $("#next").onclick = () => seek(index + 1);
    $("#live-timeline").onclick = event => seek(Math.floor(
      event.offsetX * frames.length / event.currentTarget.clientWidth,
    ));
    show(0);
  }

  async function start() {
    if (!run) {
      $("#notice").textContent = "run query가 없습니다.";
      return;
    }
    try {
      report = await json(`/api/run/${encodeURIComponent(run)}/report`);
      if (report.kind === "live_replay") {
        startLiveReplay();
        return;
      }
      await preparePreview();
      const requested = Number(query.get("frame"));
      if (Number.isInteger(requested)) {
        index = Math.max(0, Math.min(report.frames.length - 1, requested));
      }
      $("#title").textContent = `${run} · PEM Pose Explorer`;
      $("#dataset-source").textContent = (
        `${report.dataset.id} · ${report.dataset.camera.role || "camera"}`
      );
      $("#frame-count").textContent = (
        `${report.dataset.bag_frame_count.toLocaleString()} frames · ` +
        `처리 ${report.dataset.processed_frame_count.toLocaleString()}`
      );
      $("#profile-badge").textContent = report.capture_profile;
      $("#notice").textContent = (
        "실제 생산 결과가 주 결과입니다. Legacy shadow와 pseudo-GT는 비교 계측이며 생산 pose를 변경하지 않습니다."
      );
      const slider = $("#frame-slider");
      const search = $("#frame-search");
      slider.max = search.max = report.frames.length - 1;
      slider.oninput = event => {
        selectedCandidate = null; renderFrame(Number(event.target.value));
      };
      search.onchange = event => {
        const target = Math.max(0, Math.min(
          report.frames.length - 1, Number(event.target.value) || 0,
        ));
        selectedCandidate = null; renderFrame(target);
      };
      $("#prev").onclick = () => {
        selectedCandidate = null; renderFrame(Math.max(0, index - 1));
      };
      $("#next").onclick = () => {
        selectedCandidate = null; renderFrame(Math.min(report.frames.length - 1, index + 1));
      };
      renderFrame(index);
    } catch (error) {
      $("#notice").textContent = error.message;
    }
  }
  start();
})();
