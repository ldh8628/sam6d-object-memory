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

  function openSlotStatus(slot) {
    epoch += 1;
    analysisEpoch += 1;
    clearPose();
    drawAcceptedAxes(report.frames[index]);
    $("#details").className = "details";
    $("#details").innerHTML = `
      <div class="detail-head"><div><p class="eyebrow">selected object</p>
        <h2>${esc(slot.object)}</h2></div><b>${esc(labels[slot.status] || slot.status)}</b>
      </div>
      <p class="policy"><b>저장된 PEM 시도 없음</b><br>
        <small>${slot.status === "unprocessed_realtime_drop"
          ? "latest-frame 추론에서 이 bag frame을 처리하지 않았습니다. 검출 없음과 구분됩니다."
          : "이 처리 frame에는 해당 객체의 PEM 후보 또는 생산 pose가 없습니다."}</small></p>`;
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

  function renderFrame() {
    epoch += 1;
    analysisEpoch += 1;
    const frame = report.frames[index];
    $("#frame-slider").value = index;
    $("#frame-search").value = index;
    $("#frame-label").textContent = (
      `#${index} · source ${frame.source_index} · ${frame.stamp_ns} · ` +
      `${labels[frame.status] || frame.status}`
    );
    $("#frame-image").src = frameUrl(frame.stamp_ns);
    clearPose();
    drawAcceptedAxes(frame);
    renderCards(frame);
    const slot = (frame.slots.find(item => (
      item.object === selectedObject && item.attempt_id != null
    )) || frame.slots.find(item => item.attempt_id != null));
    if (slot) {
      selectedObject = slot.object;
      openAttempt(slot);
    } else {
      $("#details").className = "details empty";
      $("#details").innerHTML = frame.processed
        ? "<p>처리 완료 · 검출 또는 PEM 시도 없음</p>"
        : "<p>latest-frame 추론에서 처리되지 않은 bag frame</p>";
    }
  }

  function stage(entry) {
    if (!entry) return "미측정";
    if (entry.status !== "available") return esc(entry.status || "미측정");
    return `${entry.present ? "O" : "X"} · ${entry.match_count ?? 0} matches`;
  }

  async function openAttempt(slot) {
    const myEpoch = ++epoch;
    try {
      const data = await json(
        `/api/run/${encodeURIComponent(run)}/candidates?attempt=${slot.attempt_id}`,
      );
      if (myEpoch !== epoch) return;
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
      if (hasCandidates) analyze(slot.attempt_id, selectedCandidate);
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

  async function start() {
    if (!run) {
      $("#notice").textContent = "run query가 없습니다.";
      return;
    }
    try {
      report = await json(`/api/run/${encodeURIComponent(run)}/report`);
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
        index = Number(event.target.value); selectedCandidate = null; renderFrame();
      };
      search.onchange = event => {
        index = Math.max(0, Math.min(report.frames.length - 1, Number(event.target.value) || 0));
        selectedCandidate = null; renderFrame();
      };
      $("#prev").onclick = () => {
        index = Math.max(0, index - 1); selectedCandidate = null; renderFrame();
      };
      $("#next").onclick = () => {
        index = Math.min(report.frames.length - 1, index + 1); selectedCandidate = null; renderFrame();
      };
      renderFrame();
    } catch (error) {
      $("#notice").textContent = error.message;
    }
  }
  start();
})();
