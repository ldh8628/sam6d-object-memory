(() => {
  "use strict";
  const storageKey = `pem-score-analysis-json:${location.pathname}`;
  const validAnalysis = value => value?.schema_version === 1 &&
    typeof value.dataset_id === "string" &&
    value.objects && !Array.isArray(value.objects) && Object.keys(value.objects).length > 0 &&
    value.definitions?.units?.stage6000_residual && value.definitions.channels &&
    value.definitions.symmetry_policy && value.overall?.stage_attrition;
  let stored = null;
  try {
    const raw = sessionStorage.getItem(storageKey);
    if (raw) {
      const candidate = JSON.parse(raw);
      if (validAnalysis(candidate)) stored = candidate;
      else sessionStorage.removeItem(storageKey);
    }
  } catch (_error) {
    try { sessionStorage.removeItem(storageKey); } catch (_ignored) {}
  }
  const D = stored || window.PEM_SCORE_ANALYSIS;
  if (!D) {
    document.body.textContent = "analysis-data.js unavailable";
    return;
  }
  const clearAnalysis = document.querySelector("#clear-analysis");
  if (clearAnalysis && stored) {
    clearAnalysis.hidden = false;
    clearAnalysis.textContent = `업로드 데이터 해제 (${D.dataset_id || "unknown"})`;
    clearAnalysis.onclick = () => {
      sessionStorage.removeItem(storageKey);
      location.reload();
    };
  }
  const esc = value => String(value ?? "").replace(
    /[&<>\"]/g, char => ({"&":"&amp;", "<":"&lt;", ">":"&gt;", '"':"&quot;"}[char]));
  const pct = value => value == null ? "—" : `${(100 * value).toFixed(1)}%`;
  const num = value => value == null ? "—" : Number(value).toLocaleString(
    undefined, {maximumFractionDigits: 5});
  const defs = D.definitions;
  const symmetry = Object.entries(defs.symmetry_policy || {})
    .filter(([_name, value]) => value.axis)
    .map(([name, value]) => `${name} axis=[${value.axis.join(",")}] step=${value.step_deg}°`)
    .join("; ") || "none";

  document.querySelector("#notice").textContent =
    `${defs.correct_candidate}. ${defs.split}. ${defs.self_slam_warning}. ` +
    `symmetry: ${symmetry}. 6000 residual unit: ${defs.units.stage6000_residual}. ` +
    `임계값은 ${defs.threshold_activation}.`;
  const attr = D.overall.stage_attrition;
  const stage6000Ceiling = attr.detections ? attr.stage6000_with_correct / attr.detections : null;
  const stage300Ceiling = attr.detections ? attr.stage300_with_correct / attr.detections : null;
  document.querySelector("#summary").innerHTML =
    `<span class="chip">trusted evaluation rows ${num(D.overall.trusted_evaluation_detection_count)}</span>` +
    `<span class="chip">GT available ${num(attr.detections)}</span>` +
    `<span class="chip">GT unavailable ${num(attr.gt_unavailable_detections)}</span>` +
    `<span class="chip ceiling">6000 ceiling ${pct(stage6000Ceiling)}</span>` +
    `<span class="chip ceiling">300 ceiling ${pct(stage300Ceiling)}</span>` +
    `<span class="chip">6000→300 loss ${num(attr.lost_6000_to_300)}</span>`;

  const names = Object.keys(D.objects);
  const requestedObject = new URLSearchParams(location.search).get("object");
  let selected = new Set([
    requestedObject && names.includes(requestedObject) ? requestedObject : names[0]
  ]);
  const nav = document.querySelector("#objects");

  function histogram(dist, kind) {
    if (!dist || !dist.histogram.length) {
      return `<p class="foot">score unavailable · missing ${num(dist?.missing_count || 0)}</p>`;
    }
    const maximum = Math.max(...dist.histogram.map(item => item.count), 1);
    const width = 100 / dist.histogram.length;
    const bars = dist.histogram.map((item, index) => {
      const height = Math.max(2, 96 * item.count / maximum);
      return `<rect class="bar ${kind}" x="${index * width}" y="${100 - height}" ` +
        `width="${Math.max(.3, width - .25)}" height="${height}">` +
        `<title>bin ${num(item.lo)}…${num(item.hi)} · ${kind} ${num(item.count)} · ` +
        `missing ${num(dist.missing_count)}</title></rect>`;
    }).join("");
    return `<svg class="hist" viewBox="0 0 100 100" preserveAspectRatio="none" ` +
      `role="img" aria-label="${esc(kind)} histogram">${bars}</svg>`;
  }

  function pairedHistogram(correct, incorrect) {
    if (!correct?.histogram?.length && incorrect?.histogram?.length) {
      return histogram(incorrect, "incorrect");
    }
    if (!incorrect?.histogram?.length && correct?.histogram?.length) {
      return histogram(correct, "correct");
    }
    if (!correct?.histogram?.length || !incorrect?.histogram?.length) {
      return `<p class="foot">paired histogram unavailable</p>`;
    }
    const maximum = Math.max(...correct.histogram.map(item => item.count),
      ...incorrect.histogram.map(item => item.count), 1);
    const width = 100 / correct.histogram.length;
    const bars = correct.histogram.map((item, index) => {
      const other = incorrect.histogram[index];
      const hc = Math.max(1, 96 * item.count / maximum);
      const hi = Math.max(1, 96 * other.count / maximum);
      const title = `bin ${num(item.lo)}…${num(item.hi)} · correct ${num(item.count)} · ` +
        `incorrect ${num(other.count)} · missing C/I ${num(correct.missing_count)}/${num(incorrect.missing_count)}`;
      return `<g><title>${title}</title>` +
        `<rect class="bar incorrect" x="${index * width}" y="${100 - hi}" ` +
        `width="${Math.max(.2, width / 2 - .12)}" height="${hi}"/>` +
        `<rect class="bar correct" x="${index * width + width / 2}" y="${100 - hc}" ` +
        `width="${Math.max(.2, width / 2 - .12)}" height="${hc}"/></g>`;
    }).join("");
    return `<svg class="hist paired" viewBox="0 0 100 100" preserveAspectRatio="none" ` +
      `role="img" aria-label="correct and incorrect histogram on shared axes">${bars}</svg>`;
  }

  function ecdfPlot(series) {
    const points = series.flatMap(item => item.dist?.ecdf || []);
    if (!points.length) return `<p class="foot">ECDF unavailable</p>`;
    const lo = Math.min(...points.map(item => item.score));
    const hi = Math.max(...points.map(item => item.score));
    const span = hi - lo || 1;
    const paths = series.map(item => {
      const coords = (item.dist?.ecdf || []).map(point =>
        `${(100 * (point.score - lo) / span).toFixed(3)},${(100 - 96 * point.fraction).toFixed(3)}`
      ).join(" ");
      return `<polyline class="ecdf-line ${item.kind}" points="${coords}">` +
        `<title>${esc(item.kind)} ECDF · x ${num(lo)}…${num(hi)}</title></polyline>`;
    }).join("");
    return `<svg class="ecdf" viewBox="0 0 100 100" preserveAspectRatio="none" ` +
      `role="img" aria-label="ECDF on shared score axis">${paths}</svg>` +
      `<p class="axis"><span>${num(lo)}</span><span>shared score axis</span><span>${num(hi)}</span></p>`;
  }

  function channelCard(name, value, trusted) {
    const channel = D.definitions.channels[name];
    if (!trusted) {
      return `<article class="channel"><h3>${esc(channel.label)}</h3>` +
        `${histogram(value.unlabelled, "unlabelled")}` +
        `${ecdfPlot([{dist:value.unlabelled, kind:"unlabelled"}])}` +
        `<p class="legend">라벨 없는 전체 후보 · missing ${num(value.unlabelled.missing_count)}</p></article>`;
    }
    return `<article class="channel"><h3>${esc(channel.label)}</h3>` +
      `${pairedHistogram(value.correct, value.incorrect)}` +
      `${ecdfPlot([{dist:value.incorrect, kind:"incorrect"}, {dist:value.correct, kind:"correct"}])}` +
      `<p class="legend">green correct ${num(value.correct.count)} · ` +
      `red incorrect ${num(value.incorrect.count)} · ` +
      `missing ${num(value.correct.missing_count + value.incorrect.missing_count)} · ` +
      `invalid pose ${num(value.invalid_pose_count)} · ` +
      `GT unavailable ${num(value.gt_unavailable_count)} · overlap ` +
      `${value.overlap_interval ? `${num(value.overlap_interval.lo)}…${num(value.overlap_interval.hi)}` : "none"}</p></article>`;
  }

  function residualCard(value, trusted) {
    if (!trusted) {
      return `<article class="channel"><h3>6000 initial 3-point residual</h3>` +
        `${histogram(value.overall, "unlabelled")}` +
        `${ecdfPlot([{dist:value.overall, kind:"unlabelled"}])}` +
        `<p class="legend">라벨 없는 aggregate · normalized object radius · PEM geometry 아님</p></article>`;
    }
    return `<article class="channel"><h3>6000 initial 3-point residual</h3>` +
      `${pairedHistogram(value.correct, value.incorrect)}` +
      `${ecdfPlot([{dist:value.incorrect, kind:"incorrect"}, {dist:value.correct, kind:"correct"}])}` +
      `<p class="legend">lower is better · normalized object radius · invalid pose ` +
      `${num(value.invalid_pose?.count)} · overflow right-censored · PEM geometry 아님</p></article>`;
  }

  const metric = (value, key) => value?.[key];
  function thresholdRows(panel) {
    return Object.entries(panel.thresholds).map(([channel, threshold]) => {
      const tune = threshold.tune || threshold.tune_sample;
      const holdout = threshold.holdout || threshold.holdout_sample;
      return `<tr><td>${esc(D.definitions.channels[channel].label)}</td>` +
        `<td class="status ${esc(threshold.status)}">${esc(threshold.status)}</td>` +
        `<td>${num(threshold.threshold)}</td>` +
        `<td>${pct(metric(tune, "correct_candidate_retention"))}</td>` +
        `<td>${pct(metric(tune, "incorrect_candidate_rejection"))}</td>` +
        `<td>${pct(metric(tune, "detection_level_recall"))}</td>` +
        `<td>${num(metric(tune, "correct_candidate_count"))}/` +
        `${num(metric(tune, "incorrect_candidate_count"))}/` +
        `${num(metric(tune, "eligible_detection_count"))}</td>` +
        `<td>${num(metric(tune, "surviving_candidates_mean"))}/` +
        `${num(metric(tune, "surviving_candidates_median"))}</td>` +
        `<td>${pct(metric(holdout, "correct_candidate_retention"))}</td>` +
        `<td>${pct(metric(holdout, "incorrect_candidate_rejection"))}</td>` +
        `<td>${pct(metric(holdout, "detection_level_recall"))}</td>` +
        `<td>${num(metric(holdout, "correct_candidate_count"))}/` +
        `${num(metric(holdout, "incorrect_candidate_count"))}/` +
        `${num(metric(holdout, "eligible_detection_count"))}</td>` +
        `<td>${num(metric(holdout, "surviving_candidates_mean"))}/` +
        `${num(metric(holdout, "surviving_candidates_median"))}</td></tr>`;
    }).join("");
  }

  function sweepCard(channel, threshold) {
    if (threshold.status !== "proposed" || !threshold.sweep?.length) {
      return `<article class="sweep"><h4>${esc(D.definitions.channels[channel].label)}</h4>` +
        `<p class="foot">${esc(threshold.reason || "sweep unavailable")}</p></article>`;
    }
    const sweep = threshold.sweep;
    const lo = Math.min(...sweep.map(point => point.threshold));
    const hi = Math.max(...sweep.map(point => point.threshold));
    const span = hi - lo || 1;
    const line = (key, kind) => {
      const valid = sweep.filter(point => point[key] != null);
      const coords = valid.map(point =>
        `${(100 * (point.threshold - lo) / span).toFixed(3)},${(100 - 96 * point[key]).toFixed(3)}`
      ).join(" ");
      const dots = valid.map(point => `<circle class="sweep-dot ${kind}" ` +
        `cx="${(100 * (point.threshold - lo) / span).toFixed(3)}" ` +
        `cy="${(100 - 96 * point[key]).toFixed(3)}" r=".65">` +
        `<title>threshold ${num(point.threshold)} · ${esc(key)} ${pct(point[key])} · ` +
        `correct ${num(point.correct_candidate_count)} · incorrect ${num(point.incorrect_candidate_count)} · ` +
        `missing ${num(point.missing_score_count)}</title></circle>`).join("");
      return `<polyline class="sweep-line ${kind}" points="${coords}"/>${dots}`;
    };
    return `<article class="sweep"><h4>${esc(D.definitions.channels[channel].label)} · tune sweep</h4>` +
      `<svg viewBox="0 0 100 100" preserveAspectRatio="none" role="img" ` +
      `aria-label="threshold sweep">${line("correct_candidate_retention", "correct")}` +
      `${line("incorrect_candidate_rejection", "incorrect")}` +
      `${line("detection_level_recall", "recall")}</svg>` +
      `<p class="axis"><span>${num(lo)}</span><span>threshold</span><span>${num(hi)}</span></p>` +
      `<p class="legend">green correct retention · red incorrect rejection · cyan detection recall</p></article>`;
  }

  function postFilterStages(panel) {
    return Object.entries(panel.thresholds).map(([channel, threshold]) => {
      const stage = threshold.holdout_stage_survival;
      if (!stage) return "";
      return `<div title="same holdout GT-available detection population; proposal only">` +
        `<span>${esc(D.definitions.channels[channel].label)} holdout survival</span>` +
        `<strong>${num(stage.stage6000_with_correct)}→${num(stage.stage300_with_correct)}→` +
        `${num(stage.post_filter_with_correct)}</strong>` +
        `<small>loss ${num(stage.lost_6000_to_300)} + ${num(stage.lost_300_to_filter)} · ` +
        `population ${num(stage.gt_available)}</small></div>`;
    }).join("");
  }

  function panel(name) {
    const value = D.objects[name];
    const stage = value.stage_attrition;
    const stageHtml = stage ?
      `<div class="stage"><div title="${esc(stage.stage6000_score)}">` +
      `<span>6000 initial residual</span><strong>${num(stage.stage6000_with_correct)}</strong></div>` +
      `<div title="${esc(stage.stage300_scores)}"><span>300 full scores</span>` +
      `<strong>${num(stage.stage300_with_correct)}</strong></div>` +
      `<div><span>attrition</span><strong>${num(stage.lost_6000_to_300)}</strong></div></div>` : "";
    const thresholdHtml = value.trusted ?
      `<h3>독립 threshold 제안</h3><div class="post-filter">${postFilterStages(value)}</div>` +
      `<div class="table-wrap"><table><thead><tr>` +
      `<th>channel</th><th>status</th><th>threshold</th>` +
      `<th>tune keep</th><th>tune reject</th><th>tune det. recall</th><th>tune C/I/D</th><th>tune mean/median</th>` +
      `<th>hold keep</th><th>hold reject</th><th>hold det. recall</th><th>hold C/I/D</th><th>hold mean/median</th>` +
      `</tr></thead><tbody>${thresholdRows(value)}</tbody></table></div>` +
      `<div class="sweeps">${Object.entries(value.thresholds).map(([channel, threshold]) =>
        sweepCard(channel, threshold)).join("")}</div>` +
      `<p class="foot">각 sweep 점은 analysis JSON에 threshold, 정답·오답 수, 결측 수와 ` +
      `함께 저장됩니다. Weighted/composite score 없음. Realtime activation 없음.</p>` : "";
    return `<section class="panel"><h2>${esc(name)}</h2>` +
      `<p class="${value.trusted ? "" : "unavailable"}">` +
      (value.trusted ?
        `trusted pseudo-GT · evaluation ${num(value.input_detection_count)} · ` +
        `reference excluded ${num(value.excluded_reference_member_count)}` :
        `GT 신뢰 부족 (${esc(value.unavailable_reason)}) — 정답/오답 분포와 threshold unavailable`) +
      `</p>${stageHtml}<div class="channels">` +
      `${residualCard(value.stage6000_residual, value.trusted)}` +
      `${Object.entries(value.channels).map(([key, item]) =>
        channelCard(key, item, value.trusted)).join("")}</div>${thresholdHtml}</section>`;
  }

  function render() {
    nav.innerHTML = names.map(name =>
      `<button data-name="${esc(name)}" class="${selected.has(name) ? "selected" : ""}">` +
      `<b>${esc(name)}</b><small>${D.objects[name].trusted ? "trusted" : "GT 신뢰 부족"}</small>` +
      `</button>`).join("");
    document.querySelector("#panels").innerHTML = [...selected].map(panel).join("");
    nav.querySelectorAll("button").forEach(button => {
      button.onclick = event => {
        const name = event.currentTarget.dataset.name;
        if (event.shiftKey) {
          selected.has(name) ? selected.delete(name) : selected.add(name);
          if (!selected.size) selected.add(name);
        } else {
          selected = new Set([name]);
        }
        render();
      };
    });
  }
  render();
  const picker = document.querySelector("#analysis-file");
  if (picker) picker.onchange = event => {
    const file = event.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const value = JSON.parse(String(reader.result));
        if (!validAnalysis(value)) throw new Error("invalid analysis JSON");
        sessionStorage.setItem(storageKey, JSON.stringify(value));
        location.reload();
      } catch (error) {
        document.querySelector("#notice").textContent =
          `analysis.json 로드 실패: ${error.message}`;
      }
    };
    reader.readAsText(file);
  };
})();
