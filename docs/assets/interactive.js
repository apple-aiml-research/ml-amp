//
// For licensing see accompanying LICENSE file.
// Copyright (C) 2026 Apple Inc. All Rights Reserved.
//

function h(tag, attrs = {}, ...children) {
    const el = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
        if (v == null) continue;
        if (k === "class") el.setAttribute("class", v);
        else if (k === "text") el.textContent = v;
        else el.setAttribute(k, v);
    }
    for (const c of children.flat()) if (c != null) el.append(c.nodeType ? c : document.createTextNode(c));
    return el;
}

function segmented(options, active, onPick, cls = "") {
    const bar = h("div", {class: `amp-seg ${cls}`, role: "tablist"});
    options.forEach((o, i) => {
        const b = h("button", {type: "button", class: "amp-seg-btn" + (i === active ? " active" : ""), role: "tab",
                               "aria-selected": i === active ? "true" : "false", text: o});
        b.addEventListener("click", () => {
            bar.querySelectorAll(".amp-seg-btn").forEach((x, j) => {
                x.classList.toggle("active", j === i);
                x.setAttribute("aria-selected", j === i ? "true" : "false");
            });
            onPick(i);
        });
        bar.append(b);
    });
    return bar;
}

const KIND_LABEL = {ours: "AMP1 (ours)", agent: "Agent", vla: "VLA", wam: "WAM/LWM", vlmwam: "VLM+WAM", tamp: "TAMP+FM",
                    agentvla: "Agent+VLA", agentwam: "Agent+WAM", human: "Human oracle"};
const REGIME_LABEL = {zeroshot: "Zero-shot", fewshot: "Few-shot", finetuned: "Fine-tuned"};
const OURS_KINDS = new Set(["ours"]);

function kindLegend(rows) {
    const tinted = r => r.kind !== "human" && !!r.regime;
    const keys = Object.keys(KIND_LABEL).filter(k => rows.some(r => r.kind === k)).map(k => {
        const own = rows.filter(r => r.kind === k);
        const label = k === "ours" && own.every(r => r.name.startsWith("AMP1+")) ? "AMP1+ (ours)" : KIND_LABEL[k];
        return h("span", {class: `amp-key kind-${k}`, text: label});
    });
    const regimes = new Set(rows.filter(tinted).map(r => r.regime));
    if (regimes.size)
        keys.push(h("span", {class: "amp-legend-gap", "aria-hidden": "true"}),
                  rows.some(r => r.kind !== "human" && !r.regime) ? h("span", {class: "amp-key amp-key-regime", text: REGIME_LABEL.zeroshot}) : "",
                  h("span", {class: "amp-key amp-key-regime regime-tint",
                             text: regimes.size > 1 ? "Few-shot or fine-tuned" : REGIME_LABEL[[...regimes][0]]}));
    return h("div", {class: "amp-legend"}, keys);
}

function barChart(rows, {max = 100, unit = "%"} = {}) {
    const wrap = h("div", {class: "amp-bars"});
    rows.forEach((r, i) => {
        const bar = h("div", {class: `amp-bar kind-${r.kind}${r.regime ? ` regime-${r.regime}` : ""}`});
        const ours = OURS_KINDS.has(r.kind);
        const row = h("div", {class: `amp-bar-row${ours ? " is-ours" : ""}`, title: `${r.name}: ${r.value}${unit} (${KIND_LABEL[r.kind] || r.kind}${r.regime ? `, ${REGIME_LABEL[r.regime].toLowerCase()}` : ""})`},
                      h("div", {class: "amp-bar-label"}, r.name, ours ? h("span", {class: "amp-ours", text: "Ours"}) : null),
                      h("div", {class: "amp-bar-track"}, bar),
                      h("span", {class: "amp-bar-value", text: r.value == null ? "n/a" : `${r.value.toFixed(1)}${unit}`}));
        wrap.append(row);
        requestAnimationFrame(() => setTimeout(() => { bar.style.width = r.value == null ? "0" : `${100 * r.value / max}%`; }, 30 * i));
    });
    return wrap;
}

const TASK_SPEED = {"robolab-120": {speed: "5× speed", rate: 0.5},
                    "robodojo": {speed: "5× speed", rate: 0.5}, "behavior-1k": {speed: "5× speed", rate: 0.5}};
const SIM_SPEED = {speed: "10× speed"};
const END_HOLD = {"RoboLab-120": 250};
const whenParsed = fn => document.readyState === "loading" ? document.addEventListener("DOMContentLoaded", fn) : fn();
const LOADING_EVENTS = ["play", "playing", "pause", "waiting", "seeking", "seeked", "canplay", "loadeddata", "ended", "error", "emptied"];
const showLoading = ({target: video}) => {
    if (!(video instanceof HTMLVideoElement) || video.classList.contains("hero-vid")) return;
    video.parentElement?.classList.toggle("is-loading",
        !video.error && (video.seeking || (!video.paused && video.readyState < video.HAVE_FUTURE_DATA)));
};
LOADING_EVENTS.forEach(type => document.addEventListener(type, showLoading, true));
function holdLastFrame(video, ms) {
    let timer = 0;
    const pause = video.pause.bind(video);
    video.pause = () => { clearTimeout(timer); pause(); };
    video.addEventListener("ended", () => { timer = setTimeout(() => video.play()?.catch(() => {}), ms); });
}
const TASK_NEAR = ("IntersectionObserver" in window) ? new IntersectionObserver((entries, observer) => entries.forEach(e => {
    if (!e.isIntersecting) return;
    observer.unobserve(e.target);
    if (e.target.dataset.poster) e.target.poster = e.target.dataset.poster;
}), {rootMargin: "100% 0px"}) : null;
const TASK_SEEN = ("IntersectionObserver" in window) ? new IntersectionObserver(entries => entries.forEach(e => {
    if (e.isIntersecting) e.target.play()?.catch(() => {});
    else e.target.pause();
})) : null;
const dropTasks = el => el.querySelectorAll("video").forEach(v => { TASK_NEAR?.unobserve(v); TASK_SEEN?.unobserve(v); v.pause(); });

const METHOD_LABEL = {"amp1": "AMP1", "flux 3 action": "FLUX 3 Action", "pi0.5": "π0.5"};
const SHORT_LABEL = {"FLUX 3 Action": "FLUX 3"};
const AMP_NAME = {"RoboDojo": "AMP1+"};
const galleryCategories = name => (typeof AMP_VIDEOS !== "undefined" ? AMP_VIDEOS.galleries?.[name]?.categories : null) || [];
const galleryCategory = (gallery, name) => galleryCategories(gallery).find(c => c.name.toLowerCase() === name.toLowerCase());

function taskVideo(v, rate = 1, hold = 0) {
    const poster = v.thumb ? encodeURI(v.thumb) : null;
    const video = h("video", {src: encodeURI(v.file), loop: hold ? null : "", muted: "", playsinline: "", "aria-label": v.title,
                              ...(TASK_SEEN ? {preload: "none", "data-poster": poster} : {poster, autoplay: "", preload: "none"})});
    video.muted = true;
    video.defaultPlaybackRate = video.playbackRate = rate;
    if (hold) holdLastFrame(video, hold);
    if (TASK_SEEN) { TASK_NEAR.observe(video); TASK_SEEN.observe(video); }
    else video.addEventListener("canplay", () => { if (video.paused) video.play()?.catch(() => {}); });
    return video;
}

function rolloutPlayer(methods, {speed, rate = 1, hold = 0, compare = false, start = 0, onTask} = {}) {
    let task = Math.min(start, methods[0].clips.length - 1);
    const side = compare && methods.length > 1;
    const shown = new Set(methods.map((_, i) => i));
    const picks = h("div", {class: "amp-rp-picks", hidden: ""});
    const stage = h("div", {class: "amp-rp-stage"}), caption = h("p", {class: "amp-rp-caption"});
    const list = h("div", {class: "amp-rp-list", role: "list"});
    const first = methods[0].clips[0];
    if (first?.thumb) {
        const img = new Image();
        img.onload = () => {
            stage.style.setProperty("--task-aspect", `${img.naturalWidth} / ${img.naturalHeight}`);
            stage.classList.toggle("squarish", img.naturalWidth / img.naturalHeight < 1.2);
        };
        img.src = encodeURI(first.thumb);
    }
    const outcome = clip => clip.note ? h("span", {class: `amp-rp-outcome ${clip.note.toLowerCase()}`, text: clip.note}) : null;
    const show = () => {
        dropTasks(stage);
        const on = side ? methods.filter((_, i) => shown.has(i)) : [methods[0]];
        stage.classList.toggle("side", side);
        picks.hidden = !side;
        const figure = (m, small) => h("figure", {},
            taskVideo(m.clips[task], rate, hold),
            side || m.clips[task].note ? h("figcaption", {}, side ? h("b", {text: small ? SHORT_LABEL[m.label] || m.label : m.label}) : null,
                                           outcome(m.clips[task])) : null);
        stage.replaceChildren(figure(on[0], false), ...(on.length > 1 ? [h("div", {class: "amp-rp-rest"}, on.slice(1).map(m => figure(m, true)))] : []));
        caption.textContent = methods[0].clips[task].prompt || methods[0].clips[task].title;
        [...list.children].forEach((b, i) => { b.classList.toggle("active", i === task); b.setAttribute("aria-current", i === task ? "true" : "false"); });
    };
    methods[0].clips.forEach((v, i) => {
        const item = h("button", {type: "button", class: "amp-rp-item", role: "listitem", title: v.prompt || v.title},
                       v.thumb ? h("img", {src: encodeURI(v.thumb), alt: "", loading: "lazy", decoding: "async"}) : null, h("span", {text: v.title}));
        item.addEventListener("click", () => { task = i; onTask?.(i); show(); });
        list.append(item);
    });
    const head = h("div", {class: "amp-rp-head"});
    if (side) {
        methods.forEach((m, i) => {
            const chip = h("button", {type: "button", class: "amp-rp-pick active", "aria-pressed": "true", text: m.label});
            chip.addEventListener("click", () => {
                if (shown.has(i) && shown.size === 1) return;
                shown.has(i) ? shown.delete(i) : shown.add(i);
                chip.classList.toggle("active", shown.has(i));
                chip.setAttribute("aria-pressed", String(shown.has(i)));
                show();
            });
            picks.append(chip);
        });
        head.append(picks);
    }
    else head.append(h("span", {class: "amp-rp-pick active amp-rp-tag", title: `Rollouts by ${methods[0].label}`, text: methods[0].label}));
    if (speed) head.append(h("span", {class: "amp-rp-speed", text: speed}));
    show();
    return h("div", {class: `amp-rp${side ? " side" : ""}`}, head, caption, list, stage);
}

const methodsOf = gallery => galleryCategories(gallery).map(c => ({label: METHOD_LABEL[c.name.toLowerCase()] || c.name, clips: c.videos}));

function realworldModule(root) {
    const R = AMP_DATA.realworld, compare = root.dataset.baselines === "on";
    const rows = R.entries.map(e => ({name: e.name, kind: e.kind, regime: e.regime, value: e.v.default})).sort((x, y) => y.value - x.value);
    root.append(h("p", {class: "amp-note mt-0 mb-3", text: R.note}), barChart(rows), kindLegend(rows));
    const methods = methodsOf("realworld");
    if (methods.length) root.append(h("div", {class: "mt-4"}, rolloutPlayer(compare ? methods : methods.slice(0, 1), {compare})));
}

function benchModule(root) {
    const B = AMP_DATA.benchmarks, names = Object.keys(B), compare = root.dataset.baselines === "on";
    const body = h("div"), tasks = h("div");
    let bench = 0, cond = 0, task = 0;
    const benchPlayer = (name, c) => {
        const amp = galleryCategory("benchmarks", name);
        const byCond = name === "RoboLab-120" ? methodsOf(`robolab_${c}`) : [];
        let methods = compare ? byCond : byCond.slice(0, 1);
        if (!methods.length && amp?.videos.length) methods = [{label: AMP_NAME[name] || "AMP1", clips: amp.videos}];
        return methods.length ? rolloutPlayer(methods, {...(TASK_SPEED[name.toLowerCase()] || SIM_SPEED), hold: END_HOLD[name], compare,
                                                        start: task, onTask: i => { task = i; }}) : null;
    };
    const condWrap = h("div");
    const render = () => {
        const b = B[names[bench]], c = (b.conditions || ["default"])[cond] || "default";
        condWrap.innerHTML = "";
        if (b.conditions) condWrap.append(segmented(b.conditions.map(x => `${x[0].toUpperCase()}${x.slice(1)} instructions`), cond,
                                                     i => { cond = i; render(); }, "amp-conds"));
        const rows = b.entries.filter(e => c === "default" || e.v[c] != null)
            .map(e => ({name: e.name, kind: e.kind, regime: e.regime, value: e.v[c]})).sort((x, y) => y.value - x.value);
        body.innerHTML = "";
        const footnote = b.footnote && rows.some(r => r.name.includes(b.footnote[0])) ? h("p", {class: "amp-note", text: b.footnote}) : "";
        body.append(barChart(rows), footnote, kindLegend(rows));
        dropTasks(tasks);
        tasks.replaceChildren(...[benchPlayer(names[bench], c)].filter(Boolean));
    };
    const cards = [...document.querySelectorAll(".stat-card[data-bench]")].filter(el => el.dataset.bench in B);
    const pick = i => {
        bench = i; cond = 0; task = 0;
        cards.forEach(el => {
            el.classList.toggle("active", el.dataset.bench === names[i]);
            el.setAttribute("aria-selected", el.dataset.bench === names[i] ? "true" : "false");
        });
        render();
    };
    cards.forEach(el => {
        const i = names.indexOf(el.dataset.bench);
        el.setAttribute("role", "tab");
        el.tabIndex = 0;
        el.addEventListener("click", () => pick(i));
        el.addEventListener("keydown", e => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); pick(i); } });
    });
    cards[0]?.closest(".row").setAttribute("role", "tablist");
    tasks.classList.add("mt-4");
    root.append(condWrap, body, tasks);
    pick(0);
}

whenParsed(() => {
    const mods = {realworld: realworldModule, bench: benchModule};
    for (const [id, fn] of Object.entries(mods)) {
        const el = document.getElementById(`${id}-module`);
        if (el) try { fn(el); } catch (e) { el.append(h("p", {class: "amp-note", text: `This module failed to load: ${e.message}`})); }
    }
});
