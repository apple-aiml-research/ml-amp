//
// For licensing see accompanying LICENSE file.
// Copyright (C) 2026 Apple Inc. All Rights Reserved.
//

(() => {
    const fig = document.getElementById("taskflow");
    if (!fig) return;
    const q = (s, r) => (r || fig).querySelector(s), qa = (s, r) => [...(r || fig).querySelectorAll(s)];
    const view = q(".amp-w3-view"), canvas = q(".amp-w3-gl"), T = window.THREE;
    let renderer = null;
    const webgl = (() => { try { const c = document.createElement("canvas"); return !!(c.getContext("webgl2") || c.getContext("webgl")); } catch (e) { return false; } })();
    try { if (T && canvas && webgl) renderer = new T.WebGLRenderer({canvas, antialias: true, alpha: true}); } catch (e) { renderer = null; }
    if (!renderer) { fig.classList.add("no-gl"); return; }

    const clamp = (x, a, b) => x < a ? a : x > b ? b : x, c01 = x => clamp(x, 0, 1);
    const win = (t, a, b) => c01((t - a) / (b - a));
    const eo = x => 1 - Math.pow(1 - c01(x), 3);
    const eio = x => { x = c01(x); return x < .5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2; };
    const lerp = (a, b, t) => a + (b - a) * t;
    const f2 = v => v.toFixed(2);
    const C = new WeakMap(), cc = el => { let c = C.get(el); if (!c) C.set(el, c = {}); return c; };
    const css = (el, k, v) => { const c = cc(el); if (c[k] !== v) { c[k] = v; el.style[k] = v; } };
    const op = (el, v) => css(el, "opacity", v <= .002 ? "0" : v >= .998 ? "1" : v.toFixed(3));
    const tx = (el, v) => { const c = cc(el); if (c.$t !== v) { c.$t = v; el.textContent = v; } };
    const tg = (el, n, on) => { const c = cc(el), k = "." + n; on = !!on; if (c[k] !== on) { c[k] = on; el.classList.toggle(n, on); } };
    const h = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };
    const V = (x, y, z) => new T.Vector3(x, y, z), VA = a => new T.Vector3(a[0], a[1], a[2]);
    const vis = (t, a, b, fi = .25, fo = .4) => Math.min(win(t, a, a + fi), 1 - win(t, b, b + fo));

    const IMG = "assets/task-flow/";
    const TURNS = [
        {title: "Read task and perceive scene", say: {k: "Inputs", q: "First user message: read the instruction from the episode, then begin by looking at the scene"},
         calls: [{code: "ep.observe()", to: "sensing", f: ["observe"], env: true, fx: "observe", up: [{chip: "“Pick up the banana and place it in the bowl”"}]},
                 {code: "ep.objects()", to: "perception", f: ["objects"], fx: "objects", ret: [{chip: "obj1 yellow · obj2 brown", sw: ["#f5c518", "#c4453a"]}]},
                 {code: 'ep.annotate("ego_l") · ep.rgb("ego_r")', to: "perception", f: ["annotate", "rgb"], env: true, fx: "cams", cams: ["left", "right"],
                  up: [{img: "t1-left.webp"}, {img: "t1-right.webp"}]}]},
        {title: "Fit oriented box to object", say: {q: "I'll grab its points, fit an oriented bounding box"},
         calls: [{code: 'rh.fit_obb(ep.object_points("obj1"))', to: "perception", skill: "Scene Reasoning", fx: "obb", ret: [{chip: "box 18.4 × 6.2 cm · long side 65.5°"}]},
                 {code: 'ep.closeup("obj1")', to: "perception", f: ["render"], fx: "closeup", ret: [{img: "t2-closeups.webp"}]}]},
        {title: "Choose grasp and preview approach", say: {q: "its centroid might not fall on the fruit"},
         calls: [{code: "gp = xyz[np.argmin(d)]", to: "skills", skill: "Scene Reasoning", fx: "grasp", ret: [{chip: "grasp at the point nearest the mean"}]},
                 {code: "ep.preview(wps)", to: "motion", f: ["preview"], skill: "Motion", fx: "path", ret: [{chip: "✓ path valid"}]},
                 {code: 'ep.check_batch(pv["q"][::4])', to: "motion", f: ["check_collision"], fx: "check", ret: [{chip: "✗ gripper touches obj1, table", bad: true}]}]},
        {title: "Approach object", say: {q: "Settling on a TCP height of 0.025 and proceeding with execution"},
         calls: [{code: "ep.execute(ep.preview(wps))", to: "motion", f: ["preview", "execute"], skill: "Motion", act: [0, 64], mv: "approach"},
                 {code: 'ep.rgb("wrist") · ep.rgb("ego_r")', to: "sensing", f: ["rgb"], env: true, fx: "cams", cams: ["wrist", "right"],
                  up: [{img: "t4-wrist.webp"}, {img: "t4-right.webp"}]}]},
        {title: "Grasp and lift object", say: {q: "the banana looks positioned between the fingers correctly"},
         calls: [{code: "ep.gripper(True, 1.0)", to: "motion", f: ["gripper"], act: [64, 79], mv: "close", up: [{chip: "gripper reads 28% closed: something between the fingers"}]},
                 {code: "ep.execute(pv)  # lift 12 cm", to: "motion", f: ["preview", "execute"], act: [79, 105], mv: "lift", env: true, cams: ["right"],
                  up: [{chip: "gripper reads 55% closed: still holding"}, {img: "t5-right.webp"}]}]},
        {title: "Plan object placement", say: {q: "I need to recompute the bowl's center"},
         calls: [{code: "ep.objects()", to: "perception", f: ["objects"], fx: "bowl", ret: [{chip: "obj2 brown · 16×15×6 cm", sw: "#c4453a"}]},
                 {code: "ep.preview(wps)", to: "motion", f: ["preview"], skill: "Motion", fx: "path2", ret: [{chip: "✓ path to above the bowl"}]}]},
        {title: "Carry object above target", say: {q: "the bottom should clear the bowl's rim and floor fine"},
         calls: [{code: "ep.execute(pv)", to: "motion", f: ["execute"], act: [105, 161], mv: "carry"},
                 {code: 'ep.rgb("ego_r") · ep.rgb("ego_l")', to: "sensing", f: ["rgb"], env: true, fx: "cams", cams: ["left", "right"],
                  up: [{img: "t7-left.webp"}, {img: "t7-right.webp"}]}]},
        {title: "Release object", say: {q: "the action is to open and lift the gripper"},
         calls: [{code: "ep.gripper(False, 1.0)", to: "motion", f: ["gripper"], act: [161, 162], mv: "release", end: true, up: [{chip: "# RoboLab ended the episode."}]}]},
    ];
    const PART = {objects: "perception", annotate: "perception", render: "perception", observe: "sensing", rgb: "sensing",
                  preview: "motion", check_collision: "motion", execute: "motion", gripper: "motion"};
    const FPS = 15, SPEED = 2.2, LEAD = .5, HOLD = 2.6, BASE = .2;
    const STOPS = [0, .25, .5, 1, 2, 5, 10, 15, 20], LABEL = ["paused", "¼×", "½×", "1×", "2×", "5×", "10×", "15×", "20×"];

    const EVENTS = [], MOTIONS = [], MV = {}, FX = {};
    let T_END = 0;
    (() => {
        let t = LEAD;
        TURNS.forEach((tu, i) => {
            tu.n = i + 1; tu.t0 = t; tu.items = [];
            let c = t + .5;
            const think = clamp(tu.say.q.length / 42, 1.0, 1.7);
            tu.think = [c, c + think]; c += think + .2;
            tu.calls.forEach(call => {
                call.blocks = [...new Set([call.to, ...(call.f || []).map(f => PART[f])])];
                call.code0 = call.code; call.code = [c, c + .7];
                EVENTS.push({k: "code", t0: c, t1: c + .7});
                let e = c + .7;
                call.hit = [c + .55, 0];
                if (call.act) {
                    EVENTS.push({k: "act", t0: e, t1: e + .6}); e += .6;
                    const d = Math.max(.6, (call.act[1] - call.act[0]) / FPS / SPEED);
                    const m = {t0: e, t1: e + d, s0: call.act[0], s1: call.act[1]};
                    MOTIONS.push(m); MV[call.mv] = m; e += d;
                    if (call.end) tu.endAt = e;
                }
                const run = [call.hit[0], e];
                if (call.up) {
                    e += call.act ? .1 : .3;
                    call.envOn = call.env ? [e - .35, e + .55] : null;
                    EVENTS.push({k: "obs", t0: e, t1: e + 1.2}); call.out = [e, e + 1.2]; e += 1.2;
                    call.up.forEach(d => tu.items.push({at: e, d}));
                } else if (call.ret) {
                    e += .15;
                    EVENTS.push({k: "ret", t0: e, t1: e + .7}); call.out = [e, e + .7]; e += .7;
                    call.ret.forEach(d => tu.items.push({at: e, d}));
                }
                call.hit[1] = e;
                call.run = run;
                if (call.fx) (FX[call.fx] = FX[call.fx] || []).push(call);
                c = e + .25;
            });
            tu.t1 = c + .9;
            t = tu.t1;
        });
        T_END = t + HOLD;
    })();
    const END_AT = TURNS[TURNS.length - 1].endAt;
    const turnAt = t => { let k = 0; TURNS.forEach((tu, i) => { if (t >= tu.t0) k = i; }); return k; };
    const stepAt = t => {
        let s = 0;
        for (const m of MOTIONS) { if (t >= m.t1) s = m.s1; else if (t >= m.t0) return lerp(m.s0, m.s1, (t - m.t0) / (m.t1 - m.t0)); else break; }
        return s;
    };
    const fx = (k, i = 0) => FX[k][i];

    const P = {
        tcp0: [0.359, 0, 0.337], above: [0.481, -0.107, 0.13], grasp0: [0.481, -0.107, 0.022], grasp: [0.481, -0.107, 0.025],
        lift: [0.481, -0.107, 0.145], bowlHi: [0.58, 0.154, 0.16], bowlLo: [0.58, 0.154, 0.11], gp: [0.481, -0.107, 0.0389],
        aabb1: {c: [0.484, -0.105, 0.031], s: [0.108, 0.178, 0.037]}, aabb2: {c: [0.579, 0.154, 0.034], s: [0.16, 0.16, 0.056]},
        obb: {c: [0.491, -0.105, 0.026], s: [0.184, 0.062, 0.025], yaw: 65.515}, mean: [0.482, -0.107, 0.031],
        bowl2: {c: [0.58, 0.154, 0.035], s: [0.16, 0.15, 0.0585]},
    };
    const SCENE = {banana: [0.5097, -0.1158, 0.0212], bowl: [0.5611, 0.1526, 0.0028], top: .003};
    const YAW_G = (P.obb.yaw - 90) * Math.PI / 180;
    const AXIS = V(Math.cos(YAW_G), Math.sin(YAW_G), 0);
    const DEG = Math.PI / 180;

    const scene = new T.Scene(), camera = new T.PerspectiveCamera(34, 1.6, .02, 20);
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.outputEncoding = T.sRGBEncoding; renderer.toneMapping = T.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.05;
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
    const world = new T.Group();
    world.quaternion.setFromRotationMatrix(new T.Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1));
    scene.add(world);
    scene.add(new T.HemisphereLight(0xdfe8ff, 0x2a2420, .75));
    const sun = new T.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1.1, -.7, 1.7); sun.castShadow = true;
    Object.assign(sun.shadow.camera, {left: -.9, right: .9, top: .9, bottom: -.9, near: .1, far: 5});
    sun.shadow.mapSize.set(1536, 1536); sun.shadow.bias = -.0004; sun.shadow.radius = 3;
    sun.target.position.set(.5, 0, 0); world.add(sun, sun.target);
    const std = (color, o = {}) => new T.MeshStandardMaterial({color, roughness: .55, metalness: .05, ...o});
    const mesh = (geo, mat, shadow = true) => { const m = new T.Mesh(geo, mat); m.castShadow = shadow; m.receiveShadow = true; return m; };
    const wood = (() => {
        const cv = document.createElement("canvas"); cv.width = 512; cv.height = 512;
        const g = cv.getContext("2d"); g.fillStyle = "#c28b54"; g.fillRect(0, 0, 512, 512);
        for (let i = 0; i < 260; i++) {
            const y = Math.random() * 512, a = .04 + Math.random() * .1, w = 1 + Math.random() * 3;
            g.strokeStyle = Math.random() < .5 ? `rgba(120,70,30,${a})` : `rgba(240,200,150,${a})`; g.lineWidth = w;
            g.beginPath(); g.moveTo(0, y); for (let x = 0; x <= 512; x += 32) g.lineTo(x, y + Math.sin(x / 70 + i) * 4); g.stroke();
        }
        const tex = new T.CanvasTexture(cv); tex.encoding = T.sRGBEncoding; tex.wrapS = tex.wrapT = T.RepeatWrapping; tex.repeat.set(1.4, 1.4);
        return tex;
    })();
    const table = mesh(new T.BoxGeometry(.70, 1.0, .04), std(0xffffff, {map: wood, roughness: .7}), false);
    table.position.set(.547, 0, SCENE.top - .02); world.add(table);
    [[.22, .46], [.87, .46], [.22, -.46], [.87, -.46]].forEach(([x, y]) => { const l = mesh(new T.BoxGeometry(.045, .045, .72), std(0x3a2a1e)); l.position.set(x, y, -.4); world.add(l); });
    scene.fog = new T.Fog(0x0b0c0f, 2.2, 4.2);
    const floor = mesh(new T.PlaneGeometry(8, 8), std(0x0d0e11, {roughness: 1}), false);
    floor.position.set(.5, 0, -.76); world.add(floor);
    const dark = std(0x2b2d33, {roughness: .45});
    const YUP = V(0, 1, 0);
    const ped = mesh(new T.BoxGeometry(.22, .22, .76), std(0x24262b)); ped.position.set(-.04, 0, -.38); world.add(ped);
    const R = window.AMP_ROBOT, bin = b => Uint8Array.from(atob(b), ch => ch.charCodeAt(0));
    function creased(pos, idx, cos) {
        const nF = idx.length / 3, fn = new Float32Array(nF * 3), adj = Array.from({length: pos.length / 3}, () => []);
        for (let f = 0; f < nF; f++) {
            const [a, b, c] = [idx[3 * f], idx[3 * f + 1], idx[3 * f + 2]].map(i => 3 * i);
            const ux = pos[b] - pos[a], uy = pos[b + 1] - pos[a + 1], uz = pos[b + 2] - pos[a + 2], vx = pos[c] - pos[a], vy = pos[c + 1] - pos[a + 1], vz = pos[c + 2] - pos[a + 2];
            fn[3 * f] = uy * vz - uz * vy; fn[3 * f + 1] = uz * vx - ux * vz; fn[3 * f + 2] = ux * vy - uy * vx;
            adj[a / 3].push(f); adj[b / 3].push(f); adj[c / 3].push(f);
        }
        const unit = f => { const x = fn[3 * f], y = fn[3 * f + 1], z = fn[3 * f + 2], l = Math.hypot(x, y, z) || 1; return [x / l, y / l, z / l]; };
        const P = new Float32Array(nF * 9), N = new Float32Array(nF * 9);
        for (let f = 0; f < nF; f++) {
            const n0 = unit(f);
            for (let k = 0; k < 3; k++) {
                const v = idx[3 * f + k]; let x = 0, y = 0, z = 0;
                adj[v].forEach(g => { const n = unit(g); if (n[0] * n0[0] + n[1] * n0[1] + n[2] * n0[2] > cos) { x += fn[3 * g]; y += fn[3 * g + 1]; z += fn[3 * g + 2]; } });
                const l = Math.hypot(x, y, z) || 1, o = 9 * f + 3 * k;
                P[o] = pos[3 * v]; P[o + 1] = pos[3 * v + 1]; P[o + 2] = pos[3 * v + 2]; N[o] = x / l; N[o + 1] = y / l; N[o + 2] = z / l;
            }
        }
        return [P, N];
    }
    const linkGeo = (L, G) => {
        const q = new Uint16Array(bin(G.p).buffer), pos = new Float32Array(q.length);
        for (let i = 0; i < q.length; i++) pos[i] = L.lo[i % 3] + q[i] / 65535 * (L.hi[i % 3] - L.lo[i % 3]);
        const [P, N] = creased(pos, new Uint16Array(bin(G.i).buffer), Math.cos((L.f === "ee" ? 45 : 60) * DEG)), g = new T.BufferGeometry();
        g.setAttribute("position", new T.BufferAttribute(P, 3)); g.setAttribute("normal", new T.BufferAttribute(N, 3));
        return g;
    };
    const MATS = {}, matFor = (c, metal) => MATS[c + metal] || (MATS[c + metal] = std(new T.Color().setRGB(...c), metal ? {metalness: .75, roughness: .3} : {roughness: .4}));
    const LINKS = Object.fromEntries(R.links.map(L => [L.n, {...L, geos: L.groups.map(G => ({geo: linkGeo(L, G), mat: matFor(G.c, G.m)}))}]));
    const armMeshes = [], eeGroup = new T.Group(); eeGroup.matrixAutoUpdate = false; world.add(eeGroup);
    R.links.forEach(L => {
        const o = new T.Group(); o.matrixAutoUpdate = false; LINKS[L.n].mesh = o;
        LINKS[L.n].geos.forEach(({geo, mat}) => o.add(mesh(geo, mat)));
        if (L.f === "ee") eeGroup.add(o); else { o.userData.k = +L.f.slice(4); world.add(o); armMeshes.push(o); }
    });
    const M4 = a => new T.Matrix4().set(...a.flat()), FLANGE_EE = M4(R.flangeEe), TCP_EE = M4(R.eeTcp).invert();
    const adapter = mesh(new T.CylinderGeometry(R.adapter.r, R.adapter.r, R.adapter.h, 48), std(0xa3a6ab, {metalness: .75, roughness: .3})); adapter.matrixAutoUpdate = false; world.add(adapter);
    const ADAPTER = new T.Matrix4().makeTranslation(0, 0, R.adapter.z).multiply(new T.Matrix4().makeRotationX(Math.PI / 2));
    function fk(q) {
        const out = [new T.Matrix4()]; let M = new T.Matrix4();
        R.dh.forEach(([a, d, al], i) => {
            const th = i < 7 ? q[i] : 0, ct = Math.cos(th), st = Math.sin(th), ca = Math.cos(al), sa = Math.sin(al);
            M = M.clone().multiply(new T.Matrix4().set(ct, -st, 0, a, st * ca, ct * ca, -sa, -sa * d, st * sa, ct * sa, ca, ca * d, 0, 0, 0, 1));
            out.push(M);
        });
        return out;
    }
    function fingerPoses(theta) {
        const out = {}, isFinger = n => !!(LINKS[n] && LINKS[n].par);
        const pose = n => {
            if (out[n]) return out[n];
            const L = LINKS[n], Tp = isFinger(L.par) ? pose(L.par) : new T.Matrix4(), J = new T.Matrix4();
            if (L.g) { const piv = VA(L.piv); J.makeRotationAxis(VA(L.ax).normalize(), L.g * theta); J.setPosition(piv.clone().sub(piv.clone().applyMatrix4(J))); }
            return (out[n] = Tp.clone().multiply(J));
        };
        R.links.forEach(L => { if (L.par) pose(L.n); });
        return out;
    }
    function poseRobot(q, theta) {
        const F = fk(q), fp = fingerPoses(theta);
        armMeshes.forEach(m => m.matrix.copy(F[m.userData.k]));
        eeGroup.matrix.copy(F[8]).multiply(FLANGE_EE); adapter.matrix.copy(F[8]).multiply(ADAPTER);
        R.links.forEach(L => { if (L.f === "ee") LINKS[L.n].mesh.matrix.copy(L.par ? fp[L.n] : new T.Matrix4()); });
    }
    const tcpPose = p => { const x = V(Math.cos(YAW_G), Math.sin(YAW_G), 0).multiplyScalar(R.flip ? -1 : 1), z = V(0, 0, -1); return new T.Matrix4().makeBasis(x, z.clone().cross(x), z).setPosition(VA(p)).multiply(TCP_EE); };
    const thetaFor = w => {
        const G = R.gaps;
        for (let i = 1; i < G.length; i++) if (G[i][1] <= w) { const [t0, g0] = G[i - 1], [t1, g1] = G[i]; return t0 + (t1 - t0) * (g0 - w) / (g0 - g1); }
        return R.closed;
    };
    const banana = (() => {
        const Y = window.AMP_YCB_BANANA, bin = b => Uint8Array.from(atob(b), ch => ch.charCodeAt(0));
        const q = new Uint16Array(bin(Y.p).buffer), pos = new Float32Array(q.length), c8 = bin(Y.c), col = new Float32Array(c8.length), c = new T.Color();
        for (let i = 0; i < q.length; i++) pos[i] = Y.lo[i % 3] + q[i] / 65535 * (Y.hi[i % 3] - Y.lo[i % 3]);
        for (let i = 0; i < c8.length; i += 3) { c.setRGB(c8[i] / 255, c8[i + 1] / 255, c8[i + 2] / 255).convertSRGBToLinear(); col[i] = c.r; col[i + 1] = c.g; col[i + 2] = c.b; }
        const geo = new T.BufferGeometry();
        geo.setAttribute("position", new T.BufferAttribute(pos, 3)); geo.setAttribute("color", new T.BufferAttribute(col, 3));
        geo.setIndex(new T.BufferAttribute(new Uint16Array(bin(Y.i).buffer), 1)); geo.computeVertexNormals();
        const g = new T.Group(); g.add(mesh(geo, std(0xffffff, {vertexColors: true, roughness: .55}))); world.add(g); g.geo = geo;
        return g;
    })();
    const BAN_REST = new T.Matrix4().makeTranslation(...SCENE.banana);
    // YCB bowl profile: YCB-Video, Copyright (c) 2017 Yu Xiang, MIT (task-flow/ycb.LICENSE).
    const bowlMesh = (() => {
        const pts = [[0, 0], [.0485, 0], [.0526, .006], [.0567, .010], [.0598, .014], [.0627, .018], [.065, .022], [.0671, .026], [.0689, .030], [.0704, .034],
            [.0717, .038], [.0725, .042], [.0733, .046], [.0811, .0495], [.0811, .055], [.0727, .055], [.0716, .050], [.0702, .046], [.0697, .042], [.0685, .038],
            [.0671, .034], [.0653, .030], [.0634, .026], [.0608, .022], [.0582, .018], [.0549, .014], [.0508, .010], [.0467, .006], [.040, .004], [0, .004]].map(([r, z]) => new T.Vector2(r, z));
        const m = mesh(new T.LatheGeometry(pts, 64), std(0x6e1212, {roughness: .3, side: T.DoubleSide}));
        m.rotation.x = Math.PI / 2; m.position.set(...SCENE.bowl); world.add(m);
        return m;
    })();
    function camRig(hfovDeg, depth, size = 1) {
        const g = new T.Group(), body = mesh(new T.BoxGeometry(.05 * size, .035 * size, .035 * size), dark, false);
        const hw = Math.tan(hfovDeg * DEG / 2) * depth, hh = hw * 9 / 16, c = [V(-hw, -hh, -depth), V(hw, -hh, -depth), V(hw, hh, -depth), V(-hw, hh, -depth)];
        const lp = []; c.forEach((p, i) => { lp.push(0, 0, 0, p.x, p.y, p.z); const n = c[(i + 1) % 4]; lp.push(p.x, p.y, p.z, n.x, n.y, n.z); });
        const lg = new T.BufferGeometry(); lg.setAttribute("position", new T.Float32BufferAttribute(lp, 3));
        const lines = new T.LineSegments(lg, new T.LineBasicMaterial({color: 0x40c8e0, transparent: true, opacity: .35}));
        const fg = new T.BufferGeometry(); fg.setAttribute("position", new T.Float32BufferAttribute([0, 0, 0, ...c.flatMap(p => [p.x, p.y, p.z])], 3)); fg.setIndex([0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1]);
        const fill = new T.Mesh(fg, new T.MeshBasicMaterial({color: 0x40c8e0, transparent: true, opacity: 0, side: T.DoubleSide, depthWrite: false}));
        g.add(body, lines, fill); g.lines = lines; g.fill = fill;
        return g;
    }
    const camQ = (w, x, y, z) => new T.Quaternion(x, y, z, w);
    const CAMS = {left: camRig(104, .2), right: camRig(104, .2), wrist: camRig(88, .1, .4)};
    CAMS.left.position.set(.05, .57, .66); CAMS.left.quaternion.copy(camQ(-.393, -.195, .399, .805));
    CAMS.right.position.set(.05, -.57, .66); CAMS.right.quaternion.copy(camQ(.805, .399, -.195, -.393));
    world.add(CAMS.left, CAMS.right);
    CAMS.wrist.position.set(.011, -.031, -.074); CAMS.wrist.quaternion.copy(camQ(-.420, .570, .576, -.409)); eeGroup.add(CAMS.wrist);

    const lineMat = (color, o = 1) => new T.LineBasicMaterial({color, transparent: true, opacity: o});
    const ROD = new T.CylinderGeometry(1, 1, 1, 8);
    function boxRig(color) {
        const g = new T.Group(), mat = new T.MeshBasicMaterial({color, transparent: true}), f = new T.Mesh(new T.BoxGeometry(1, 1, 1), new T.MeshBasicMaterial({color, transparent: true, opacity: .1, depthWrite: false}));
        g.rods = Array.from({length: 12}, () => { const r = new T.Mesh(ROD, mat); g.add(r); return r; }); g.add(f); g.f = f; g.mat = mat; world.add(g);
        return g;
    }
    const EDGES = [[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
    function setBox(g, c, s, yaw, o) {
        g.visible = o > .01; if (!g.visible) return;
        g.mat.opacity = o; g.f.material.opacity = .1 * o;
        const q_ = new T.Quaternion().setFromAxisAngle(V(0, 0, 1), yaw), ctr = VA(c);
        const pts = [0, 1, 2, 3, 4, 5, 6, 7].map(i => V((i & 1 ? .5 : -.5) * s[0], (i & 2 ? .5 : -.5) * s[1], (i & 4 ? .5 : -.5) * s[2]).applyQuaternion(q_).add(ctr));
        EDGES.forEach(([a, b], i) => { const r = g.rods[i], d = pts[b].clone().sub(pts[a]); r.position.copy(pts[a]).addScaledVector(d, .5); r.quaternion.setFromUnitVectors(YUP, d.clone().normalize()); r.scale.set(.0011, d.length(), .0011); });
        g.f.position.copy(ctr); g.f.quaternion.copy(q_); g.f.scale.set(s[0], s[1], s[2]);
    }
    const aabb1 = boxRig(0xf5c518), aabb2 = boxRig(0xff6b5a), obb = boxRig(0x3ad29f), bowlBox = boxRig(0xff6b5a);
    const cloud = (() => {
        const pos = banana.geo.attributes.position, pts = [];
        for (let i = 0; i < pos.count; i += 2) { const p = V(pos.getX(i), pos.getY(i), pos.getZ(i)); if (p.z < -.004) continue; p.x += (Math.random() - .5) * .004; p.y += (Math.random() - .5) * .004; p.z += (Math.random() - .5) * .003; p.applyMatrix4(BAN_REST); pts.push(p.x, p.y, p.z); }
        const g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(pts, 3));
        const m = new T.Points(g, new T.PointsMaterial({color: 0xffe066, size: .006, transparent: true, opacity: 0, depthWrite: false})); world.add(m); m.n = pts.length / 3;
        return m;
    })();
    const dot = (r, color) => { const m = new T.Mesh(new T.SphereGeometry(r, 16, 12), new T.MeshBasicMaterial({color, transparent: true})); world.add(m); return m; };
    const ring = (r, color) => {
        const pts = []; for (let i = 0; i <= 64; i++) { const a = i / 64 * Math.PI * 2; pts.push(Math.cos(a) * r, Math.sin(a) * r, 0); }
        const g = new T.BufferGeometry(); g.setAttribute("position", new T.Float32BufferAttribute(pts, 3));
        const l = new T.Line(g, lineMat(color)); world.add(l); return l;
    };
    const meanDot = dot(.006, 0xffffff), meanRing = ring(.02, 0xffffff);
    meanDot.position.copy(VA(P.mean)).setZ(.041); meanRing.position.copy(meanDot.position);
    const gpDot = dot(.008, 0x6ea8ff); gpDot.position.copy(VA(P.gp));
    const arrows = [1, -1].map(s => { const a = new T.ArrowHelper(AXIS.clone().multiplyScalar(-s), VA(P.gp).addScaledVector(AXIS, s * .075).setZ(.045), .045, 0xffffff, .016, .012); world.add(a); return a; });
    const ghost = new T.Group(), ghostMat = new T.MeshBasicMaterial({color: 0x6ea8ff, transparent: true, opacity: .25, depthWrite: false});
    { const fp = fingerPoses(0); R.links.forEach(L => { if (L.f !== "ee") return; const o = new T.Group(); o.matrixAutoUpdate = false; o.matrix.copy(L.par ? fp[L.n] : new T.Matrix4()); LINKS[L.n].geos.forEach(({geo}) => o.add(new T.Mesh(geo, ghostMat))); ghost.add(o); }); }
    ghost.matrixAutoUpdate = false; ghost.matrix.copy(tcpPose(P.grasp0)); world.add(ghost);
    function polyline(points, color, dotted) {
        const N = 120, segs = [];
        for (let i = 1; i < points.length; i++) segs.push([VA(points[i - 1]), VA(points[i])]);
        const L = segs.map(([a, b]) => a.distanceTo(b)), LT = L.reduce((s, x) => s + x, 0), at = [];
        for (let k = 0; k <= N; k++) { let d = k / N * LT, i = 0; while (i < segs.length - 1 && d > L[i]) { d -= L[i]; i++; } at.push(segs[i][0].clone().lerp(segs[i][1], clamp(d / L[i], 0, 1))); }
        const mat = new T.MeshBasicMaterial({color, transparent: true});
        let obj;
        if (dotted) {
            obj = new T.Group(); const bead = new T.SphereGeometry(.0032, 10, 8);
            for (let k = 0; k <= N; k += 4) { const b = new T.Mesh(bead, mat); b.position.copy(at[k]); b.k = k; obj.add(b); }
            obj.reveal = f => obj.children.forEach(b => { b.visible = b.k <= f * N; });
        } else {
            const RS = 8, geo = new T.TubeGeometry(new T.CatmullRomCurve3(at, false, "catmullrom", 0), N, .0024, RS, false);
            obj = new T.Mesh(geo, mat);
            obj.reveal = f => geo.setDrawRange(0, Math.round(f * N) * RS * 6);
        }
        world.add(obj); obj.material = mat; obj.N = N + 1; obj.pt = k => at[k].clone();
        return obj;
    }
    const path1 = polyline([P.tcp0, P.above, P.grasp0], 0x6ea8ff), path2 = polyline([P.lift, P.bowlHi, P.bowlLo], 0x6ea8ff, true);
    const wps = [P.above, P.grasp0, P.bowlHi, P.bowlLo].map(p => { const d = dot(.007, 0x6ea8ff); d.position.copy(VA(p)); return d; });
    const SAMPLES = 11, checks = [];
    for (let i = 0; i < SAMPLES; i++) { const k = Math.round((i + 1) / SAMPLES * (path1.N - 1)); const d = dot(.0075, 0x30d158); d.position.copy(path1.pt(k)); d.bad = i >= SAMPLES - 3; if (d.bad) d.material.color.set(0xff6961); checks.push(d); }
    const contacts = [[P.gp[0] + AXIS.x * .03, P.gp[1] + AXIS.y * .03, .038], [P.gp[0] - AXIS.x * .03, P.gp[1] - AXIS.y * .03, .038], [P.gp[0], P.gp[1], .002]].map(p => { const d = dot(.009, 0xff6961); d.position.copy(VA(p)); return d; });
    const okRing = ring(.095, 0x30d158); okRing.position.set(SCENE.bowl[0], SCENE.bowl[1], SCENE.bowl[2] + .056);

    const labLayer = q(".amp-w3-labels"), tmpV = new T.Vector3();
    function label(text, color, big) { const el = h("span", "amp-w3-lab" + (big ? " big" : "")); el.innerHTML = text; if (color) el.style.setProperty("--lc", color); labLayer.append(el); return {el, at: V(0, 0, 0)}; }
    const LAB = {
        instr: label("“Pick up the banana and place it in the bowl”", "rgba(48,209,88,.8)", true),
        obj1: label("obj1<small>yellow · 18×11×4 cm</small>", "#f5c518"), obj2: label("obj2<small>brown · 16×16×6 cm</small>", "#ff6b5a"),
        obbL: label("oriented box<small>18.4 × 6.2 cm · 65.5°</small>", "#3ad29f"), mean: label("mean<small>150 points within 2 cm</small>"),
        gp: label("grasp point<small>nearest the mean</small>", "#6ea8ff"), wp: label("13 cm", "#6ea8ff"), g: label("grasp 2.2 cm", "#6ea8ff"),
        contact: label("contacts<small>obj1 · table</small>", "#ff6961"), read: label("", "rgba(48,209,88,.8)"),
        bowl: label("obj2<small>16×15×6 cm · centre</small>", "#ff6b5a"), hi: label("16 cm", "#6ea8ff"), lo: label("11 cm", "#6ea8ff"),
        done: label("✓ RoboLab ended the episode", "rgba(48,209,88,.9)", true),
        camL: label("left camera", "#40c8e0"), camR: label("right camera", "#40c8e0"), camW: label("wrist camera", "#40c8e0"),
    };
    LAB.instr.at.set(.55, 0, .25); LAB.obj1.at.set(.484, -.105, .07); LAB.obj2.at.set(.579, .154, .09); LAB.obbL.at.set(.47, -.2, .05);
    LAB.mean.at.set(.53, -.06, .06); LAB.gp.at.set(.43, -.16, .07); LAB.wp.at.copy(VA(P.above)).add(V(0, -.03, .01)); LAB.g.at.set(.48, -.2, .03);
    LAB.contact.at.set(.53, -.03, .015); LAB.bowl.at.set(.58, .154, .1); LAB.hi.at.copy(VA(P.bowlHi)).add(V(0, .04, .01)); LAB.lo.at.copy(VA(P.bowlLo)).add(V(0, .04, 0));
    LAB.done.at.set(.58, .154, .17); LAB.camL.at.set(.05, .57, .72); LAB.camR.at.set(.05, -.57, .72);

    const CLOSED = .036, OPEN = .085;
    const PATH = [[83, .5097, -.1158, .0212], [87, .5112, -.1163, .0492], [91, .5117, -.1168, .0892], [95, .5127, -.1168, .1227], [99, .5127, -.1183, .1342],
        [107, .5127, -.1183, .1342], [111, .5147, -.1133, .1327], [115, .5252, -.0898, .1347], [119, .5432, -.0423, .1382], [123, .5642, .0157, .1402],
        [127, .5807, .0677, .1442], [131, .5962, .1097, .1462], [135, .6062, .1362, .1477], [139, .6092, .1442, .1487], [143, .6092, .1442, .1487],
        [147, .6107, .1442, .1372], [151, .6107, .1442, .1132], [155, .6097, .1442, .0992], [160, .6092, .1437, .0982]];
    const ON_RIM = V(.6102, .1452, .0802), G_OFF = VA(P.grasp).sub(VA(SCENE.banana));
    const frameAt = t => clamp(stepAt(t) - 1, 0, 161.4);
    function bananaPos(f) {
        const n = PATH.length;
        if (f <= PATH[0][0]) return VA(SCENE.banana);
        if (f >= PATH[n - 1][0]) return VA(PATH[n - 1].slice(1));
        let i = 0; while (f > PATH[i + 1][0]) i++;
        const k = j => VA(PATH[clamp(j, 0, n - 1)].slice(1)), u = (f - PATH[i][0]) / (PATH[i + 1][0] - PATH[i][0]);
        const [p0, p1, p2, p3] = [k(i - 1), k(i), k(i + 1), k(i + 2)];
        return p1.clone().multiplyScalar(2).addScaledVector(p2.clone().sub(p0), u)
            .addScaledVector(p0.clone().multiplyScalar(2).addScaledVector(p1, -5).addScaledVector(p2, 4).sub(p3), u * u)
            .addScaledVector(p1.clone().multiplyScalar(3).sub(p0).addScaledVector(p2, -3).add(p3), u * u * u).multiplyScalar(.5);
    }
    function tcpAt(t) {
        const pa = (m, segs) => {
            const u = win(t, m.t0, m.t1); let acc = 0;
            for (let i = 0; i < segs.length; i++) { const [a, b, w] = segs[i]; if (u <= acc + w || i === segs.length - 1) return VA(a).lerp(VA(b), eio((u - acc) / w)); acc += w; }
        };
        const A = MV.approach;
        if (t < A.t0) return VA(P.tcp0);
        if (t < A.t1) return pa(A, [[P.tcp0, P.above, .625], [P.above, P.grasp, .375]]);
        if (t < MV.lift.t0) return VA(P.grasp);
        return bananaPos(Math.min(frameAt(t), 160)).add(G_OFF);
    }
    const openAt = t => { const c = MV.close, r = MV.release; return t < c.t0 ? OPEN : t < c.t1 ? lerp(OPEN, CLOSED, eio(win(t, c.t0, c.t1))) : t < r.t0 ? CLOSED : lerp(CLOSED, OPEN, eio(win(t, r.t0, r.t1))); };
    const qRow = x => { const i = clamp(Math.floor(x), 0, R.q.length - 1), j = Math.min(i + 1, R.q.length - 1), u = x - Math.floor(x); return R.q[i].map((v, k) => v + (R.q[j][k] - v) * u); };
    function qAt(t) {
        const A = MV.approach, K = R.k;
        if (t < A.t0) return R.q[0];
        if (t < A.t1) { const u = win(t, A.t0, A.t1); return u <= .625 ? qRow(eio(u / .625) * (K - 1)) : qRow(K + eio((u - .625) / .375) * (K - 1)); }
        if (t < MV.lift.t0) return R.q[2 * K - 1];
        return qRow(2 * K + clamp(frameAt(t), R.f0, 160) - R.f0);
    }
    function bananaAt(t) {
        const f = frameAt(t);
        if (t < MV.lift.t0 || f <= PATH[0][0]) return BAN_REST;
        const u = c01(f - 160);
        return new T.Matrix4().makeTranslation(...bananaPos(Math.min(f, 160)).lerp(ON_RIM, u * u).toArray());
    }
    function applyScene(t) {
        const tcp = tcpAt(t);
        poseRobot(qAt(t), thetaFor(openAt(t)));
        banana.matrixAutoUpdate = false; banana.matrix.copy(bananaAt(t)); banana.matrixWorldNeedsUpdate = true;
        const ob = fx("observe"), objs = fx("objects"), fit = fx("obb"), gr = fx("grasp"), pth = fx("path"), chk = fx("check"), bw = fx("bowl"), p2 = fx("path2");
        op(LAB.instr.el, vis(t, ob.envOn[0], ob.out[1] + .6));
        const boxes = vis(t, objs.run[0], fit.run[1] + 1.5) * (1 - .6 * win(t, fit.run[0], fit.run[0] + .6));
        setBox(aabb1, P.aabb1.c, P.aabb1.s, 0, boxes); setBox(aabb2, P.aabb2.c, P.aabb2.s, 0, vis(t, objs.run[0], TURNS[1].t1));
        op(LAB.obj1.el, vis(t, objs.run[0] + .2, fit.run[0])); op(LAB.obj2.el, vis(t, objs.run[0] + .3, TURNS[1].t1));
        const cl = vis(t, fit.run[0], TURNS[2].t0, .3, .6);
        cloud.material.opacity = .95 * cl; cloud.visible = cl > .01; cloud.geometry.setDrawRange(0, Math.round(cloud.n * win(t, fit.run[0], fit.run[0] + .5)));
        const f = eio(win(t, fit.run[0] + .45, fit.run[0] + 1.1)), ov = vis(t, fit.run[0] + .4, MV.approach.t0) * (t > TURNS[2].t1 ? .35 : 1);
        setBox(obb, [lerp(P.aabb1.c[0], P.obb.c[0], f), lerp(P.aabb1.c[1], P.obb.c[1], f), lerp(P.aabb1.c[2], P.obb.c[2], f)],
               [lerp(P.aabb1.s[0], P.obb.s[0], f), lerp(P.aabb1.s[1], P.obb.s[1], f), lerp(P.aabb1.s[2], P.obb.s[2], f)], lerp(0, P.obb.yaw * DEG, f), ov);
        op(LAB.obbL.el, vis(t, fit.run[0] + 1.0, TURNS[2].t0));
        const mn = vis(t, fit.run[0] + 1.0, TURNS[2].t1); meanDot.material.opacity = mn; meanRing.material.opacity = mn; meanDot.visible = meanRing.visible = mn > .01;
        op(LAB.mean.el, vis(t, fit.run[0] + 1.1, TURNS[1].t1));
        const g1 = vis(t, gr.run[0], MV.approach.t0 + .3);
        gpDot.material.opacity = g1; gpDot.visible = g1 > .01; arrows.forEach(a => { a.visible = g1 > .05; a.line.material.transparent = a.cone.material.transparent = true; a.line.material.opacity = a.cone.material.opacity = g1; });
        op(LAB.gp.el, vis(t, gr.run[0] + .2, pth.run[0]));
        const gh = vis(t, gr.run[0] + .3, MV.approach.t0, .3, .3);
        ghost.visible = gh > .01; ghost.traverse(o => { if (o.material) o.material.opacity = (o === ghost ? 1 : .25) * gh; });
        ghost.matrix.copy(tcpPose(t >= TURNS[3].think[1] ? P.grasp : P.grasp0));
        const pv = vis(t, pth.run[0], MV.approach.t1 + .5, .2, .6);
        path1.visible = pv > .01; path1.material.opacity = pv; path1.reveal(win(t, pth.run[0], pth.run[0] + .8));
        [0, 1].forEach(i => { wps[i].visible = pv > .01 && t > pth.run[0] + .4 * (i + 1); wps[i].material.opacity = pv; });
        op(LAB.wp.el, vis(t, pth.run[0] + .5, MV.approach.t0)); op(LAB.g.el, vis(t, pth.run[0] + .8, MV.approach.t0));
        tx(LAB.g.el, t >= TURNS[3].think[1] ? "grasp 2.5 cm" : "grasp 2.2 cm");
        const cv_ = vis(t, chk.run[0], MV.approach.t0, .1, .4);
        checks.forEach((d, i) => { const a = win(t, chk.run[0] + i * .06, chk.run[0] + i * .06 + .12) * cv_; d.material.opacity = a; d.visible = a > .01; });
        const ct = vis(t, chk.run[0] + SAMPLES * .06, MV.approach.t0, .15, .4) * (.75 + .25 * Math.sin(t * 9));
        contacts.forEach(d => { d.material.opacity = ct; d.visible = ct > .02; });
        op(LAB.contact.el, vis(t, chk.run[0] + SAMPLES * .06, MV.approach.t0));
        const rd = fx("observe") && TURNS[4].calls, c5 = rd[0], c6 = rd[1];
        const r1 = vis(t, c5.out[0] + .4, c6.code[0]), r2 = vis(t, c6.out[0] + .3, TURNS[5].t0 + .8);
        LAB.read.at.copy(tcp).add(V(0, -.06, .1));
        LAB.read.el.innerHTML = r2 > .01 ? "gripper reads 55% closed<small>still holding</small>" : "gripper reads 28% closed<small>something between the fingers</small>";
        op(LAB.read.el, Math.max(r1, r2));
        const bb = vis(t, bw.run[0], TURNS[6].t0); setBox(bowlBox, P.bowl2.c, P.bowl2.s, 0, bb); op(LAB.bowl.el, vis(t, bw.run[0] + .2, p2.run[0] + .4));
        const q2 = vis(t, p2.run[0], MV.carry.t1 + .6, .2, .6);
        path2.visible = q2 > .01; path2.material.opacity = q2; path2.reveal(win(t, p2.run[0], p2.run[0] + .8));
        [2, 3].forEach((i, j) => { wps[i].visible = q2 > .01 && t > p2.run[0] + .4 * (j + 1); wps[i].material.opacity = q2; });
        op(LAB.hi.el, vis(t, p2.run[0] + .5, MV.carry.t0)); op(LAB.lo.el, vis(t, p2.run[0] + .8, MV.carry.t0));
        const ok = END_AT ? win(t, END_AT + .5, END_AT + 1.0) : 0;
        okRing.material.opacity = ok * (.7 + .3 * Math.sin(t * 6)); okRing.visible = ok > .01; op(LAB.done.el, ok);
        const flash = {left: 0, right: 0, wrist: 0};
        TURNS.forEach(tu => tu.calls.forEach(c => { if (c.cams && c.envOn) c.cams.forEach(k => { flash[k] = Math.max(flash[k], vis(t, c.envOn[0] - .1, c.envOn[1] - .3, .1, .4)); }); }));
        Object.entries(CAMS).forEach(([k, g]) => { g.fill.material.opacity = .22 * flash[k]; g.lines.material.opacity = .3 + .7 * flash[k]; });
        op(LAB.camL.el, flash.left); op(LAB.camR.el, flash.right); op(LAB.camW.el, flash.wrist);
        LAB.camW.at.copy(tcp).add(V(0, -.07, .16));
    }

    const SHOTS = [
        {eye: [1.62, -1.02, 1.02], at: [.42, 0, .2]}, {eye: [.86, -.46, .4], at: [.49, -.1, .02]}, {eye: [.8, -.72, .3], at: [.45, -.07, .12]},
        {eye: [1.0, -.72, .46], at: "tcp"}, {eye: [.9, -.68, .4], at: "tcp"}, {eye: [1.08, -.38, .6], at: [.53, .03, .08]},
        {eye: [1.02, .14, .55], at: "tcp"}, {eye: [.86, .46, .4], at: [.58, .15, .05]},
    ];
    let yawOff = 0, pitchOff = 0, lastDrag = -1e9;
    function shotAt(t, k, tcp) {
        const s = SHOTS[k], at = s.at === "tcp" ? tcp.clone().lerp(V(.5, 0, .05), .35) : VA(s.at);
        return {eye: VA(s.eye), at};
    }
    function placeCamera(t, k, tcp, now) {
        const tu = TURNS[k], cur = shotAt(t, k, tcp), u = eio(win(t, tu.t0, tu.t0 + .5));
        let eye = cur.eye, at = cur.at;
        if (k > 0 && u < 1) { const prev = shotAt(t, k - 1, tcp); eye = prev.eye.lerp(cur.eye, u); at = prev.at.lerp(cur.at, u); }
        if (now - lastDrag > 3500) { yawOff *= .96; pitchOff *= .96; }
        const d = eye.clone().sub(at), r = d.length(), yaw = Math.atan2(d.y, d.x) + yawOff, pitch = clamp(Math.asin(d.z / r) + pitchOff, .08, 1.35);
        eye = at.clone().add(V(Math.cos(pitch) * Math.cos(yaw) * r, Math.cos(pitch) * Math.sin(yaw) * r, Math.sin(pitch) * r));
        camera.position.copy(world.localToWorld(eye)); camera.up.set(0, 1, 0); camera.lookAt(world.localToWorld(at.clone()));
    }
    let dragging = false, dx0 = 0, dy0 = 0;
    view.addEventListener("pointerdown", e => { if (e.button) return; dragging = true; dx0 = e.clientX; dy0 = e.clientY; tg(view, "is-drag", true); view.setPointerCapture && view.setPointerCapture(e.pointerId); });
    view.addEventListener("pointermove", e => {
        if (!dragging) return;
        yawOff -= (e.clientX - dx0) * .006; pitchOff += (e.clientY - dy0) * .004; dx0 = e.clientX; dy0 = e.clientY; lastDrag = performance.now(); wake();
    });
    const endDrag = () => { dragging = false; tg(view, "is-drag", false); };
    view.addEventListener("pointerup", endDrag); view.addEventListener("pointercancel", endDrag);
    view.addEventListener("dblclick", () => { yawOff = 0; pitchOff = 0; wake(); });

    const M = Object.fromEntries(qa("[data-m]").map(el => [el.dataset.m, el])), CHIP = Object.fromEntries(qa("[data-c]").map(el => [el.dataset.c, el]));
    const LINK = Object.fromEntries(qa("[data-l]").map(el => [el.dataset.l, el]));
    const proc = q(".amp-w3-proc");
    let openPop = null;
    const closePop = () => { if (!openPop) return; openPop.pop.hidden = true; openPop.btn.setAttribute("aria-expanded", "false"); openPop = null; };
    function placePop({btn, pop}) {
        const pr = proc.getBoundingClientRect(), br = btn.closest(".amp-w3-m").getBoundingClientRect();
        if (window.innerWidth >= 992) Object.assign(pop.style, {left: "", width: "", right: `calc(100% + 12px)`, top: `${f2(Math.max(0, Math.min(br.top - pr.top, pr.height - pop.offsetHeight)))}px`});
        else Object.assign(pop.style, {right: "", left: "0", width: "100%", top: `${f2(br.bottom - pr.top + 8)}px`});
    }
    qa(".amp-w3-info").forEach(btn => btn.addEventListener("click", e => {
        e.stopPropagation();
        const was = openPop && openPop.btn === btn;
        closePop();
        if (was) return;
        openPop = {btn, pop: document.getElementById(btn.getAttribute("aria-controls"))};
        openPop.pop.hidden = false; btn.setAttribute("aria-expanded", "true"); placePop(openPop);
    }));
    qa(".amp-w3-pop").forEach(pop => {
        pop.addEventListener("click", e => e.stopPropagation());
        q(".amp-w3-pop-x", pop).addEventListener("click", () => { const b = openPop && openPop.btn; closePop(); if (b) b.focus(); });
    });
    document.addEventListener("click", closePop);
    document.addEventListener("keydown", e => { if (e.key === "Escape" && openPop) { const b = openPop.btn; closePop(); b.focus(); } });
    window.addEventListener("resize", () => { if (openPop) placePop(openPop); });
    const say = q(".amp-w3-say"), sayK = q(".amp-w3-k"), sayQ = q(".amp-w3-q"), codeEl = q(".amp-w3-c"), outEl = q(".amp-w3-out"), stepEl = q(".amp-w3-step");
    const itemEl = d => {
        if (d.img) { const e = h("span", "amp-hw-item img"), im = h("img"); im.src = IMG + d.img; im.alt = ""; e.append(im); return e; }
        const e = h("span", "amp-hw-item" + (d.bad ? " bad" : ""));
        [].concat(d.sw || []).forEach(c => { const i = h("i"); i.style.background = c; e.append(i); });
        e.append(d.chip); return e;
    };
    let shown = -1;
    function panel(t, k) {
        const tu = TURNS[k];
        if (shown !== k) { shown = k; outEl.replaceChildren(...tu.items.map(it => { it.el = itemEl(it.d); return it.el; })); tx(sayK, tu.say.k || "AMP1 reasons"); tg(say, "in", !!tu.say.k); }
        const n = Math.round(tu.say.q.length * win(t, tu.think[0], tu.think[1]));
        tx(sayQ, tu.say.q.slice(0, n)); op(say, win(t, tu.think[0] - .2, tu.think[0]));
        let call = null; tu.calls.forEach(c => { if (t >= c.code[0]) call = c; });
        tx(codeEl, call ? call.code0.slice(0, Math.max(1, Math.ceil(call.code0.length * win(t, call.code[0], call.code[0] + .35)))) : "");
        op(codeEl.parentNode, call ? 1 : .35);
        tu.items.forEach(it => { const p = win(t, it.at - .05, it.at + .25); op(it.el, p); css(it.el, "transform", p >= 1 ? "" : `scale(${f2(.7 + .3 * eo(p))})`); });
        const on = new Set(); let running = false, obs = false;
        tu.calls.forEach(c => { if (t >= c.hit[0] && t < c.hit[1]) { running = true; c.blocks.forEach(b => on.add(b)); if (c.skill) on.add("skills"); } if (c.envOn && t >= c.envOn[0] && t < c.envOn[1]) obs = true; });
        Object.entries(CHIP).forEach(([c, el]) => tg(el, "is-on", on.has(c)));
        tg(M.model, "is-r", t >= tu.think[0] - .2 && t < tu.think[1] + .3);
        tg(M.model, "is-code", tu.calls.some(c => t >= c.code[0] - .15 && t < c.code[0] + .3));
        tg(M.py, "is-on", running);
        const moving = MOTIONS.some(m => t >= m.t0 && t < m.t1);
        tg(M.env, "is-act", moving); tg(M.env, "is-obs", obs);
        const lk = (el, p, color, o) => { el.style.setProperty("--p", `${f2(p * 100)}%`); el.style.setProperty("--dc", color); el.style.setProperty("--o", o.toFixed(2)); };
        let tc = null, io = null;
        EVENTS.forEach(ev => {
            if (t < ev.t0 || t > ev.t1) return;
            const u = win(t, ev.t0, ev.t1);
            if (ev.k === "code") tc = [eio(u), "#4b8dff"];
            else if (ev.k === "ret") tc = [1 - eio(u), "#30d158"];
            else if (ev.k === "act") io = [eio(u), "#ff9f40"];
            else if (ev.k === "obs") { if (u < .5) io = [1 - eio(u * 2), "#30d158"]; else tc = [1 - eio((u - .5) * 2), "#30d158"]; }
        });
        lk(LINK.tc, tc ? tc[0] : 0, tc ? tc[1] : "#fff", tc ? 1 : 0); lk(LINK.io, io ? io[0] : 0, io ? io[1] : "#fff", io ? 1 : 0);
        tx(stepEl, String(Math.round(stepAt(t))));
    }

    const video = q(".amp-w3-pip video");
    function syncVideo(t) {
        if (video.readyState < 1) return;
        const want = frameAt(t) / FPS, moving = MOTIONS.some(m => t >= m.t0 && t < m.t1), rate = SPEED * BASE * factor;
        if (moving && playing && !scrubbing && rate >= .5) {
            if (video.playbackRate !== rate) video.playbackRate = rate;
            if (video.paused) { video.currentTime = want; video.play().catch(() => {}); }
            else if (Math.abs(video.currentTime - want) > .25) video.currentTime = want;
        } else {
            if (!video.paused) video.pause();
            const mid = (Math.floor(want * FPS + 1e-6) + .5) / FPS;
            if (Math.abs(video.currentTime - mid) > .25 / FPS && !video.seeking) video.currentTime = mid;
        }
    }

    const steps = q(".amp-hw-steps"), railOn = q(".amp-hw-rail-on"), tok = q(".amp-hw-tok"), fin = q(".amp-hw-fin");
    const nowN = q(".amp-hw-now-n"), nowT = q(".amp-hw-now-t"), playBtn = q(".amp-hw-play"), thr = q(".amp-hw-thr"), thrIn = q(".amp-hw-thr-in"), thrVal = q(".amp-hw-thr-val");
    const stepBtns = TURNS.map((tu, i) => {
        const b = h("button", "amp-hw-step", String(i + 1));
        b.type = "button"; b.setAttribute("aria-label", `Turn ${i + 1}: ${tu.title}`);
        b.addEventListener("click", e => { if (e.detail === 0) jump(i); });
        steps.insertBefore(b, fin);
        return b;
    });
    const railX = i => i < stepBtns.length ? stepBtns[i].offsetLeft + stepBtns[i].offsetWidth / 2 : fin.offsetLeft + fin.offsetWidth / 2;
    const xAt = tt => { if (tt < TURNS[0].t0) return railX(0); const k = turnAt(tt), tu = TURNS[k]; return lerp(railX(k), railX(k + 1), win(tt, tu.t0, tu.t1)); };
    const tAt = x => { for (let k = 0; k < TURNS.length; k++) { const a = railX(k), b = railX(k + 1); if (x <= b || k === TURNS.length - 1) return lerp(TURNS[k].t0, TURNS[k].t1 - .01, c01((x - a) / (b - a))); } return 0; };

    let t = 0, factor = 5, lastFactor = 5, playing = true, scrubbing = false, inView = false, started = false, reduced = false, rTurn = 0;
    function setFactor(f) {
        factor = f; if (f > 0) lastFactor = f; playing = f > 0;
        const i = STOPS.indexOf(f);
        thrIn.value = String(i); tx(thrVal, LABEL[i]); thrIn.setAttribute("aria-valuetext", f === 5 ? "5×, the default speed" : LABEL[i]);
        thr.style.setProperty("--fill", `${f2(i / (STOPS.length - 1) * 100)}%`);
        tg(thr, "is-paused", !playing); tg(playBtn, "is-paused", !playing);
        playBtn.setAttribute("aria-label", playing ? "Pause animation" : "Play animation");
        if (!playing && !video.paused) video.pause();
        if (playing) { started = true; wake(); }
    }
    const setPlaying = on => setFactor(on ? lastFactor : 0);
    playBtn.addEventListener("click", () => setPlaying(!playing));
    thrIn.addEventListener("input", () => setFactor(STOPS[+thrIn.value]));
    function jump(i) { if (reduced) { rTurn = i; wake(); return; } t = TURNS[i].t0; started = true; if (!playing) setPlaying(true); wake(); }
    let downX = 0, moved = false, downBtn = null;
    const railPos = e => e.clientX - steps.getBoundingClientRect().left;
    steps.addEventListener("pointerdown", e => { if (e.button) return; downX = e.clientX; moved = false; scrubbing = true; tg(steps, "is-scrub", true); downBtn = e.target.closest ? e.target.closest(".amp-hw-step") : null; steps.setPointerCapture && steps.setPointerCapture(e.pointerId); });
    steps.addEventListener("pointermove", e => {
        if (!scrubbing) return; if (Math.abs(e.clientX - downX) > 3) moved = true; if (!moved) return;
        if (reduced) rTurn = turnAt(tAt(railPos(e))); else { t = tAt(railPos(e)); started = true; }
        wake();
    });
    const endScrub = () => { if (!scrubbing) return; scrubbing = false; tg(steps, "is-scrub", false); if (!moved && downBtn) jump(stepBtns.indexOf(downBtn)); wake(); };
    steps.addEventListener("pointerup", endScrub); steps.addEventListener("pointercancel", endScrub);

    function resize() {
        const w = view.clientWidth, hh = view.clientHeight;
        if (!w || !hh) return;
        renderer.setSize(w, hh, false); camera.aspect = w / hh;
        camera.fov = Math.max(34, 2 * Math.atan(Math.tan(17 * DEG) * 1.6 / camera.aspect) / DEG); camera.updateProjectionMatrix();
    }
    function frame(now) {
        const k = reduced ? rTurn : turnAt(t), tt = reduced ? TURNS[k].t1 - .01 : t, tu = TURNS[k];
        stepBtns.forEach((b, i) => { tg(b, "is-now", i === k); tg(b, "is-past", i < k); });
        tx(nowN, `Turn ${tu.n}`); tx(nowT, tu.title);
        const x = reduced ? railX(k) : xAt(tt);
        css(tok, "left", `${f2(x)}px`); css(railOn, "width", `${f2(Math.max(0, x - railX(0)))}px`); op(tok, reduced ? 0 : 1);
        tg(fin, "is-on", END_AT && tt >= END_AT);
        applyScene(tt);
        panel(tt, k);
        placeCamera(tt, k, tcpAt(tt), now);
        scene.updateMatrixWorld(true);
        const w = view.clientWidth, hh = view.clientHeight;
        Object.values(LAB).forEach(L => {
            if (cc(L.el).opacity === "0") return;
            tmpV.copy(L.at); world.localToWorld(tmpV); tmpV.project(camera);
            const vx = (tmpV.x + 1) / 2 * w, vy = (1 - tmpV.y) / 2 * hh;
            css(L.el, "transform", `translate3d(${f2(vx - L.el.offsetWidth / 2)}px,${f2(vy - L.el.offsetHeight / 2)}px,0)`);
            if (tmpV.z > 1) op(L.el, 0);
        });
        renderer.render(scene, camera);
        syncVideo(tt);
    }

    let raf = 0, last = 0;
    const tick = now => {
        raf = 0;
        const dt = last ? Math.min(.05, (now - last) / 1000) : 0; last = now;
        if (started && playing && !reduced && !scrubbing) { t += dt * BASE * factor; if (t >= T_END) t = 0; }
        frame(now);
        const easing = Math.abs(yawOff) + Math.abs(pitchOff) > .002;
        if ((inView && (playing || easing || dragging)) && !reduced || dragging) raf = requestAnimationFrame(tick); else last = 0;
    };
    const wake = () => { if (!raf) raf = requestAnimationFrame(tick); };
    const mq = window.matchMedia ? window.matchMedia("(prefers-reduced-motion: reduce)") : null;
    function applyReduced() { reduced = !!(mq && mq.matches); playBtn.hidden = reduced; thr.hidden = reduced; if (reduced && !video.paused) video.pause(); wake(); }
    if (mq && mq.addEventListener) mq.addEventListener("change", applyReduced);
    if ("ResizeObserver" in window) new ResizeObserver(() => { resize(); wake(); }).observe(view); else window.addEventListener("resize", () => { resize(); wake(); });
    if ("IntersectionObserver" in window) {
        new IntersectionObserver(es => es.forEach(e => { inView = e.isIntersecting; if (inView) { if (video.preload !== "auto") { video.preload = "auto"; video.load(); } started = true; wake(); } else if (!video.paused) video.pause(); }), {threshold: .2}).observe(view);
    } else { inView = true; started = true; }
    resize();
    setFactor(5);
    applyReduced();
})();
