# helpers.py -- paste/exec in an episode:  exec(open("workspace/helpers.py").read())
# needs: ep, rh, np in globals. All functions verified in e001/e002 (dual ARX X5) unless marked (untested).
L=ep.ee("left_hand"); R=ep.ee("right_hand"); BASE_Y={"left":0.3,"right":-0.3,"left_hand":0.3,"right_hand":-0.3}
seeds=[None,[0.0065,1.712,1.225,-1.0836,0,-1.5643],[-0.0065,1.712,1.225,-1.0836,0,1.5643],[0,1.712,1.225,-1.0836,0,0]]

def gen_ik(h,pose,extra=()):
    """first reachable IK solution (try several seeds) or None. IK calls are SLOW (~0.3-1 s): no big grids."""
    for sd in list(extra)+seeds:
        r=h.ik(pose,seed=sd)
        if r["reachable"]: return r["solutions"][0]["q"]

def best_td(h,pos,yaw,qseed):
    """top-down pose nearest qseed (that arm's 6 q), trying yaw and yaw+-180 -> (dist,yaw,pose7) or None"""
    best=None
    for yw in [yaw,yaw-180,yaw+180]:
        for sd in [list(qseed)]+seeds:
            r=h.ik(h.top_down(pos,yw),seed=sd)
            if r["reachable"]:
                d=np.abs(np.array(r["solutions"][0]["q"])-qseed).sum()
                if best is None or d<best[0]: best=(d,yw,h.top_down(pos,yw))
                break
    return best

def axes(p):
    Rm=rh.quat_to_mat(p[3:7]); return Rm[:,0],Rm[:,1],Rm[:,2]   # tx (closing), ty, tz (approach)

def up(P,z):
    P=list(P); P[2]=z; return P

# ---------- WRIST CAMERA (any wrist cam, any TCP orientation) ----------
# camera centre = TCP - 0.1025*approach + 0.0047*ty ; f=518 ; pp (320,240);
# image right = -tx, image down = -ty.  Accurate ~0.5-1 cm near the image centre, worse (1-2 cm) at edges.
# (old biased wrist_proj/wrist_backproj removed after e026: use wrist_proj2/wrist_backproj2)
# ---------- HEAD CAMERA (coarse, +-2-3 cm; refine with the wrist cam) ----------
# fitted pinhole: camera at (0.102, 0, 0.493), pitched down 1.103 rad, f=247, pp (320,240), image right = -y.
HEAD=(0.102,0.493,1.103,247.2)
def head_proj(P,p=HEAD):
    cx,cz,th,f=p; P=np.atleast_2d(P); d=P-np.array([cx,0,cz])
    fw=np.array([np.cos(th),0,-np.sin(th)]); rt=np.array([0,-1,0]); dn=np.array([-np.sin(th),0,-np.cos(th)])
    z=d@fw; return np.c_[320+f*(d@rt)/z, 240+f*(d@dn)/z]
def head_backproj(u,v,h,p=HEAD):
    cx,cz,th,f=p
    fw=np.array([np.cos(th),0,-np.sin(th)]); rt=np.array([0,-1,0]); dn=np.array([-np.sin(th),0,-np.cos(th)])
    d=fw+(u-320)/f*rt+(v-240)/f*dn; C=np.array([cx,0,cz]); t=(h-cz)/d[2]; return C+t*d

# ---------- POSES ----------
def tilt_pose(h,pos,tilt,base_y):
    """approach along the azimuth from the arm base, tilted down `tilt` deg, closing axis HORIZONTAL"""
    az=np.arctan2(pos[1]-base_y,pos[0]); t=np.radians(tilt)
    app=[np.cos(t)*np.cos(az),np.cos(t)*np.sin(az),-np.sin(t)]; cl=np.array([-np.sin(az),np.cos(az),0])
    for s in [1,-1]:
        p=rh.pose_from_axes(pos,app,s*cl)
        if gen_ik(h,p) is not None: return p

def tilt_grasp(center, R_, zc, theta_deg, base_y=0.3, back=0.0):
    """WALL CLAMP for a bowl/rim/sloped wall: TCP on the near rim point (center - R_*u, u = unit vector arm-base->center),
    approach = (u cos th, -sin th) (down along the wall), closing axis in that vertical plane (perpendicular to the wall).
    back>0 backs off along -approach (pre-grasp). Returns [pose(s=+1), pose(s=-1)]; pick the one whose preview is fast."""
    c=np.array(center); u=np.array([c[0], c[1]-base_y]); u/=np.linalg.norm(u)
    th=np.radians(theta_deg)
    a=np.array([u[0]*np.cos(th),u[1]*np.cos(th),-np.sin(th)])
    n=np.array([u[0]*np.sin(th),u[1]*np.sin(th),np.cos(th)])
    pos=np.array([c[0]-R_*u[0], c[1]-R_*u[1], zc]) - back*a
    return [rh.pose_from_axes(list(pos),list(a),list(s*n)) for s in [1,-1]]

def lean_pose(pos,yaw,phi):
    """top-down tilted by phi deg about the closing axis (closing axis horizontal along yaw). Held objects keep their
    orientation about the closing axis. Left arm: lean_pose(p,0,-20/-30) reaches z 0.12-0.145 at x 0.25-0.32."""
    c=np.array([np.cos(np.radians(yaw)),np.sin(np.radians(yaw)),0])
    perp=np.cross([0,0,1],c)
    a=np.array([0,0,-1])*np.cos(np.radians(phi))+perp*np.sin(np.radians(phi))
    return rh.pose_from_axes(list(pos),list(a),list(c))

def probe(h,pos,qseed,yaws=(0,30,60,90,120,150,180,-30,-60,-90,-120,-150)):
    """yaws whose top-down pose at pos has IK (seeded with the current arm q) -- cheap reach map"""
    return [yw for yw in yaws if h.ik(rh.top_down(pos,yw),seed=list(qseed))["reachable"]]

def limits_ok(q,arm):
    """after a 'stopped' chunk: a collision can push a joint past its limit -> all previews fail; then home()."""
    q=np.array(q); return bool(np.all(q>=np.array(arm["q_min"])-1e-3) and np.all(q<=np.array(arm["q_max"])+1e-3))


# ---------- added after e004 (verified there) ----------
def rotz_pose(p,dth_deg,newpos):
    """rotate pose p about WORLD z by dth_deg and put it at newpos: a held object keeps its tilt (standing stays standing)"""
    c,s=np.cos(np.radians(dth_deg)),np.sin(np.radians(dth_deg)); Rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return list(newpos)+list(rh.mat_to_quat(Rz@rh.quat_to_mat(p[3:7])))

def lean_look(h,T,yaw,phi,d=0.07,qseed=None):
    """look pose d back along the approach of lean_pose(T,yaw,phi): T is at the wrist image centre. -> (pose, q or None)"""
    a=axes(lean_pose([0,0,0],yaw,phi))[2]; p=lean_pose(list(np.array(T)-d*a),yaw,phi)
    return p, gen_ik(h,p,extra=[list(qseed)] if qseed is not None else ())

def seq(pg,lift,ta=0.6):
    """waypoints: reach grasp pose pg at ta (open), close, hold 0.4 s, lift by `lift` -> ~ta+0.85 s. Use the result in
    execute_both / execute. 0.45 s was too fast for a 10 cm advance; 0.5-0.6 ok."""
    pu=list(pg); pu[2]+=lift
    return [rh.waypoint(ta,pg,0),rh.waypoint(ta+0.1,pg,1),rh.waypoint(ta+0.5,pg,1),rh.waypoint(ta+0.85,pu,1)]

def try_pv(wl,wr):
    """preview both arms; print ok / schedule / reason / duration"""
    pv=ep.preview_both({"left_hand":wl,"right_hand":wr}); print(pv["ok"],pv.get("schedule"),rh.explain(pv)[:300],pv.get("duration")); return pv


# ---------- added after e005/e005b: CORRECTED WRIST MODEL (camera pitched ~-20 deg about tool x, +2.3 cm along ty) ----------
# Fitted in e005 on 3 wrist views of a static object vs the head model; it reproduced e005b's head estimates to ~1 cm.
WR_DL=np.radians(-20.0); WR_OFF=0.023
def _wr_frame(p,dl=None,off=None):
    dl=WR_DL if dl is None else dl; off=WR_OFF if off is None else off
    tx,ty,a=axes(p); c,s=np.cos(dl),np.sin(dl)
    a2=c*a+s*ty; ty2=c*ty-s*a; C=np.array(p[:3])-0.1025*a+(0.0047+off)*ty
    return C,tx,ty2,a2
def wrist_backproj2(p,u,v,h,dl=None,off=None):
    """pixel (u,v) of the wrist cam of TCP pose p -> world xy on plane z=h (corrected model)"""
    C,tx,ty2,a2=_wr_frame(p,dl,off); d=a2-(u-320)/518*tx-(v-240)/518*ty2; t=(h-C[2])/d[2]; return (C+t*d)[:2]
def wrist_proj2(p,P,dl=None,off=None):
    """world point P -> (u,v,depth) in the wrist image of TCP pose p (corrected model)"""
    C,tx,ty2,a2=_wr_frame(p,dl,off); rel=np.array(P)-C; Z=rel@a2
    return 320-518*(rel@tx)/Z, 240-518*(rel@ty2)/Z, Z


# ---------- added after e006 (all ran in e006; pour geometry IK-verified, the pour itself lost the cup: grip near the rim!) ----------
def rotx_pose(p,th_deg,newpos):
    """rotate pose p about WORLD x by th_deg (+th tilts tool-up toward -y) and put it at newpos"""
    c,s=np.cos(np.radians(th_deg)),np.sin(np.radians(th_deg)); Rx=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    return list(newpos)+list(rh.mat_to_quat(Rx@rh.quat_to_mat(p[3:7])))

def rodr(k,ang):
    k=np.array(k,float);k/=np.linalg.norm(k);K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K

def pour_R(mouth,tilt_deg,az_deg,psi_deg,d,Rg):
    """TCP pose that tilts a held container (grasped upright with TCP rotation Rg, TCP on its axis, mouth d m above TCP along
    the axis) by tilt_deg toward horizontal azimuth az_deg, rotated psi_deg about the container axis; mouth centre at `mouth`."""
    az=np.radians(az_deg); t=np.radians(tilt_deg)
    u=np.array([np.sin(t)*np.cos(az),np.sin(t)*np.sin(az),np.cos(t)])
    ax=np.cross([0,0,1],u); ang=np.arccos(np.clip(u[2],-1,1))
    Ra=rodr(ax,ang) if np.linalg.norm(ax)>1e-6 else np.eye(3)
    Rp=rodr(u,np.radians(psi_deg))@Ra@Rg
    return list(np.array(mouth)-d*u)+list(rh.mat_to_quat(Rp))

def lip_pose(Plip,t,az,psi,r_rim,d,Rg):
    """pour_R with the LIP (rim point toward az, the lowest rim point while tilting) held at Plip for any tilt t"""
    dv=np.array([np.cos(np.radians(az)),np.sin(np.radians(az)),0]); tr=np.radians(t)
    mouth=np.array(Plip)-r_rim*np.cos(tr)*dv+r_rim*np.sin(tr)*np.array([0,0,1])
    return pour_R(mouth,t,az,psi,d,Rg)

def chain(h,poses,q0):
    """IK each pose seeded with the previous solution -> list of q or None (check joint continuity before previewing)"""
    qs=[];q=list(q0)
    for p in poses:
        rr=h.ik(p,seed=q)
        if not rr["reachable"]: qs.append(None); continue
        q=rr["solutions"][0]["q"]; qs.append(q)
    return qs

def rimpts(c,h,r,n=36):
    a=np.linspace(0,2*np.pi,n,endpoint=False); return np.c_[c[0]+r*np.cos(a),c[1]+r*np.sin(a),np.full(n,h)]

def head_fit_cylinder(v_top,v_rimbot,u_left,u_right,v_bottom,xr,yr,hr=(0.03,0.15),rr=(0.02,0.05)):
    """grid-fit an upright cylinder/cup from head-image pixels: rim ellipse top/bottom rows, left/right columns, lowest pixel of
    the base. xr, yr: (lo,hi) search ranges. -> (err, cx, cy, h, r). e006: accurate to ~3 px / <1 cm."""
    import itertools; best=None
    for cx,cy,h_,r_ in itertools.product(np.arange(xr[0],xr[1],0.005),np.arange(yr[0],yr[1],0.005),
                                         np.arange(hr[0],hr[1],0.005),np.arange(rr[0],rr[1],0.0025)):
        P=head_proj(rimpts((cx,cy),h_,r_)); B=head_proj(rimpts((cx,cy),0.0,r_))
        e=(P[:,1].min()-v_top)**2+(P[:,1].max()-v_rimbot)**2+(P[:,0].min()-u_left)**2+(P[:,0].max()-u_right)**2+(B[:,1].max()-v_bottom)**2
        if best is None or e<best[0]: best=(e,cx,cy,h_,r_)
    return best


# ---------- added after e007b (ran there) ----------
HOME_TCP={"left_hand":[0.248,0.30,0.155,0.49992,0.49992,0.50008,0.50008],"right_hand":[0.248,-0.30,0.155,0.49992,0.49992,0.50008,0.50008]}
# (a TCP waypoint to HOME_TCP inside another execute brought q to ~0 in e007: homes an arm without a separate home() call)

def tilt_pose_s(h,pos,tilt,base_y,qs):
    """tilt_pose seeded with the arm's current q (qs = that arm's 6 q): approach along the azimuth from the arm base, tilted
    down `tilt` deg, closing axis horizontal; tries both closing signs. Reached the opposite side of the table (cross-midline)."""
    az=np.arctan2(pos[1]-base_y,pos[0]); t=np.radians(tilt)
    app=[np.cos(t)*np.cos(az),np.cos(t)*np.sin(az),-np.sin(t)]; cl=np.array([-np.sin(az),np.cos(az),0])
    for s in [1,-1]:
        p=rh.pose_from_axes(list(pos),app,list(s*cl))
        if h.ik(p,seed=list(qs))["reachable"]: return p

def Rz2(th_deg):
    c,s=np.cos(np.radians(th_deg)),np.sin(np.radians(th_deg)); return np.array([[c,-s],[s,c]])

def dp_map2(p,f,c0,c1,TH,rot_first=True):
    """rigid planar move of a dragged object: rotate by TH deg about c0, then translate c0->c1. f in 0..1.
    rot_first: rotation over f 0..0.5, translation over 0.5..1 (kept the outline clear of neighbours in e009)."""
    g=min(1,2*f) if rot_first else f; hh=max(0,2*f-1) if rot_first else f
    return Rz2(TH*g)@(np.array(p)-np.array(c0))+np.array(c0)+(np.array(c1)-np.array(c0))*hh

def drag_rotate_waypoints(h,G0,yaw0,phi,z,c0,c1,TH,t_rot=1.0,t_tr=0.6,n=9,qseed=None):
    """waypoints (gripper closed) that drag-rotate a held object; also IK-chains them -> (wps, ok). Grip pose = lean_pose(G,yaw0+TH*g,phi).
    e009: right arm, dustpan handle, lean(135,-20), TH=60, 1.0 s rotation + 0.6 s translation was fine."""
    wps=[];q=qseed;ok=True
    for i in range(1,n+1):
        f=i/n; g=min(1,2*f); G=list(dp_map2(G0,f,c0,c1,TH))+[z]; p=lean_pose(G,yaw0+TH*g,phi)
        rr=h.ik(p,seed=q) if q is not None else h.ik(p)
        if not rr["reachable"]: ok=False
        else: q=rr["solutions"][0]["q"]
        tt=t_rot*min(f,0.5)/0.5+(t_tr*(f-0.5)/0.5 if f>0.5 else 0)
        wps.append(rh.waypoint(round(tt,3),p,1))
    return wps,ok

def pick_seq(t0,c,dpt,ta=0.45,yaw=95,phi=-15,zg=0.018,zd=0.055):
    """pick a small cube at xy c (lean grasp, in-plan close) and drop it at xy dpt; 1.55 s after t0+ta. Gripper must start open
    or opens on the first waypoint. e009: 3/3 cubes (2.5 cm) dropped into a dustpan. Lengthen ta after a big wrist reorientation."""
    P=lambda xy,z: lean_pose(list(xy)+[z],yaw,phi)
    return [rh.waypoint(t0+ta,P(c,zd),0),rh.waypoint(t0+ta+0.25,P(c,zg),0),rh.waypoint(t0+ta+0.35,P(c,zg),1),
            rh.waypoint(t0+ta+0.6,P(c,zg),1),rh.waypoint(t0+ta+0.85,P(c,0.06),1),rh.waypoint(t0+ta+1.3,P(dpt,zd),1),
            rh.waypoint(t0+ta+1.4,P(dpt,zd),0),rh.waypoint(t0+ta+1.55,P(dpt,zd),0)]

def head_blobs(img,mask_fn,min_px=200,h=0.045,vmin=150):
    """colour-mask blobs in the HEAD image -> [(n_px,u,v,xy at height h)]. mask_fn(I) on int array, e.g.
    white: lambda I:(I[...,0]>185)&(I[...,1]>185)&(I[...,2]>185); green: lambda I:(I[...,1]>150)&(I[...,0]<170)&(I[...,2]<90).
    Centroid back-projected at the object's top height was within 3-5 mm of wrist fits (standing blocks)."""
    from scipy import ndimage
    m=mask_fn(np.asarray(img).astype(int)); lab,n=ndimage.label(m); out=[]
    for j in range(1,n+1):
        vv,uu=np.nonzero(lab==j)
        if len(vv)>=min_px and vv.mean()>vmin: out.append((len(vv),uu.mean(),vv.mean(),head_backproj(uu.mean(),vv.mean(),h)[:2]))
    return out

def blob_obb(img,mask_fn,u,v,h=0.045):
    """OBB (rh.OBB) of the head-mask blob containing pixel (u,v), pixels back-projected at height h: centre + long_yaw_deg."""
    from scipy import ndimage
    m=mask_fn(np.asarray(img).astype(int)); lab,_=ndimage.label(m); vv,uu=np.nonzero(lab==lab[int(v),int(u)])
    P=np.array([head_backproj(a,b,h) for a,b in zip(uu,vv)]); return rh.fit_obb(P)

def box_corners(cx,cy,yaw,a,b,hgt,z0=0.0):
    """8 corners of a z-up box (a along yaw, b across), order: dx,dy,z (bottom/top alternate)"""
    c,s_=np.cos(yaw),np.sin(yaw); pts=[]
    for dx in (-a/2,a/2):
        for dy in (-b/2,b/2):
            for z in (z0,z0+hgt): pts.append([cx+c*dx-s_*dy,cy+s_*dx+c*dy,z])
    return np.array(pts)

def wrist_rect(tcp,corners_uv,h):
    """4 wrist-image corners of a top face (in order around) -> (centre xy, edge lengths, edge yaws deg) via wrist_backproj2 at
    height h. e010: from lean(.,-20) and top-down looks, centre within ~5 mm of the head estimate."""
    W=np.array([wrist_backproj2(tcp,u,v,h) for u,v in corners_uv]); e1=W[1]-W[0]; e2=W[3]-W[0]
    return W.mean(0),(np.linalg.norm(e1),np.linalg.norm(e2)),(np.degrees(np.arctan2(e1[1],e1[0])),np.degrees(np.arctan2(e2[1],e2[0])))

def place_waypoints(h_pose_grasp,dth,site_xy,z_above,z_rel,t0=0.0,t_move=0.6):
    """carry an UPRIGHT held piece: rotz the grasp pose by dth to above site, lower to z_rel, open, hold, rise 3 cm straight up.
    -> waypoints (gripper closed until the release). e010 timing: move 0.6-0.8 s, lower 0.35 s, open 0.1, hold 0.2, rise 0.3."""
    P=lambda z: rotz_pose(h_pose_grasp,dth,list(site_xy)+[z])
    return [rh.waypoint(t0+t_move,P(z_above),1),rh.waypoint(t0+t_move+0.35,P(z_rel),1),rh.waypoint(t0+t_move+0.45,P(z_rel),0),
            rh.waypoint(t0+t_move+0.65,P(z_rel),0),rh.waypoint(t0+t_move+0.95,P(z_rel+0.03),0)]


# ---------- added after e011/e011b (pose_axes, pitched, cyl-clearance logic ran in e011b; roll_pour_pose/pivot_ratio package its lesson) ----------
def pose_axes(pos,a,tyv):
    """Pose7 from an approach axis a and a desired tool-y axis (made orthogonal); tx = ty x a (right-handed, home: a=+x,ty=+z,tx=+y)."""
    a=np.array(a,float);a/=np.linalg.norm(a); tyv=np.array(tyv,float); tyv-=a*(a@tyv); tyv/=np.linalg.norm(tyv)
    txv=np.cross(tyv,a); return list(pos)+list(rh.mat_to_quat(np.c_[txv,tyv,a]))

def pitched(pos,th_deg,az_deg=0):
    """horizontal home-like pose (approach along az) pitched nose-down by th about the horizontal closing axis"""
    t=np.radians(th_deg); c,s_=np.cos(np.radians(az_deg)),np.sin(np.radians(az_deg))
    return pose_axes(pos,[np.cos(t)*c,np.cos(t)*s_,-np.sin(t)],[np.sin(t)*c,np.sin(t)*s_,np.cos(t)])

def roll_pour_pose(mouth,T_deg,az_deg,side=+1,d=0.04,mouth_sign=+1):
    """PIVOT-SAFE pour pose for an upright object gripped from the side with a HORIZONTAL approach along az_deg (home-like: ty=+z at
    grasp, mouth along mouth_sign*ty, d m from the TCP). The object is tilted T_deg from upright purely by ROLLING about the approach;
    the mouth tilts toward side*(z x a) (side=+1: to the left of the approach). Mouth placed at `mouth`. The roll uses joint 6 (+-1.74 rad
    -> about 100 deg max from a j6~0 grasp): IK-chain T = 0..T_max with seed = current q and check which side works."""
    a=np.array([np.cos(np.radians(az_deg)),np.sin(np.radians(az_deg)),0.0]); h=np.cross([0,0,1.0],a)*side
    t=np.radians(T_deg); m=np.cos(t)*np.array([0,0,1.0])+np.sin(t)*h
    return pose_axes(list(np.array(mouth)-d*m),a,list(mouth_sign*m))

def pivot_ratio(pose,b):
    """gravity torque on a held body (b = unit direction grip->COM, world) about the CLOSING axis, as a fraction of the maximum.
    Only friction resists it: keep < ~0.2 on every waypoint of a tilt (e011b: the pour pose that let the bottle swing had a large one)."""
    tx=axes(pose)[0]; tq=np.cross(np.array(b,float),[0,0,-1.0]); n=np.linalg.norm(tq)
    return 0.0 if n<1e-9 else abs(tq@tx)/n

def cyl_clear(h_arm_name,q_full,c_xy,r,top_z):
    """min clearance (m) of one arm's collision spheres (e.g. 'left') to an upright cylinder (cup/obstacle) of radius r, top top_z,
    for the FULL q (12). Fingertip spheres overestimate by ~1 cm. e011b: >= 0.9 cm along a pour plan was contact-free."""
    rr=ep.robot(list(q_full)); S=np.array(rr["spheres"]); m=np.array([l.startswith(h_arm_name) for l in rr["sphere_links"]]); S=S[m]
    dxy=np.hypot(S[:,0]-c_xy[0],S[:,1]-c_xy[1])-r-S[:,3]; dz=S[:,2]-S[:,3]-top_z
    d=np.where(dz>0,np.where(dxy>0,np.hypot(np.maximum(dxy,0),dz),dz),dxy); return float(d.min())


# ---------- added after e012 (all ran in e012) ----------
def ikq(hd,pose,seed):
    """IK with the given seed first, then the standard seeds -> q (np array) or None"""
    for sd in [seed]+seeds:
        r=hd.ik(pose,seed=None if sd is None else list(sd))
        if r["reachable"]: return np.array(r["solutions"][0]["q"])
    return None

def td_tx(pos,az):
    """top-down pose (a=-z) with the CLOSING axis tx horizontal at azimuth az (deg); ty = a x tx. az and az+180 are different wrist poses."""
    c,s=np.cos(np.radians(az)),np.sin(np.radians(az)); return pose_axes(pos,[0,0,-1],[s,-c,0])

def lean_tx(pos,tx_az,beta):
    """td_tx tilted by beta deg ABOUT tx (closing axis stays horizontal at tx_az). beta=+20 reached z 0.16 over (0.27,0.175) with the left arm
    where top-down stops at 0.105. A key/object hanging from a hinge grip stays vertical under it."""
    tx=np.array([np.cos(np.radians(tx_az)),np.sin(np.radians(tx_az)),0]); ty_td=np.cross([0,0,-1],tx)
    b=np.radians(beta); a=np.array([0,0,-1])*np.cos(b)+ty_td*np.sin(b)
    return pose_axes(pos,a,np.cross(a,tx))

def interp_poses(P0,P1,n):
    """n poses from P0 to P1 (linear position, slerp orientation)"""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    sl=Slerp([0,1],Rot.from_matrix([rh.quat_to_mat(P0[3:]),rh.quat_to_mat(P1[3:])])); out=[]
    for f in np.linspace(0,1,n):
        out.append(list(np.array(P0[:3])*(1-f)+np.array(P1[:3])*f)+list(rh.mat_to_quat(sl([f]).as_matrix()[0])))
    return out

def chainq(hd,poses,q0):
    """IK-chain poses, each seeded with the previous solution -> (qs, all_ok)"""
    qs=[];q=q0
    for p in poses:
        r=hd.ik(p,seed=list(q))
        if not r["reachable"]: return qs,False
        q=np.array(r["solutions"][0]["q"]); qs.append(q)
    return qs,True

def arm_spheres(hd,qfull,prefix):
    """collision spheres (x,y,z,r) of one arm ('left'/'right') for the FULL 12-q. ep.robot() returns only the primary (left) arm: use R.robot for the right."""
    rr=hd.robot(list(qfull)); S=np.array(rr["spheres"]); nm=np.array(rr["sphere_links"]); m=np.array([n.startswith(prefix) for n in nm]); return S[m],nm[m]

def arms_min_clearance(ql,qr):
    """min sphere clearance between the two arms (m) and the two link names; finger (link7/8) spheres overestimate by ~1 cm"""
    qf=list(ql)+list(qr); SL,nL=arm_spheres(L,qf,'left'); SR,nR=arm_spheres(R,qf,'right')
    D=np.linalg.norm(SL[:,None,:3]-SR[None,:,:3],axis=2)-SL[:,None,3]-SR[None,:,3]; i,j=np.unravel_index(D.argmin(),D.shape)
    return float(D.min()),nL[i],nR[j]
# Gripper geometry from the spheres (tool frame): finger plates at +-tx, along ty about +-0.9 cm at the tips widening to +-3 cm 7 cm back,
# along the approach from -0.077 to the tip. Two grippers on one small object need >= ~2 cm between their grip points.


# ---------- added after e013/e013b (flat glyph tiles onto a row of pads; all ran in those episodes) ----------
def purple_or_blue_mask(img,kind="purple"):
    """colour mask for coloured foam tiles: 'purple' (b>g+40 & r>g+10) or 'blue' (b>120,r<90,g<100)"""
    I=np.asarray(img).astype(int)
    if kind=="purple": return (I[...,2]>I[...,1]+40)&(I[...,0]>I[...,1]+10)
    return (I[...,2]>120)&(I[...,0]<90)&(I[...,1]<100)

def glyph_features(img,box,proj,mask=None,h=0.016):
    """Flat glyph with a HOLE (6,8,9,0,4..): world features from ONE image. box=(u0,u1,v0,v1) around the tile,
    proj(u,v)->xy (e.g. lambda u,v: wrist_backproj2(tcp,u,v,h)[:2], valid for top-down/lean wrist looks).
    Returns (hole_xy, centroid_xy, yaw_deg of hole->farthest point, handedness).
    YAW: compare yaw_deg between the start image and now (worked to ~5 deg; filled-blob chamfer did NOT).
    HANDEDNESS: sign of mean cross(axis, pts-hole); a sign change vs the start image = the tile is FLIPPED FACE-DOWN
    (mirror glyph). The head camera is too coarse for this (no hole visible): use a wrist look."""
    from scipy import ndimage
    m=purple_or_blue_mask(img) if mask is None else mask
    u0,u1,v0,v1=box; mm=np.zeros_like(m); mm[v0:v1,u0:u1]=m[v0:v1,u0:u1]; mm=ndimage.binary_opening(mm,iterations=1)
    lab,n=ndimage.label(mm); sz=ndimage.sum(mm,lab,range(1,n+1)); blob=lab==(np.argmax(sz)+1)
    hole=ndimage.binary_fill_holes(blob)&~blob; hl,hn=ndimage.label(hole)
    if hn==0: return None
    hs=ndimage.sum(hole,hl,range(1,hn+1)); hv,hu=np.nonzero(hl==(np.argmax(hs)+1)); H=np.array(proj(hu.mean(),hv.mean()))
    vv,uu=np.nonzero(blob); Pp=np.array([proj(uu[i],vv[i]) for i in range(0,len(vv),3)])
    d=np.linalg.norm(Pp-H,axis=1); T=Pp[np.argsort(d)[-15:]].mean(0); v=(T-H)/np.linalg.norm(T-H)
    rel=Pp-H; cr=v[0]*rel[:,1]-v[1]*rel[:,0]
    return H,Pp.mean(0),float(np.degrees(np.arctan2(v[1],v[0]))),float(np.sign(cr.mean()))

def flip_stick_wps(hd,c,z_grip=0.019,t0=0.0,dy=-0.015,ph=-45):
    """e013 (SUCCESS, first try): turn a lying stick/tile 180 deg about its LONG axis (x here) on the spot (right arm, tile at
    c=(0.37,-0.18) on a pad): top-down grip on its two ENDS (fingers closing along the long axis, lean_pose yaw 180), lift 2.6 cm,
    tilt 45 deg about the closing axis, lower the edge onto the surface, push 1.5 cm sideways (dy sign = the side that continues
    the rotation), open, rise. 4.2 s = 104 steps. Probe ph (-45 reachable, -60 not) and the dy sign with preview first."""
    TD=lambda x,y,z: lean_pose([x,y,z],180,0); LN=lambda x,y,z: lean_pose([x,y,z],180,ph); x,y=c
    return [rh.waypoint(t0+0.9,TD(x,y,0.06),0),rh.waypoint(t0+1.3,TD(x,y,z_grip),0),rh.waypoint(t0+1.4,TD(x,y,z_grip),1),
            rh.waypoint(t0+1.65,TD(x,y,z_grip),1),rh.waypoint(t0+1.95,TD(x,y,0.045),1),rh.waypoint(t0+2.7,LN(x,y,0.045),1),
            rh.waypoint(t0+3.0,LN(x,y,0.022),1),rh.waypoint(t0+3.3,LN(x,y+dy,0.020),1),rh.waypoint(t0+3.4,LN(x,y+dy,0.020),0),
            rh.waypoint(t0+3.55,LN(x,y+dy,0.020),0),rh.waypoint(t0+3.85,LN(x,y+dy,0.06),0)]


# ---------- added after e014 (ran there: 6/6 cup moves first try) ----------
def Ry(t):
    c,s=np.cos(np.radians(t)),np.sin(np.radians(t)); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(t):
    c,s=np.cos(np.radians(t)),np.sin(np.radians(t)); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def P7(p,Rm): return list(np.asarray(p,float))+list(rh.mat_to_quat(Rm))
def grasp_pose(pos,d_az,gamma,sgn=1):
    """grasp whose approach is tilted gamma deg from straight down TOWARD horizontal azimuth d_az; closing axis horizontal, perpendicular to d.
    e015: grasp_pose(G,-90,20,-1) (left arm) on a lying charger whose prongs point -y. After rotating the object by M (e.g. Rz(psi)@Ry(-90) for
    object -x -> down), the gripper pose is P7(T, M@R_grasp) with T = target_point - M@(object_point - G)."""
    d=np.array([np.cos(np.radians(d_az)),np.sin(np.radians(d_az)),0])
    a=-np.cos(np.radians(gamma))*np.array([0,0,1.])+np.sin(np.radians(gamma))*d
    tx=sgn*np.cross([0,0,1.],d); ty=np.cross(a,tx); return P7(pos,np.c_[tx,ty,a])
def rlook2(dist,elev,az_from,aim,tilt_fix=-15):
    """wrist-camera look pose (tool y up) viewing 3D point `aim` from horizontal azimuth az_from, elevation elev, camera distance dist.
    tilt_fix -15 compensates the wrist camera pitch so the aim lands near v 232 (wrist_proj2). e015 right arm: rlook2(0.36,25,-80,S) for S=(0.375,0.296,0.02)."""
    h=np.array([np.cos(np.radians(az_from)),np.sin(np.radians(az_from)),0])
    e=elev+tilt_fix
    d=-(np.cos(np.radians(e))*h)+np.array([0,0,-np.sin(np.radians(e))])
    d0=-(np.cos(np.radians(elev))*h)+np.array([0,0,-np.sin(np.radians(elev))])
    cam=np.asarray(aim)-dist*d0; tcp=cam+0.1025*d
    return pose_axes(tcp,d,[0,0,1])


# ---------- added after e016 (pens into a pen holder; all ran there, 4/4 insertions; episode FAILED for another reason, see NOTES 16) ----------
def unit(v):
    v=np.array(v,float); return v/np.linalg.norm(v)

def pen_bottom(P,Lb):
    """lower end of a pen held along -ty, Lb m below the grip, for TCP pose P"""
    Rm=rh.quat_to_mat(P[3:]); return np.array(P[:3])+Lb*(Rm@np.array([0,-1,0]))

def rotated_pen(pen,Mxy,D):
    """pen dict after rotating it by D deg about the vertical through Mxy (on-table reversal planning)"""
    c,s=np.cos(np.radians(D)),np.sin(np.radians(D)); Rm=np.array([[c,-s],[s,c]])
    p=dict(pen); p["tip"]=Mxy+Rm@(pen["tip"]-Mxy); p["rear"]=Mxy+Rm@(pen["rear"]-Mxy); return p

def hexv(cx,cy,R,th,z):
    a=np.radians(th)+np.arange(6)*np.pi/3; return np.c_[cx+R*np.cos(a),cy+R*np.sin(a),np.full(6,z)]

def fit_hex_head(top_uv,botfront_uv,x0=(0.44,-0.07,0.12,0.037)):
    """hexagonal container from the HEAD image: 6 top-rim vertex pixels + the lowest (front) bottom vertex pixel -> (cx,cy,h,R,th).
    e016: residuals ~1 px, gave a centre that the insertions confirmed."""
    from scipy.optimize import least_squares
    top_uv=np.asarray(top_uv,float); botfront_uv=np.asarray(botfront_uv,float)
    def res(p):
        cx,cy,h,R,th=p; uv=np.array([np.ravel(head_proj(v)) for v in hexv(cx,cy,R,th,h)]); r=[]
        for o_ in top_uv: d=np.linalg.norm(uv-o_,axis=1); r.extend(uv[d.argmin()]-o_)
        uvb=np.array([np.ravel(head_proj(v)) for v in hexv(cx,cy,R,th,0.0)]); r.extend(uvb[uvb[:,1].argmax()]-botfront_uv)
        return np.array(r)
    best=None
    for th0 in [0,15,30,45]:
        s=least_squares(res,list(x0)+[th0])
        if best is None or s.cost<best.cost: best=s
    return best.x


# ---------- added after e017 (all ran there) ----------
def hpose(pos,az):
    """horizontal home-like grasp pose: approach along azimuth az (deg), tool y up (closing axis horizontal)"""
    return pose_axes(list(pos),[np.cos(np.radians(az)),np.sin(np.radians(az)),0],[0,0,1])

def clear_arm(hd,armname,q6,obs_):
    """min clearance of ONE arm's spheres to upright cylinders obs_=[((cx,cy),r,top_z),...]; hd = L or R handle, q6 that arm's q.
    (ep.robot(full q) only returns the primary arm's spheres -> use R.robot for the right arm.)"""
    rr=hd.robot(list(q6)); S=np.array(rr["spheres"]); nm=rr["sphere_links"]; m=np.array([l.startswith(armname) for l in nm]); S=S[m]
    out=[]
    for c,r_,t_ in obs_:
        dxy=np.hypot(S[:,0]-c[0],S[:,1]-c[1])-r_-S[:,3]; dz=S[:,2]-S[:,3]-t_
        d=np.where(dz>0,np.where(dxy>0,np.hypot(np.maximum(dxy,0),dz),dz),dxy); out.append(round(float(d.min()),3))
    return out

# ---------- added after e018 (all ran there; vertical hanging stick tool, SUCCESS) ----------
def strike_pose(B,d,az,sign=1):
    """horizontal gripper (approach az deg, tool y = sign*z) holding a VERTICAL stick whose tip/ball centre B is d m below the TCP.
    e018 right arm: az 75 (80-90 for y<-0.04), d 0.05, B over keys at x 0.43-0.48. Grasp first top-down with ty toward the up-end."""
    T=np.array(B,float)+[0,0,d*sign]
    return pose_axes(list(T),[np.cos(np.radians(az)),np.sin(np.radians(az)),0],[0,0,sign])

def hdir(al,be):
    """unit vector at azimuth al deg, tilted be deg below horizontal"""
    al,be=np.radians(al),np.radians(be); return np.array([np.cos(be)*np.cos(al),np.cos(be)*np.sin(al),-np.sin(be)])

def slerp_pose(A,B,f):
    """pose fraction f of the way from Pose7 A to B (lerp position, slerp rotation): IK-check segments of a waypoint path at f=0..1"""
    from scipy.spatial.transform import Rotation as Rr, Slerp
    qa=np.array(A[3:]);qb=np.array(B[3:])
    rs=Rr.from_quat([[qa[1],qa[2],qa[3],qa[0]],[qb[1],qb[2],qb[3],qb[0]]])
    q=Slerp([0,1],rs)([f]).as_quat()[0]
    return list((1-f)*np.array(A[:3])+f*np.array(B[:3]))+[q[3],q[0],q[1],q[2]]

def box_clear(hd,armname,qs,lo,hi):
    """min clearance (m) of ONE arm's collision spheres to an axis-aligned box [lo,hi] over a list of q (6-long or 12-long).
    e020b: homing path vs a box around a 3-cube tower; finger spheres overestimate the real fingers by ~1 cm."""
    lo=np.asarray(lo,float); hi=np.asarray(hi,float); best=9.0
    for qq in qs:
        qq=np.asarray(qq,float); qq=(qq[6:] if armname=="right" else qq[:6]) if len(qq)==12 else qq
        rr=hd.robot(list(qq)); S=np.array(rr["spheres"]); m=np.array([l.startswith(armname) for l in rr["sphere_links"]]); S=S[m]
        d=np.linalg.norm(np.maximum(np.maximum(lo-S[:,:3],S[:,:3]-hi),0),axis=1)-S[:,3]; best=min(best,float(d.min()))
    return best


# ---------- added after e021 (two-T pose exchange, SUCCESS; all ran there) ----------
def color_mask(img, crop, kind):
    """bool mask in a head-image crop=(v0,v1,u0,u1). kind 'red' (R>200&G<130), 'blue' (B>150&R<100), or a callable on the int sub-image"""
    I=img.astype(int); v0,v1,u0,u1=crop; sub=I[v0:v1,u0:u1]; m=np.zeros(I.shape[:2],bool)
    if kind=="red": m[v0:v1,u0:u1]=(sub[:,:,0]>200)&(sub[:,:,1]<130)
    elif kind=="blue": m[v0:v1,u0:u1]=(sub[:,:,2]>150)&(sub[:,:,0]<100)
    else: m[v0:v1,u0:u1]=kind(sub)
    return m

def bp_mask(m,z=0.02):
    """head-mask pixels -> world xy on plane z (head model)"""
    v,u=np.nonzero(m); return np.array([head_backproj(uu,vv,z)[:2] for uu,vv in zip(u,v)])

def mask_iou(a,b):
    """IoU of two pixel masks + centroid shift (du,dv): compare a moved object NOW with the target object's mask in the START image"""
    va,ua=np.nonzero(a); vb,ub=np.nonzero(b)
    return (a&b).sum()/max((a|b).sum(),1), (ua.mean()-ub.mean(), va.mean()-vb.mean())

def T_mask_pts(cx,cy,ang,dims,grid):
    """T template: origin = centre of the crossbar's OUTER edge, ang = stem direction; dims (crossbar len, crossbar width, stem width, stem len)"""
    Lc,Wc,Ws,Ls=dims
    d=np.array([np.cos(ang),np.sin(ang)]); n=np.array([-d[1],d[0]])
    rel=grid-np.array([cx,cy]); t=rel@d; s_=rel@n
    return ((t>=0)&(t<=Wc)&(np.abs(s_)<=Lc/2))|((t>Wc)&(t<=Wc+Ls)&(np.abs(s_)<=Ws/2))

def _T_iou_search(P,dims,c0,angs,offs,res):
    from scipy.spatial import cKDTree
    lo=P.min(0)-0.03; hi=P.max(0)+0.03
    xs=np.arange(lo[0],hi[0],res); ys=np.arange(lo[1],hi[1],res); G=np.stack(np.meshgrid(xs,ys),-1).reshape(-1,2)
    occ=cKDTree(P).query(G)[0]<res*1.3; best=(0,None,None)
    for ang in angs:
        for dx in offs:
            for dy in offs:
                c=c0+[dx,dy]; m=T_mask_pts(c[0],c[1],ang,dims,G); iou=(m&occ).sum()/max((m|occ).sum(),1)
                if iou>best[0]: best=(iou,c,ang)
    return best

def fit_T(P,dims=(0.064,0.024,0.02,0.05)):
    """coarse (3 deg, 4 mm) then fine (1 deg, 1.5 mm) IoU fit of a T to back-projected points -> (iou, edge centre xy, stem angle rad).
    e021: head masks at z 0.02 -> IoU ~0.8, agreed with a wrist look to 3 mm."""
    b=_T_iou_search(P,dims,P.mean(0),np.radians(np.arange(0,360,3)),np.arange(-0.04,0.041,0.004),0.002)
    return _T_iou_search(P,dims,b[1],b[2]+np.radians(np.arange(-5,5.1,1)),np.arange(-0.006,0.0061,0.0015),0.0015)


# ---------- added after e022b (mug hanging; all ran there) ----------
from PIL import ImageDraw as _IDraw
def gridcrop(img,box,scale=3,step=20):
    """crop (u0,v0,u1,v1) of an image, upscale, draw a labelled pixel grid -> read pixel coordinates precisely from show()"""
    im=Image.fromarray(img).crop(box); W,Hh=im.size; im=im.resize((W*scale,Hh*scale),Image.LANCZOS); d=_IDraw.Draw(im)
    for u in range((box[0]//step+1)*step, box[2], step):
        x=(u-box[0])*scale; d.line([(x,0),(x,Hh*scale)],fill=(0,255,255) if u%100 else (255,0,0),width=1); d.text((x+2,2),str(u),fill=(255,255,0))
    for v in range((box[1]//step+1)*step, box[3], step):
        y=(v-box[1])*scale; d.line([(0,y),(W*scale,y)],fill=(0,255,255) if v%100 else (255,0,0),width=1); d.text((2,y+2),str(v),fill=(255,255,0))
    return im

TRI_DL=np.radians(-24)   # wrist pitch fitted at pitch-10 look poses (both arms, e022b); NOT valid at pitch 45
def tri(obsv,x0=(0.44,0.03,0.18),dl=None):
    """least-squares 3D point from pixels: obsv=[("h",None,(u,v)) head, ("w",tcp_pose,(u,v)) wrist,...] -> (xyz, residuals px).
    e022b: head+wrist triangulation of peg tips/handles agreed ~5 mm with later observations (head residual ~7 px)."""
    from scipy.optimize import least_squares
    dl=TRI_DL if dl is None else dl
    def res(P):
        r=[]
        for k,p,uv in obsv:
            pr=head_proj(P)[0] if k=="h" else wrist_proj2(p,P,dl=dl)[:2]
            r+=[pr[0]-uv[0],pr[1]-uv[1]]
        return r
    s=least_squares(res,list(x0)); return np.round(s.x,3), np.round(s.fun,1)

def look_at_pose(pos,target):
    """home-like (pitched) pose at pos whose approach points at target (camera looks ~24 deg further down)"""
    d=np.array(target)-np.array(pos); az=np.degrees(np.arctan2(d[1],d[0])); th=np.degrees(np.arctan2(-d[2],np.hypot(d[0],d[1])))
    return pitched(list(pos),th,az)

def reach_map(hd,P,heads,ths=(30,45,60),zs=(0.041,0.09,0.15)):
    """{heading: ['101',...] per pitch}: IK reachability of pitched(P xy, z, th, heading) -> pick a grasp heading/pitch fast"""
    return {h:["".join(str(int(gen_ik(hd,pitched([P[0],P[1],z],th,h)) is not None)) for z in zs) for th in ths] for h in heads}

def offp(p,d):
    """Pose7 translated by world vector d (orientation kept)"""
    q=list(p); q[0]+=d[0]; q[1]+=d[1]; q[2]+=d[2]; return q

def body_grasp(C,z,th,heading,back=0.06):
    """across-body mug grasp: TCP on the mug axis C at height z, approach heading = handle azimuth, pitch th -> (grasp, look/pre pose back along approach)"""
    g=pitched([C[0],C[1],z],th,heading); a_=axes(g)[2]; return g, pitched(list(np.array(g[:3])-back*a_),th,heading)


# ---------- added after e025 (6-object basket sort, SUCCESS; ran there) ----------
def best_pose(h,pos,yaws,qseed,phi=0):
    """yaw with IK closest to qseed -> (dist,yaw,pose7,q)"""
    best=None
    for yw in yaws:
        p=lean_pose(pos,yw,phi)
        for sd in [list(qseed)]+seeds:
            r=h.ik(p,seed=sd)
            if r["reachable"]:
                q=np.array(r["solutions"][0]["q"]); d=np.abs(q-qseed).max()
                if best is None or d<best[0]: best=(d,yw,p,q)
                break
    return best

def retime(wps,ts):
    
    return [dict(w,t=t) for w,t in zip(wps,ts)]


# ---------- added after e026 (3 lying cone-tipped tubes -> upright into a rack, SUCCESS 281/500; all ran there) ----------
def tube_from_head(top_uv,bot_uv,z=0.015):
    """lying stick/tube from head pixels of its two end centres (top_uv = end that must end UP) -> dict(cap,tip,d,L,az)"""
    c=head_backproj(*top_uv,z)[:2]; t=head_backproj(*bot_uv,z)[:2]; d=c-t; Lt=np.linalg.norm(d); d=d/Lt
    return dict(cap=c,tip=t,d=d,L=Lt,az=np.degrees(np.arctan2(d[1],d[0])))

def hang_rot_pose(T,pos,psi):
    """grasp-family pose: closing axis horizontal across the tube, rotated psi deg about it (0 = top-down with ty toward the UP end,
    90 = horizontal approach along T's az with ty=+z -> tube hangs vertical, up-end up)"""
    d=np.array([T["d"][0],T["d"][1],0]); f=np.radians(psi); z=np.array([0,0,1.])
    return pose_axes(list(pos),-z*np.cos(f)+d*np.sin(f),d*np.cos(f)+z*np.sin(f))

def hang_carry_wps(T,hole,az,t0=0.95,off=0.045,zg=0.016,z_hang=0.18,nrot=5,seg=0.21,psi1=25,z1=0.06,tl=0.3):
    """pre-grasp (25-deg tilted, 5 cm back) -> top-down grasp `off` m from the up-end at TCP z zg -> close -> 25-deg rotated lift to z1
    -> slerp in nrot segments to the hanging pose pitched(hole,0,az) at z_hang. az is free (tube symmetric): pick it to keep open
    fingers off neighbours. e026: ~3.3 s / 85 steps per arm, both arms in one execute_both."""
    G=T["cap"]-off*T["d"]; Gp=np.array([G[0],G[1],zg]); Pg=hang_rot_pose(T,Gp,0); P25=hang_rot_pose(T,Gp,25)
    Ppre=list(Gp-0.05*axes(P25)[2])+list(P25[3:]); P1=hang_rot_pose(T,[G[0],G[1],z1],psi1); Ph=pitched([hole[0],hole[1],z_hang],0,az)
    w=[rh.waypoint(t0,Ppre,0),rh.waypoint(t0+0.35,Pg,0),rh.waypoint(t0+0.45,Pg,1),rh.waypoint(t0+0.8,Pg,1),rh.waypoint(t0+0.8+tl,P1,1)]
    t=t0+0.8+tl
    for P in interp_poses(P1,Ph,nrot+1)[1:]:
        t+=seg; w.append(rh.waypoint(t,P,1))
    return w

def hang_release_wps(p_now,hole,zr=0.14,ts=(0.6,0.7,1.0,1.4)):
    """from a hanging pose: go to pitched(hole,0,same az) at TCP z zr, open, wait, back off 5 cm along -approach and up 5 cm (~43 steps)"""
    a0=axes(p_now)[2]; az=np.degrees(np.arctan2(a0[1],a0[0])); Pd=pitched([hole[0],hole[1],zr],0,az); a=axes(Pd)[2]
    Pb=list(np.array(Pd[:3])-0.05*a+[0,0,0.05])+list(Pd[3:])
    return [rh.waypoint(ts[0],Pd,1),rh.waypoint(ts[1],Pd,0),rh.waypoint(ts[2],Pd,0),rh.waypoint(ts[3],Pb,0)]


# ---------- added after e027/e027b (doll row; both ran there) ----------
def cyl_pts(cx,cy,r,h,n=12,m=6):
    """points on an upright cylinder -> ep.define_object(points=cyl_pts(...),label=..,kind="fixture") gives check_batch an obstacle without depth"""
    pts=[[cx+r*np.cos(a),cy+r*np.sin(a),z] for z in np.linspace(0,h,m) for a in np.linspace(0,2*np.pi,n,endpoint=False)]
    pts.append([cx,cy,h]); return np.array(pts)

def inv_pitched(pos,th,az):
    """pitched(pos,th,az) rolled 180 deg about the approach (ty ~ -z): final pose to stand up a lying object whose head was along -ty at the grasp"""
    t=np.radians(th); c,s_=np.cos(np.radians(az)),np.sin(np.radians(az))
    a=[np.cos(t)*c,np.cos(t)*s_,-np.sin(t)]; tyv=[-np.sin(t)*c,-np.sin(t)*s_,-np.cos(t)]
    return pose_axes(pos,a,tyv)

def reach2(h,poses):
    """IK reachability of many poses trying every seed in `seeds` (ik_batch per seed) -> (ok bool array, q list)"""
    ok=np.zeros(len(poses),bool); Q=[None]*len(poses)
    for sd in seeds:
        idx=[i for i in range(len(poses)) if not ok[i]]
        if not idx: break
        for i,r in zip(idx,h.ik_batch([poses[i] for i in idx],seed=sd)):
            if r["reachable"]: ok[i]=True; Q[i]=r["solutions"][0]["q"]
    return ok,Q


# ---------- added after e028 (hand tools into a tray; swing_wps ran there and worked) ----------
def swing_wps(g_pose,pivot_xy,a0_deg,a1_deg,z_arc=0.085,z_rel=0.055,t0=0.0,seg=0.4,n=3,close_at=None):
    """rotate a LYING long object about its end resting on the floor: g_pose = grasp pose at the elevated end (already closed, or pass
    close_at=(t_open_to_g, t_close) handled by caller); grip point moves on an arc of radius |grip-pivot| from azimuth a0 to a1 (deg, seen
    from the pivot), gripper yawed by (a1-a0) with rotz_pose, then lowered to z_rel, opened, raised. e028: hammer 147->180 deg."""
    piv=np.array(pivot_xy,float); r=np.linalg.norm(np.array(g_pose[:2])-piv)
    pt=lambda a: list(piv+r*np.array([np.cos(np.radians(a)),np.sin(np.radians(a))]))
    w=[]; t=t0
    for k in range(1,n+1):
        a=a0_deg+(a1_deg-a0_deg)*k/n; t+=seg; w.append(rh.waypoint(t,rotz_pose(g_pose,a-a0_deg,pt(a)+[z_arc]),1))
    fin=rotz_pose(g_pose,a1_deg-a0_deg,pt(a1_deg)+[z_rel])
    w+=[rh.waypoint(t+0.35,fin,1),rh.waypoint(t+0.45,fin,0),rh.waypoint(t+0.6,fin,0),
        rh.waypoint(t+0.9,rotz_pose(g_pose,a1_deg-a0_deg,pt(a1_deg)+[z_rel+0.055]),0)]
    return w


# ---------- added after e029 (desk tidy with a cabinet; both ran there) ----------
def box_pts(lo,hi,n=8):
    """surface points of an aabb -> ep.define_object(points=box_pts(lo,hi),kind="fixture") (cloud is empty without depth)"""
    xs,ys,zs=[np.linspace(lo[i],hi[i],n) for i in range(3)]
    P=np.array([[x,y,z] for x in xs for y in ys for z in zs]); m=(np.isclose(P,lo)|np.isclose(P,hi)).any(1); return P[m]

def cab_clear(qs,lo,hi,prefix="left"):
    """min clearance (m) of one arm's collision spheres (FULL 12-q list) to an aabb [lo,hi]; ep.check missed a 3 cm penetration in e029.
    For the right arm pass prefix="right" (ep.robot may return only the primary arm's spheres: then use arm_spheres/box_clear)."""
    lo=np.array(lo); hi=np.array(hi); m=9
    for qq in qs:
        rb=ep.robot(list(qq)); S=np.array(rb["spheres"]); sel=np.array([l.startswith(prefix) for l in rb["sphere_links"]])
        if not sel.any(): continue
        d=np.linalg.norm(np.maximum(0,np.maximum(lo-S[sel,:3],S[sel,:3]-hi)),axis=1)-S[sel,3]; m=min(m,d.min())
    return m


# ---------- added after e030 (holed pieces onto pegs) ----------
def dip_line(tcp_pose,u_tip,mm_per_px=0.2):
    """straddle-dip measurement (open fingers around a peg tip at ~TCP height): returns (tx_xy, d) meaning
    peg_xy . tx_xy = tcp_xy . tx_xy + d  (u reliable; image right = -tx). Two dips at yaws >= 60 deg apart -> solve_dips."""
    tx=axes(tcp_pose)[0][:2]; tx=tx/np.linalg.norm(tx); return tx, (320-u_tip)*mm_per_px/1000.0

def solve_dips(dips):
    """dips = [(tcp_pose,u_tip),...] -> peg xy (least squares)"""
    A=[];b=[]
    for p,u in dips:
        tx,d=dip_line(p,u); A.append(tx); b.append(np.dot(np.array(p[:2]),tx)+d)
    return np.linalg.lstsq(np.array(A),np.array(b),rcond=None)[0]
