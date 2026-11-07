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
def wrist_proj(p,P):
    tx,ty,a=axes(p); C=np.array(p[:3])-0.1025*a+0.0047*ty; rel=np.array(P)-C; Z=rel@a
    return 320+518*(-rel@tx)/Z, 240+518*(-rel@ty)/Z, Z
def wrist_backproj(p,u,v,h):
    """pixel (u,v) of the wrist cam of TCP pose p -> world xy on the plane z=h"""
    tx,ty,a=axes(p); C=np.array(p[:3])-0.1025*a+0.0047*ty
    d=a-(u-320)/518*tx-(v-240)/518*ty; t=(h-C[2])/d[2]; return (C+t*d)[:2]

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

def rim_points(img, side, u_lim=(0,640), v_rng=range(0,480,10), thr=150):
    """edge of a bright (white) object per row: side='right' -> leftmost bright pixel, 'left' -> rightmost"""
    I=img.astype(float); out=[]
    for v in v_rng:
        idx=np.where(I[v,u_lim[0]:u_lim[1]].mean(axis=1)>thr)[0]
        if len(idx): out.append((u_lim[0]+(idx.min() if side=="right" else idx.max()), v))
    return out


# ---------- added after e003 (verified there) ----------
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

def cloth_mask(img):
    """light-blue / pale cloth mask on the head image (e007b shirt); adjust thresholds per colour. Largest blob, holes filled."""
    from scipy import ndimage
    I=img.astype(int); r,g,b=I[...,0],I[...,1],I[...,2]
    m=(b>170)&(g-r>8)&(b-r>-30)
    lab,n=ndimage.label(m)
    if n==0: return m
    k=np.argmax(ndimage.sum(m,lab,range(1,n+1)))+1; m=lab==k
    return ndimage.binary_fill_holes(ndimage.binary_closing(m,iterations=3))

def shirt_raster(mask,x0=0.0,x1=0.55,y0=-0.35,y1=0.4,res=0.005,h=0.003,marks=()):
    """head-image mask -> top-down PIL image (x up, +y left), 5 cm grid labelled; marks = [(x,y,(r,g,b))]. Read keypoints off it."""
    from PIL import Image, ImageDraw
    vv,uu=np.nonzero(mask); P=np.array([head_backproj(u,v,h) for u,v in zip(uu,vv)])
    xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res); H=np.zeros((len(xs),len(ys)),np.uint8)
    ix=((P[:,0]-x0)/res).astype(int); iy=((P[:,1]-y0)/res).astype(int)
    ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys)); H[ix[ok],iy[ok]]=255
    im=Image.fromarray(H[::-1,::-1].copy()).convert("RGB").resize((len(ys)*4,len(xs)*4),0); d=ImageDraw.Draw(im)
    tr=lambda x,y:((len(ys)-1-(y-y0)/res)*4,(len(xs)-1-(x-x0)/res)*4)
    for xv in np.arange(x0,x1,0.05):
        row=tr(xv,0)[1]; d.line([(0,row),(im.width,row)],fill=(120,0,0)); d.text((2,row),f"{xv:.2f}",fill=(255,255,0))
    for yv in np.arange(-0.35,0.4,0.05):
        col=tr(0,yv)[0]; d.line([(col,0),(col,im.height)],fill=(0,0,140)); d.text((col,2),f"{yv:.2f}",fill=(0,255,255))
    for (x,y,c) in marks:
        a,bb=tr(x,y); d.ellipse([a-5,bb-5,a+5,bb+5],outline=c,width=2)
    return im

def pinch_seq(pos,yaw,t_hover=0.7,z_grip=0.009,z_lift=0.06,h=None):
    """cloth/flat-object pinch: hover z 0.04 -> down to z_grip -> close in-plan -> hold 0.35 s -> lift. ~t_hover+1.1 s."""
    td=(h.top_down if h is not None else rh.top_down)
    ph=td(list(pos)[:2]+[0.04],yaw); pg=td(list(pos)[:2]+[z_grip],yaw); pu=td(list(pos)[:2]+[z_lift],yaw)
    return [rh.waypoint(t_hover,ph,0),rh.waypoint(t_hover+0.3,pg,0),rh.waypoint(t_hover+0.4,pg,1),
            rh.waypoint(t_hover+0.75,pg,1),rh.waypoint(t_hover+1.1,pu,1)]


# ---------- added after e009 (all ran in e009) ----------
def frame_ts(M_deg):
    """object-placement frame for a container whose MOUTH faces azimuth M_deg: returns (t_, s_) unit xy vectors;
    s_ = sweep/push direction into the mouth (az M-180), t_ = along the lip (s_ rotated -90). xy = t*t_ + s*s_"""
    sg=np.radians(M_deg-180); s_=np.array([np.cos(sg),np.sin(sg)]); t_=np.array([np.cos(sg-np.pi/2),np.sin(sg-np.pi/2)])
    return t_,s_

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

def color_blobs(img,mask,uv_box=(0,640,0,480),h=0.015,min_px=10):
    """connected blobs of a colour mask inside a head-image box (u0,u1,v0,v1) -> [(n_px,u,v,xy world via head_backproj at h)]"""
    from scipy import ndimage
    u0,u1,v0,v1=uv_box; m=np.zeros_like(mask); m[v0:v1,u0:u1]=mask[v0:v1,u0:u1]
    lab,n=ndimage.label(m); out=[]
    for j in range(1,n+1):
        vv,uu=np.nonzero(lab==j)
        if len(vv)>=min_px: out.append((len(vv),uu.mean(),vv.mean(),head_backproj(uu.mean(),vv.mean(),h)[:2]))
    return out
# colour masks used (head image, int arrays r,g,b): white r,g,b>200 | magenta r>150,b>100,g<80 | yellow r>200,g>170,b<60 | lime/green g>150,r<170,b<90


# ---------- added after e010 (head_blobs/blob_obb/box_corners/wrist_rect logic ran in e010; place_waypoints packages the e010 place sequence, untested as a function) ----------
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
