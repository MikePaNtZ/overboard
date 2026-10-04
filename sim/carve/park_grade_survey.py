import numpy as np
from scipy import ndimage as nd
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
S=0.25; x0,y0=-915,-465
h=np.load('/tmp/parkprobe/park_h.npy'); f=np.isfinite(h)
# fill small gaps between splats, then reject big holes
w=f.astype(float); hz=np.where(f,h,0)
k=5; num=nd.uniform_filter(hz,k); den=nd.uniform_filter(w,k)
hf=np.where(den>0.3,num/np.maximum(den,1e-9),np.nan); cov=den>0.3
# local step check: max-min in 1 m window over raw data
hmax=nd.maximum_filter(np.where(f,h,-1e9),5); hmin=nd.minimum_filter(np.where(f,h,1e9),5)
smooth=(hmax-hmin)<0.12   # allows 6-8% over 1.25 m (0.1 m) but not kerbs or steps
# slope over a 2 m baseline
k2=9; num=nd.uniform_filter(np.nan_to_num(hf),k2); den=nd.uniform_filter(cov.astype(float),k2)
hs=num/np.maximum(den,1e-9)
gy,gx=np.gradient(hs,S); g=np.hypot(gx,gy)*100
valid=cov&smooth&(den>0.95)
band=valid&(g>=5.5)&(g<=8.5)
lab,n=nd.label(nd.binary_closing(band,iterations=2)&valid)
objs=nd.find_objects(lab); rows=[]
for i,o in enumerate(objs):
    m=lab[o]==i+1; area=m.sum()*S*S
    if area<15: continue
    yy,xx=np.nonzero(m); yy=yy+o[0].start; xx=xx+o[1].start
    dx,dy=gx[yy,xx].mean(),gy[yy,xx].mean(); d=np.array([dx,dy]); d/=np.linalg.norm(d)
    proj=(xx*S)*d[0]+(yy*S)*d[1]; L=np.ptp(proj)
    z=hs[yy,xx]
    rows.append((area,L,np.ptp(z),g[yy,xx].mean(),x0+xx.mean()*S,y0+yy.mean()*S,np.degrees(np.arctan2(-d[1],-d[0]))))
rows.sort(key=lambda r:-r[1])
print("area_m2 fall_line_len_m drop_m grade% cx cy downhill_dir_deg")
for r in rows[:15]: print("%7.0f %6.1f %6.2f %5.1f %7.1f %7.1f %6.0f"%r)
np.save('/tmp/parkprobe/rows.npy',np.array(rows))
fig,ax=plt.subplots(figsize=(14,11.5),dpi=110)
ext=[x0,x0+h.shape[1]*S,y0,y0+h.shape[0]*S]
ls=matplotlib.colors.LightSource(315,45)
base=np.where(cov,np.nan_to_num(hf),np.nan)
ax.imshow(np.where(cov,hf,np.nan),origin='lower',extent=ext,cmap='Greys_r',alpha=0.9)
gg=np.where(valid,g,np.nan)
im=ax.imshow(np.where(valid&(g>=3),gg,np.nan),origin='lower',extent=ext,cmap='turbo',vmin=0,vmax=12)
plt.colorbar(im,ax=ax,label='grade % (smooth surfaces, >=3%)',shrink=0.7)
for j,r in enumerate(rows[:8]): ax.annotate(str(j+1),(r[4],r[5]),color='white',fontsize=13,weight='bold',bbox=dict(fc='k',alpha=0.6))
ax.plot(-38.8,-74.5,'r*',ms=16); ax.annotate('spawn',(-38.8,-74.5),color='r',xytext=(5,5),textcoords='offset points')
ax.set_title('City Park hard surfaces: grade map (UE world, m). Numbered = 6-8% candidates, longest fall line first')
ax.set_xlabel('UE X (m)'); ax.set_ylabel('UE Y (m)')
fig.savefig('/tmp/parkprobe/park_grade_map.png',bbox_inches='tight')
