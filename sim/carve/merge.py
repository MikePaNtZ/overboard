import numpy as np, json, struct
from scipy import ndimage as nd
A=np.load('/tmp/carve_terrain/carve_height.npy'); HA=np.load('/tmp/carve_terrain/holes.npy')
B=np.load('/tmp/carve_terrain_big/carve_height.npy'); HB=np.load('/tmp/carve_terrain_big/holes.npy')
d=A-B; sub=(~HB)&(d>0)&(d<0.03)
F=np.where(sub,B,A).astype(np.float32)
print('posts replaced %.2f%%'%(sub.mean()*100))
open('/tmp/carve_terrain_v2/carve_hfield.bin','wb').write(struct.pack('<ii',*F.shape)+F.astype('<f4').tobytes())
np.save('/tmp/carve_terrain_v2/carve_height.npy',F); np.save('/tmp/carve_terrain_v2/holes.npy',HA)
m=json.load(open('/tmp/carve_terrain/metadata.json')); m['z_min_m']=float(F.min()); m['z_max_m']=float(F.max())
m['overlay_removal']="posts where the full max-Z raster is 0-3 cm above a raster of WIDE triangles (2*area/longest edge >= 30 cm) take the base value: lane paint and edge blocks are modelled as flush, kerbs are kept"
json.dump(m,open('/tmp/carve_terrain_v2/metadata.json','w'),indent=1)
c=1500; print('centre post', F[c,c])
hh=F[c-240:c+240,c-1440:c+80]
g=np.zeros_like(hh); g[:,1:]=np.abs(np.diff(hh,axis=1)); g[1:,:]=np.maximum(g[1:,:],np.abs(np.diff(hh,axis=0)))
step=g-nd.median_filter(g,9); lane=step[240-70:240+60]
print('lane |y|<3.5 posts with >3 mm step: %.3f%%, >1 mm %.3f%%, max step %.1f mm'%((lane>0.003).mean()*100,(lane>0.001).mean()*100, lane.max()*1000))
