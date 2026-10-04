import sys, json, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
sys.path.insert(0,'/tmp/carve')
run=sys.argv[1]; terr=sys.argv[2] if len(sys.argv)>2 else '/tmp/carve_terrain_v2'
d=np.load(f'/tmp/carve/{run}.npz'); h=np.load(f'{terr}/carve_height.npy'); S=0.05; c=1500
lane=np.array(json.load(open('/tmp/carve/lane.json')))
t=d['t']; sp=np.hypot(d['vx'],d['vy']); fell=(d['flags'].astype(int)&4)>0
lo=np.interp(d['px'],lane[::-1,0],lane[::-1,1]); hi=np.interp(d['px'],lane[::-1,0],lane[::-1,2])
margin=np.minimum(d['py']-lo,hi-d['py'])
print(f"{run}: fell={'%.2fs'%t[fell][0] if fell.any() else 'never'} max speed {sp.max():.2f} end x {d['px'][-1]:.1f} min lane margin {margin[d['px']>-70].min():.2f} m pitch {np.degrees(d['pitch']).min():.1f}..{np.degrees(d['pitch']).max():.1f}")
fig,ax=plt.subplots(2,1,figsize=(15,9),dpi=100,gridspec_kw={'height_ratios':[1.3,1]})
sl=(slice(c-int(6/S),c+int(12/S)),slice(c-int(74/S),c+int(4/S)))
ax[0].imshow(h[sl],origin='lower',extent=[-74,4,-6,12],cmap='Greys_r')
ax[0].fill_between(lane[:,0],lane[:,1],lane[:,2],color='tab:green',alpha=0.15,label='clean lane')
ax[0].plot(d['px'],d['py'],'r-',lw=2,label='board path'); ax[0].legend(loc='lower left'); ax[0].set_aspect('equal'); ax[0].set_title(f'{run}: path in lane (MuJoCo frame, downhill = -x)')
ax[1].plot(t,sp,label='speed m/s'); ax[1].plot(t,np.degrees(d['pitch']),label='pitch deg'); ax[1].plot(t,d['current']/10,label='current /10 A'); ax[1].plot(t,np.degrees(np.unwrap(d['yaw']))/10,label='heading /10 deg')
ax[1].legend(); ax[1].grid(alpha=.3); ax[1].set_xlabel('sim time s')
fig.tight_layout(); fig.savefig(f'/tmp/carve/{run}_analysis.png')
