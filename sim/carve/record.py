"""Record sim-host state packets (wire v3, 104 B) to an .npz pose track."""
import socket, struct, sys, time, numpy as np
port=int(sys.argv[1]); out=sys.argv[2]; secs=float(sys.argv[3])
F=struct.Struct('<IHHQd3f4f5f2f3f3f'); assert F.size==104
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,8<<20)
s.bind(('127.0.0.1',port)); s.settimeout(1.0)
rows=[]; t_end=time.time()+secs; idle=0
while time.time()<t_end:
    try: b,_=s.recvfrom(256)
    except socket.timeout:
        idle+=1
        if rows and idle>=3: break
        continue
    idle=0
    if len(b)==104: rows.append(F.unpack(b))
a=np.array([r[2:] for r in rows],dtype=np.float64)
cols='flags seq t px py pz qw qx qy qz wheel_angle wheel_rate pitch yaw current rider_fa rider_lat vx vy vz wx wy wz'.split()
np.savez(out,**{c:a[:,i] for i,c in enumerate(cols)})
seq=a[:,1]; print(f"{len(a)} packets, dropped {int(seq[-1]-seq[0]+1-len(a))}, t {a[0,2]:.2f}..{a[-1,2]:.2f}s")
