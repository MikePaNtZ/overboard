"""Is there a quasi-steady window where residual/unaided-specific-force settles?"""
import csv, math
from pathlib import Path
OUT = Path("/private/tmp/claude-501/-Users-mike-projects-overboard/94162fbe-f42c-4c5a-98c3-d48ab1fac1dc/scratchpad")
ACCEL_FF_GAIN = 0.0584
ONE = math.degrees(1.0) / 9.80665

for tag in ("full-stick_reserve0.6", "full-stick_reserve0.8", "stick-reversal_shipped"):
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader((OUT / f"{tag}.csv").open())]
    half = 50
    print(f"\n=== {tag} ===")
    print(f"{'t':>6} {'speed':>7} {'a_unaid':>8} {'resid':>7} {'ratio':>7} {'pred':>7}")
    for i in range(half, len(rows) - half, 250):
        t = rows[i]["sim_time_s"]
        if t > 22: break
        a = (rows[i+half]["forward_speed_m_s"] - rows[i-half]["forward_speed_m_s"]) / (
            rows[i+half]["sim_time_s"] - rows[i-half]["sim_time_s"])
        au = a - ACCEL_FF_GAIN * rows[i]["applied_amps"]
        res = rows[i]["est_pitch_deg"] - rows[i]["truth_pitch_deg"]
        pred = math.degrees(math.atan(au / 9.80665))
        print(f"{t:6.2f} {rows[i]['forward_speed_m_s']:7.3f} {au:8.4f} {res:7.4f} "
              f"{res/au if abs(au)>1e-3 else float('nan'):7.3f} {pred:7.4f}")
