"""Load the real Rust control law and expose it as a scenario controller hook.

The point of this module is that the sim exercises the SAME control code that
will eventually run on the Pi -- `control_core::PitchRegulator` behind
`safety::Envelope` -- rather than a Python reimplementation that would have to
be kept in step with it.

The physics calls the controller here, which is the reverse of the hardware
arrangement (where `board-app` owns the loop and calls `BoardActuate`). That is
deliberate and cheap to undo: Python owns MuJoCo, the collision geometry, the
determinism and the metrics, and re-implementing all of that in Rust would buy
no control knowledge. Only the direction of the call differs; the law and the
envelope are the same objects either way.

ctypes rather than PyO3/maturin: no build integration, no interpreter coupling,
and the crate stays linkable from the hardware binary unchanged. Call overhead
is ~1-2 us, so a 4000-step run spends single-digit milliseconds in the FFI.
"""

from __future__ import annotations

import ctypes
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

#: Default gains, AMPS PER RADIAN of nose-up-positive pitch (ICD 10.1).
#:
#: Chosen analytically, not tuned: with effective pitch inertia ~0.403 kg*m^2,
#: restoring stiffness m*g*l = 2.354 N*m/rad and kt = 0.7 N*m/A, these place the
#: closed-loop pole pair at omega ~= 12 rad/s with zeta ~= 0.8 -- about 5x the
#: plant's own 2.42 rad/s. They are a starting point for a sweep, not a result.
DEFAULT_KP_A_PER_RAD = 80.0
DEFAULT_KD_A_PER_RAD_S = 11.0

#: Envelope clamp, amps. Matches the model's derived ctrlrange (40 A * kt).
DEFAULT_MAX_CURRENT_A = 40.0

#: Loaded rolling radius, m (ICD 10.5). Stage-0 will measure it.
DEFAULT_R_EFF_M = 0.14605

#: Outer-loop clamp: the most lean it may ask the inner loop to hold.
DEFAULT_MAX_PITCH_REF_RAD = 0.087   # 5 degrees

_OK = 0
_ERR = {
    -1: "null pointer",
    -2: "struct size mismatch -- the header and the library disagree",
    -3: "non-finite pitch or rate reached the controller",
}


class ObParams(ctypes.Structure):
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("kp_a_per_rad", ctypes.c_float),
        ("kd_a_per_rad_s", ctypes.c_float),
        ("max_current_a", ctypes.c_float),
        ("kp_v_rad_per_m_s", ctypes.c_float),
        ("ki_v_rad_per_m", ctypes.c_float),
        ("max_pitch_ref_rad", ctypes.c_float),
        ("v_ref_m_s", ctypes.c_float),
        ("r_eff_m", ctypes.c_float),
        ("com_above_axle", ctypes.c_uint32),
    ]


class ObObs(ctypes.Structure):
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("t_ns", ctypes.c_uint64),
        ("pitch_rad", ctypes.c_float),
        ("pitch_rate_rad_s", ctypes.c_float),
        ("wheel_rate_rad_s", ctypes.c_float),
        ("motor_current_a", ctypes.c_float),
    ]


class ObCmd(ctypes.Structure):
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("amps", ctypes.c_float),
        ("saturated", ctypes.c_uint32),
        ("pitch_ref_rad", ctypes.c_float),
    ]


def _library_filename() -> str:
    """What cargo names a cdylib on this platform.

    Deliberately NOT `sysconfig.get_config_var("SHLIB_SUFFIX")`: on macOS that
    reports `.so`, because it describes Python extension modules, while cargo
    emits `.dylib`. Asking Python what Rust called its output is the wrong
    question, and it silently produces a path that never exists.
    """
    if sys.platform == "darwin":
        return "libcontrol_ffi.dylib"
    if sys.platform == "win32":
        return "control_ffi.dll"
    return "libcontrol_ffi.so"


def library_path() -> Path | None:
    """Newest built `control-ffi` shared library, or None if it is not built."""
    stem = _library_filename()
    found = [
        p
        for p in (REPO / "target" / prof / stem for prof in ("release", "debug"))
        if p.exists()
    ]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


def build(release: bool = True) -> Path:
    """Build the cdylib and return its path. Raises on a build failure."""
    cmd = ["cargo", "build", "-p", "control-ffi"] + (["--release"] if release else [])
    subprocess.run(cmd, cwd=REPO, check=True)
    path = library_path()
    if path is None:
        raise RuntimeError(f"{' '.join(cmd)} succeeded but produced no shared library")
    return path


class RustController:
    """The Rust control law, as a callable matching the scenario hook.

    Call signature is the scenario's: ``(t_s, pitch_rad, pitch_rate_rad_s,
    wheel_rate_rad_s) -> amps``. Radians, nose-up-positive, amps out.
    """

    def __init__(
        self,
        kp_a_per_rad: float = DEFAULT_KP_A_PER_RAD,
        kd_a_per_rad_s: float = DEFAULT_KD_A_PER_RAD_S,
        max_current_a: float = DEFAULT_MAX_CURRENT_A,
        kp_v_rad_per_m_s: float = 0.0,
        ki_v_rad_per_m: float = 0.0,
        max_pitch_ref_rad: float = DEFAULT_MAX_PITCH_REF_RAD,
        v_ref_m_s: float = 0.0,
        r_eff_m: float = DEFAULT_R_EFF_M,
        com_above_axle: bool = True,
        lib_path: Path | None = None,
    ) -> None:
        path = lib_path or library_path()
        if path is None:
            # Deliberately an error, never a skip. A closed-loop test that
            # quietly passes because the controller was absent is precisely the
            # "green build that proves nothing" failure this project has
            # already hit twice in CI.
            raise FileNotFoundError(
                "control-ffi shared library not found. Build it first:\n"
                "    cargo build --release -p control-ffi"
            )

        self._lib = lib = ctypes.CDLL(str(path))
        self.lib_path = path

        lib.ob_abi_version.restype = ctypes.c_uint32
        lib.ob_controller_new.argtypes = [ctypes.POINTER(ObParams)]
        lib.ob_controller_new.restype = ctypes.c_void_p
        lib.ob_controller_arm.argtypes = [ctypes.c_void_p]
        lib.ob_controller_arm.restype = ctypes.c_int32
        lib.ob_controller_free.argtypes = [ctypes.c_void_p]
        lib.ob_controller_free.restype = None
        lib.ob_controller_update.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ObObs),
            ctypes.POINTER(ObCmd),
        ]
        lib.ob_controller_update.restype = ctypes.c_int32

        self.abi_version = int(lib.ob_abi_version())

        params = ObParams(
            size=ctypes.sizeof(ObParams),
            kp_a_per_rad=kp_a_per_rad,
            kd_a_per_rad_s=kd_a_per_rad_s,
            max_current_a=max_current_a,
            kp_v_rad_per_m_s=kp_v_rad_per_m_s,
            ki_v_rad_per_m=ki_v_rad_per_m,
            max_pitch_ref_rad=max_pitch_ref_rad,
            v_ref_m_s=v_ref_m_s,
            r_eff_m=r_eff_m,
            com_above_axle=1 if com_above_axle else 0,
        )
        self._handle = lib.ob_controller_new(ctypes.byref(params))
        if not self._handle:
            raise RuntimeError("ob_controller_new rejected the parameters")

        if lib.ob_controller_arm(self._handle) != _OK:
            raise RuntimeError("ob_controller_arm failed")

        # Reused across the run so a 4000-step scenario does not allocate two
        # structs per step; the FFI copies out of them synchronously.
        self._obs = ObObs(size=ctypes.sizeof(ObObs))
        self._cmd = ObCmd(size=ctypes.sizeof(ObCmd))

        #: Cycles on which the envelope clamped. Anti-windup will need this;
        #: for now it is evidence about whether the gains have any headroom.
        self.saturated_cycles = 0
        self.cycles = 0
        #: Last pitch reference the outer loop asked for, radians.
        self.pitch_ref_rad = 0.0
        #: Largest magnitude it ever asked for -- shows whether the outer loop
        #: is living against its clamp.
        self.peak_abs_pitch_ref_rad = 0.0

    def __call__(
        self,
        t_s: float,
        pitch_rad: float,
        pitch_rate_rad_s: float,
        wheel_rate_rad_s: float,
    ) -> float:
        o, c = self._obs, self._cmd
        o.t_ns = int(t_s * 1e9)
        o.pitch_rad = pitch_rad
        o.pitch_rate_rad_s = pitch_rate_rad_s
        o.wheel_rate_rad_s = wheel_rate_rad_s

        rc = self._lib.ob_controller_update(self._handle, ctypes.byref(o), ctypes.byref(c))
        if rc != _OK:
            raise RuntimeError(f"ob_controller_update failed: {_ERR.get(rc, rc)}")

        self.cycles += 1
        if c.saturated == 1:
            self.saturated_cycles += 1
        self.pitch_ref_rad = float(c.pitch_ref_rad)
        self.peak_abs_pitch_ref_rad = max(self.peak_abs_pitch_ref_rad,
                                          abs(self.pitch_ref_rad))
        return float(c.amps)

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.ob_controller_free(self._handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()

    def __enter__(self) -> "RustController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
