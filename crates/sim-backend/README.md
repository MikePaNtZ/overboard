# sim-backend

Implements `hal::BoardIo` against the MuJoCo onewheel model
(`sim/models/overboard_onewheel.xml`). Currently a **stub**: it returns
synthetic all-zero observations with an incrementing synthetic clock, just
enough to exercise the `hal` seam end-to-end. Actually stepping MuJoCo via
its C API (FFI) is the next milestone and is intentionally out of scope for
this scaffold.
