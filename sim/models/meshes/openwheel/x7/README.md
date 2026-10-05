# X7 build meshes (proxy)

Per-material meshes `<body>__<material>.stl` (body = frame or wheel) and
`look.json`: the outside surfaces and the one board look of Mike's build
(Fungineers X7 rails, Superflux HT 6 in, Thunder 11.5 in tyre).

PROXY geometry: sized from vendor numbers and photos by the hardware track
(no vendor CAD exists); any dimension can be off by 1-2 cm. Copied from
`overboard-viz-kit` (`out/kit-guide/models/by_material/`, the same scene as
`overboard_x7_exterior_dark.glb`, branch feat/content/kit-assembly-concept)
by `sim/carve/x7_look.py`, which also writes `x7_visual.xml`. Re-run it when
the hardware track re-exports the look (the grip and fender colours are
provisional).

Units metres. Z up, +X = nose, +Y = right rail. Origin = the ground point
under the axle (axle at z = 0.146). sim-host `--plant x7` places them with
pos (0, 0, -0.146) and a 180 deg turn about Z, because the model's forward is
-X. Visual only; the collision pads are boxes from the same proxy (the end boxes
kicked 4.2 deg with the deck: the deck strikes at 20.4 deg).
