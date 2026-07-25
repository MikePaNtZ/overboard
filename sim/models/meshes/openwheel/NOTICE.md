# Attribution

The STL shells in this directory (`front_enclosure.stl`, `rear_enclosure.stl`,
`front_footpad.stl`, `rear_footpad.stl`, `front_bumper.stl`, `rear_bumper.stl`,
`electronics_platform.stl`) are the non-print-split, full versions imported
from:

**Openwheel** by Byte Sized Engineering -- https://github.com/bytesizedengineering/Openwheel

Licensed **MIT** -- see `LICENSE` in this directory (verbatim copy of the
upstream repo's `LICENSE` file).

## README/LICENSE discrepancy

The upstream repo's `README.md` states the project is licensed under **GPLv3**,
but the actual `LICENSE` file committed to the repo is the **MIT License**
(copyright (c) 2021 byte sized). The LICENSE file is the authoritative legal
text for a GitHub repo, so these meshes are used and redistributed here under
the MIT terms as written in that file. If Byte Sized Engineering later
clarifies/corrects this discrepancy upstream, revisit this attribution.

## What was NOT imported

- The print-split "A"/"B" variants of the front/rear enclosures (we use the
  single non-split STL for each).
- Openwheel has no tire mesh (their build reuses a hoverboard hub tire); the
  tire in `overboard_onewheel.xml` is a hand-authored MuJoCo primitive
  cylinder, not derived from Openwheel.
