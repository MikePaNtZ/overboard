# Notebooks

Four notebooks that explain the maths behind the controls work and tell the
story of getting it wrong along the way.

| | |
|---|---|
| `01-equations-of-motion.ipynb` | What kind of plant this is, and why `mgl`'s sign decides everything |
| `02-closed-loop-control.ipynb` | Inner loop, outer loop, and the traps between them |
| `03-attitude-estimation.ipynb` | Why the accelerometer lies, and what to do about it |
| `04-estimator-in-the-loop.ipynb` | Bode plots: why an accurate estimator still crashed the board, and the fix |

## Read them without cloning

GitHub renders `.ipynb` natively, so these are readable in the browser with all
figures:

- [01 · Equations of motion](https://github.com/MikePaNtZ/overboard/blob/master/notebooks/01-equations-of-motion.ipynb)
- [02 · Closed-loop control](https://github.com/MikePaNtZ/overboard/blob/master/notebooks/02-closed-loop-control.ipynb)
- [03 · Attitude estimation](https://github.com/MikePaNtZ/overboard/blob/master/notebooks/03-attitude-estimation.ipynb)
- [04 · Estimator in the loop](https://github.com/MikePaNtZ/overboard/blob/master/notebooks/04-estimator-in-the-loop.ipynb)

**They are committed with their outputs.** That is a deliberate exception to the
usual "strip outputs before committing" habit: the whole point of these is to be
*read*, and an unexecuted notebook renders on GitHub as code with no figures at
all — which is what notebooks 1–3 did until 2026-07-26. The cost is a noisier
diff; the benefit is that the analysis is legible to someone who will never run
it. Re-execute before committing a change, or the prose and the plots drift
apart.

## They load data, they do not generate it

Each notebook reads archived datasets from `sim/out/experiments/`. Produce them
first:

```sh
.venv/bin/python scripts/analyse_control.py     # notebooks 1 and 2
.venv/bin/python scripts/analyse_estimator.py   # notebook 3
.venv/bin/python scripts/analyse_estimator_phase.py  # notebook 4
```

Keeping generation in scripts rather than in the notebooks is deliberate: the
scripts run in CI's environment, they diff cleanly in review, and a notebook
that re-runs physics on every open is a notebook nobody opens.

## On the provenance of the numbers

Notebook 4 is **generated** from `scripts/make_notebook_04.py` rather than
hand-edited, then executed against the archived data. A notebook is JSON with
embedded outputs, and hand-merging one is how you get a file that opens but does
not run; keeping the prose in a diffable `.py` avoids that.

Notebooks 3 and 4's datasets were captured **during** the estimator work. Notebooks 1
and 2 read datasets **regenerated afterwards** — the plant and control sweeps
originally ran in throwaway scripts and only their conclusions were written
down.

That regeneration is legitimate rather than a reconstruction: every scenario is
deterministic and seeded, so re-running a configuration reproduces it
bit-for-bit. They are the same numbers the decisions were made on. Each
notebook says which it is, because the distinction matters.

## Running them

```sh
.venv/bin/pip install -r requirements-sim.txt
.venv/bin/jupyter lab notebooks/
```

Nothing in CI depends on them.
