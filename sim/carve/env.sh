export PATH="$HOME/.cargo/bin:/usr/sbin:/sbin:$PATH"
export PY=/Users/mike/projects/overboard/.venv/bin/python
export MUJOCO_DIR=$($PY -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))")
export DYLD_FRAMEWORK_PATH="$MUJOCO_DIR/.dylibs:$MUJOCO_DIR"
export DYLD_LIBRARY_PATH="$MUJOCO_DIR"
export SIMHOST=/Users/mike/projects/overboard-carve/target/release/sim-host
export DYLD_LIBRARY_PATH=$(ls -d /Users/mike/projects/overboard-carve/target/release/build/plant-mujoco-*/out | head -1)
