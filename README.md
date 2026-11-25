# RACP

Setup guide for the full Adaptive Compliance Policy stack on the Franka workstation:
Python training/inference (`racp`) plus the C++ robot side
(`hardware_interfaces`, `controllers` and their dependencies).

Tested on Ubuntu 24.04 with an RTX 5080 (CUDA 12.8).

## Layout

The paths below assume the workspace is at `~/RACP`. The code itself only relies on the
folder layout: `hardware_interfaces` finds `libfranka-0.9.2` next to it, and the Python code
imports `hardware_interfaces` from the folder next to `racp`.

| Folder | What it is | Source |
|---|---|---|
| `racp/` | Training, data processing, inference (branch `dev`) | submodule, `mikelasa` fork |
| `hardware_interfaces/` | `ManipServer`, hardware drivers, `main_teleop` (branch `v1.0`) | submodule, `mikelasa` fork |
| `controllers/` | Impedance/admittance controllers (branch `mainline`) | submodule, `mikelasa` fork |
| `cpplibrary/` | RobotUtilities (`RUT`) | submodule |
| `yaml-cpp/`, `opencv/`, `librealsense/`, `ur_rtde/` | Third-party C++ libraries | submodules |
| `eigen-3.4.0/` | Eigen 3.4 sources | downloaded |
| `libfranka-0.9.2/` | libfranka **0.9.2** (the version the robot uses) | cloned separately |

All C++ libraries are installed to `~/.local`, so `sudo` is only needed for `apt`.

## 1. Clone

```sh
cd ~
git clone --recursive --branch master git@github.com:mikelasa/adaptive_compliance_policy.git ACP
cd ACP
git -C adaptive_compliance_policy checkout dev
git -C hardware_interfaces checkout v1.0
git -C controllers checkout mainline
```

## 2. System packages and `~/.local`

```sh
sudo apt update
sudo apt install -y build-essential cmake git wget unzip pkg-config \
    libpoco-dev libeigen3-dev libhidapi-dev \
    libssl-dev libusb-1.0-0-dev libudev-dev libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at v4l-utils

mkdir -p ~/.local/bin ~/.local/include ~/.local/lib
```

Add to `~/.bashrc`:

```sh
# Home installations
export PATH=$HOME/.local/bin:$PATH
export C_INCLUDE_PATH=$HOME/.local/include/:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=$HOME/.local/include/:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH

# Pyrite folders
export PYRITE_RAW_DATASET_FOLDERS=$HOME/data/real               # raw recordings
export PYRITE_DATASET_FOLDERS=$HOME/data/real_processed         # processed zarr datasets
export PYRITE_CHECKPOINT_FOLDERS=$HOME/training_outputs         # one folder per training run
export PYRITE_HARDWARE_CONFIG_FOLDERS=$HOME/RACP/hardware_interfaces/workcell/table_top_manip/config
export PYRITE_CONTROL_LOG_FOLDERS=$HOME/data/inference_log      # inference logs
```

## 3. Python environment (`pyrite`)

Uses [micromamba](https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html)
(`mamba` works the same). `conda_environment.yaml` in the fork already leaves PyTorch out
and pins `zarr<3` and `numcodecs<0.13`, so no edits are needed.

```sh
cd ~/RACP/racp
micromamba env create -f conda_environment.yaml
micromamba activate pyrite

# PyTorch with CUDA 12.8 (required for RTX 50xx; PyTorch >= 2.7)
pip install --upgrade --index-url https://download.pytorch.org/whl/cu128 torch torchvision torchaudio

pip install v4l2py toppra atomics
pip install vit-pytorch    # needs >= 1.7.12, not available in conda
pip install imagecodecs    # needs >= 2023.9.18, conflicts in conda

# Dual-attention encoder used by the models
pip install -e PyriteML/dual-attention-encoder
```

Make the env's own libraries visible to the C++ code when it is active:

```sh
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_LIBRARY_PATH=$CONDA_PREFIX/lib/:$LD_LIBRARY_PATH' > $CONDA_PREFIX/etc/conda/activate.d/linker.sh
```

## 4. C++ dependencies

Keep `pyrite` activated for every build below. Each block starts from `~/RACP`.

### yaml-cpp (must be C++17)

```sh
cd ~/RACP/yaml-cpp && mkdir -p build && cd build
cmake .. -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX=$HOME/.local
make -j && make install
```

### Eigen 3.4

```sh
cd ~/RACP
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar xzf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local
make -j && make install
```

### cpplibrary (RobotUtilities)

```sh
cd ~/RACP/cpplibrary && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_CXX_FLAGS="-I$HOME/.local/include"
make -j && make install
```

### ur_rtde

```sh
cd ~/RACP/ur_rtde && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DPYTHON_EXECUTABLE=$(which python3) \
         -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j && make install
```

### OpenCV 4.12

```sh
cd ~/RACP/opencv && git checkout 4.12.0
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local \
         -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
         -DWITH_PNG=OFF -DBUILD_PNG=OFF
make -j$(($(nproc)-1)) && make install
```

If VS Code IntelliSense can't find the OpenCV headers afterwards, refresh its include paths.

### librealsense

```sh
cd ~/RACP/librealsense
./scripts/setup_udev_rules.sh
# Only on Ubuntu 20/22/24 with an LTS kernel (5.15, 5.19, 6.5):
./scripts/patch-realsense-ubuntu-lts-hwe.sh
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_EXAMPLES=true
make -j$(($(nproc)-1)) && make install
```

### libfranka 0.9.2

The robot firmware needs **0.9.2**, cloned into its own folder (not the `libfranka` submodule).
`hardware_interfaces` includes headers straight from `libfranka-0.9.2/` next to it.

```sh
cd ~/RACP
git clone --branch 0.9.2 --recursive https://github.com/frankaemika/libfranka libfranka-0.9.2
cd libfranka-0.9.2 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON \
         -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(($(nproc)-1)) && make install
```

### controllers

The fork already includes the `cpplibrary`/`yaml-cpp` include directories in its `CMakeLists.txt`.

```sh
cd ~/RACP/controllers && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j && make install
```

## 5. hardware_interfaces

The fork already has the fixes from the upstream repo applied (`TaskSpaceInterpolationController`
in `manip_server_loops.cc`, libfranka 0.9.2 include paths in the CMake files).

```sh
cd ~/RACP/hardware_interfaces && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local \
         -DCMAKE_PREFIX_PATH="$CONDA_PREFIX;$HOME/.local" \
         -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j && make install
```

This produces:
- `build/applications/manipulation_data_collection/main_teleop`: SpaceMouse teleoperation and data collection.
- `workcell/table_top_manip/python/manip_server_pybind*.so`: the Python binding used by inference.

Robot configs live in `workcell/table_top_manip/config/`:
- `single_arm_data_collection_franka.yaml`: teleop and ACP/RACP inference.
- `single_arm_DP_franka.yaml`: DP inference.

## 6. Workflow

Activate `pyrite` first. Python commands run from `~/RACP/racp` unless noted.

1. **Collect data**
   ```sh
   ~/RACP/hardware_interfaces/build/applications/manipulation_data_collection/main_teleop
   ```
2. **Process raw data into zarr.** Set `input_dir`/`output_dir` at the top of the script, then:
   ```sh
   python PyriteUtility/data_pipeline/real_data_processing.py
   ```
3. **Add virtual-target/stiffness labels.** Set `dataset_path` and the ACP/RACP settings described in the script's docstring, then:
   ```sh
   python PyriteEnvSuites/scripts/postprocess_dataset.py
   ```
4. **Train**
   ```sh
   cd PyriteML
   HYDRA_FULL_ERROR=1 accelerate launch train.py --config-name=train_ACP.yaml
   # or train_RACP.yaml, train_RACP_dual_camera.yaml, train_DP.yaml
   ```
5. **Run on the robot**
   ```sh
   python PyriteEnvSuites/env_runners/inference_ACP_RACP.py --ckpt <checkpoint>
   python PyriteEnvSuites/env_runners/inference_DP.py --ckpt <checkpoint>
   ```

See `racp/README.md` for more on the method, data format and training options.
