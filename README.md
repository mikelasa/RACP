# RACP

Tested on Ubuntu 24.04 with an RTX 5080 (CUDA 12.8).

## Layout

The following table shows how the framework parts are distributed.

| Folder | What it is | Source |
|---|---|---|
| `racp/` | Training, data processing, inference | this repo |
| `hardware_interfaces/` | `ManipServer`, hardware drivers, `main_teleop` | this repo |
| `controllers/` | Impedance/admittance/position controllers | this repo |
| `libfranka-0.9.2/` | libfranka **0.9.2** (the version the robot uses) | submodule |
| `cpplibrary/` | RobotUtilities (`RUT`) | submodule |
| `yaml-cpp/`, `opencv/`, `librealsense/`, `ur_rtde/` | Third-party C++ libraries | submodules |
| `racp/PyriteML/dual-attention-encoder/`, `racp/PyriteML/multimodal_representation/` | Model components | submodules |
| `eigen-3.4.0/` | Eigen 3.4 sources | downloaded (step 4) |

Submodules are pinned to the exact commits used on the workstation.

All C++ libraries are installed to `~/.local`, so `sudo` is only needed for `apt`.

## 1. Clone

```sh
git clone --recursive https://github.com/mikelasa/RACP.git ~/RACP
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
(`mamba` works the same). `racp/conda_environment.yaml` already leaves PyTorch out
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

## 4. Activate env
```
mamba activate pyrite
```

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

### ur_rtde (if UR robot is used only)

```sh
cd ~/RACP/ur_rtde && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DPYTHON_EXECUTABLE=$(which python3) \
         -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j && make install
```

### OpenCV

```sh
cd ~/RACP/opencv && mkdir -p build && cd build
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

The robot firmware needs **0.9.2**; the submodule is pinned to it.
`hardware_interfaces` includes headers straight from `libfranka-0.9.2/` next to it.

```sh
cd ~/RACP/libfranka-0.9.2 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON \
         -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(($(nproc)-1)) && make install
```

### controllers

Its `CMakeLists.txt` already includes the `cpplibrary`/`yaml-cpp` include directories.

```sh
cd ~/RACP/controllers && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j && make install
```

## 5. hardware_interfaces

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
- `build/applications/manipulation_data_collection/main_teleop`: Teleoperation and data collection interface.
- `workcell/table_top_manip/python/manip_server_pybind*.so`: the Python binding used by inference to connect C++ and python.

Robot configs live in `workcell/table_top_manip/config/`:
- `single_arm_data_collection_franka.yaml`: teleop and ACP/RACP inference.
- `single_arm_DP_franka.yaml`: DP inference.

## 6. Workflow

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
   # for multirun session using hydra sweeped you can add --multirun argument
   ```
   > **Warning:** check `VISUALIZE_ATTENTION` in
   > `racp/PyriteML/diffusion_policy/model/vision/utils/attention_viz.py` before training RACP.
   > If it is `True`, the observation encoder and the action transformer compute and keep their
   > attention weights (forcing PyTorch's slower attention path), and every validation sampling
   > step writes an `attn_weights_<n>.pkl` file to `racp/PyriteML/attn_viz/`. Training slows
   > down and that folder fills up. Set it to `False` for training and turn it on only for
   > inference when you want to plot attention (step 6).
5. **Run on the robot**
   ```sh
   python PyriteEnvSuites/env_runners/inference_ACP_RACP.py --ckpt <checkpoint>
   python PyriteEnvSuites/env_runners/inference_DP.py --ckpt <checkpoint>
   ```
6. **Results**

   **Trained models.** Each training run creates a folder in `$PYRITE_CHECKPOINT_FOLDERS`
   (`~/training_outputs`) named `<date>_<time>_<task>_<name>` (`..._multirun/<n>_...` for sweeps):
   - `checkpoints/`: the best `epoch=XXXX-train_loss=X.XXX.ckpt` files (top 20 by validation
     action error) and `latest.ckpt`
   - `train.log`, `logs.json.txt`: training/validation metrics (also on wandb)
   - `sparse_normalizer.pkl`, `.hydra/`: normalizer and the full config of the run

   `--ckpt` in step 5 takes a path relative to `$PYRITE_CHECKPOINT_FOLDERS`: a run folder
   (uses its `checkpoints/latest.ckpt`) or a specific `.ckpt` file.

   **Inference logs.** Each robot run is logged to `$PYRITE_CONTROL_LOG_FOLDERS`
   (`~/data/inference_log`): `temp_worldframe/` for ACP/RACP and `temp_DP/` for DP, one
   `<date>_<time>_<checkpoint>/` folder per launch with one `episode_<n>.zarr` per episode.
   The impedance controller writes its own log, with the measured wrench, to
   `~/data/controller_log/impedance_controller.log`.

   **Force, trajectory and stiffness plots (ACP/RACP)** with
   `PyriteEnvSuites/env_runners/plot_logs.py`:
   1. Copy `~/data/controller_log/impedance_controller.log` into the episode folder
      (`.../episode_<n>.zarr/`) right after the rollout. Without it the wrench plot is skipped.
   2. Set `TEST_PATH` at the top of the script to that `episode_<n>.zarr` folder. Optionally set
      `FLAG_GROUND_TRUTH = True` with `TRAIN_PATH` pointing at a training episode, or
      `FLAG_COMPARE = True` with `COMPARE_PATH` pointing at another rollout.
   3. Run it:
      ```sh
      python PyriteEnvSuites/env_runners/plot_logs.py
      ```
   It plots the predicted stiffness along each axis, the nominal vs virtual target
   trajectory in 3D, the measured forces/torques and ||F||, and the scalar stiffness (inference,
   post-blend and, with `FLAG_GROUND_TRUTH`, ground truth).

   **Attention plots (RACP only)** with `PyriteEnvSuites/env_runners/plot_attention.py`:
   1. Set `VISUALIZE_ATTENTION = True` in `attention_viz.py` (see the warning in step 4).
   2. Empty or remove `attn_viz/` in the folder you launch inference from (`racp/` with the
      commands above). Each run numbers its files from 0, so leftovers from a longer earlier
      run would get mixed in.
   3. Run inference (step 5). Every control step writes `attn_viz/attn_weights_<n>.pkl`.
   4. Plot the image vs force attention mass and the wrench magnitude over the rollout:
      ```sh
      python PyriteEnvSuites/env_runners/plot_attention.py attn_viz
      ```
      It saves `attn_viz/attn_force_correlation.png` and `figs/attention/attention.pdf`
      (next to `attn_viz/`). Without an argument it reads
      `PyriteEnvSuites/env_runners/attn_viz/`.
   5. Set `VISUALIZE_ATTENTION` back to `False` before training again.

See `racp/README.md` for more on the method, data format and training options.

When you change C++ code in `hardware_interfaces` or `controllers`, run `sudo make install` again

## 7. Data format

See [`data/README.md`](data/README.md) for what raw and processed episodes look like, with example
episodes to download.

## Credits

RACP builds on code by Yifan Hou (Stanford):
- `racp/` started from [adaptive_compliance_policy](https://github.com/yifan-hou/adaptive_compliance_policy) (MIT, see `racp/LICENSE`).
- `hardware_interfaces/` started from [hardware_interfaces](https://github.com/yifan-hou/hardware_interfaces).
- `controllers/` started from [force_control](https://github.com/yifan-hou/force_control) (MIT, see `controllers/LICENSE`).

Their full commit history is kept in this repo.
