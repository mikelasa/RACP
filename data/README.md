# Data format

What the data looks like after collection (Workflow step 1) and after processing and labelling (steps 2 and 3) in the main [README](../README.md).

Example episodes to download: [raw episode](LINK_TO_RAW_EPISODE) ·
[processed episode](LINK_TO_PROCESSED_EPISODE)

## Raw episode (step 1, `main_teleop`)

One folder per demonstration, written to `data_folder` in
`single_arm_data_collection_franka.yaml` (under `$PYRITE_RAW_DATASET_FOLDERS`).
A 14 s flip-up demo is about 150 MB:

```
episode_<id>/
├── robot_data_0.json    # 1 kHz: robot_time_stamps [ms], ts_pose_fb [x y z qw qx qy qz]
├── wrench_data_0.json   # 1 kHz: wrench_time_stamps [ms], wrench [Fx Fy Fz Tx Ty Tz]
├── joint_data_0.json    # 1 kHz: robot_time_stamps [ms], q (7 joint angles)
├── torque_data_0.json   # 1 kHz: robot_time_stamps [ms], tau_J (7 joint torques)
├── rgb_0/               # camera 0 (wrist), ~30 fps: img_<n>_<timestamp>_ms.jpg
└── rgb_1/               # camera 1 (external D415), ~30 fps
```

The `.json` files use trailing commas, so read them with the processing script rather than
`json.load`. `_0` is the robot id; there is only one robot.

## Processed dataset (steps 2 and 3)

One zarr store per dataset (under `$PYRITE_DATASET_FOLDERS`), about 55 MB per episode:

```
<dataset>/
├── data/
│   └── episode_<id>/
│       ├── rgb_0                       (T_img, 1080, 1080, 3) uint8, JPEG-XL compressed
│       ├── rgb_1                       (T_img, 480, 848, 3)   uint8, JPEG-XL compressed
│       ├── rgb_time_stamps_0, _1       (T_img,)
│       ├── robot_time_stamps_0         (T,)
│       ├── ts_pose_fb_0                (T, 7)    measured pose [x y z qw qx qy qz]
│       ├── ts_pose_command_0           (T, 7)    copy of ts_pose_fb_0
│       ├── wrench_0                    (T_w, 6)  raw wrench
│       ├── wrench_filtered_0           (T_w, 6)  low-pass filtered wrench
│       ├── wrench_time_stamps_0        (T_w,)
│       ├── ts_pose_virtual_target_0    (T, 7)    label, added by step 3
│       ├── stiffness_0                 (T,)      label, added by step 3
│       └── camera0_rgb.mp4, camera1_rgb.mp4   preview videos
└── meta/
    └── episode_rgb0_len, episode_rgb1_len, episode_robot0_len, episode_wrench0_len
                                        (n_episodes,) samples per episode
```
