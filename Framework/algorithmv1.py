#!/usr/bin/env python3

"""
Author: Christiaan Peppelman
Reviewed by: Fardien Azizi
Created: 12 may 2026

This algorithm finds the newest bag file, peforms a data quality check, synchronizes the data and then converts it into te zarr file and bag file.
The bag file conversion is solely for validation via Rviz and can be omitted.
To better suit your specifiec application the thressholds for the blurr and brightness in the data quality check can be modified.
The outputs of each session will be stored in a seperate session folder.

AI contribution: LLM were used in development of this algorith. All outputs were thoroughly reviewd before implementation.
"""

import os
import glob
import json
import sys
from datetime import datetime

import cv2
import numpy as np
import zarr
import rosbag
from sensor_msgs.msg import CompressedImage, JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import rospy



CAMERA_TOPIC    = "/camera/color/image_raw/compressed"
JOINT_TOPIC     = "/dingo1/dinova/joint_states"
TF_TOPIC        = "/tf"
TF_STATIC_TOPIC = "/tf_static"

# Main data folder
BAG_DIR = os.path.expanduser("~/ros_data")

BLUR_THRESHOLD       = 30.0
BRIGHTNESS_THRESHOLD = 40.0

SAVE_REJECTED_IMAGES = True



def find_newest_bag() -> str:
    """
    Fallback function for manual use.
    Searches recursively in ~/ros_data for the newest .bag file.
    Replay bags are ignored.
    """
    bags = glob.glob(os.path.join(BAG_DIR, "**", "*.bag"), recursive=True)

    
    bags = [b for b in bags if not b.endswith("_replay.bag")]

    if not bags:
        raise FileNotFoundError(f"No .bag files found in: {BAG_DIR}")

    return max(bags, key=os.path.getctime)


def get_session_dir_from_bag_path(bag_path: str) -> str:
    """
    Expected structure:

    ~/ros_data/session_xxx/bag/session_xxx.bag

    This returns:

    ~/ros_data/session_xxx

    Also works if the bag is not inside a bag/ folder.
    """
    bag_folder = os.path.dirname(bag_path)
    parent_name = os.path.basename(bag_folder)

    if parent_name in ["bag", "raw"]:
        return os.path.dirname(bag_folder)

    return bag_folder


def setup_rejected_folders(bag_path: str) -> dict:
    """
    Store rejected images inside the session folder:

    session_xxx/rejected_images/blurry
    session_xxx/rejected_images/dark
    session_xxx/rejected_images/blurry_and_dark
    """
    session_dir = get_session_dir_from_bag_path(bag_path)
    base = os.path.join(session_dir, "rejected_images")

    folders = {
        "blurry":      os.path.join(base, "blurry"),
        "dark":        os.path.join(base, "dark"),
        "blurry_dark": os.path.join(base, "blurry_and_dark"),
    }

    for path in folders.values():
        os.makedirs(path, exist_ok=True)

    return folders


def assess_image_quality(image_bgr: np.ndarray) -> dict:
    """Return blur score, brightness, and quality flags for one frame."""
    gray       = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    small_gray = cv2.resize(gray, (320, 240))

    blur_score = float(cv2.Laplacian(small_gray, cv2.CV_32F).var())
    brightness = float(gray.mean())

    is_blurry = blur_score < BLUR_THRESHOLD
    is_dark   = brightness < BRIGHTNESS_THRESHOLD

    return {
        "blur_score": blur_score,
        "brightness": brightness,
        "is_blurry": is_blurry,
        "is_dark": is_dark,
        "is_good": not is_blurry and not is_dark,
    }



# Reading bag file

def read_bag(bag_path: str, rejected_folders: dict) -> dict:
    """
    Read topics from the bag.
    Returns raw lists ready for numpy conversion and raw TF messages.
    """
    cam = {
        "times": [],
        "blur": [],
        "brightness": [],
        "is_blurry": [],
        "is_dark": [],
        "is_good": [],
        "images_rgb": [],
        "images_bgr": [],
    }

    joints = {
        "times": [],
        "positions": [],
        "velocities": [],
        "efforts": [],
    }

    tf_data = {
        "times": [],
        "msgs": [],
    }

    tf_static_data = {
        "times": [],
        "msgs": [],
    }

    frame_idx = 0

    print(f"Reading bag: {bag_path}")

    with rosbag.Bag(bag_path, "r") as bag:
        topics = [CAMERA_TOPIC, JOINT_TOPIC, TF_TOPIC, TF_STATIC_TOPIC]

        for topic, msg, t in bag.read_messages(topics=topics):

            
            if topic == CAMERA_TOPIC:
                timestamp = msg.header.stamp.to_sec()

                np_arr = np.frombuffer(msg.data, np.uint8)
                image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if image_bgr is None:
                    print(f"  [WARN] Could not decode frame at t={timestamp:.3f}")
                    continue

                q = assess_image_quality(image_bgr)

                cam["times"].append(timestamp)
                cam["blur"].append(q["blur_score"])
                cam["brightness"].append(q["brightness"])
                cam["is_blurry"].append(q["is_blurry"])
                cam["is_dark"].append(q["is_dark"])
                cam["is_good"].append(q["is_good"])

                cam["images_rgb"].append(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
                cam["images_bgr"].append(image_bgr)

                if SAVE_REJECTED_IMAGES and not q["is_good"]:
                    fname = f"frame_{frame_idx:06d}.jpg"

                    if q["is_blurry"] and q["is_dark"]:
                        cv2.imwrite(
                            os.path.join(rejected_folders["blurry_dark"], fname),
                            image_bgr,
                        )
                    elif q["is_blurry"]:
                        cv2.imwrite(
                            os.path.join(rejected_folders["blurry"], fname),
                            image_bgr,
                        )
                    else:
                        cv2.imwrite(
                            os.path.join(rejected_folders["dark"], fname),
                            image_bgr,
                        )

                frame_idx += 1

            
            elif topic == JOINT_TOPIC:
                joints["times"].append(msg.header.stamp.to_sec())
                joints["positions"].append(list(msg.position))
                joints["velocities"].append(list(msg.velocity))
                joints["efforts"].append(list(msg.effort))

            
            elif topic == TF_TOPIC:
                tf_data["times"].append(t.to_sec())
                tf_data["msgs"].append(msg)

            elif topic == TF_STATIC_TOPIC:
                tf_static_data["times"].append(t.to_sec())
                tf_static_data["msgs"].append(msg)

    
    for key in ["times", "blur", "brightness", "is_blurry", "is_dark", "is_good"]:
        cam[key] = np.array(cam[key])

    cam["images_rgb"] = np.array(cam["images_rgb"], dtype=np.uint8)
    cam["images_bgr"] = cam["images_bgr"]

    
    for key in ["times", "positions", "velocities", "efforts"]:
        joints[key] = np.array(joints[key])

    return cam, joints, tf_data, tf_static_data


# Validation and Synchronization

def validate_and_sync(cam: dict, joints: dict) -> dict:
    """
    Keep only frames that:
    - pass quality checks
    - fall inside the joint-state time range

    Then interpolate joint data to camera timestamps.
    """
    if len(cam["times"]) == 0:
        raise ValueError(f"No camera messages found on topic: {CAMERA_TOPIC}")

    if len(joints["times"]) == 0:
        raise ValueError(f"No joint messages found on topic: {JOINT_TOPIC}")

    valid_mask = (
        cam["is_good"]
        & (cam["times"] >= joints["times"][0])
        & (cam["times"] <= joints["times"][-1])
    )

    n_original = len(cam["times"])
    n_valid = int(valid_mask.sum())
    n_rejected = n_original - n_valid

    print("\nValidation summary:")
    print(f"  Total camera frames : {n_original}")
    print(f"  Rejected (blurry)   : {int(cam['is_blurry'].sum())}")
    print(f"  Rejected (dark)     : {int(cam['is_dark'].sum())}")
    print(f"  Outside joint range : {n_original - n_valid - int((~cam['is_good']).sum())}")
    print(f"  Valid frames        : {n_valid}")

    times_v = cam["times"][valid_mask]
    blur_v = cam["blur"][valid_mask]
    brightness_v = cam["brightness"][valid_mask]
    images_rgb_v = cam["images_rgb"][valid_mask]
    images_bgr_v = [cam["images_bgr"][i] for i in np.where(valid_mask)[0]]

    n_joints = joints["positions"].shape[1]
    n_frames = n_valid

    synced_pos = np.zeros((n_frames, n_joints), dtype=np.float32)
    synced_vel = np.zeros((n_frames, n_joints), dtype=np.float32)
    synced_eff = np.zeros((n_frames, n_joints), dtype=np.float32)

    for j in range(n_joints):
        synced_pos[:, j] = np.interp(
            times_v,
            joints["times"],
            joints["positions"][:, j],
        )
        synced_vel[:, j] = np.interp(
            times_v,
            joints["times"],
            joints["velocities"][:, j],
        )
        synced_eff[:, j] = np.interp(
            times_v,
            joints["times"],
            joints["efforts"][:, j],
        )

    print("\nSynchronisation done:")
    print(f"  images_rgb : {images_rgb_v.shape}")
    print(f"  positions  : {synced_pos.shape}")

    return {
        "times": times_v,
        "blur": blur_v,
        "brightness": brightness_v,
        "images_rgb": images_rgb_v,
        "images_bgr": images_bgr_v,
        "positions": synced_pos,
        "velocities": synced_vel,
        "efforts": synced_eff,
        "n_joints": n_joints,
        "n_frames": n_frames,
        "n_rejected": n_rejected,
        "n_original": n_original,
    }


# Create the Zarr format

def write_zarr(synced: dict, bag_path: str) -> str:
    """
    Write Zarr dataset to:

    session_xxx/zarr/session_xxx_dataset.zarr
    """
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]

    session_dir = get_session_dir_from_bag_path(bag_path)
    zarr_dir = os.path.join(session_dir, "zarr")
    os.makedirs(zarr_dir, exist_ok=True)

    zarr_path = os.path.join(zarr_dir, f"{bag_name}_dataset.zarr")

    print(f"\n[Fork A] Writing Zarr store → {zarr_path}")

    store = zarr.open(zarr_path, mode="w")

    H = synced["images_rgb"].shape[1]
    W = synced["images_rgb"].shape[2]
    n_f = synced["n_frames"]
    n_j = synced["n_joints"]

    store.create_dataset(
        "camera/images",
        data=synced["images_rgb"],
        chunks=(1, H, W, 3),
        dtype=np.uint8,
        compressor=zarr.Blosc(cname="lz4", clevel=5, shuffle=zarr.Blosc.BITSHUFFLE),
        overwrite=True,
    )

    store.create_dataset(
        "camera/timestamps",
        data=synced["times"].astype(np.float64),
        chunks=(n_f,),
        overwrite=True,
    )

    store.create_dataset(
        "camera/blur_scores",
        data=synced["blur"].astype(np.float32),
        chunks=(n_f,),
        overwrite=True,
    )

    store.create_dataset(
        "camera/brightness",
        data=synced["brightness"].astype(np.float32),
        chunks=(n_f,),
        overwrite=True,
    )

    store.create_dataset(
        "joints/positions",
        data=synced["positions"],
        chunks=(n_f, n_j),
        overwrite=True,
    )

    store.create_dataset(
        "joints/velocities",
        data=synced["velocities"],
        chunks=(n_f, n_j),
        overwrite=True,
    )

    store.create_dataset(
        "joints/efforts",
        data=synced["efforts"],
        chunks=(n_f, n_j),
        overwrite=True,
    )

    store.create_dataset(
        "joints/timestamps",
        data=synced["times"].astype(np.float64),
        chunks=(n_f,),
        overwrite=True,
    )

    store.attrs.update({
        "source_bag": bag_path,
        "created_at": datetime.utcnow().isoformat(),
        "n_frames": int(n_f),
        "n_joints": int(n_j),
        "image_shape": [int(H), int(W), 3],
        "image_color_format": "RGB",
        "camera_topic": CAMERA_TOPIC,
        "joint_topic": JOINT_TOPIC,
        "tf_topic": TF_TOPIC,
        "tf_static_topic": TF_STATIC_TOPIC,
        "blur_threshold": BLUR_THRESHOLD,
        "brightness_threshold": BRIGHTNESS_THRESHOLD,
        "n_original_frames": int(synced["n_original"]),
        "n_rejected_frames": int(synced["n_rejected"]),
        "joint_names": [f"joint_{j}" for j in range(n_j)],
        "time_start": float(synced["times"][0]),
        "time_end": float(synced["times"][-1]),
        "duration_seconds": float(synced["times"][-1] - synced["times"][0]),
    })

    print("  Zarr store written successfully.")
    print(store.tree())

    return zarr_path


def write_replay_bag(
    synced: dict,
    tf_data: dict,
    tf_static_data: dict,
    bag_path: str,
) -> str:
    """
    Write replay bag to the same bag/ folder as the original bag:

    session_xxx/bag/session_xxx_replay.bag
    """
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]

    bag_dir = os.path.dirname(bag_path)
    os.makedirs(bag_dir, exist_ok=True)

    replay_path = os.path.join(bag_dir, f"{bag_name}_replay.bag")

    print(f"\n[Fork B] Writing replay bag → {replay_path}")

    with rosbag.Bag(replay_path, "w") as out_bag:

        
        for i, (ts, image_bgr) in enumerate(
            zip(synced["times"], synced["images_bgr"])
        ):
            ros_time = rospy.Time.from_sec(ts)

            success, buffer = cv2.imencode(
                ".jpg",
                image_bgr,
                [cv2.IMWRITE_JPEG_QUALITY, 95],
            )

            if not success:
                print(f"  [WARN] Could not re-encode frame {i}, skipping.")
                continue

            cam_msg = CompressedImage()
            cam_msg.header.stamp = ros_time
            cam_msg.header.seq = i
            cam_msg.format = "jpeg"
            cam_msg.data = buffer.tobytes()

            out_bag.write(CAMERA_TOPIC, cam_msg, ros_time)

        print(f"  Camera frames written    : {len(synced['times'])}")

        
        for i, (ts, pos, vel, eff) in enumerate(
            zip(
                synced["times"],
                synced["positions"],
                synced["velocities"],
                synced["efforts"],
            )
        ):
            ros_time = rospy.Time.from_sec(ts)

            js_msg = JointState()
            js_msg.header.stamp = ros_time
            js_msg.header.seq = i
            js_msg.name = [f"joint_{j}" for j in range(synced["n_joints"])]
            js_msg.position = pos.tolist()
            js_msg.velocity = vel.tolist()
            js_msg.effort = eff.tolist()

            out_bag.write(JOINT_TOPIC, js_msg, ros_time)

        print(f"  Joint state msgs written : {len(synced['times'])}")

        
        for ts, msg in zip(tf_data["times"], tf_data["msgs"]):
            out_bag.write(TF_TOPIC, msg, rospy.Time.from_sec(ts))

        for ts, msg in zip(tf_static_data["times"], tf_static_data["msgs"]):
            out_bag.write(TF_STATIC_TOPIC, msg, rospy.Time.from_sec(ts))

        print(f"  /tf msgs written         : {len(tf_data['times'])}")
        print(f"  /tf_static msgs written  : {len(tf_static_data['times'])}")

    print("  Replay bag written successfully.")

    return replay_path



def sanity_check(zarr_path: str, replay_path: str):
    print("\n── Sanity checks ──────────────────────────────────")

    store = zarr.open(zarr_path, mode="r")

    frame_0 = store["camera/images"][0]
    pos_0 = store["joints/positions"][0]

    print(f"[Zarr]  frame 0 shape : {frame_0.shape}  dtype={frame_0.dtype}")
    print(f"[Zarr]  joint pos 0   : {pos_0}")

    with rosbag.Bag(replay_path, "r") as bag:
        info = bag.get_type_and_topic_info()
        print("[Bag]   topics in replay bag:")

        for topic, topic_info in info.topics.items():
            print(f"          {topic:50s}  {topic_info.message_count} msgs")

    print("───────────────────────────────────────────────────")



def main():
    """
    Normal use from the recorder:

    python3 bag_to_zarr_algorithm_compatible.py /path/to/session_xxx.bag

    Manual fallback use:

    python3 bag_to_zarr_algorithm_compatible.py

    In fallback mode, the newest .bag under ~/ros_data is used.
    """
    if len(sys.argv) >= 2:
        bag_path = os.path.expanduser(sys.argv[1])
    else:
        print("[WARN] No bag path given. Falling back to newest bag in ~/ros_data.")
        bag_path = find_newest_bag()

    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Given bag file does not exist: {bag_path}")

    print(f"Using bag file: {bag_path}")

    rejected_folders = setup_rejected_folders(bag_path)

    
    cam, joints, tf_data, tf_static_data = read_bag(bag_path, rejected_folders)

    
    synced = validate_and_sync(cam, joints)

    
    zarr_path = write_zarr(synced, bag_path)

    
    replay_path = write_replay_bag(synced, tf_data, tf_static_data, bag_path)

    # Sanity check
    sanity_check(zarr_path, replay_path)

    print("\n✓ Done.")
    print(f"  ML dataset  : {zarr_path}")
    print(f"  Replay bag  : {replay_path}")

    print("\nTo visualise in RViz on the robot:")
    print(f"  scp {replay_path} bep2026@192.168.0.121:~/bags/")
    print("  ssh bep2026@192.168.0.121")
    print(f"  rosbag play ~/bags/{os.path.basename(replay_path)}")


if __name__ == "__main__":
    main()
