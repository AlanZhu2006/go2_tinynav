#!/usr/bin/env python3
import argparse
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IMAGE_TOPICS = (
    "/camera/camera/infra1/image_rect_raw",
    "/camera/camera/infra2/image_rect_raw",
    "/camera/camera/depth/image_rect_raw",
    "/camera/camera/color/image_raw",
)

CAMERA_INFO_TOPICS = (
    "/camera/camera/infra1/camera_info",
    "/camera/camera/infra2/camera_info",
    "/camera/camera/color/camera_info",
)

TIMESTAMP_TOPICS = IMAGE_TOPICS + CAMERA_INFO_TOPICS + ("/camera/camera/imu",)


def count_messages(bag_path: str) -> Counter:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    counts = Counter()
    while reader.has_next():
        topic, _data, _stamp = reader.read_next()
        counts[topic] += 1
    return counts


def topic_types(reader: rosbag2_py.SequentialReader) -> dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def read_stamps(
    bag_path: str,
    topics: tuple[str, ...],
) -> tuple[dict[str, list[float]], dict[str, list[float]]]:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    types = topic_types(reader)
    message_types = {
        topic: get_message(types[topic])
        for topic in topics
        if topic in types
    }
    header_stamps = {topic: [] for topic in topics}
    receive_stamps = {topic: [] for topic in topics}

    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        msg_type = message_types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            header_stamps[topic].append(stamp.sec + stamp.nanosec * 1e-9)
            receive_stamps[topic].append(_stamp * 1e-9)

    return header_stamps, receive_stamps


def validate_timestamp_continuity(
    bag_path: str,
    max_image_gap: float,
    max_imu_gap: float,
    allow_startup_stale_frame: bool,
) -> list[str]:
    header_stamps, receive_stamps = read_stamps(bag_path, TIMESTAMP_TOPICS)
    failures = []

    print("TinyNav timestamp validation:")
    for topic in TIMESTAMP_TOPICS:
        topic_stamps = header_stamps.get(topic, [])
        topic_receive_stamps = receive_stamps.get(topic, [])
        if len(topic_stamps) < 2:
            print(f"  {topic}: not enough header stamps for continuity check")
            continue

        gaps = [
            topic_stamps[index + 1] - topic_stamps[index]
            for index in range(len(topic_stamps) - 1)
        ]
        receive_gaps = [
            topic_receive_stamps[index + 1] - topic_receive_stamps[index]
            for index in range(len(topic_receive_stamps) - 1)
        ]
        max_gap = max(gaps)
        median_gap = sorted(gaps)[len(gaps) // 2]
        allowed_gap = max_imu_gap if topic == "/camera/camera/imu" else max_image_gap
        big_gaps = [
            (index, gap)
            for index, gap in enumerate(gaps)
            if gap > allowed_gap
        ]
        ignored_startup_gap = False
        if (
            allow_startup_stale_frame
            and topic in IMAGE_TOPICS
            and len(big_gaps) == 1
            and big_gaps[0][0] == 0
            and receive_gaps
            and receive_gaps[0] <= allowed_gap
            and (len(gaps) == 1 or max(gaps[1:]) <= allowed_gap)
        ):
            ignored_startup_gap = True

        print(
            f"  {topic}: median_dt={median_gap:.6f}s max_dt={max_gap:.6f}s "
            f"gaps>{allowed_gap:.3f}s={len(big_gaps)}"
        )
        if ignored_startup_gap:
            print(
                f"    warning: ignored stale startup frame; receive_dt="
                f"{receive_gaps[0]:.6f}s and subsequent headers are continuous"
            )
        elif big_gaps:
            preview = ", ".join(
                f"#{index}:{gap:.3f}s" for index, gap in big_gaps[:5]
            )
            failures.append(f"{topic} has timestamp gaps: {preview}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--min-images", type=int, default=30)
    parser.add_argument("--min-camera-info", type=int, default=1)
    parser.add_argument("--min-imu", type=int, default=100)
    parser.add_argument("--max-image-gap", type=float, default=0.2)
    parser.add_argument("--max-imu-gap", type=float, default=0.05)
    parser.add_argument("--skip-timestamp-check", action="store_true")
    parser.add_argument(
        "--strict-startup-timestamps",
        action="store_true",
        help="Treat stale first image frames as failures instead of startup warnings.",
    )
    args = parser.parse_args()

    counts = count_messages(args.bag)
    failures = []

    print("TinyNav bag validation:")
    for topic in IMAGE_TOPICS:
        count = counts.get(topic, 0)
        print(f"  {topic}: {count}")
        if count < args.min_images:
            failures.append(f"{topic} has {count} messages, expected >= {args.min_images}")

    for topic in CAMERA_INFO_TOPICS:
        count = counts.get(topic, 0)
        print(f"  {topic}: {count}")
        if count < args.min_camera_info:
            failures.append(
                f"{topic} has {count} messages, expected >= {args.min_camera_info}"
            )

    imu_count = counts.get("/camera/camera/imu", 0)
    print(f"  /camera/camera/imu: {imu_count}")
    if imu_count < args.min_imu:
        failures.append(f"/camera/camera/imu has {imu_count} messages, expected >= {args.min_imu}")

    tf_static_count = counts.get("/tf_static", 0)
    print(f"  /tf_static: {tf_static_count}")
    if tf_static_count < 1:
        failures.append("/tf_static is missing")

    if not args.skip_timestamp_check:
        print()
        failures.extend(
            validate_timestamp_continuity(
                args.bag,
                max_image_gap=args.max_image_gap,
                max_imu_gap=args.max_imu_gap,
                allow_startup_stale_frame=not args.strict_startup_timestamps,
            )
        )

    if failures:
        print()
        print("Bag is not suitable for TinyNav mapping:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("Bag has the required TinyNav mapping streams and timestamp continuity.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
