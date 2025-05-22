"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import argparse

import rosbag2_py
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from collections import defaultdict
from scipy.spatial.transform import Rotation as R



def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader

def flatten_msg(msg, prefix=""):
    """Recursively flatten a ROS message into a flat dict."""
    out = {}
    dir_msg = dir(msg)
    for attr in dir_msg:
        if attr == "orientation":
            rpy = R.from_quat([getattr(msg, attr).x,getattr(msg, attr).y,getattr(msg, attr).z,getattr(msg, attr).w]).as_euler('xyz', degrees=True)
            out['roll'] = rpy[0]
            out['pitch'] = rpy[1]
            out['yaw'] = rpy[2]
            continue
            
        if attr.startswith("_") or callable(getattr(msg, attr)):
            continue
        value = getattr(msg, attr)
        if attr == "SLOT_TYPES":
            continue
        key = f"{prefix}{attr}"
        if hasattr(value, "__slots__"):  # another ROS message
            out.update(flatten_msg(value, prefix=key + "."))
        else:
            out[key] = value
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )

    shit = defaultdict(list)

    args = parser.parse_args()
    for topic, msg, timestamp in read_messages(args.input):
        fmsg = flatten_msg(msg)
        fmsg["time"] = timestamp
        topic = topic.replace("/vicon/Group466CF/Group466CF","/vicon")
        shit[topic].append(fmsg)
    dfs = {}
    for topic, records in shit.items():
        df = pd.DataFrame(records)
        # Optional: prefix columns with topic name to prevent conflicts
        if "time" in df.columns:
            df = df.set_index("time")
        dfs[topic] = df.add_prefix(topic + "/")

    # Merge all DataFrames on time (outer join to preserve all data)
    df_all = pd.concat(dfs.values(), axis=1).sort_index().reset_index()
    df_all.to_csv(args.input.replace(".mcap",".csv"))
    print(df_all)


if __name__ == "__main__":
    main()
