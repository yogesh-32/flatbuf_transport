#!/usr/bin/env python3
import rospy, socket, struct, time
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Path

from serialize_navdata import serialize_navdata

# HOST = "127.0.0.1"
HOST = "192.168.144.214"
PORT = 9001
MESSAGE_TYPE_NAVDATA = 1
message_id = 0

# def mock_pose(): return (1.0, 2.0, 0.5)

latest_obstacles = []
latest_path = []

def get_tf_pose(tf_buffer):
    try:
        trans = tf_buffer.lookup_transform("nav_local_frame", "base_link", rospy.Time(0), rospy.Duration(0.5))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)
    except Exception as e:
        rospy.logwarn_throttle(5, f"[POSE] TF lookup failed: {e}")
        return (0.0, 0.0, 0.0)

def costmap_callback(msg: OccupancyGrid):
    global latest_obstacles
    latest_obstacles = []

    width = msg.info.width
    resolution = msg.info.resolution
    origin = msg.info.origin

    for idx, val in enumerate(msg.data):
        if val > 95:    # Stricter limit 50 -> 95 -> 100
            x_idx = idx % width
            y_idx = idx // width
            x = origin.position.x + x_idx * resolution
            y = origin.position.y + y_idx * resolution
            latest_obstacles.append((x, y))

def path_callback(msg: Path):
    global latest_path
    latest_path = []
    for pose in msg.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        latest_path.append((x, y))

def get_obstacles_from_costmap():
    return latest_obstacles[:10000] # Limit increased 100 -> 10000

def get_path_from_global_planner():
    # return latest_path[:200]
    return latest_path

def connect():
    while not rospy.is_shutdown():
        try:
            s = socket.socket()
            s.connect((HOST, PORT))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            rospy.loginfo("[SENDER] Connected to receiver.")
            return s
        except Exception:
            rospy.logwarn_throttle(5, "[SENDER] Waiting for receiver...")
            time.sleep(1)

def main():
    global message_id
    rospy.init_node("flatbuffer_sender")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, costmap_callback, queue_size=1)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, path_callback, queue_size=1)

    sock = connect()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            pose = get_tf_pose(tf_buffer)
            # pose = mock_pose()
            obs = get_obstacles_from_costmap()
            path = get_path_from_global_planner()
            payload = serialize_navdata(pose, obs, path)

            header = struct.pack('<III', message_id, MESSAGE_TYPE_NAVDATA, len(payload))
            sock.sendall(header + payload)

            rospy.loginfo_throttle(1, f"[SENT] ID={message_id} | Obs={len(obs)} | Path={len(path)} | {len(payload)} bytes")
            message_id += 1
        except Exception as e:
            rospy.logwarn(f"[ERROR] Send failed: {e}")
            sock.close()
            sock = connect()

        rate.sleep()

if __name__ == "__main__":
    main()
