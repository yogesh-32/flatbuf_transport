#!/usr/bin/env python3
import rospy, socket, struct, time
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

from serialize_navdata import serialize_navdata

HOST = "127.0.0.1"
PORT = 9001
MESSAGE_TYPE_NAVDATA = 1
message_id = 0

# def mock_pose(): return (1.0, 2.0, 0.5)

def mock_obstacles(): return [(0.5, 0.5), (1.5, 1.5)]
def mock_path(): return [(1.0, 1.0), (2.0, 2.0)]

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

    sock = connect()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            pose = get_tf_pose(tf_buffer)
            # pose = mock_pose()
            obs = mock_obstacles()
            path = mock_path()
            payload = serialize_navdata(pose, obs, path)

            header = struct.pack('<III', message_id, MESSAGE_TYPE_NAVDATA, len(payload))
            sock.sendall(header + payload)

            # rospy.loginfo_throttle(1, f"[SENT] ID={message_id} | Type={MESSAGE_TYPE_NAVDATA} | {len(payload)} bytes")
            rospy.loginfo(f"[SENT] ID={message_id} | Type={MESSAGE_TYPE_NAVDATA} | {len(payload)} bytes")
            message_id += 1
        except Exception as e:
            rospy.logwarn(f"[ERROR] Send failed: {e}")
            sock.close()
            sock = connect()

        rate.sleep()

if __name__ == "__main__":
    main()
