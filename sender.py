#!/usr/bin/env python3
import rospy, socket, struct, time
from serialize_navdata import serialize_navdata

HOST = "127.0.0.1"
PORT = 9001
MESSAGE_TYPE_NAVDATA = 1
message_id = 0

def mock_pose(): return (1.0, 2.0, 0.5)
def mock_obstacles(): return [(0.5, 0.5), (1.5, 1.5)]
def mock_path(): return [(1.0, 1.0), (2.0, 2.0)]

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
    sock = connect()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            pose = mock_pose()
            obs = mock_obstacles()
            path = mock_path()
            payload = serialize_navdata(pose, obs, path)

            header = struct.pack('<III', message_id, MESSAGE_TYPE_NAVDATA, len(payload))
            sock.sendall(header + payload)

            rospy.loginfo_throttle(1, f"[SENT] ID={message_id} | Type={MESSAGE_TYPE_NAVDATA} | {len(payload)} bytes")
            message_id += 1
        except Exception as e:
            rospy.logwarn(f"[ERROR] Send failed: {e}")
            sock.close()
            sock = connect()

        rate.sleep()

if __name__ == "__main__":
    main()
