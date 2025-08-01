#!/usr/bin/env python3
import rospy, socket, struct
from serialize_navdata import serialize_navdata

def mock_pose(): return (1.0, 2.0, 0.5)
def mock_obstacles(): return [(0.5, 0.5), (1.5, 1.5)]
def mock_path(): return [(1.0, 1.0), (2.0, 2.0)]

def main():
    rospy.init_node("flatbuffer_sender")
    s = socket.socket()
    s.connect(('127.0.0.1', 9001))
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pose = mock_pose()
        obs = mock_obstacles()
        path = mock_path()
        data = serialize_navdata(pose, obs, path)
        packet = struct.pack('<I', len(data)) + data
        s.sendall(packet)
        rospy.loginfo_throttle(1, f"[SENT] {len(data)} bytes")
        rate.sleep()

if __name__ == "__main__":
    main()
