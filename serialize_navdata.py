import flatbuffers
import time

import NavData
from builders import create_pose2d, create_obstacle_point, create_path_point

def serialize_navdata(pose, obstacles, path):
    builder = flatbuffers.Builder(1024)

    # Pose
    pose_offset = create_pose2d(builder, *pose)

    # Obstacles
    obstacle_offsets = [create_obstacle_point(builder, x, y) for x, y in obstacles]
    NavData.NavDataStartObstaclesVector(builder, len(obstacle_offsets))
    for offset in reversed(obstacle_offsets):
        builder.PrependUOffsetTRelative(offset)
    obstacles_vec = builder.EndVector()

    # Path
    path_offsets = [create_path_point(builder, x, y) for x, y in path]
    NavData.NavDataStartPathVector(builder, len(path_offsets))
    for offset in reversed(path_offsets):
        builder.PrependUOffsetTRelative(offset)
    path_vec = builder.EndVector()

    # Build NavData table
    NavData.NavDataStart(builder)
    NavData.NavDataAddTimestamp(builder, time.time())
    NavData.NavDataAddPose(builder, pose_offset)
    NavData.NavDataAddObstacles(builder, obstacles_vec)
    NavData.NavDataAddPath(builder, path_vec)
    navdata_offset = NavData.NavDataEnd(builder)

    builder.Finish(navdata_offset)
    return bytes(builder.Output())  # ensure immutable byte string
