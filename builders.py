import flatbuffers
import Pose2D
import ObstaclePoint
import PathPoint

def create_pose2d(builder, x, y, yaw):
    Pose2D.Pose2DStart(builder)
    Pose2D.Pose2DAddX(builder, x)
    Pose2D.Pose2DAddY(builder, y)
    Pose2D.Pose2DAddYaw(builder, yaw)
    return Pose2D.Pose2DEnd(builder)

def create_obstacle_point(builder, x, y):
    ObstaclePoint.ObstaclePointStart(builder)
    ObstaclePoint.ObstaclePointAddX(builder, x)
    ObstaclePoint.ObstaclePointAddY(builder, y)
    return ObstaclePoint.ObstaclePointEnd(builder)

def create_path_point(builder, x, y):
    PathPoint.PathPointStart(builder)
    PathPoint.PathPointAddX(builder, x)
    PathPoint.PathPointAddY(builder, y)
    return PathPoint.PathPointEnd(builder)
