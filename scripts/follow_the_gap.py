#! /usr/bin/env python3
# Created by Chen Yuxuan
# Modified by Tian Bo
# @Time: 2023/10/20 17:02:12
# @Author: Jeff Wang(Lr_2002)
import os
import rospy
import yaml
import tf
import rospkg
import speed_control
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point,Vector3, Quaternion, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion # radi
from math import cos, sin, atan2, hypot, pi
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

"""
todo
1. fix the problem to let the car avoide the collision
2. avoide the U-turn to the back
"""

robot_name = os.getenv("TIANRACER_NAME", "tianracer")

def display_direction(scale, tail, tip, idnum):
    """
    generate arrow makers
    scale: the scale of the marker
    tail: which side has no arrow
    tip: the side has arrow
    idnum: the num of the arrow
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = robot_name + "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "points_arrow"
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale = scale
    m.color.r = 1.0
    m.color.g = 0.1
    m.color.b = 0.1
    m.color.a = 0.6
    m.points = [tail, tip]
    return m

def display_threshold(scale, points, idnum):
    """
    generate the round (using cylinder to generate)
    scale: the size of the round(in Vector3)
    points: the position of the marker
    idnum: the idnum
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = robot_name + "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "round"
    m.id = idnum
    m.type = Marker.CYLINDER
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.pose.position.x = points[0]
    m.pose.position.y = points[1]
    m.pose.position.z = points[2]
    m.scale = scale
    m.color.r = 0
    m.color.g = 0.
    m.color.b = 1
    m.color.a = 0.3
    return m

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    """
    from data(recv from the sub) to get the specific angle data
    data: from the sub
    angle: the angle you want to get, usually in deg
    deg: whether your angle is deg
    """
    if deg:
        angle = np.deg2rad(angle)
    idx = int((angle - data.angle_min) / data.angle_increment)
    if idx < 0 or idx >= len(data.ranges):
        return FILTER_VALUE
    dis = data.ranges[idx]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def get_gap(dis_list, threshold):
    """
    from the distance_list to get the gap
    dis_list: distance_list, generated from 'get_range'
    threshold: the threshold to filter the gap
    """
    lis = [1 if a > threshold else 0 for a in dis_list]
    tmp = 0
    ans_list = []
    for i in lis:
        if i == 0 :
            tmp = 0
            ans_list.append(0)
        else:
            tmp +=1
            ans_list.append(tmp)
    return ans_list


def load_waypoints():
    """
    load waypoints from the yaml file via rospkg (no hard-coded absolute path)
    returns a list of (x, y, yaw, max_steer_override, k_yaw_override) tuples in file order
    """
    pkg_name = rospy.get_param("~waypoint_pkg", "tianracer_gazebo")
    rel_path = rospy.get_param("~waypoint_file", "scripts/waypoint_race/tianracer_racetrack_points.yaml")
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound:
        rospy.logerr("waypoint package not found: %s", pkg_name)
        return []
    path = os.path.join(pkg_path, rel_path)
    if not os.path.exists(path):
        rospy.logerr("waypoint file not found: %s", path)
        return []
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    wps = []
    for w in data.get("waypoints", []):
        pos = w["pose"]["position"]
        ori = w["pose"].get("orientation")
        if ori is not None:
            q = [ori["x"], ori["y"], ori["z"], ori["w"]]
            (_, _, yaw) = euler_from_quaternion(q)
        else:
            yaw = None  # 兼容旧格式：点没有朝向，后续按“纯朝点”处理
        max_steer_override = w.get("max_steer")  # 单点最大转向角覆盖；无该字段则为 None
        k_yaw_override = w.get("k_yaw")          # 单点朝向融合增益覆盖；无该字段则为 None
        speed_override = w.get("speed")
        wps.append((pos["x"], pos["y"], yaw, max_steer_override, k_yaw_override, speed_override))
    rospy.loginfo("loaded %d waypoints from %s", len(wps), path)
    return wps


def resolve_closing_idx(wps):
    """
    find the waypoint closest to (start_wp_x, start_wp_y) and return its index;
    this is the target of the "closing" phase, so reordering the yaml won't break it
    """
    if not wps:
        return 0
    sx = rospy.get_param("~start_wp_x", -0.037)
    sy = rospy.get_param("~start_wp_y", 0.735)
    best = min(range(len(wps)), key=lambda i: hypot(wps[i][0] - sx, wps[i][1] - sy))
    rospy.loginfo("closing point = wp[%d] (%.3f, %.3f)", best, wps[best][0], wps[best][1])
    return best


def publish_waypoint_markers(event=None):
    """
    publish every loaded waypoint as a red SPHERE marker (frame = "map") so the
    waypoints show up in RViz
    """
    if not waypoints:
        return
    for i, (wx, wy, _wyaw, _ms, _ky, _spd) in enumerate(waypoints):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "waypoints"
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.4
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        wp_marker_pub.publish(m)
    rospy.loginfo("published %d waypoint markers", len(waypoints))


def get_car_pose():
    """
    get the current car pose (x, y, yaw) from TF map -> base_link
    returns None if the transform is not available yet
    """
    global tf_listener
    if tf_listener is None:
        return None
    map_frame = rospy.get_param("~map_frame", "tianracer/odom")
    base_frame = rospy.get_param("~base_frame", "tianracer/base_link")
    try:
        (trans, rot) = tf_listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    x, y = trans[0], trans[1]
    (_, _, yaw) = euler_from_quaternion(rot)
    return (x, y, yaw)


def pure_pursuit_steering(car, target, target_yaw=None, max_steer_override=None, k_yaw_override=None):
    """
    pure-pursuit steering toward a single target point
    car: (x, y, yaw), target: (x, y), target_yaw: optional desired heading at the point
    """
    x, y, yaw = car
    tx, ty = target
    dx, dy = tx - x, ty - y
    goal_angle = atan2(dy, dx)
    alpha = goal_angle - yaw
    alpha = atan2(sin(alpha), cos(alpha))  # normalize to [-pi, pi]
    wheelbase = rospy.get_param("~wheelbase", 0.42)
    dist = hypot(dx, dy)
    lookahead = max(dist, rospy.get_param("~lookahead_min", 1.0))
    steering = atan2(2.0 * wheelbase * sin(alpha), lookahead)
    max_steer = max_steer_override if max_steer_override is not None else rospy.get_param("~max_steer", 0.5)
    steering = max(-max_steer, min(max_steer, steering))
    steer_to_point = steering

    # 朝向融合：接近点时，车头逐渐对齐到点的朝向
    if target_yaw is not None:
        k_yaw = k_yaw_override if k_yaw_override is not None else rospy.get_param("~k_yaw", 0.7)
        yaw_err = target_yaw - yaw
        yaw_err = atan2(sin(yaw_err), cos(yaw_err))  # normalize to [-pi, pi]
        steer_to_yaw = k_yaw * yaw_err
        w = 1.0 - min(dist / 1.0, 1.0)
        steering = (1 - w) * steer_to_point + w * steer_to_yaw
        steering = max(-max_steer, min(max_steer, steering))

    return steering


gx, gy, gz = 0, 0 ,0
gr, gp, gyaw = 0, 0 ,0
g_ang = 0
last_steering = 0.0
emergency_dir = 0       # 0 = not locked, 1 = locked left turn, -1 = locked right turn
emergency_until = rospy.Time(0)
waypoints = []          # [(x, y, yaw), ...] loaded once at startup
visited = set()         # 已访问 waypoint 的下标集合
wp_phase = "nearest"    # "nearest"=追最近未访问点；"closing"=去收尾点(closing_wp_idx)
closing_wp_idx = 0      # 收尾点索引：启动时按 (start_wp_x, start_wp_y) 最近匹配得到
tf_listener = None
lap_reset_time = rospy.Time(0)
lap_start = None
lap_count = 0

def get_pose(data):
    """
    read the position of the model
    data: recv from the sub
    """
    global gx, gy, gz, gr, gp, gyaw
    gx = data.pose.pose.position.x
    gy = data.pose.pose.position.y
    gz = data.pose.pose.position.z
    ori = [
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    ]
    (gr, gp, gyaw) = euler_from_quaternion(ori)

INTERVAL = 112 # total interval
scan_interval = [i - 60 for i in range(INTERVAL)] # the interval you want to scan

def follow_the_gap_callback(data):
    global gx, gy, gz, last_steering, emergency_dir, emergency_until, visited, wp_phase, lap_reset_time, lap_start, lap_count
    ang_min_deg = rospy.get_param("~ang_min_deg", -60)
    ang_max_deg = rospy.get_param("~ang_max_deg", 60)
    STEP = 1 # step_length of the scan
    threshold = rospy.get_param("~threshold", 1.05) # clip threshold (walls within 1.05 m count as obstacles)
    dis_list = [] # value to go through to find the gap
    for ang in range(ang_min_deg, ang_max_deg, STEP):
        dis_list.append(get_range(data, ang))

    # collect all gaps (continuous runs where distance > threshold)
    gaps = []  # each: (start_idx, end_idx, length, mid_angle_deg)
    i = 0
    n = len(dis_list)
    while i < n:
        if dis_list[i] > threshold:
            start = i
            while i < n and dis_list[i] > threshold:
                i += 1
            end = i - 1
            length = end - start + 1
            mid_angle = (start + end) / 2.0 * STEP + ang_min_deg
            gaps.append((start, end, length, mid_angle))
        else:
            i += 1

    # follow the gap:
    #  always steer toward the widest gap (classic follow-the-gap), so the car
    #  starts turning early instead of waiting until it is on top of a wall.
    FRONT_HALF_ANGLE = 10  # degrees, "front" = +/- this around 0 deg
    front_min = min(get_range(data, ang) for ang in range(-FRONT_HALF_ANGLE, FRONT_HALF_ANGLE + 1, STEP))

    # ---- waypoint assist (point mode) ----
    wp_mode = False
    wp_steering = 0.0
    spd_override = None
    lap_delay = rospy.get_param("~lap_delay", 3.0)
    if waypoints and tf_listener is not None and (rospy.Time.now() - lap_reset_time).to_sec() > lap_delay:
        car = get_car_pose()
        if car is not None:
            pass_dist = rospy.get_param("~waypoint_pass_dist", 0.5)
            switch_dist = rospy.get_param("~waypoint_switch_dist", 2.0)

            if wp_phase == "closing":
                # 去收尾点：固定追 closing_wp_idx，直到 dist < pass_dist
                best_i = closing_wp_idx
                wx, wy, wyaw, ms_override, ky_override, spd_override = waypoints[best_i]
                dist = hypot(wx - car[0], wy - car[1])
                if dist < switch_dist:
                    wp_mode = True
                    wp_steering = pure_pursuit_steering(car, (wx, wy), target_yaw=wyaw,
                                                        max_steer_override=ms_override, k_yaw_override=ky_override)
                    rospy.loginfo_throttle(1.0, "[closing] wp[%d] dist=%.2f steering=%.3f",
                                           best_i, dist, wp_steering)
                if dist < pass_dist:
                    wp_phase = "nearest"
                    lap_count += 1
                    rospy.logwarn("===== LAP %d COMPLETE: %.2f s =====", lap_count, (rospy.Time.now() - lap_start).to_sec())
                    lap_start = rospy.Time.now()
                    gov.stability = 1.0
                    gov._flips = []

            else:  # wp_phase == "nearest"
                # 候选 = 未访问的点；全访问过就进入"去收尾点"状态
                candidates = [i for i in range(len(waypoints)) if i not in visited]
                if not candidates:
                    visited.clear()
                    wp_phase = "closing"
                else:
                    # 每帧在候选中选离车最近的那个
                    best_i = min(candidates, key=lambda i: hypot(waypoints[i][0] - car[0],
                                                                 waypoints[i][1] - car[1]))
                    wx, wy, wyaw, ms_override, ky_override, spd_override = waypoints[best_i]
                    dist = hypot(wx - car[0], wy - car[1])
                    if dist < switch_dist:
                        wp_mode = True
                        wp_steering = pure_pursuit_steering(car, (wx, wy), target_yaw=wyaw,
                                                            max_steer_override=ms_override, k_yaw_override=ky_override)
                        rospy.loginfo_throttle(1.0, "[nearest] chasing wp[%d] dist=%.2f wp_yaw=%.2f car_yaw=%.2f steering=%.3f",
                                               best_i, dist, wyaw if wyaw is not None else 0.0, car[2], wp_steering)
                    # 到过该点(dist < pass_dist)就标记为已访问，不再作为候选
                    if dist < pass_dist:
                        visited.add(best_i)

    # early-turn lookahead: start steering toward the widest gap as soon as a wall
    # is within this distance, instead of waiting until it is only `threshold` away
    TURN_LOOKAHEAD = rospy.get_param("~turn_lookahead", 2.0)  # meters

    EMERGENCY_THRESHOLD = rospy.get_param("~emergency_threshold", 0.3)  # meters
    EMERGENCY_STEER = rospy.get_param("~emergency_steer", 0.5)          # radians
    EMERGENCY_LOCK_TIME = rospy.get_param("~emergency_lock_time", 1.5)  # seconds

    MIN_GAP_WIDTH = 5  # samples (~5 deg), narrower gaps are treated as noise

    # early turn: once a wall is within TURN_LOOKAHEAD, steer toward the widest
    # gap right away; when the front is clear, keep going straight
    if front_min < TURN_LOOKAHEAD:
        real_gaps = [g for g in gaps if g[2] >= MIN_GAP_WIDTH]
        if real_gaps:
            target_angle = max(real_gaps, key=lambda g: g[2])[3]
        elif gaps:
            target_angle = max(gaps, key=lambda g: g[2])[3]
        else:
            target_angle = 0.0
    else:
        target_angle = 0.0

    # emergency: wall very close AND no gap anywhere (a dead end)
    emergency = front_min < EMERGENCY_THRESHOLD and not gaps
    forced_steering = None

    # emergency direction lock: hold one side for EMERGENCY_LOCK_TIME
    # instead of flipping every frame (which made the car spin in place)
    if emergency_dir != 0 and (rospy.Time.now() > emergency_until or not emergency):
        emergency_dir = 0
    if emergency and emergency_dir == 0:
        # not locked yet: lock opposite to the last steering direction
        emergency_dir = -1 if last_steering > 0 else 1
        emergency_until = rospy.Time.now() + rospy.Duration(EMERGENCY_LOCK_TIME)
    if emergency:
        # dead end: hold the locked emergency direction
        forced_steering = emergency_dir * EMERGENCY_STEER

    avoidance_angle = 0  # avoidance angle of the gap(usually according to the size of the model)
    if target_angle > 0:
        target_angle -= avoidance_angle
    elif target_angle < 0:
        target_angle += avoidance_angle

    P = 1
    global gyaw

    if forced_steering is not None:
        # emergency hard turn: bypass the normal pipeline and smoothing
        steering_angle = forced_steering
        last_steering = steering_angle
    else:
        if wp_mode:
            steering_angle = wp_steering
        else:
            target_angle_rad = np.deg2rad(target_angle)
            steering_angle = target_angle_rad * P
        clip_threshold = 0.5
        steering_angle = steering_angle if steering_angle < clip_threshold else clip_threshold
        steering_angle = steering_angle if steering_angle > -clip_threshold else -clip_threshold

        # smooth the steering to avoid per-frame full-lock oscillation
        smooth = rospy.get_param("~smooth", 0.4)  # 0~1, larger = smoother
        steering_angle = smooth * last_steering + (1 - smooth) * steering_angle
        last_steering = steering_angle
    global g_ang

    dis_ml = Float64MultiArray(data=dis_list)
    frame_pub.publish(dis_ml)
    # ---- 倒车：回正方向盘，直线后退 ----
    reversing = front_min < params.reverse_threshold
    if reversing:
        steering_angle = 0.0
        last_steering = 0.0

    # ---- 速度决策：纯逻辑 + 稳定性回退 ----
    stability = gov.step(steering_angle, emergency, rospy.Time.now().to_sec())
    speed = speed_control.compute_speed(params, front_min, wp_mode, emergency, stability, spd_override)
    length = 1
    gap_angle = target_angle / 180 * 3.14
    ref_yaw = gyaw + gap_angle

    arrow_pub.publish(display_direction(scale, Point(gx,gy,gz), Point(gx + length * cos(ref_yaw),gy + length * sin(ref_yaw) , gz), 3))
    round_pub.publish(display_threshold(Vector3(threshold, threshold, 0.1), (gx, gy, gz), 4))
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle
    drive_msg.drive.speed=speed
    angle = steering_angle * 180 / np.pi
    rospy.loginfo("front_min=%.2f target_angle=%.1f steering=%.3f", front_min, target_angle, steering_angle)
    if emergency:
        rospy.logwarn("EMERGENCY: front_min=%.2f, steering=%.2f", front_min, steering_angle)
    # rospy.loginfo(f"rad_steering_angle: {steering_angle:1.1f} steering_angle: {angle:1.1f}  speed: {speed:1.1f}")
    print("rad_steering_angle: %.1f steering_angle: %.1f  speed: %.1f" % (steering_angle, angle, speed))
    drive_pub.publish(drive_msg)

if __name__ == '__main__':
  try:
    rospy.init_node("follow_the_gap")
    params = speed_control.SpeedParams(
        speed=rospy.get_param("~speed", 3.4),
        straight_boost=rospy.get_param("~straight_boost", 2.8),
        corner_tight_speed=rospy.get_param("~corner_tight_speed", 2.6),
        corner_near_speed=rospy.get_param("~corner_near_speed", 3.6),
        corner_tight_threshold=rospy.get_param("~corner_tight_threshold", 1.0),
        corner_near_threshold=rospy.get_param("~corner_near_threshold", 2.0),
        wp_speed=rospy.get_param("~wp_speed", 3.15),
        emergency_speed=rospy.get_param("~emergency_speed", 0.2),
        reverse_threshold=rospy.get_param("~reverse_threshold", 0.15),
        reverse_speed=rospy.get_param("~reverse_speed", 0.3),
        stab_osc_window=rospy.get_param("~stab_osc_window", 2.0),
        stab_osc_limit=rospy.get_param("~stab_osc_limit", 5),
        stab_cooldown=rospy.get_param("~stab_cooldown", 1.5),
        stab_recover_rate=rospy.get_param("~stab_recover_rate", 0.25),
    )
    gov = speed_control.StabilityGovernor(params)
    waypoints = load_waypoints()
    closing_wp_idx = resolve_closing_idx(waypoints)
    tf_listener = tf.TransformListener()
    wp_marker_pub = rospy.Publisher("waypoint_markers", Marker, queue_size=10)
    publish_waypoint_markers()
    rospy.Timer(rospy.Duration(1.0), publish_waypoint_markers)
    arrow_pub = rospy.Publisher("visualization_marker_show_orientation", Marker, queue_size=10)
    round_pub = rospy.Publisher("round_show", Marker, queue_size=10)
    scale = Vector3(0.05,0.2,0.2)
    pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, get_pose)
    scan_sub = rospy.Subscriber(robot_name + '/scan', LaserScan, follow_the_gap_callback)
    drive_pub = rospy.Publisher(robot_name + '/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    frame_pub = rospy.Publisher('image', Float64MultiArray,queue_size=150)
    lap_start = rospy.Time.now()
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
