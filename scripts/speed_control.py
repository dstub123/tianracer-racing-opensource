# -*- coding: utf-8 -*-
"""
follow_the_gap 的速度决策 + 稳定性回退（纯逻辑，无 ROS/GPU 依赖，可单独单测）。

设计：
  - 直道速度 = speed（保守基准 3.3） + stability * straight_boost（默认 +2.8）
    稳定时 stability=1 → 3.3+2.8=6.1；失控时 stability=0 → 回到保守 3.3。
  - 弯道速度（紧弯/缓弯）保持不变，不受 stability 影响。
  - 弯道判断条件（front_min 与阈值）不在这里，仍由调用方沿用原逻辑。
"""


class SpeedParams(object):
    """所有速度与稳定性参数，带默认值（与 follow_the_gap.py 原逻辑一致 + 提速增量）。"""

    def __init__(self,
                 speed=3.3,                 # 直道保守速度（原 ~speed）
                 straight_boost=2.8,        # 直道提速增量（3.3+2.8=6.1）
                 corner_tight_speed=2.5,    # 紧弯（front_min < 1.0）速度
                 corner_near_speed=3.0,     # 缓弯（1.0<=front_min<2.0）速度
                 corner_tight_threshold=1.0,
                 corner_near_threshold=2.0,
                 wp_speed=2.55,             # waypoint 辅助速度，不变
                 emergency_speed=0.2,       # 紧急限速，不变
                 reverse_threshold=0.15,    # 倒车触发距离，不变
                 reverse_speed=0.3,         # 倒车速度，不变
                 # ---- 稳定性回退参数 ----
                 stab_osc_window=2.0,       # 统计转向变号的窗口（秒）
                 stab_osc_limit=5,          # 窗口内变号次数超过此值判为“画龙”
                 stab_cooldown=1.5,         # 触发后保持保守的时长（秒）
                 stab_recover_rate=0.25,    # 冷却后每秒钟恢复的 stability 量（0~1/s）
                 ):
        self.speed = float(speed)
        self.straight_boost = float(straight_boost)
        self.corner_tight_speed = float(corner_tight_speed)
        self.corner_near_speed = float(corner_near_speed)
        self.corner_tight_threshold = float(corner_tight_threshold)
        self.corner_near_threshold = float(corner_near_threshold)
        self.wp_speed = float(wp_speed)
        self.emergency_speed = float(emergency_speed)
        self.reverse_threshold = float(reverse_threshold)
        self.reverse_speed = float(reverse_speed)
        self.stab_osc_window = float(stab_osc_window)
        self.stab_osc_limit = int(stab_osc_limit)
        self.stab_cooldown = float(stab_cooldown)
        self.stab_recover_rate = float(stab_recover_rate)


def compute_speed(params, front_min, wp_mode, emergency, stability, wp_speed_override=None):
    """
    纯函数：返回目标速度（m/s）。

    params   : SpeedParams
    front_min: 正前方最近障碍距离（米）
    wp_mode  : 是否处于 waypoint 辅助模式
    emergency: 是否紧急贴墙
    stability: 0~1，直道提速的缩放系数（0=保守，1=全速）
    """
    if front_min < params.reverse_threshold:
        return -params.reverse_speed          # 倒车
    if front_min < params.corner_tight_threshold:
        speed = params.corner_tight_speed     # 紧弯，不变
    elif front_min < params.corner_near_threshold:
        speed = params.corner_near_speed      # 缓弯，不变
    else:
        # 直道：保守基准 + stability 缩放的提速增量
        speed = params.speed + stability * params.straight_boost
    if wp_mode:
        wp_speed = wp_speed_override if wp_speed_override is not None else params.wp_speed
        speed = min(speed, wp_speed)
    if emergency:
        speed = min(speed, params.emergency_speed)
    return speed


class StabilityGovernor(object):
    """根据 steering 变号频率和 emergency 判断是否失控，输出 stability(0~1)。"""

    DEADBAND = 0.05  # 转向死区（弧度），忽略微小抖动

    def __init__(self, params):
        self.params = params
        self.stability = 1.0
        self._sign = 0
        self._flips = []          # 转向变号的时间戳列表
        self._cooldown_until = 0.0
        self._last_time = None

    def _trigger(self, now):
        self.stability = 0.0
        self._cooldown_until = now + self.params.stab_cooldown
        self._flips = []

    def step(self, steering, emergency, now):
        """喂入当前转向角/紧急标志/当前时间，返回 stability(0~1)。"""
        if steering > self.DEADBAND:
            s = 1
        elif steering < -self.DEADBAND:
            s = -1
        else:
            s = 0

        if s != 0 and s != self._sign:
            self._flips.append(now)
        self._sign = s

        # 丢掉窗口之外的旧变号记录
        cutoff = now - self.params.stab_osc_window
        self._flips = [t for t in self._flips if t >= cutoff]

        if emergency or len(self._flips) >= self.params.stab_osc_limit:
            self._trigger(now)
        elif now > self._cooldown_until and self.stability < 1.0:
            dt = now - self._last_time if self._last_time is not None else 0.0
            self.stability = min(
                1.0, self.stability + self.params.stab_recover_rate * max(dt, 0.0))

        self._last_time = now
        return self.stability
