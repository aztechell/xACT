# === xAct_robot_steer.py ===
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Button
from pybricks.tools import StopWatch
from xAct_action import Action
import umath

__all__ = [
    "Robot",
    "HeadingTracker",
    "speed_control",
    "clamp",
    "angle_wrap",
]


def speed_control(ratio, max_speed, min_speed=30):
    direction = 1 if max_speed >= 0 else -1
    max_spd = abs(max_speed)
    min_spd = abs(min_speed)

    if ratio > 0.7:
        k = (min_spd - max_spd) / 0.3
        b = min_spd - k
        spd = k * ratio + b
    elif ratio < 0.3:
        k = (max_spd - min_spd) / 0.3
        spd = k * ratio + min_spd
    else:
        spd = max_spd
    return clamp(spd * direction, -100, 100)


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def angle_wrap(value):
    return (value + 180) % 360 - 180


class HeadingTracker:
    def __init__(self, imu):
        self.imu = imu
        self._last = 0
        self._accum = 0

    def zero(self, value=0):
        self.imu.reset_heading(value)
        self._last = self.imu.heading()
        self._accum = value

    def heading_abs(self):
        h = self.imu.heading()
        dh = h - self._last
        if dh > 180:
            dh -= 360
        elif dh < -180:
            dh += 360
        self._accum += dh
        self._last = h
        return self._accum


class Robot:
    """
    Single-drive + steering motor robot.

    Units:
      - X, Y are in millimeters.
      - Heading is in degrees (absolute, unwrapped).
    """
    def __init__(
        self,
        hub,
        drive_port=Port.A,
        steer_port=Port.B,
        drive_dir=Direction.CLOCKWISE,
        steer_dir=Direction.COUNTERCLOCKWISE,
        wheel_diameter_mm=62.4,
        drive_gear_ratio=1.0,
        steer_speed=300,
        max_steer_deg=120,
        steer_center=0,
        steer_calibrate_offset=15,
        steer_inverted=False,
        heading_zero=0,
    ):
        self.hub = hub
        self.drive = Motor(drive_port, drive_dir) if drive_port is not None else None
        self.steer = Motor(steer_port, steer_dir) if steer_port is not None else None

        self.drive_gear_ratio = drive_gear_ratio
        self.wheel_diameter_mm = wheel_diameter_mm
        self.wheel_mm_per_deg = (umath.pi * self.wheel_diameter_mm) / 360.0

        self.steer_speed = steer_speed
        self.max_steer_deg = max_steer_deg
        self.steer_center = steer_center
        self.steer_sign = -1 if steer_inverted else 1
        self.steer_calibrate_offset = steer_calibrate_offset

        try:
            while not self.hub.imu.ready():
                pass
        except Exception:
            pass

        self._heading = HeadingTracker(self.hub.imu)
        self._heading.zero(heading_zero)

        if self.steer is not None:
            self._calibrate_steer()

        self.X = 0.0
        self.Y = 0.0
        self.heading = angle_wrap(heading_zero)
        self.heading_abs = heading_zero
        self.velocity = 0.0

        # Default tuning for drive_to_point_action
        self.drive_to_point_kp = 2.0
        self.drive_to_point_ki = 0.0
        self.drive_to_point_kd = 0.5
        self.drive_to_point_max_steer_deg = None
        self.drive_to_point_min_speed = 30
        self.drive_to_point_stop_dist_mm = 25
        self.drive_to_point_i_limit = 200
        self.drive_to_point_pose_offset_mm = 0
        self.drive_to_point_max_accel_dist_mm = 100
        self.drive_to_point_max_decel_dist_mm = 100

    def beep(self):
        try:
            self.hub.speaker.beep(880, 100)
        except AttributeError:
            pass

    def _require_drive(self):
        if self.drive is None:
            raise RuntimeError("Drive motor not configured. Set drive_port.")

    def _require_steer(self):
        if self.steer is None:
            raise RuntimeError("Steer motor not configured. Set steer_port.")

    def _calibrate_steer(self):
        if self.steer_calibrate_offset is not None:
            self.steer.run_target(
                self.steer_speed,
                self.steer_calibrate_offset,
                then=Stop.HOLD,
                wait=True,
            )
        self.steer.reset_angle(0)
        self._set_steer_target(0, wait=True)

    def _update_heading(self):
        self.heading_abs = self._heading.heading_abs()
        self.heading = angle_wrap(self.heading_abs)

    def _set_steer_target(self, target_deg, wait=False):
        self._require_steer()
        target = clamp(target_deg, -self.max_steer_deg, self.max_steer_deg)
        target = target * self.steer_sign + self.steer_center
        self.steer.run_target(self.steer_speed, target, then=Stop.HOLD, wait=wait)

    def _stop_drive(self, stop=Stop.BRAKE, center_steer=True):
        if self.drive is not None:
            try:
                self.drive.stop(stop)
            except TypeError:
                self.drive.stop()
        if center_steer and self.steer is not None:
            self._set_steer_target(0, wait=False)

    def set_drive_to_point_tuning(
        self,
        kp=None,
        ki=None,
        kd=None,
        max_steer_deg=None,
        min_speed=None,
        stop_dist_mm=None,
        i_limit=None,
        pose_offset_mm=None,
        max_accel_dist_mm=None,
        max_decel_dist_mm=None,
    ):
        if kp is not None:
            self.drive_to_point_kp = kp
        if ki is not None:
            self.drive_to_point_ki = ki
        if kd is not None:
            self.drive_to_point_kd = kd
        if max_steer_deg is not None:
            self.drive_to_point_max_steer_deg = max_steer_deg
        if min_speed is not None:
            self.drive_to_point_min_speed = min_speed
        if stop_dist_mm is not None:
            self.drive_to_point_stop_dist_mm = stop_dist_mm
        if i_limit is not None:
            self.drive_to_point_i_limit = i_limit
        if pose_offset_mm is not None:
            self.drive_to_point_pose_offset_mm = pose_offset_mm
        if max_accel_dist_mm is not None:
            self.drive_to_point_max_accel_dist_mm = max_accel_dist_mm
        if max_decel_dist_mm is not None:
            self.drive_to_point_max_decel_dist_mm = max_decel_dist_mm

    # === Odometry ===
    def odometry_action(self):
        self._require_drive()
        class Odom(Action):
            def __init__(inner, robot):
                super().__init__(robot)
                inner.prev_drive = self.drive.angle()
                inner.prev_x = self.X
                inner.prev_y = self.Y
                inner.timer = StopWatch()
                inner.last_time = inner.timer.time()

            def update(inner):
                self._update_heading()
                cur_drive = self.drive.angle()
                d_deg = cur_drive - inner.prev_drive
                dist_mm = d_deg * self.wheel_mm_per_deg / self.drive_gear_ratio
                self.X += dist_mm * umath.cos(umath.radians(self.heading_abs))
                self.Y += dist_mm * umath.sin(umath.radians(self.heading_abs))
                inner.prev_drive = cur_drive

                t = inner.timer.time()
                dt = (t - inner.last_time) / 1000
                if dt > 0.05:
                    dx = self.X - inner.prev_x
                    dy = self.Y - inner.prev_y
                    self.velocity = umath.sqrt(dx * dx + dy * dy) / dt
                    inner.prev_x, inner.prev_y = self.X, self.Y
                    inner.last_time = t
                return False
        return Odom(self)

    # === Drive distance with gyro hold ===
    def drive_distance_action(
        self,
        distance_mm,
        speed=60,
        heading=None,
        kp=2.0,
        kd=0.5,
        ki=0.0,
        max_steer_deg=None,
        min_speed=30,
        stop=Stop.HOLD,
        timeout_ms=None,
    ):
        self._require_drive()
        self._require_steer()
        class DriveDistance(Action):
            def on_start(inner):
                dist_mm = distance_mm() if callable(distance_mm) else distance_mm
                inner.target_mm = abs(dist_mm)
                inner.target_deg = inner.target_mm / self.wheel_mm_per_deg * self.drive_gear_ratio
                self.drive.reset_angle(0)

                inner.max_speed = speed
                if speed >= 0:
                    inner.max_speed = abs(speed) * (1 if dist_mm >= 0 else -1)

                inner.direction = 1 if inner.max_speed >= 0 else -1
                inner.target_heading = self.heading_abs if heading is None else heading

                inner.prev_error = 0
                inner.integral = 0
                inner.timer = StopWatch()
                inner.last_time = inner.timer.time()
                inner.start_time = inner.last_time

            def update(inner):
                if timeout_ms is not None:
                    if inner.timer.time() - inner.start_time >= timeout_ms:
                        self._stop_drive(stop=stop, center_steer=True)
                        return True

                self._update_heading()
                cur_deg = self.drive.angle()
                if inner.direction >= 0:
                    done = cur_deg >= inner.target_deg
                else:
                    done = cur_deg <= -inner.target_deg
                if done:
                    self._stop_drive(stop=stop, center_steer=True)
                    self.beep()
                    return True

                ratio = min(1.0, abs(cur_deg) / inner.target_deg) if inner.target_deg > 0 else 1.0
                current_speed = speed_control(ratio, inner.max_speed, min_speed=min_speed)
                self.drive.dc(current_speed)

                err = inner.target_heading - self.heading_abs
                now = inner.timer.time()
                dt = (now - inner.last_time) / 1000
                inner.last_time = now

                inner.integral = inner.integral * 0.8 + err * dt
                derivative = (err - inner.prev_error) / dt if dt > 0 else 0
                inner.prev_error = err

                steer_cmd = kp * err + ki * inner.integral + kd * derivative
                limit = self.max_steer_deg if max_steer_deg is None else max_steer_deg
                steer_cmd = clamp(steer_cmd, -limit, limit)
                self._set_steer_target(steer_cmd, wait=False)
                return False
        return DriveDistance(self)

    # === Drive to target point (requires odometry_action running) ===
    def drive_to_point_action(
        self,
        _X=None,
        _Y=None,
        speed=60,
        kp=None,
        kd=None,
        ki=None,
        max_steer_deg=None,
        min_speed=None,
        stop_dist_mm=None,
        i_limit=None,
        pose_offset_mm=None,
        max_accel_dist_mm=None,
        max_decel_dist_mm=None,
        stop=Stop.COAST,
    ):
        self._require_drive()
        self._require_steer()
        class DriveToPoint(Action):
            def on_start(inner):
                self._update_heading()
                inner.start_x = self.X
                inner.start_y = self.Y
                inner.target_x = self.X if _X is None else _X
                inner.target_y = self.Y if _Y is None else _Y
                inner.offset_mm = (
                    self.drive_to_point_pose_offset_mm
                    if pose_offset_mm is None
                    else pose_offset_mm
                )
                inner.start_ex = inner.start_x + inner.offset_mm * umath.cos(umath.radians(self.heading_abs))
                inner.start_ey = inner.start_y + inner.offset_mm * umath.sin(umath.radians(self.heading_abs))
                inner.dx = inner.target_x - inner.start_ex
                inner.dy = inner.target_y - inner.start_ey
                inner.total_dist = umath.sqrt(inner.dx * inner.dx + inner.dy * inner.dy)
                inner.no_move = inner.total_dist <= 0.001
                inner.direction = 1 if speed >= 0 else -1
                inner.prev_error = 0
                inner.integral = 0
                inner.timer = StopWatch()
                inner.last_time = inner.timer.time()

            def update(inner):
                local_kp = self.drive_to_point_kp if kp is None else kp
                local_ki = self.drive_to_point_ki if ki is None else ki
                local_kd = self.drive_to_point_kd if kd is None else kd
                local_min_speed = self.drive_to_point_min_speed if min_speed is None else min_speed
                local_stop_dist = self.drive_to_point_stop_dist_mm if stop_dist_mm is None else stop_dist_mm
                local_max_steer = self.drive_to_point_max_steer_deg if max_steer_deg is None else max_steer_deg
                local_i_limit = self.drive_to_point_i_limit if i_limit is None else i_limit
                local_max_accel_dist = (
                    self.drive_to_point_max_accel_dist_mm
                    if max_accel_dist_mm is None
                    else max_accel_dist_mm
                )
                local_max_decel_dist = (
                    self.drive_to_point_max_decel_dist_mm
                    if max_decel_dist_mm is None
                    else max_decel_dist_mm
                )

                self._update_heading()
                if inner.no_move:
                    self._stop_drive(stop=stop, center_steer=True)
                    self.beep()
                    return True

                ex = self.X + inner.offset_mm * umath.cos(umath.radians(self.heading_abs))
                ey = self.Y + inner.offset_mm * umath.sin(umath.radians(self.heading_abs))
                dx = inner.target_x - ex
                dy = inner.target_y - ey
                dist = umath.sqrt(dx * dx + dy * dy)
                if dist <= local_stop_dist:
                    self._stop_drive(stop=stop, center_steer=True)
                    self.beep()
                    print(f"X = {self.X:.1f} mm, Y = {self.Y:.1f} mm, H = {self.heading_abs:.1f}")
                    return True
                if inner.total_dist > 0:
                    proj_len = (
                        (ex - inner.start_ex) * inner.dx
                        + (ey - inner.start_ey) * inner.dy
                    ) / inner.total_dist
                    if proj_len >= inner.total_dist:
                        self._stop_drive(stop=stop, center_steer=True)
                        self.beep()
                        print(f"X = {self.X:.1f} mm, Y = {self.Y:.1f} mm, H = {self.heading_abs:.1f}")
                        return True

                target_heading = umath.degrees(umath.atan2(dy, dx))
                desired_heading = target_heading if inner.direction == 1 else angle_wrap(target_heading + 180)
                err = angle_wrap(desired_heading - self.heading_abs)

                now = inner.timer.time()
                dt = (now - inner.last_time) / 1000
                inner.last_time = now

                if abs(err) < 45:
                    inner.integral = inner.integral * 0.7 + err * dt
                else:
                    inner.integral *= 0.7
                if local_i_limit is not None:
                    inner.integral = clamp(inner.integral, -local_i_limit, local_i_limit)
                derivative = angle_wrap(err - inner.prev_error) / dt if dt > 0 else 0
                inner.prev_error = err

                steer_cmd = local_kp * err + local_ki * inner.integral + local_kd * derivative
                limit = self.max_steer_deg if local_max_steer is None else local_max_steer
                steer_cmd = clamp(steer_cmd, -limit, limit)
                self._set_steer_target(steer_cmd, wait=False)

                if inner.total_dist > 0:
                    if local_max_accel_dist is None and local_max_decel_dist is None:
                        progress = 1.0 - min(1.0, dist / inner.total_dist)
                    else:
                        accel_dist_raw = local_max_accel_dist
                        decel_dist_raw = local_max_decel_dist
                        if accel_dist_raw is None and decel_dist_raw is not None:
                            accel_dist_raw = decel_dist_raw
                        if decel_dist_raw is None and accel_dist_raw is not None:
                            decel_dist_raw = accel_dist_raw

                        accel_dist = (
                            min(accel_dist_raw, inner.total_dist * 0.3)
                            if accel_dist_raw is not None
                            else inner.total_dist * 0.3
                        )
                        decel_dist = (
                            min(decel_dist_raw, inner.total_dist * 0.3)
                            if decel_dist_raw is not None
                            else inner.total_dist * 0.3
                        )
                        cruise_dist = max(0.0, inner.total_dist - accel_dist - decel_dist)
                        traveled = clamp(inner.total_dist - dist, 0.0, inner.total_dist)

                        if accel_dist > 0 and traveled <= accel_dist:
                            progress = 0.3 * (traveled / accel_dist)
                        elif cruise_dist > 0 and traveled <= accel_dist + cruise_dist:
                            progress = 0.3 + 0.4 * ((traveled - accel_dist) / cruise_dist)
                        elif decel_dist > 0:
                            progress = 0.7 + 0.3 * (
                                (traveled - accel_dist - cruise_dist) / decel_dist
                            )
                        else:
                            progress = 1.0
                    progress = clamp(progress, 0.0, 1.0)
                else:
                    progress = 1.0
                current_speed = speed_control(progress, speed, min_speed=local_min_speed)
                self.drive.dc(current_speed)
                return False
        return DriveToPoint(self)

    # === Car-like turn to heading (drives while steering) ===
    def turn_to_heading_action(
        self,
        target_heading,
        speed=30,
        max_steer_deg=None,
        kp=2.0,
        tolerance_deg=2.0,
        stop=Stop.HOLD,
    ):
        self._require_drive()
        self._require_steer()
        class Turn(Action):
            def update(inner):
                self._update_heading()
                err = angle_wrap(target_heading - self.heading_abs)
                if abs(err) <= tolerance_deg:
                    self._stop_drive(stop=stop, center_steer=True)
                    self.beep()
                    return True

                steer_cmd = kp * err
                limit = self.max_steer_deg if max_steer_deg is None else max_steer_deg
                steer_cmd = clamp(steer_cmd, -limit, limit)
                self._set_steer_target(steer_cmd, wait=False)
                self.drive.dc(clamp(speed, -100, 100))
                return False
        return Turn(self)

    def steer_action(self, target_deg, wait=False):
        self._require_steer()
        class Steer(Action):
            def update(inner):
                self._set_steer_target(target_deg, wait=wait)
                return True
        return Steer(self)

    def drive_dc_action(self, speed, duration_ms=None, stop=Stop.BRAKE):
        self._require_drive()
        class DriveDC(Action):
            def on_start(inner):
                inner.timer = StopWatch()

            def update(inner):
                self.drive.dc(clamp(speed, -100, 100))
                if duration_ms is None:
                    return False
                if inner.timer.time() >= duration_ms:
                    self._stop_drive(stop=stop, center_steer=False)
                    return True
                return False
        return DriveDC(self)

    def stop_action(self, stop=Stop.BRAKE, center_steer=True):
        class StopAction(Action):
            def update(inner):
                self._stop_drive(stop=stop, center_steer=center_steer)
                return True
        return StopAction(self)

    def wait_action(self, duration_ms):
        class WaitAction(Action):
            def update(inner):
                return inner.wait(duration_ms)
        return WaitAction(self)

    def sound_action(self):
        class Sound(Action):
            def update(inner):
                self.beep()
                return True
        return Sound(self)

    def print_pose_action(self):
        class PrintPose(Action):
            def update(inner):
                print(f"X = {self.X:.1f} mm, Y = {self.Y:.1f} mm, H = {self.heading_abs:.1f}")
                return True
        return PrintPose(self)

    def reset_odometry_action(self, _X=None, _Y=None, heading=None):
        class ResetOdometry(Action):
            def update(inner):
                if _X is not None:
                    self.X = _X
                if _Y is not None:
                    self.Y = _Y
                if heading is not None:
                    self._heading.zero(heading)
                    self.heading_abs = heading
                    self.heading = angle_wrap(heading)
                return True
        return ResetOdometry(self)
