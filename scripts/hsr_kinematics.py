"""HSR 台車の運動学・オドメトリのデータクラス (hsr.py から分離)。"""
import math



class hsr_config:
    def __init__(self) -> None:
        self.use_ros = True
class BaseOdometry:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.ang = 0.0
class JointSpace:
    def __init__(self) -> None:
        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0
        self.vel_steer = 0.0
class CartSpace:
    def __init__(self) -> None:
        self.dot_x = 0.0
        self.dot_y = 0.0
        self.dot_r = 0.0
class VehicleState:
    def __init__(self) -> None:
        self.steer_angle = 0.0
class VehicleDynamics:
    """
    Dynamics of offset diff drive vehicle
    Equations are from the paper written by Masayoshi Wada etal.
    https://www.jstage.jst.go.jp/article/jrsj1983/18/8/18_8_1166/_pdf
    """

    def __init__(self, wheel_radius: float, wheel_separation: float, wheel_offset: float) -> None:
        self._wheel_radius = wheel_radius
        self._wheel_separation = wheel_separation
        self._wheel_offset = wheel_offset

    def forward(self, joint_space: JointSpace, state: VehicleState) -> CartSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = CartSpace()
        output.dot_x = (
            self._wheel_radius / 2.0 * cos_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * cos_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * sin_s
        ) * joint_space.vel_wheel_l
        output.dot_y = (
            self._wheel_radius / 2.0 * sin_s
            + self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_r + (
            self._wheel_radius / 2.0 * sin_s
            - self._wheel_radius * self._wheel_offset / self._wheel_separation * cos_s
        ) * joint_space.vel_wheel_l
        output.dot_r = (
            self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_r
            - self._wheel_radius / self._wheel_separation * joint_space.vel_wheel_l
            - joint_space.vel_steer
        )
        return output

    def inverse(self, cart_space: CartSpace, state: VehicleState) -> JointSpace:
        cos_s = math.cos(state.steer_angle)
        sin_s = math.sin(state.steer_angle)
        output = JointSpace()
        output.vel_wheel_r = (
            cos_s / self._wheel_radius
            - self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            + self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_wheel_l = (
            cos_s / self._wheel_radius
            + self._wheel_separation * sin_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_x + (
            sin_s / self._wheel_radius
            - self._wheel_separation * cos_s / 2.0 /
            self._wheel_radius / self._wheel_offset
        ) * cart_space.dot_y
        output.vel_steer = (
            -sin_s / self._wheel_offset * cart_space.dot_x
            + cos_s / self._wheel_offset * cart_space.dot_y
            - cart_space.dot_r
        )
        return output
class WheelOdometry:
    def __init__(self) -> None:
        self.pose = BaseOdometry()

    def integrate(self, cart_velocity: CartSpace, dt: float):
        diff_r = cart_velocity.dot_r * dt
        cosr = math.cos(self.pose.ang + 0.5 * diff_r)
        sinr = math.sin(self.pose.ang + 0.5 * diff_r)
        world_dot_x = cart_velocity.dot_x * cosr - cart_velocity.dot_y * sinr
        world_dot_y = cart_velocity.dot_x * sinr + cart_velocity.dot_y * cosr
        if dt > 0.0:
            self.pose.x += world_dot_x * dt
            self.pose.y += world_dot_y * dt
            self.pose.ang += diff_r
        return world_dot_x, world_dot_y

    def set_pose(self, x: float, y: float, ang: float) -> None:
        self.pose.x = x
        self.pose.y = y
        self.pose.ang = ang
