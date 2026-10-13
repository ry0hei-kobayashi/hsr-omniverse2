"""クォータニオン小物 (hsr.py から分離)。

tf_transformations には依存しない (Isaac 同梱の ROS 2 には無く、Python 3.11 向けに配る手間が
増えるだけだった)。quaternion_from_euler / euler_from_quaternion は tf_transformations の既定
(axes='sxyz' = 固定軸 roll→pitch→yaw、クォータニオンは (x, y, z, w)) と同じ結果を返す。
"""
import math

# tf_transformations の _EPS (numpy.finfo(float).eps * 4) と同じ gimbal lock 判定閾値
_EULER_EPS = 8.881784197001252e-16


# --- 案A(アタッチ把持)用クォータニオン補助 (x,y,z,w) ---
def _q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])
def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
def _q_rot(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quaternion_from_euler(r, p, y):
    """roll, pitch, yaw [rad] → (x, y, z, w)。tf_transformations.quaternion_from_euler(r, p, y) と同値。"""
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def euler_from_quaternion(x, y, z, w):
    """(x, y, z, w) → (roll, pitch, yaw) [rad]。tf_transformations.euler_from_quaternion((x, y, z, w)) と同値。

    正規化していないクォータニオンも受け付ける (tf_transformations と同じく回転行列経由で計算)。
    pitch = ±90° 付近は gimbal lock で roll/yaw が一意に決まらないが、同じ規約 (roll を 0 側に寄せず
    行列要素から求める) に揃えてある。
    """
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    s = 2.0 / n
    # 回転行列の必要な要素だけ (tf_transformations.quaternion_matrix と同じ式)
    m00 = 1.0 - s * (y * y + z * z)
    m10 = s * (x * y + z * w)
    m20 = s * (x * z - y * w)
    m21 = s * (y * z + x * w)
    m22 = 1.0 - s * (x * x + y * y)
    m11 = 1.0 - s * (x * x + z * z)
    m12 = s * (y * z - x * w)
    cy = math.sqrt(m00 * m00 + m10 * m10)
    if cy > _EULER_EPS:
        roll = math.atan2(m21, m22)
        pitch = math.atan2(-m20, cy)
        yaw = math.atan2(m10, m00)
    else:  # gimbal lock (tf_transformations.euler_from_matrix の else 分岐と同じ)
        roll = math.atan2(-m12, m11)
        pitch = math.atan2(-m20, cy)
        yaw = 0.0
    return (roll, pitch, yaw)
