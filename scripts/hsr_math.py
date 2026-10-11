"""クォータニオン小物 (hsr.py から分離)。"""
import math

import tf_transformations


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
    return tf_transformations.quaternion_from_euler(r, p, y)


def euler_from_quaternion(x, y, z, w):
    return tf_transformations.euler_from_quaternion((x, y, z, w))
