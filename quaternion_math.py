import math


def quaternion_from_axis_angle(axis, angle):
    return (
        axis[0] * math.sin(angle/2),
        axis[1] * math.sin(angle/2),
        axis[2] * math.sin(angle/2),
        math.cos(angle/2))


def quaternion_from_rotation_vector(vec):
    mag = math.sqrt(sum(v**2 for v in vec))
    return quaternion_from_rotation_vector([v / mag for v in vec], mag)


def quaternion_vector_mult(q, v):
    r = quaternion_mult((v[0], v[1], v[2], 0), quaternion_conjugate(q))
    return quaternion_mult(q, r)[:3]


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_mult(q, r):
    q0, q1, q2, q3 = q
    r0, r1, r2, r3 = r
    t0 = + r0*q3 - r1*q2 + r2*q1 + r3*q0
    t1 = + r0*q2 + r1*q3 - r2*q0 + r3*q1
    t2 = - r0*q1 + r1*q0 + r2*q3 + r3*q2
    t3 = - r0*q0 - r1*q1 - r2*q2 + r3*q3
    return (t0, t1, t2, t3)


def angle_between_quaternions(q1, q2):
    def dot(x, y):
        return sum(x_i * y_i for x_i, y_i in zip(x, y))
    return math.acos(2 * dot(q1, q2) ** 2 - 1)
