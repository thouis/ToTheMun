from numpy import pi, dot, cross, array, arccos, sign
from numpy.linalg import norm


def angle_between(obj1, obj2):
    frame = obj1.orbit.body.non_rotating_reference_frame
    pos1 = array(obj1.position(frame))
    pos2 = array(obj2.position(frame))

    # find the cross product from pos1 to pos2
    pos1 /= norm(pos1)
    pos2 /= norm(pos2)

    # angle from dot product
    angle = arccos(dot(pos1, pos2))

    # flip if cross product opposite orbital normal of first object
    cp = cross(pos1, pos2)
    cp_local = conn.space_center.transform_direction(cp, frame, obj1.orbital_reference_frame)
    if cp_local[2] > 0:  # left handed coordinate system
        angle = - angle

    return angle


def time_until_phase(obj1, obj2, phase):
    ''' find time until angle between obj1 and obj2 is phase '''
    angle = angle_between(obj1, obj2)
    print("angle between is", angle)

    # find orbital radial velocity
    radians_per_second_1 = 2 * pi / obj1.orbit.period
    radians_per_second_2 = 2 * pi / obj2.orbit.period

    # assume obj2 is running away, obj1 is chasing
    relative_angle_change_per_second = radians_per_second_2 - radians_per_second_1
    print("p, a, r", phase, angle, relative_angle_change_per_second)
    delta = phase - angle
    while sign(delta) != sign(relative_angle_change_per_second):
        delta += sign(relative_angle_change_per_second) * 2 * pi
    return delta / relative_angle_change_per_second


if __name__ == '__main__':
    import krpc

    conn = krpc.connect(name='Launch into orbit')
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    vessel = conn.space_center.active_vessel
    r_kerbin = vessel.orbit.semi_major_axis
    mun = conn.space_center.bodies['Mun']
    r_mun = mun.orbit.semi_major_axis
    transfer_phase = pi * (1 - (((r_kerbin + r_mun) / (2 * r_mun)) ** 1.5))
    print("phase", transfer_phase)

    dt = time_until_phase(vessel, mun, transfer_phase)

    print("warp", dt)
    conn.space_center.warp_to(ut() + dt)
    print("ang", angle_between(vessel, mun))
