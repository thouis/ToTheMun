import math
import time
import krpc
from quaternion_math import quaternion_from_rotation_vector
from orbit_math import time_until_phase, pi

turn_start_altitude = 8000
turn_end_altitude = 45000
target_altitude = 400000

conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel


# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
stage_3_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=True)
srb_fuel = conn.add_stream(stage_3_resources.amount, 'SolidFuel')
rframe = vessel.orbit.body.reference_frame
altitude = conn.add_stream(getattr, vessel.flight(rframe), 'mean_altitude')
termv = conn.add_stream(getattr, vessel.flight(rframe), 'terminal_velocity')
speed = conn.add_stream(getattr, vessel.flight(rframe), 'speed')

assert srb_fuel() > .5

# Pre-launch setup
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 0.0

# Countdown...
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

# Activate the first stage
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

# Main ascent loop
srbs_separated = False
turn_angle = 0
last_print = time.time()
desired_throttle = 0.01  # to keep reaction wheels charged
while True:
    # Gravity turn
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    # Separate SRBs when finished
    if not srbs_separated:
        if srb_fuel() < 0.1:
            vessel.control.activate_next_stage()
            srbs_separated = True
            print('SRBs separated')
            desired_throttle = vessel.control.throttle = 0.1

    # Decrease throttle when approaching target apoapsis
    if apoapsis() > target_altitude*0.95:
        print('Approaching target apoapsis')
        break

    if srbs_separated:
        if speed() > termv():
            desired_throttle *= 0.95
        else:
            desired_throttle *= 1.0 / 0.95
        desired_throttle = min(1.0, max(0.05, desired_throttle))
        vessel.control.throttle = desired_throttle
        time.sleep(0.1)  # allow throttle to adjust

    if time.time() - last_print >= 10.0:
        print("Ap at %f of target" % (apoapsis() / target_altitude))
        print("  vel %f termvel %f throttle %f" % (speed(), termv(), desired_throttle))
        last_print = time.time()

# Disable engines when target apoapsis is reached
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    pass
print('Target apoapsis reached')

# Wait until out of atmosphere
print('Disengaging autopilot, Coasting out of atmosphere')
vessel.control.throttle = 0.0
time.sleep(0.1)  # allow throttle to happen
vessel.auto_pilot.disengage()
# use SAS for prograde burn
time.sleep(0.1)  # allow auto to disengage
vessel.control.sas = True
time.sleep(0.1)  # allow SAS to turn on
vessel.control.sas_mode = conn.space_center.SASMode.prograde

# Orientate ship
print('Orientating ship for circularization burn')

# coast out of atmosphere
# while altitude() < 70500:
#     pass

# Plan circularization burn (using vis-viva equation)
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Wait until burn
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 15  # warp turns off SAS, give time to reorient
conn.space_center.warp_to(burn_ut - lead_time)

# Execute burn
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time)
vessel.control.throttle = 0.0
node.remove()

print('Launch complete, computing angle for Hohmann transfer')
r_kerbin = vessel.orbit.semi_major_axis
mun = conn.space_center.bodies['Mun']
r_mun = mun.orbit.semi_major_axis
transfer_phase = pi * (1 - (((r_kerbin + r_mun) / (2 * r_mun)) ** 1.5))
print("phase", transfer_phase)

# compute node and delta-v for transfer
dt = time_until_phase(vessel, mun, transfer_phase, conn)
v1 = math.sqrt(mu / r_kerbin)
r_ellipse = (r_kerbin + r_mun) / 2
v2 = math.sqrt(mu * ((2/r_kerbin) - (1/r_ellipse)))
delta_v = v2 - v1
print("dV", delta_v)

node = vessel.control.add_node(ut() + dt, prograde=delta_v)
conn.space_center.warp_to(ut() + dt - 5)
time.sleep(5)  # reorient

print('Executing transfer burn')
vessel.control.throttle = 0.0
time.sleep(0.01)  # let throttle go to zero
vessel.control.activate_next_stage()  # separation
time.sleep(0.01)
vessel.control.activate_next_stage()  # transfer stage
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate
print("burn time", burn_time)
vessel.control.throttle = 1.0
time.sleep(burn_time * 0.95)
vessel.control.throttle = 0.25
while apoapsis() < r_mun:
    time.sleep(0.05)
vessel.control.throttle = 0.0
node.remove()

# warp to change of influence
time_to_change_of_soi = vessel.orbit.time_to_soi_change
assert time_to_change_of_soi > 0
conn.space_center.warp_to(ut() + time_to_change_of_soi + 10)

# get new orbit parameters and prepare to stabilize orbit
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.periapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
assert delta_v < 0
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_periapsis, prograde=delta_v)  # negative delta v

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(- delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Wait until burn
print('Waiting until second circularization burn')
time_to_periapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
burn_ut = ut() + time_to_periapsis() - (burn_time / 2)
lead_time = 15  # warp turns off SAS, give time to reorient
conn.space_center.warp_to(burn_ut - lead_time)

# use SAS for reotrgrade burn
vessel.control.sas = True
time.sleep(0.1)  # allow SAS to turn on
vessel.control.sas_mode = conn.space_center.SASMode.retrograde

# Orientate ship
print('Orientating ship for second circularization burn')
while time_to_periapsis() - (burn_time/2.) > 0:
    pass
print('Executing burn - {} seconds'.format(burn_time))
vessel.control.throttle = 1.0
time.sleep(burn_time)
vessel.control.throttle = 0.0
node.remove()

# drop to last stage, prepare to land
vessel.control.activate_next_stage()  # separate transfer stage
vessel.control.activate_next_stage()  # landing & return stage
vessel.control.sas = True
time.sleep(10)  # let stage float away

# retrograde burn until we land
vessel.control.sas_mode = conn.space_center.SASMode.retrograde

# this frame has x pointing toward the surface
srf_frame = vessel.orbit.body.reference_frame


def vertical_velocity():
    velocity = vessel.velocity(srf_frame)
    position = vessel.position(srf_frame)
    normp = math.sqrt(sum(p ** 2 for p in position))
    return sum(v * p / normp for v, p in zip(velocity, position))


def status(str):
    alt = vessel.flight().bedrock_altitude
    spd = vessel.flight(srf_frame).speed
    vel = vertical_velocity()
    ttc = alt / -vel
    throttle = vessel.control.throttle
    print("  alt: {:.2f}  ttc: {:2f}  vel: {:.2f}  spd: {:.2f}  thrtl: {:.2f}".format(alt, ttc, vel, spd, throttle))


# drop orbital speed and begin fall to surface
while vessel.flight(srf_frame).speed > 100:
    status("dropping out of orbit")
    vessel.control.throttle = 1
    time.sleep(1)

vessel.control.throttle = 0

# find time to fall to around 50000 m using free fall equation
y0 = vessel.orbit.radius
mun_radius = 200000
y1 = 50000 + mun_radius
if y1 < y0:
    mu = vessel.orbit.body.gravitational_parameter
    tff = math.sqrt(y0 ** 3 / (2 * mu)) * (math.sqrt((y1 / y0) * (1 - y1 / y0)) + math.acos(math.sqrt(y1 / y0)))
    print("free fall time", tff, y0, y1, mu)
    if tff > 0:
        conn.space_center.warp_to(ut() + tff)


def adjust_throttle(too_fast, throttle, factor=0.9, minval=0.05):
    # make sure we can adjust upward from 0
    if throttle == 0:
        throttle = minval
    if too_fast:
        throttle = throttle / factor
    else:
        throttle = throttle * factor
    # clamp to 0,1 with anything below minval set to 0
    if throttle < minval:
        throttle = 0.0
    elif throttle > 1.0:
        throttle = 1.0
    return throttle


# landing loop - keep time to crash (= height / vertical velocity) above 50
# seconds or velocity above 20 m/s, until we reach 100m.
high_loop_min_fall_rate = 20
while vessel.flight().bedrock_altitude > 100:
    rate_of_fall = - vertical_velocity()
    time_to_crash = (vessel.flight().bedrock_altitude / rate_of_fall)
    too_fast = (rate_of_fall > high_loop_min_fall_rate) and (time_to_crash < 50)
    vessel.control.throttle = adjust_throttle(too_fast, vessel.control.throttle, factor=0.97)
    status("high landing loop")
    time.sleep(.2)


def linterp(a, b, w):
    # linear interpolation between a & b via weight w
    if w < 0:
        return a
    elif w > 1:
        return b
    return a + (b - a) * w


# landing - drop below 5 m/s
while vessel.flight().bedrock_altitude > 5:
    rate_of_fall = - vertical_velocity()
    # desired fall = 20 m/s @ 100m, 2 m/s @ 5m
    desired_fall = linterp(2, high_loop_min_fall_rate, (vessel.flight().bedrock_altitude - 5) / 95)
    too_fast = rate_of_fall > desired_fall
    vessel.control.throttle = adjust_throttle(too_fast, vessel.control.throttle, factor=0.97)
    status("low landing loop")
    time.sleep(0.01)

status("throttle 0 and deploying gear")
vessel.control.gear = True
time.sleep(0.1)
vessel.control.throttle = 0
