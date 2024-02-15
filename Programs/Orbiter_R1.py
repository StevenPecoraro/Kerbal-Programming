import math
import time
import krpc
from Maneuvers import *

# Orbital Parameters
turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 100000

# Set Up
conn = krpc.connect(name='Launch into orbit')
sc = conn.space_center
vessel = sc.active_vessel

# Set up streams for telemetry
ut = conn.add_stream(getattr, sc, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

# Pre-launch setup
Autopilot(vessel)
vessel.control.throttle = 1.0
pitch = 90.0
vessel.auto_pilot.target_pitch_and_heading(90.0, 90.0)

# Countdown...
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')
vessel.control.activate_next_stage()

# Main ascent loop
turn_angle = 0.0
while True:
    # Gravity turn
    if turn_start_altitude < altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90.0
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90.0 - turn_angle, 90.0)
    check_engine(vessel)
    if apoapsis() > target_altitude * 0.95:
        print('Approaching target apoapsis')
        break

# Disable engines when target apoapsis is reached
vessel.control.throttle = 0.5

while apoapsis() < target_altitude:
    check_engine(vessel)
    pass

vessel.control.throttle = 0.0
print('Target apoapsis reached')
time.sleep(1)

# Wait until out of atmosphere to Jettison Fairings
print('Coasting out of atmosphere')
while altitude() < 70100:
    time.sleep(.1)
    pass
time.sleep(1)
vessel.parts.fairings[0].jettison()
time.sleep(1)

# Plan circularization burn (using vis-viva equation)
print('Planning circularization burn')
GM = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(GM * ((2.0 / r) - (1.0 / a1)))
v2 = math.sqrt(GM * ((2.0 / r) - (1.0 / a2)))
delta_v = v2 - v1
node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=-delta_v)
# burning time

# Rendezvous.main(conn)

# Node_Execute.main(conn)

# Ending orbital maneuvers
vessel.control.remove_nodes()
vessel.auto_pilot.disengage()
vessel.control.sas = True
print('Launch complete')
