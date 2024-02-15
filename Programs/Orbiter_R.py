from multiprocessing import Process
from time import sleep
from Maneuvers import *
from multiprocessing import Process

# Orbital Parameters
turn_start_altitude = 250
turn_end_altitude = 45000
target_apoapsis = 100000

# Set Up
conn = krpc.connect(name='Launch into orbit')
sc = conn.space_center
vessel = sc.active_vessel

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')


"""def run_cpu_tasks_in_parallel(tasks):
    running_tasks = [Process(target=task) for task in tasks]
    for running_task in running_tasks:
        running_task.start()
    for running_task in running_tasks:
        running_task.join()"""


def apoapsis_please(vessel):
    while True:
        print(vessel.orbit.apoapsis)
        time.sleep(1)


def main_func(conn):
    Autopilot(vessel)
    conn.space_center.active_vessel.control.throttle = 1.0
    pitch = 90.0
    vessel.auto_pilot.target_pitch_and_heading(pitch, 90.0)
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')
    vessel.control.activate_next_stage()


if __name__ == '__main__':
    run_cpu_tasks_in_parallel([
        apoapsis_please(vessel),
        main_func(conn),
    ])

# Countdown...

# Main ascent loop
# Gravity turn
while True:
    if turn_start_altitude < altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        # curve equation
        new_pitch = 90 - (pow(frac, 2) * 90)
        if abs(new_pitch - pitch) > 0.5:
            pitch = new_pitch
            vessel.auto_pilot.target_pitch_and_heading(new_pitch, 90.0)
    check_engine(vessel)
    if apoapsis() > target_apoapsis * 0.95:
        print('Approaching target apoapsis')
        break

slow_down_apoapsis = target_apoapsis * 0.95
# Disable engines when target apoapsis is reached
while apoapsis() < target_apoapsis:
    starting_burn_power = vessel.available_thrust / vessel.mass
    slow_down_remaining = (target_apoapsis - apoapsis()) / slow_down_apoapsis
    new_burn_power = starting_burn_power * pow(slow_down_remaining, 2)
    vessel.control.throttle = new_burn_power / starting_burn_power
    pass

print('Target apoapsis reached')
vessel.control.throttle = 0.0
time.sleep(1)

# Wait until out of atmosphere to Jettison Fairings
print('Coasting out of atmosphere')
while altitude() < 70500:
    pass
vessel.parts.fairings[0].jettison()

# Plan circularization burn (using vis-viva equation)
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
v3 = math.sqrt(mu * ((2.0 / r) - (1.0 / a3)))
delta_v = v2 - v1
node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Orientate ship
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Wait until burn
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis() - (burn_time / 2.0)
lead_time = 15.0
conn.space_center.warp_to(burn_ut - lead_time)

# Re-Orientate ship
print('Re-Orientate Ship')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Execute burn
print('Ready to execute burn')
while ut() < burn_ut:
    pass

print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)

# Fine Tuning Orbital Burn
print('Fine tuning')
vessel.control.throttle = 0.2
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 0.5:
    pass
vessel.control.throttle = 0.0

# Ending orbital maneuvers
print('Launch complete')
node.remove()

vessel.control.sas = True
