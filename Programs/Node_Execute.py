import time
import krpc
import math
from Maneuvers import *

def execute_burn(conn, vessel, node):

    #targeting
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    Autopilot(vessel)
    rf = vessel.orbit.body.reference_frame
    vessel.auto_pilot.reference_frame = rf
    vessel.auto_pilot.target_direction = node.remaining_burn_vector(rf)
    vessel.auto_pilot.wait()
    delta_v = node.delta_v

    #calculate burn time
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.81
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    #time warp to node
    burn_ut = node.ut - (burn_time/2.0)
    conn.space_center.warp_to(burn_ut - 5.0)

    #executing burn
    while ut() < burn_ut:
        pass

    starting_burn_power = burn_power(vessel)
    vessel.control.throttle = starting_burn_power
    while node.remaining_delta_v > 0.1:
        if node.remaining_delta_v < delta_v/20:
            vessel.control.throttle = starting_burn_power/5
        vessel.auto_pilot.target_direction = node.remaining_burn_vector(rf)
        Check_Engine(vessel)
        pass
    vessel.control.throttle = 0.0

    #returning to user
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    vessel.control.sas_mode.stability_assist = True
    vessel.control.remove_nodes()

#tune the thrust of the burn
def burn_power(vessel, delta_v):
    TWR= vessel.max_thrust/vessel.mass
    if delta_v < TWR / 3:
        return .05
    elif delta_v < TWR / 2:
        return .1
    elif delta_v < TWR:
        return .25
    else:
        return 1.0


conn = krpc.connect(name="Current Vessel")
vessel = conn.space_center.active_vessel
node = vessel.control.nodes[0]
#burn baby burn
execute_burn(conn, vessel, node)