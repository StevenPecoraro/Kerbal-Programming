import time
import math
import krpc


def MatchPlanes(vessel, target):
    return


def circularize_at_intercept(vessel, target):
    return


def get_closer(vessel, target):
    return


def get_phase_angle(vessel, target):
    return


def orbital_progress(vessel, ut):
    return


def time_transfer(vessel, target, ut, phase_angle):
    return


def Hohmann_Transfer(vessel, target, time):
    return


def circularize_at_apoapsis(sc, vessel, ut):
    return


def matchv(sc, vessel, target):
    return


def close_dist(sc, vessel, target):
    return


def check_engine(vessel):
    for engine in vessel.parts.engines:
        if engine.active:
            engine = vessel.parts.engines[len(vessel.parts.engines) - 1]
    if engine.available_thrust == 0.0 and engine.active:
        vessel.control.activate_next_stage()


def Autopilot(vessel):
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.auto_pilot.engage()
    return


def Main_Ascent():
    Gravity_Turn()
    Circularization()


def Gravity_Turn():
    return


def Circularization():
    return


def burn_power(vessel):
    node = vessel.control.nodes[0]
    delta_v = node.delta_v
    TWR = vessel.max_thrust / vessel.mass
    if delta_v < TWR / 3:
        return .05
    elif delta_v < TWR / 2:
        return .1
    elif delta_v < TWR:
        return .25
    else:
        return 1.0


def burn_time(vessel):
    delta_v = vessel.control.node[0].delta_v
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.81
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v / Isp)
    flow_rate = F / Isp
    time_to_burn = (m0 - m1) / flow_rate
    return time_to_burn


def Return_Controls(vessel):
    vessel.auto_pilot.disengage()
    vessel.control.sas = True
    vessel.control.sas_mode.stability_assist = True
    vessel.control.remove_node()
    print("She's all yours!")
    return


# Aparent helper functions below, Add as needed


def clamp_2pi(x):
    return


def v3minus(vessel, target):
    return


def target(vessel, target):
    return


def anti_target(vessel, target):
    return


def target_vplus(vessel, target):
    return


def target_vminus(vessel, target):
    return


def dist(vessel, target):
    return


def speed(vessel, target):
    return


def dot3(vessel, target):
    return vessel[0] * target[0] + vessel[1] * target[1] + vessel[2] * target[2]


def cross3(vessel, target):
    return
