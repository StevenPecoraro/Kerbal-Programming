import time
import krpc
import math
from Node_Execute import *
from Maneuvers import *

conn = krpc.connect(name='Attempt Rendezvous')
sc = conn.space_center
vessel = sc.active_vessel

ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

Autopilot(vessel)

Main_Ascent()

Hohmann_Transfer(conn)
circularize_at_intercept(conn)
get_closer(conn)