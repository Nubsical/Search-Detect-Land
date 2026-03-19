from pymavlink import mavutil

mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
mav.wait_heartbeat()
print("heartbeat detected")

mav.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_INFO,
    "\n\nAprilTag detected!\n\n".encode('utf-8')
)