"""
April 24, 2026

Ardupilot is unable to stabilize drone. When I add throttle, 
it flips over towards its front left direction. Assuming it's due to motor 
assignment differences between Betaflight and Ardupilot (the drone flies properly
on Betaflight) which I assigned accordingly yesterday but didn't have time to test.
To efficiently test this after school today, I'm gonna code up a program to just 
detect motor values and potentially the respective yaw/pitch/roll to determine if 
the motors are working properly

"""

from pymavlink import mavutil

def estimate_orientation(motor_speeds):
    m1 = motor_speeds[0]
    m2 = motor_speeds[1]
    m3 = motor_speeds[2]
    m4 = motor_speeds[3]

    left = m1 + m4
    right = m2 + m3
    roll = right - left  
    
    front = m1 + m2
    back = m3 + m4
    pitch = front - back  

    cw = m1 + m3
    ccw = m2 + m4
    yaw = cw - ccw  

    
    if abs(roll) < 50:
        roll_dir = "stable roll"
    elif roll > 0:
        roll_dir = "roll right"
    else:
        roll_dir = "roll left"

    if abs(pitch) < 50:
        pitch_dir = "stable pitch"
    elif pitch > 0:
        pitch_dir = "pitch forward"
    else:
        pitch_dir = "pitch backward"
    
    if abs(yaw) < 50:
        yaw_dir = "stable yaw"
    elif yaw > 0:
        yaw_dir = "yaw clockwise"
    else:
        yaw_dir = "yaw counter-clockwise"

    return roll, pitch, yaw, roll_dir, pitch_dir, yaw_dir

#gpt, get heartbeat from flight controller
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Connected", master.target_system)

while True:
    #gpt
    msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    
    if msg:
        motor_speeds = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        # First 4 motor channels
        print(
            f"Motor1: {motor_speeds[0]}  "
            f"Motor2: {motor_speeds[1]}  "
            f"Motor3: {motor_speeds[2]}  "
            f"Motor4: {motor_speeds[3]}\n"
        )
        roll, pitch, yaw, roll_dir, pitch_dir, yaw_dir = estimate_orientation(motor_speeds)
        print(f"roll: {roll}\npitch: {pitch}\nyaw: {yaw}\nyaw dir: {yaw_dir}\nroll dir: {roll_dir}\npitch dir: {pitch_dir}\n")


