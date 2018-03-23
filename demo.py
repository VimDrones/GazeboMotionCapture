print "Start simulator (SITL)"
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

import time
import GazeboMotionCapture
position_estimate = GazeboMotionCapture.PositionEstimate()

def callback(id, position, orientation):
    print(id, position, orientation)
    if not position_estimate.init:
        position_estimate.first_position = position
        position_estimate.init = True

    position_estimate.receive_new_position(position[0], position[1], position[2])
    if position_estimate.is_ready():
        msg = vehicle.message_factory.att_pos_mocap_encode(
            int(round(time.time() * 1000)),
            orientation,
            position_estimate.ned_position[0],
            position_estimate.ned_position[1],
            position_estimate.ned_position[2])
        vehicle.send_mavlink(msg)


GazeboMotionCapture.callback = callback
motion_capture = GazeboMotionCapture.create_motion_capture(('192.168.1.49', 11345))

motion_capture.start()

time.sleep(10)
motion_capture.exit()
