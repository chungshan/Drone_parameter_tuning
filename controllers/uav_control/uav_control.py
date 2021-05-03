"""uav_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, InertialUnit, GPS, Compass, Gyro, Motor, Keyboard
import math
import numpy as np
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
imu = robot.getInertialUnit("inertial unit")
imu.enable(timestep)
camera = robot.getCamera("camera")
camera.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
compass = robot.getCompass("compass")
compass.enable(timestep)
gyro = robot.getGyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getCamera("camera roll")
camera_pitch_motor = robot.getCamera("camera pitch")
front_left_motor = robot.getMotor("front left propeller")
front_right_motor = robot.getMotor("front right propeller")
rear_left_motor = robot.getMotor("rear left propeller")
rear_right_motor = robot.getMotor("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

# arming
for i in range(4):
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(1.0)
print("arming")
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0  
k_roll_p = 50.0
k_pitch_p = 30.0

# k_vertical_thrust = 68.494513
# k_vertical_offset = 0.6175035
# k_vertical_p = 2.958583
# k_roll_p = 57.4724170
# k_pitch_p = 26.832327


target_altitude = 1.0 
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
roll_disturbance = 0
pitch_disturbance = 0
yaw_disturbance = 0
# target
target_z = 1
target_x = 0
target_y = 0
target_yaw = 0
i = 0
def convert_to_pitch_roll(ex, ey, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([ex, ey], R)
    # print("ex = %f, ey = %f" % (ex, ey))
    # print(yaw)
    # print("ex_ = %f, ey_ = %f" % (exy_[0], exy_[1]))
    return exy_[0], exy_[1]
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    altitude = gps.getValues()[1]
    px = gps.getValues()[0]
    py = gps.getValues()[2]
    # print("px = %f, py = %f" %(px, py))
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    
    # keyboard input

    # Compute the roll, pitch, yaw and vertical inputs.
    pitch_err, roll_err = convert_to_pitch_roll(px - target_x, target_y - py, yaw)
    # roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration + 1*(target_y - py)
    # pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration + 20*(px - target_x)
    roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration - roll_err
    pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration - pitch_err
    yaw_input = 0.1*(target_yaw - yaw)
    clamped_difference_altitude = np.clip(target_z - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)
    # Actuate the motors taking into consideration all the computed inputs.
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    
    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)


    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
