"""supervisor_tuning controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Emitter, Supervisor
from tuning import Bayes_opt
import struct


def suggest_to_gain(params_dict):
    k_vertical_thrust = params_dict["k_vertical_thrust"]
    k_vertical_offset = params_dict["k_vertical_offset"]
    k_vertical_p = params_dict["k_vertical_p"]
    k_roll_p = params_dict["k_roll_p"]
    k_pitch_p = params_dict["k_pitch_p"]
    return k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p
# create the Robot instance.
supervisor = Supervisor()
emitter = supervisor.getEmitter("emitter")
uav = supervisor.getFromDef("uav")
# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0
target_x = 0
target_y = 0
target_z = 1
reward = 0
params_dic = {"k_vertical_thrust": k_vertical_thrust, "k_vertical_offset": k_vertical_offset, "k_vertical_p": k_vertical_p, "k_roll_p": k_roll_p, "k_pitch_p": k_pitch_p}
bayes_opt = Bayes_opt(params_dic)
suggest_gain = bayes_opt.suggest()
k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p = suggest_to_gain(suggest_gain)
message = struct.pack('5f', k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p)
emitter.send(message)
iteration = 1
pose = []
eval_mode = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    message = struct.pack('5f', k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p)
    emitter.send(message)
    err_x, err_y, err_z = target_x - uav.getPosition()[0], target_y - uav.getPosition()[2], target_z - uav.getPosition()[1]
    pose.append([uav.getPosition()[0], uav.getPosition()[1], uav.getPosition()[2]])
    reward += abs(err_x) + abs(err_y) + abs(err_z)
    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    if(supervisor.getTime() > 20):
        print("===[%s] Iteration %d ===" % ("Training" if not eval_mode else "Testing", iteration))
        print(suggest_gain)
        print("total reward = %f" % -reward)

        if eval_mode:
            bayes_opt.save_pose_to_log(pose, iteration)
        else:
            bayes_opt.optimize(suggest_gain, -reward)

        if iteration % 5 == 0:
            suggest_gain = bayes_opt.best_suggest()
            eval_mode = True
        else:
            suggest_gain = bayes_opt.suggest()
            eval_mode = False

        iteration += 1
        k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p = suggest_to_gain(suggest_gain)
        reward = 0
        pose = []
        supervisor.simulationReset()
        uav.restartController()
    pass

# Enter here exit cleanup code.
