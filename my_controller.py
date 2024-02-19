"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

ds = []
for i in range(8):
    ds.append(robot.getDevice('ps' + str(i)))
    ds[-1].enable(timestep)
    
ls = []
for i in range(8):
    ls.append(robot.getDevice('ls' + str(i)))
    ls[-1].enable(timestep)


state = 'FORWARD'
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    d = []
    
    for dist in ds:
        d.append(dist.getValue())
    #print(d)
    
    if state == 'FORWARD':
        motor_left.setVelocity(3.14)
        motor_right.setVelocity(3.14)
        if (d[0] > 110):
            state = 'TURN AROUND'
    elif state == 'TURN AROUND':
        motor_left.setVelocity(-3.14)
        if (d[4] > 110):
            state = 'RETURN TRIP'
    elif state == 'RETURN TRIP':
        motor_left.setVelocity(3.14)
        if (d[0] > 110):
            state = 'SWITCH'
    elif state == 'SWITCH':
        motor_right.setVelocity(-3.14)
        if (d[5] > 121):
            state = 'FINAL STRETCH'
    elif state == 'FINAL STRETCH':
        motor_right.setVelocity(3.14)
        if (d[5] < 100):
            state = 'STOP'
    else:
        motor_left.setVelocity(0)
        motor_right.setVelocity(0)
        
    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    # motor.setPosition(10.0)
    #motor_left.setVelocity(3.14)
    #motor_right.setVelocity(3.14)

    pass

# Enter here exit cleanup code.
