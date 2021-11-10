from controller import Robot


TIME_STEP = 32
robot = Robot()

ds = []
dsNames = ['dist_sensor_left','dist_sensor_mid','dist_sensor_right']
for i in range(3):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
leftBaseSpeed =2
rightBaseSpeed=2
leftMxSpeed = 6
rightMxSpeed = 6

lasterror = 0

Kp = 0.018
Ki = 0.005
Kd = 0.45
def motorRun(leftSpeed, rightSpeed):
    wheels[0].setVelocity(leftSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[3].setVelocity(rightSpeed)

def binarizeSensorVals(sensor_vals, threshold=110):
    maximum = max(sensor_vals)
    minimum = min(sensor_vals)
    for i in range(len(sensor_vals)):
        x = sensor_vals[i]
        if abs(x-maximum) > abs(x-minimum):
            sensor_vals[i] = 1
        else:
            sensor_vals[i] = 0
    return sensor_vals

def PIDcalc(sensor_vals):
    integral = 0
    sensor_weight = 0
    sensor_sum = 0
    sensor_avg = 0
    for i in range(0,3):
        sensor_weight = sensor_weight + (i-1)*sensor_vals[i]*100
        sensor_sum = sensor_sum+sensor_vals[i]
    if sensor_sum == 0:
        sensor_sum=1
    sensor_avg = sensor_weight/sensor_sum 
    position = sensor_avg
    print(sensor_vals)
    print(position)
    error = position - 0
    integral = integral + error
    global lasterror
    motorSpeed = Kp * error + Ki*(integral) + Kd * (error - lasterror)
    lasterror = error
    rightMotorSpeed = rightBaseSpeed + motorSpeed
    leftMotorSpeed = leftBaseSpeed - motorSpeed
    
    if (rightMotorSpeed > rightMxSpeed) or (rightMotorSpeed < -1*rightMxSpeed): 
        rightMotorSpeed = rightMxSpeed if rightMotorSpeed >=0 else -1*rightMxSpeed # prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMxSpeed ) or (leftMotorSpeed < -1*leftMxSpeed):
         leftMotorSpeed = leftMxSpeed if leftMotorSpeed >=0 else -1*leftMxSpeed # prevent the motor from going beyond max speed
    print("Left Speed: ", leftMotorSpeed, " Right Speed: ", rightMotorSpeed)
    motorRun(leftMotorSpeed, rightMotorSpeed)

while robot.step(TIME_STEP) != -1:

    
    left_val= ds[0].getValue()
    mid_val= ds[1].getValue()
    right_val= ds[2].getValue()
    
    print("Left Sen {}, Middle Sen {}, Right Sen {}".format(left_val,mid_val,right_val))
    #PIDcalc([left_val, mid_val, right_val])
    PIDcalc(binarizeSensorVals([left_val, mid_val, right_val]))
    
    