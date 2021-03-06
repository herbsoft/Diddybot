################################################################################
# Main program for controlling DiddyBot. Out PiWars 2018 robot which is based on
# CamJam EduKit 3 Robot
#
# By Scott Parsley, based on code by Tom Oinn, Emma Norling and Mike Horne
#
################################################################################

################################################################################
# Various imports

# Need floating point division of integers
from __future__ import division     # Gives floating point division

import cwiid
import RPi.GPIO as GPIO
import math

from gpiozero import CamJamKitRobot
from gpiozero import PWMLED
from time import sleep, time

################################################################################
#

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

rightPinTrigger = 18
rightPinEcho = 27
leftPinTrigger = 22
leftPinEcho = 23
frontPinTrigger = 24
frontPinEcho = 25

GPIO.setup(rightPinTrigger, GPIO.OUT)
GPIO.setup(rightPinEcho, GPIO.IN)
GPIO.setup(leftPinTrigger, GPIO.OUT)
GPIO.setup(leftPinEcho, GPIO.IN)
GPIO.setup(frontPinTrigger, GPIO.OUT)
GPIO.setup(frontPinEcho, GPIO.IN)

GPIO.output(leftPinTrigger, False)
GPIO.output(rightPinTrigger, False)
GPIO.output(frontPinTrigger, False)

################################################################################
# Initialise the robot object

# Get the robot instance and the independent motor controllers
robot = CamJamKitRobot()
motor_left = robot.left_motor
motor_right = robot.right_motor

################################################################################
# Define my own exception to make it clear when I want to quit

class MainMenuException(Exception):
    pass

class RobotStopException(Exception):
    pass

################################################################################
# Initialise the LED objects

led_main = PWMLED(21)
led_left = PWMLED(20)
led_right = PWMLED(26)

################################################################################
# Methods to drive the motors

def set_speeds(power_left, power_right):
    """
    :param power_left: 
        Power to send to left motor
    :param power_right: 
        Power to send to right motor, will be inverted to reflect chassis layout
    """
    
    # Take the 0-100 inputs down to 0-1 and reverse them if necessary
    power_left = (power_left / 100)
    power_right = (power_right / 100)
    
    # If power is less than 0, we want to turn the motor backwards, otherwise
    # turn it forwards
    
    if power_left < 0:
        motor_left.backward(-power_left)
    else:
        motor_left.forward(power_left)

    if power_right < 0:
        motor_right.backward(-power_right)
    else:
        motor_right.forward(power_right)

# ==============================================================================

def stop_motors():
    robot.stop()

# =============================================================================

def rotateLeft(speed, angle):
    robot.left(speed/100)
    time = (angle / 220) * (100 / speed)
    sleep(time)         

# =============================================================================

def rotateRight(speed, angle):
    robot.right(speed/100)
    time = (angle / 220) * (100 / speed)
    sleep(time)

################################################################################
# Methods to control the lights

def count_led(number):
    if (number == 1):
        led_left.value = 1
        led_main.value = 0
        led_right.value = 0

    elif (number == 2):
        led_left.value = 0
        led_main.value = 1
        led_right.value = 0

    elif (number == 3):
        led_left.value = 1
        led_main.value = 1
        led_right.value = 0

    elif (number == 4):
        led_left.value = 0
        led_main.value = 0
        led_right.value = 1
        
# ==============================================================================

def flash_led(led, times):
    for x in range(0, times):
        led.value = 1
        sleep(0.5)
        led.value = 0
        sleep(0.5)
        
    led.value = 0

# ==============================================================================

def switch_on_leds():
    led_main.value = 1
    led_left.value = 1
    led_right.value = 1

# ==============================================================================

def switch_off_leds():
    led_main.value = 0
    led_left.value = 0
    led_right.value = 0

################################################################################
# Distance sensors

def readRightDistance():
    GPIO.output(rightPinTrigger, True)
    sleep(0.00001)
    GPIO.output(rightPinTrigger, False)
    
    start_time = time()
    
    while GPIO.input(rightPinEcho) == 0:
        start_time = time()
        
    while GPIO.input(rightPinEcho) == 1:
        stop_time = time()
        
    if stop_time - start_time >= 0.04:
        print ("Too close - Right")
        stop_time = start_time
    
    distance = (stop_time - start_time) * 34326
    distance = distance / 2

    # print "%.4f" % distance
    return distance

# =============================================================================

def readLeftDistance():
    GPIO.output(leftPinTrigger, True)
    sleep(0.00001)
    GPIO.output(leftPinTrigger, False)
    
    start_time = time()
    
    while GPIO.input(leftPinEcho) == 0:
        start_time = time()
        
    while GPIO.input(leftPinEcho) == 1:
        stop_time = time()
        
    if stop_time - start_time >= 0.04:
        print ("Too close - Left")
        stop_time = start_time
    
    distance = (stop_time - start_time) * 34326
    distance = distance / 2

    # print "%.4f" % distance
    return distance

# =============================================================================

def readFrontDistance():
    GPIO.output(frontPinTrigger, True)
    sleep(0.00001)
    GPIO.output(frontPinTrigger, False)
    
    start_time = time()
    
    while GPIO.input(frontPinEcho) == 0:
        start_time = time()
        
    while GPIO.input(frontPinEcho) == 1:
        stop_time = time()
        
    if stop_time - start_time >= 0.04:
        print ("Too close - Front")
        stop_time = start_time
    
    distance = (stop_time - start_time) * 34326
    distance = distance / 2

    # print "%.4f" % distance
    return distance

# =============================================================================

def readSideDistances():
    rightDistance = readRightDistance()
    leftDistance = readLeftDistance()
    return leftDistance, rightDistance

# =============================================================================

def readDistances():
    rightDistance = readRightDistance()
    leftDistance = readLeftDistance()
    frontDistance = readFrontDistance()
    return leftDistance, rightDistance, frontDistance

################################################################################
# Button inputs for manual driving mode

def manualDriving():
    switch_on_leds()

    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_UP):
                print ("Up pressed")        
                # set_speeds(100,100)
                robot.forward()
                sleep(button_delay)

            if (buttons & cwiid.BTN_DOWN):
                print ("Down pressed")
                # set_speeds(-100,-100)
                robot.backward()
                sleep(button_delay)

            if (buttons & cwiid.BTN_LEFT):
                print ("Left pressed")
                # set_speeds(100,-100)
                robot.left()
                sleep(button_delay)         

            if(buttons & cwiid.BTN_RIGHT):
                print ("Right pressed")
                # set_speeds(-100,100)
                robot.right()
                sleep(button_delay)          

            stop_motors()

    # ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

################################################################################
# Automated straight line driving using distance sensors

def driveStraight(speed, distance, outDistance):
    calibrate = 60.0 * (speed / 100)   # How far DiddyBot moves in one second

    l1, r1, f1 = readDistances()
    print ("Distance 1 - l:{:0.3f} r:{:0.3f} f:{:0.3f}".format(l1, r1, f1))

    moveDistance = min(f1 - 25.0, distance)
    
    # If too close to a wall in front then stop
    if (moveDistance < 0):
        stop_motors()
        print ("Too close to object")
        return 1
    
    # print ("Moving {:0.3f}".format(moveDistance))
    set_speeds(speed,speed)
    sleep(moveDistance / calibrate)

    l2, r2, f2 = readDistances()
    print ("Distance 2 - l:{:0.3f} r:{:0.3f} f:{:0.3f}".format(l2, r2, f2))

    # If moved out of walls then stop
    if (l2 > outDistance and r2 > outDistance):
        stop_motors()
        print ("No walls found")
        return 2

    angle = 0

    try:    
        # Multiply by two to bring strainght and then back towards center?
        if (l1 < outDistance and l2 < outDistance):
            print ("Covered (Left) - {:0.3f} over {:0.3f}".format(l2-l1, moveDistance))
            angle = 2 * math.degrees(math.asin((l2-l1)/moveDistance))
        elif (r1 < outDistance and r2 < outDistance):
            print ("Covered (Right) - {:0.3f} over {:0.3f}".format(r1-l2, moveDistance))
            angle = 2 * math.degrees(math.asin((r1-r2)/moveDistance))

    except ValueError:
        # Can I decide a better angle in this case?
        angle = 0

    if (angle > 0.0):
        print ("Rotate left {:0.3f}".format(angle))
        rotateLeft(speed, angle)
    else:
        print ("Rotate right {:0.3f}".format(-angle))
        rotateRight(speed, -angle)

    set_speeds(speed,speed)
    return 0

# =============================================================================

def straightLineSpeed():
    switch_on_leds()

    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1
    is_moving = False

    # Dummy read to warm sensors :)
    l1, r1, f1 = readDistances()
    print ("Starting Distances - l:{:0.3f} r:{:0.3f} f:{:0.3f}".format(l1, r1, f1))

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                print ("1 pressed - Auto Drive")
                is_moving = True

            if (buttons & cwiid.BTN_2):
                print ("2 pressed - Stop")
                is_moving = False
            
            if (is_moving):
                is_moving = (driveStraight(100, 40, 40) == 0)
            else:
                stop_motors()
            
            sleep(button_delay)
            
    # ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

################################################################################
# Minimal Maze using distance sensors
    
def minimalMazeDriving():
    speed = 50
    result = driveStraight(speed, 20, 60)
    
    # 0 - drove straight, 1 - stopped due to wall, 2 - out of maze
    # Motors still going on 0, but stopped if 1 or 2
    
    if (result == 0):
        return True
    
    if (result == 1):
        left, right = readSideDistances()
        if (left > right):
            print ("Rotate Left")
            rotateLeft(speed, 90)
        else:
            print ("Rotate Right")
            rotateRight(speed, 90)
        
        return True

    if (result == 2):
        # There's a small chance this is a false answer
        rotateRight(speed, 10)
    
    return False

# =============================================================================

def minimalMaze():
    switch_on_leds()

    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1
    is_moving = False

    # Dummy read to warm sensors :)
    l1, r1, f1 = readDistances()
    print ("Starting Distances - l:{:0.3f} r:{:0.3f} f:{:0.3f}".format(l1, r1, f1))

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                print ("1 pressed - Auto Drive")
                is_moving = True

            if (buttons & cwiid.BTN_2):
                print ("2 pressed - Stop")
                is_moving = False
            
            if (is_moving):
                is_moving = minimalMazeDriving()
            else:
                stop_motors()
            
            sleep(button_delay)
            
    # ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

# =============================================================================

def testMode():
    switch_on_leds()
            
    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1

    leftSpeed = 100
    rightSpeed = 100

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                dummy = readFrontDistance()
                
                for x in range (0, 2):
                    d1 = readFrontDistance()
                    set_speeds(leftSpeed,rightSpeed)
                    sleep(1.0)
                    stop_motors()
                    d2 = readFrontDistance()
                    print ("Discreet Move {:0.3f}".format(d1-d2))
                
            if (buttons & cwiid.BTN_2):
                dummy = readFrontDistance()
                
                set_speeds(leftSpeed,rightSpeed)
                sleep(0.5)
                for x in range (0, 2):
                    d1 = readFrontDistance()
                    sleep(1.0)
                    d2 = readFrontDistance()
                    print ("Continuous Move {:0.3f}".format(d1-d2))
                stop_motors()
                
            if (buttons & cwiid.BTN_MINUS):
                rightSpeed -= 2
                print ("Right Speed {:d}".format(rightSpeed))

            if (buttons & cwiid.BTN_PLUS):
                rightSpeed += 2
                print ("Right Speed {:d}".format(rightSpeed))

            if (buttons & cwiid.BTN_LEFT):
                set_speeds(leftSpeed, rightSpeed)
                sleep(2.0)
                rotateLeft(rightSpeed,90)
                stop_motors()

            if (buttons & cwiid.BTN_RIGHT):
                set_speeds(leftSpeed, rightSpeed)
                sleep(2.0)
                rotateRight(rightSpeed,90)
                stop_motors()
                
            sleep(button_delay)

# ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

################################################################################
# Main menu loop
#
# 1.  Manual driving
# 2.  Straight-Line Speed
# 3.  Minimal Maze
# 4.  Somewhere Over the Rainbow
#

try:
    # --------------------------------------------------------------------------
    # Connect to the Controller. If it times out
    # then quit.

    led_main.value = 1
    sleep(1)

    try:
        wii=cwiid.Wiimote()
    except RuntimeError:
        led_main.value = 0
        raise RobotStopException()
    
    wii.rumble = 1
    sleep(0.2)
    wii.rumble = 0
    wii.led = 1
    
    flash_led(led_main, 3)

    print ("Connected - Use left, right and A to pick mode")        

    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.2
    listening = True
    
    mode = 1
    max_mode = 4
    
    switch_off_leds()

    while listening:
        buttons = wii.state['buttons']
        count_led(mode)        

        if (buttons & cwiid.BTN_LEFT):
            mode -= 1
            if (mode < 1):
                mode = max_mode
            print ("Menu Left pressed")
            print (mode)
            sleep(button_delay)            

        if (buttons & cwiid.BTN_RIGHT):
            mode += 1
            if (mode > max_mode):
                mode = 1
            print ("Menu Right pressed")
            print (mode)
            sleep(button_delay)            

        if (buttons & cwiid.BTN_A):
            print ("Menu A pressed")
            
            if (mode == 1):
                print ("Manual Driving Mode - Started")        
                manualDriving()
                print ("Manual Driving Mode - Exited")        

            elif (mode == 2):
                print ("Straight Line Mode - Started")        
                straightLineSpeed()
                print ("Straight Line Mode - Exited")
                
            elif (mode == 3):
                print ("Minimal Maze - Started")        
                minimalMaze()
                print ("Minimal Maze - Exited")
                
            elif (mode == 4):
                print ("Test - Started")        
                testMode()
                print ("Test - Exited")
                
            sleep(button_delay)

        # If Plus and Minus buttons pressed
        # together then rumble and quit.
        if (buttons - cwiid.BTN_PLUS - cwiid.BTN_MINUS == 0):  
            print ("\nClosing connection ...")
            wii.rumble = 1
            sleep(0.2)
            wii.rumble = 0
            raise RobotStopException()
      
# ==============================================================================

except RobotStopException:
    # This exception will be raised when the home button is pressed, at which point we should
    # stop the motors.
    stop_motors()
    switch_off_leds()

        
         
