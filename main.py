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

# Motors are reversed. If you find your robot going backwards, set this to 1
motor_multiplier = 1

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
    power_left = (motor_multiplier * power_left) / 100
    power_right = (motor_multiplier * power_right) / 100

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
    motor_left.stop()
    motor_right.stop()

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
# Define my own exception to make it clear when I want to quit

class MainMenuException(Exception):
    pass

class RobotStopException(Exception):
    pass

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
                set_speeds(100,100)
                sleep(button_delay)

            if (buttons & cwiid.BTN_DOWN):
                print ("Down pressed")
                set_speeds(-100,-100)
                sleep(button_delay)

            if (buttons & cwiid.BTN_LEFT):
                print ("Left pressed")
                set_speeds(100,-100)
                sleep(button_delay)         

            if(buttons & cwiid.BTN_RIGHT):
                print ("Right pressed")
                set_speeds(-100,100)
                sleep(button_delay)          

            stop_motors()

    # ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

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
        print ("Too close")
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
        print ("Too close")
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
        print ("Too close")
        stop_time = start_time
    
    distance = (stop_time - start_time) * 34326
    distance = distance / 2

    # print "%.4f" % distance
    return distance

################################################################################
# Automated straight line driving using distance sensors

def straightDriving():
    rightDistance = readRightDistance()
    leftDistance = readLeftDistance()
    
    power = [100,100]
        
    if (rightDistance > leftDistance):
        power[0] = 70
        power[1] = 100
        print ("Turn Right {:0.3f} {:0.3f} - {:0.1f} {:0.1f}".format(rightDistance, leftDistance, power[0], power[1]))
        
    elif (rightDistance < leftDistance):
        power[0] = 100
        power[1] = 70
        print ("Turn Left  {:0.3f} {:0.3f} - {:0.1f} {:0.1f}".format(rightDistance, leftDistance, power[0], power[1]))
        
    else:
        power[0] = 100
        power[1] = 100
        print ("Straight   {:0.3f} {:0.3f} - {:0.1f} {:0.1f}".format(rightDistance , leftDistance, power[0], power[1]))
        
    set_speeds(power[0], power[1])

# =============================================================================

def straightLineSpeed():
    switch_on_leds()

    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1
    is_moving = False

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                print ("1 pressed - Auto Drive")
                is_moving = True
                set_speeds(100,100)

            if (buttons & cwiid.BTN_2):
                print ("2 pressed - Stop")
                is_moving = False
                stop_motors()
            
            if (is_moving):
                straightDriving()
            
            sleep(button_delay)
            
    # ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

################################################################################
# Minimal Maze using distance sensors

def rotateLeft(angle):
    #stop_motors()
    set_speeds(100,-100)
    time = angle / 140
    sleep(time)         

# =============================================================================

def rotateRight(angle):
    #stop_motors()
    set_speeds(-100,100)
    time = angle / 140
    sleep(time)         
    
# =============================================================================

def minimalMazeDriving():
    frontDistance = readFrontDistance()
    print ("Front Distance {:0.3f}".format(frontDistance))

    if (frontDistance < 25.0):
        rightDistance = readRightDistance()
        leftDistance = readLeftDistance()

        print (" - Right Distance {:0.3f}".format(rightDistance))
        print (" - Left Distance  {:0.3f}".format(leftDistance))

        if (rightDistance < leftDistance):
            print ("  - Left Turn")
            rotateLeft(90)
            
        else:
            print ("  - Right Turn")
            rotateRight(90)
    
    set_speeds(100,100)
    return True

# =============================================================================

def MinimalMaze():
    switch_on_leds()
            
    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1
    is_moving = False

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                print ("1 pressed - Auto Drive")
                is_moving = True
                set_speeds(100,100)

            if (buttons & cwiid.BTN_2):
                print ("2 pressed - Stop")
                is_moving = False
                stop_motors()
            
            if (is_moving):
                is_moving = minimalMazeDriving()

            sleep(button_delay)

# ==========================================================================

    except MainMenuException:
        # This exception will be raised when the home button is pressed, at which
        # point we should stop the motors.
        stop_motors()
        switch_off_leds()

# =============================================================================

def TestMode():
    switch_on_leds()
            
    wii.rpt_mode = cwiid.RPT_BTN
    button_delay = 0.1
    is_moving = False

    try:    
        while True:

            buttons = wii.state['buttons']

            # If Home button pressed return to menu.
            if (buttons & cwiid.BTN_HOME):
                raise MainMenuException()
          
            if (buttons & cwiid.BTN_1):
                print ("1 pressed - Straight")

                for t in range (0, 20):
                    r1 = readRightDistance()
                    l1 = readLeftDistance()
                    print ("Distance 1 - l:{:0.3f} r:{:0.3f}".format(11, r1))
                    set_speeds(100,100)
                    sleep(0.5)
                    r2 = readRightDistance()
                    l2 = readLeftDistance()
                    print ("Distance 2 - l:{:0.3f} r:{:0.3f}".format(l2, r2))
                    print ("Difference - {:0.3f}".format(r1-r2))
                    angle = math.degrees(math.asin((r1-r2)/20))
                    print ("Angle {:0.3f}".format(angle))
                    if (angle > 0):
                        rotateLeft(angle)
                    else:
                        rotateRight(-angle)
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
                MinimalMaze()
                print ("Minimal Maze - Exited")
                
            elif (mode == 4):
                print ("Test - Started")        
                TestMode()
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

        
         