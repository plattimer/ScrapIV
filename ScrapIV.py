#######################################################################################################################
# Author: Peter Lattimer and Nicholas Alva
# October 23, 2024
# Project: SCRAP IV: Rotation Senior Design Project
# Description: This is the main Driver code for the rotation of the Santa Clara Radio Antenna
#              located in SCDI 4027.
########################################################################################################################

import pyfirmata2
import time
import serial

# Defining global variables
#--------------------------------------------------------------------------#
# Delay variable for time delay in seconds
delay = 1
# Delay variable for the sound
sound_delay = 0.5

# Find where the arduino object is
port = pyfirmata2.Arduino.AUTODETECT
# Create the arduino object
board = pyfirmata2.Arduino(port)
# Pin connected to motor driver set up as pwm outputs
RMB = board.get_pin('d:10:p') #reverse motor bias
FMB = board.get_pin('d:11:p') #forward2
#LED Pins for System Diagnostics
PIN_GREEN = board.get_pin('d:5:o')
PIN_RED = board.get_pin('d:2:o')
PIN_YELLOW = board.get_pin('d:3:o')
PIN_BLUE = board.get_pin('d:4:o')

# Pin connected to the sound component
SOUND_PIN = board.get_pin('d:9:o')
# Time variables
short_time = 1
very_short_time = 0.5
# Tolerable amount of variation in the angle data when determining if the data is stable
TOLERABLE_DELTA = 1
# Size of array that will test if the data is stable. The larger the value, the more checks it will do!
test_size = 5
#--------------------------------------------------------------------------#

# BEGINING OF FUNCTION DEFINITION (!OUTDATED FUNCTION!)
# This is kind of outdated and needs to be updated to be better but it does give you roll, pitch, and yaw
# Define the function to be used to obtain and read data from the sensor
# If returns -1, it was too short, try again.
# If returns array, print what is in the array (data within array is in documentation)
def serial_O_R(ser) :
    # Listen for the data fromt the sensor
    heard_data = ser.read_until(b'UT')

    # Define the array that will hold data
    angular_array = [-1, -1, -1]

    # If statement to decide how to hand the data it         managed to read
    if len(heard_data) < 13 :

        # Data that was read was too short
        return angular_array
    else :

        # Data was long enough, process it
        data = heard_data[-13:-2]

        # Obtain and calculate Roll
        RollL = data[2]
        RollH = data[3]
        Roll_angle_x = ((RollH<<8)|(RollL))/32768.0*180.0

        # Obtain and calculate Pitch
        PitchL = data[4]
        PitchH = data[5]
        Pitch_angle_x = ((PitchH<<8)|(PitchL))/32768.0*180.0

        # Obtain and calculate Yaw
        YawL = data[6]
        YawH = data[7]
        Yaw_angle_x = ((YawH<<8)|(YawL))/32768.0*180.0

        # Load the array with data
        angular_array[0] = Roll_angle_x
        angular_array[1] = Pitch_angle_x
        angular_array[2] = Yaw_angle_x

        # Pass the angular array to the user
        return angular_array

# END OF FUNCTION DEFINITION

# BEGINING OF FUNCTION DEFINITION
# Function that drives the motor Forward
# Function takes the arduino object as an input to gain access to the arduino
# Function returns nothing
def FWB_motor(board, dangle):

    # Deactivate Stop motor
    PIN_RED.write(0)
    # Deactivate the Reverse movement
    PIN_YELLOW.write(0)
    RMB.write(0)
    # Activate the Green LED to indicate the motor is pushing forward (up)
    PIN_GREEN.write(1)
    if (dangle<10):
        FMB.write(.5)
    else:
        FMB.write(1)

# END OF FUNCTION DEFINITION

# BEGINING OF FUNCTION DEFINITION
# Function that drives the motor Backwards (Reverse)
# Function takes the arduino object as an input to gain access to the arduino
# Function returns nothing
def RVB_motor(board, dangle):

    # Deactivate Forward movement
    PIN_GREEN.write(0)
    # Deactivate stop movement
    PIN_RED.write(0)
    FMB.write(0)
    # Activate the Green LED to indicate the motor is pushing forward (up)
    PIN_YELLOW.write(1)
    if (dangle<10):
        RMB.write(.5)
    else:
        RMB.write(1)

# END OF FUNCTION DEFINITION

# BEGINING OF FUNCTION DEFINITION
# Function that drives the motor Forward
# Function takes the arduino object as an input to gain access to the arduino
# Function returns nothing
def STOP_motor(board):
    # Deactivate Forward movement
    PIN_GREEN.write(0)
    FMB.write(0)
    # Deactivate the Reverse movement
    PIN_YELLOW.write(0)
    RMB.write(0)
    # Activate the red LED to indicate the motor has been stopped
    PIN_RED.write(1)

# END OF FUNCTION DEFINITION

# BEGINING OF FUNCTION DEFINITION
# This is a function that will take no inputs
# This function returns a float that represents the roll
def serial_getRoll():

    # Open the serial port
    ser = serial.Serial('COM4', 9600, timeout = 5)

    # Listen for the data fromt the sensor
    heard_data = ser.read_until(b'UT')

    # Define a variable that will hold the angle of roll
    angle_roll = -1

    # Loop that will continue to occur until the function got the data that it needs

    # Define a flag to determine when we get the data we needed
    loop_flag = False

    while loop_flag == False :

        # If statement to decide how to hand the data it managed to read
        if len(heard_data) >= 13 :

            # There was enough data, process the angle
            # There was enough data to calculate angle of roll, process it
            data = heard_data[-13:-2]

            # Obtain and calculate Roll
            RollL = data[2]
            RollH = data[3]
            angle_roll = ((RollH<<8)|(RollL))/32768.0*180.0

            # Close the serial object
            ser.close()

            # Change the status of the loop flag just in case
            loop_flag = True

            # Return the angle of roll
            return angle_roll

        else :

            # This means that there wasn't enough data, ensure that the flag is false to keep loop going
            loop_flag = False

# END OF FUNCTION DEFINITION

# BEGINNING OF FUNCTION DEFINITION
# This is a function that will take the object connected to the arduino as an input so it knows how to communicate with the arduino
# This function returns nothing but simply plays a sound in a specific pattern
def sound_success(board) :

    # Turn the speaker on
    SOUND_PIN.write(1)
    # Wait 0.25 seconds
    time.sleep(0.25)
    # Turn the speaker off
    SOUND_PIN.write(0)

    # Wait 0.25 seconds
    time.sleep(0.25)

    # Turn the speaker on
    SOUND_PIN.write(1)
    # Wait 0.25 seconds
    time.sleep(0.25)
    # Turn the speaker off
    SOUND_PIN.write(0)

# END OF FUNCTION DEFINITION

# Function that will take some argument of angles and will oscilate over time 
# UNDER CONSTRUCTION
# def mode_osc(angle):
    # Mode Oscillate

    #if (angle< :
        #Raise untill max
    #else:
        #lower until min

# END OF FUNCTION DEFINITION

def mode_osc():
    print("")
        
# Function that will take an angle as an input and will cause actuator to push until 
# desired angle is achieved (desa)
def mode_gta(desa):
    
    # Stop the motor
    STOP_motor(board)

    # Wait short time and do nothing
    time.sleep(short_time)

    # Loop until the last five roll angles read are close in their values to prevent using crazy data as feedback

    # Flag to determine if the sensor values are stable
    tolerable = False

    # Do a while loop until the angle values are stable
    while tolerable == False :

        # Create an array that will hold the values that we're testing stability with and load it with 0s
        angle_test = [0]*test_size

        # Just a diagnostic tool
        print(angle_test)

        # Define the iterating value for the next while loop
        iter_1 = 0

        # Begin the next while loop to load angle_test
        while iter_1 != test_size:

            # Activate the blue LED to indicate that we are obtaining angular data
            PIN_BLUE.write(1)

            # Open the serial port to communicate with the sensor
            ser = serial.Serial('COM4', 9600, timeout = 5)

            # Obtain the angle of roll
            angle_temp = serial_O_R(ser)

            # Close the serial port
            ser.close()

            # Deactivate the blue LED to indicate no longer obtaining angular data
            PIN_BLUE.write(0)

            # Get the iter_1 position for our angle_test array (so we're loading the array with test_size roll angles)
            angle_test[iter_1] = angle_temp[0]

            # Diagnostic print
            print(angle_test)

            # Increate the iterator
            iter_1 = iter_1 + 1

        # Inner most while loop has ended, now we test for variation (stability of angle)

        # Define the iter_2 variable
        iter_2 = 1

        # Begin second while loop to determine stability of angle data
        while iter_2 != test_size:

            # Do an if statement to determine if the data is stable
            if abs(angle_test[0] - angle_test[iter_2]) <= TOLERABLE_DELTA :

                # Set the tolerable flag to true until told otherwise
                tolerable = True
                print('It looks good man')
                print(abs(angle_test[0] - angle_test[iter_2]))
                # The variation between them is good so iterate normally
                iter_2 = iter_2 + 1

            else :

                # If this occured, the variation was too large and hence unstable therefor it failed the check
                # Set tolerable flag to false
                tolerable = False
                # Kill the loop by setting the iter_2 to max value
                iter_2 = test_size
                print('We killed the program')

    print('The signal is stable!')

    # At this point, it has been determined that the signal from the sensor is stable, proceed with moving to desa

    # Obtain the current angle
    current_angle = round(serial_getRoll())

    # Begin the loop that will move the actuator
    while current_angle != desa :

        # Obtain the current angle
        current_angle = round(serial_getRoll())

        # If statement to determine what should be done
        if current_angle < desa :

            # Angle is too small, push the dish forward
            FWB_motor(board)
            # Delay a short amount of time
            time.sleep(short_time)
            # Stop the motor when done to prevent over correction
            STOP_motor(board)

        elif current_angle > desa :

            # The angle is too large, we need to correct our position.  Do a very small reverse
            RVB_motor(board)
            # Delay a very short amount of time
            time.sleep(very_short_time)
            # Stop the motor when done to prevent over correction
            STOP_motor(board)

        else :

            # This means that the current angle is the desired angle (desa) so indicate success with sound and end loop
            sound_success(board)

    print('You have reached angle: ', desa)
    
# END OF FUNCTION DEFINITION

#main

# This is where the function would begin to be called

# Stop the motor
STOP_motor(board)

# Wait short time and do nothing
time.sleep(short_time)

# Loop until the last five roll angles read are close in their values to prevent using crazy data as feedback

# Flag to determine if the sensor values are stable
tolerable = False

# Do a while loop until the angle values are stable
while tolerable == False :

    # Create an array that will hold the values that we're testing stability with and load it with 0s
    angle_test = [0]*test_size

    print(angle_test)

    # Define the iterating value for the next while loop
    iter_1 = 0

    # Begin the next while loop to load angle_test
    while iter_1 != test_size:

        # Activate the blue LED to indicate that we are obtaining angular data
        PIN_BLUE.write(1)

        # Open the serial port to communicate with the sensor
        ser = serial.Serial('COM4', 9600, timeout = 5)
        
        # Obtain the angle of roll
        angle_temp = serial_O_R(ser)

        # Close the serial port
        ser.close()

        # Deactivate the blue LED to indicate no longer obtaining angular data
        PIN_BLUE.write(0)

        # Get the iter_1 position for our angle_test array (so we're loading the array with test_size roll angles)
        angle_test[iter_1] = angle_temp[0]

        # Diagnostic print
        print(angle_test)

        # Increate the iterator
        iter_1 = iter_1 + 1

    # Inner most while loop has ended, now we test for variation (stability of angle)

    # Define the iter_2 variable
    iter_2 = 1

    # Begin second while loop to determine stability of angle data
    while iter_2 != test_size:

        # Do an if statement to determine if the data is stable
        if abs(angle_test[0] - angle_test[iter_2]) <= TOLERABLE_DELTA :

            # Set the tolerable flag to true until told otherwise
            tolerable = True
            print('It looks good man')
            print(abs(angle_test[0] - angle_test[iter_2]))
            # The variation between them is good so iterate normally
            iter_2 = iter_2 + 1

        else :

            # If this occured, the variation was too large and hence unstable therefor it failed the check
            # Set tolerable flag to false
            tolerable = False
            # Kill the loop by setting the iter_2 to max value
            iter_2 = test_size
            print('We killed the program')

print('The signal is stable!')

# At this point, it has been determined that the signal from the sensor is stable, proceed with moving to desa

# Obtain the current angle
current_angle = round(serial_getRoll())

#Defining desired angle, minimum angle (0) and maximum angle (unknown for now) that our dish can go to
desa = -10
mina = 0
maxa = 60   #To be determined
while (desa<mina or desa>maxa ):
    print(f"Enter angle between {mina} and {maxa}: ")
    desa = int(input(""))
    

# Begin the loop that will move the actuator
while current_angle != desa :

    # Obtain the current angle
    current_angle = round(serial_getRoll())
    dangle = abs(current_angle - desa)

    # If statement to determine what should be done
    if current_angle < desa :

        # Angle is too small, push the dish forward
        FWB_motor(board, dangle)
        # Delay a short amount of time
        time.sleep(short_time)
        # Stop the motor when done to prevent over correction
        STOP_motor(board)

    elif current_angle > desa :

        # The angle is too large, we need to correct our position.  Do a very small reverse
        RVB_motor(board, dangle)
        # Delay a very short amount of time
        time.sleep(very_short_time)
        # Stop the motor when done to prevent over correction
        STOP_motor(board)

    else :

        # This means that the current angle is the desired angle (desa) so indicate success with sound and end loop
        sound_success(board)

print('You have reached the end')