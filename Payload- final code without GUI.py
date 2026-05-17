import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from tflite_runtime.interpreter import Interpreter
import cv2
from PIL import Image
#---------------------------------------------------------------------------------------------------------
import gpiozero 
from gpiozero import Servo  # Import the Servo class from the gpiozero library
from time import sleep  # Import the sleep function from the time library
#---------------------------------------------------------------------------------------------------------
import RPi.GPIO as GPIO  # Import the GPIO library to control the Raspberry Pi's GPIO pins
#---------------------------------------------------------------------------------------------------------
import smbus
import time
distance = 0
angle=0
#servo.value = -0.5

def ultrasonic():
    #ultrasonic******************************************************
    # Set up the GPIO mode
    GPIO.setmode(GPIO.BCM)  # Use BCM (Broadcom SOC channel) numbering for GPIO pins

    # Define the GPIO pins for the TRIG and ECHO
    TRIG = 23  # Assign GPIO pin 23 to the TRIG pin
    ECHO = 24  # Assign GPIO pin 24 to the ECHO pin

    # Set up the TRIG and ECHO pins
    GPIO.setup(TRIG, GPIO.OUT)  # Set the TRIG pin as an output pin (will send the signal)
    GPIO.setup(ECHO, GPIO.IN)   # Set the ECHO pin as an input pin (will receive the signal)

    def measure_distance():
        # Send a 10us pulse to trigger the sensor
        GPIO.output(TRIG, True)  # Set the TRIG pin high to start the pulse
        time.sleep(0.00001)  # Wait for 10 microseconds
        GPIO.output(TRIG, False)  # Set the TRIG pin low to end the pulse
        # This sends a 10 microsecond pulse to the ultrasonic sensor to trigger it

        # Wait for the ECHO pin to go high
        while GPIO.input(ECHO) == 0:  # Wait until the ECHO pin goes high
            pulse_start = time.time()  # Record the current time when the ECHO pin goes high
            # This starts the timer when the signal is received

            # Wait for the ECHO pin to go low
        while GPIO.input(ECHO) == 1:  # Wait until the ECHO pin goes low
            pulse_end = time.time()  # Record the current time when the ECHO pin goes low
            # This stops the timer when the signal is received

            # Calculate the duration of the pulse
            pulse_duration = pulse_end - pulse_start  # Calculate the duration of the pulse

            # Calculate the distance in centimeters
            distance = pulse_duration * 17150  # Calculate the distance in cm (34300 cm/s / 2)
            # The speed of sound is 34300 cm/s, divided by 2 because the pulse travels to the object and back

            return distance  # Return the calculated distance
    #try:#################################################################################################################
    while True:  # Start an infinite loop to continuously measure distance
        distance = measure_distance()  # Measure the distance
        print(f"Distance: {distance:.2f} cm")  # Print the distance formatted to 2 decimal places
        time.sleep(1)  # Wait for 1 second before taking the next measurement
    #except KeyboardInterrupt:  # Catch a keyboard interrupt (e.g., Ctrl+C)###############################################
    #print("Measurement stopped by user")  # Print a message indicating the measurement was stopped#######################
    #GPIO.cleanup()  # Clean up the GPIO settings, resetting the pins used################################################
    #ultrasonic******************************************************
def MPU6050():
    # MPU6050 Registers and their addresses**************************
    PWR_MGMT_1 = 0x6B           # Power management register
    SMPLRT_DIV = 0x19           # Sample rate divisor register
    CONFIG = 0x1A               # General configuration register
    GYRO_CONFIG = 0x1B          # Gyroscope configuration register
    ACCEL_CONFIG = 0x1C         # Accelerometer configuration register
    INT_ENABLE = 0x38           # Interrupt enable register
    ACCEL_XOUT_H = 0x3B         # Accelerometer X-axis high byte
    ACCEL_YOUT_H = 0x3D         # Accelerometer Y-axis high byte
    ACCEL_ZOUT_H = 0x3F         # Accelerometer Z-axis high byte
    GYRO_XOUT_H = 0x43          # Gyroscope X-axis high byte
    GYRO_YOUT_H = 0x45          # Gyroscope Y-axis high byte
    GYRO_ZOUT_H = 0x47          # Gyroscope Z-axis high byte

    # Initialize the I2C bus
    bus = smbus.SMBus(1)        # Create a new I2C bus object for bus 1
    device_address = 0x68       # MPU6050 device address on the I2C bus

    # Initialize MPU6050
    def MPU_Init():
        # Write to sample rate register
        bus.write_byte_data(device_address, SMPLRT_DIV, 7)
        # Write to power management register to wake up the MPU6050
        bus.write_byte_data(device_address, PWR_MGMT_1, 1)
        # Write to configuration register for default settings
        bus.write_byte_data(device_address, CONFIG, 0)
        # Write to gyroscope configuration register with a full scale range of +/- 2000 degrees/second
        bus.write_byte_data(device_address, GYRO_CONFIG, 24)
        # Write to accelerometer configuration register with a full scale range of +/- 2g
        bus.write_byte_data(device_address, ACCEL_CONFIG, 0)
        # Write to interrupt enable register to enable data ready interrupt
        bus.write_byte_data(device_address, INT_ENABLE, 1)

    def read_raw_data(addr):
        # Read high and low bytes from the specified address
        high = bus.read_byte_data(device_address, addr)
        low = bus.read_byte_data(device_address, addr + 1)
        # Combine high and low bytes into a single value
        value = ((high << 8) | low)
        # Convert to signed value if necessary
        if value > 32768:
            value = value - 65536
        return value

    # Initialize the MPU6050 sensor
    MPU_Init()

    print("Reading data from MPU6050...")

    while True:
        # Read raw accelerometer data
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
    
        # Read raw gyroscope data
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
    
        # Convert raw accelerometer data to g (gravity) units
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
    
        # Convert raw gyroscope data to degrees/second units
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
    
        # Print accelerometer data
        print(f"Accelerometer: Ax={Ax:.2f} g, Ay={Ay:.2f} g, Az={Az:.2f} g")
        # Print gyroscope data
        print(f"Gyroscope: Gx={Gx:.2f} °/s, Gy={Gy:.2f} °/s, Gz={Gz:.2f} °/s")
        print("------------------------------------------------------")
    
    # Wait for 1 second before the next reading
    time.sleep(1)
    # MPU6050 ************************************************************
def servo ():
    #servo*************************************************************
    # Initialize the servo (assuming GPIO pin 18)
    servo = Servo(18)  
    servo.value = -0.5

    # Define the desired angle (0-180 degrees)

    # Convert angle in degrees (0-180) to a value between -1 and 1
    # The MG995 servo typically operates in a range from -1 to 1 in gpiozero
    value = (angle / 90.0) - 1  # Convert the angle to the corresponding servo value

    # Set the servo to the calculated value
    servo.value = value  # Set the servo to the desired position

    cv2.waitKey(5000) #wait untill the debris is passed
    MPU6050()
    servo.value = -0.5
    # Print the current servo angle setting
    print(f"Servo set to {angle} degrees")  # Print a message indicating the servo position
    #servo**********************************************************
def imgprocess():
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
    
        file_path = f'captured_image_{file_counter}.jpg'

        # Save the captured frame to the specified file path
        cv2.imwrite(file_path, frame)

        img = Image.open(file_path).convert('RGB')
        if img.size[0] > img.size[1]:
            img = img.resize((512, 512 * img.size[1] // img.size[0]), Image.LANCZOS)
        else:
            img = img.resize((512 * img.size[0] // img.size[1], 512), Image.LANCZOS)

        img_np = np.zeros((512, 512, 3)).astype('float32')
        img_np[:img.size[1], :img.size[0]] = np.array(img)
        img_np = img_np[None]

        # Set input tensor to the model
        model.set_tensor(input_details[0]['index'], img_np)
        model.invoke()

        # Get output tensors
        boxes = model.get_tensor(output_details[0]['index'])[0]
        pred_cl = model.get_tensor(output_details[1]['index'])[0]

        # Post-processing
        labels = pred_cl.argmax(-1)
        scores = pred_cl.max(-1)

        score_threshold = 0.8

        indices = tf.image.non_max_suppression(
            boxes,
            scores,
            max_output_size=50,
            iou_threshold=0.2,
            score_threshold=0.6
        ).numpy()

        boxes = boxes[indices]
        labels = labels[indices]
        scores = scores[indices]
        class_names = [classes[l] for l in labels]

        # Display results using matplotlib
        fig, ax = plt.subplots()
        ax.imshow(img_np[0].astype('uint8'))

        for box, class_name in zip(boxes, class_names):
            ax.text(box[0], box[1], class_name, color='r')
            rect = patches.Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1], linewidth=1, edgecolor='r', facecolor='none')
            ax.add_patch(rect)

        image = cv2.imread(file_path)
       
        height, width, _ = image.shape  # _ is for the number of color channels (3 for RGB)

        # Calculate the center coordinates 
        center_x_image = width // 2
        center_y_image = height // 2

        print(f"Center of the image: ({center_x_image}, {center_y_image})")

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply edge detection or thresholding to find contours
        _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour (assuming it's the rectangular object)
        contour = max(contours, key=cv2.contourArea)

        # Get the bounding box coordinates
        x, y, w, h = cv2.boundingRect(contour)

        # Calculate the center of the bounding box
        center_x = x + w // 2
        center_y = y + h // 2


        # Print the calculated center coordinates
        print(f"Center of the rectangle: ({center_x}, {center_y})")

        # Calculate the percentage of the center_x relative to the width of the image
        percentage = (center_x / width) * 100

        print(f"The percentage is: {percentage}")

        # Determine the angle based on the percentage
        if percentage <= 50:
            angle = (percentage / 50) * 45
        else:
            angle = ((percentage - 100) / 50)*45
        angle= angle + 45
        # Print the calculated angle
        print(f"Calculated angle for the motor: {angle:.2f} degrees")

        # Determine if the center of the rectangle is left or right of the center of the image
        if center_x < center_x_image:
            position = "left"
        elif center_x > center_x_image:
            position = "right"
        else:
            position = "center"

        print(f"Position relative to image center: {position}")

        #draw the bounding box
        cv2.rectangle(image, (x, y), (x+w, y+h), (0,255,0), 2 )

        # Display frame with detections
        cv2.imshow('Object Detection', image)
        # Save frame with detections
        cv2.imwrite(file_path, image)

        # Increment the file counter for the next image
        file_counter += 1

        cv2.waitKey(500) # Wait for a short delay between frames

        break
        #if 0xFF == 27 :
            #print ("ESC is clicked")
            #break
        #if file_counter == 100 :
            #print("100 pic is taken")
            #break 
    print("done")
    # Release the video capture object
    cap.release()

model_path = "/home/mark/Downloads/old model/model.tflite"
interpreter = Interpreter(model_path)
interpreter.allocate_tensors()
# Specify absolute path to the model file
model = tf.lite.Interpreter("/home/mark/Downloads/old model/model.tflite")
classes = ["Kosmo_v1 - v5 1", "-"]

# Learn about its input and output details
input_details = model.get_input_details()
output_details = model.get_output_details()

# Assuming you resize the input tensor
model.resize_tensor_input(input_details[0]['index'], (1, 512, 512, 3))
model.allocate_tensors()

# Open a connection to the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, or specify another camera index if you have multiple cameras

# Initialize a counter for file names
file_counter = 1

MPU6050()
ultrasonic()
if 15 <= distance <= 25 :
    imgprocess()
    servo()
    