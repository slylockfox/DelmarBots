import face_recognition
import cv2
from datetime import datetime, timedelta
import numpy as np
import platform
import pickle
import threading
import time

from jetbot import Robot

position_tolerance = 10
view_position_center = 140 # was 125
# multiply error * P instead, was... swivel_speed = 0.4
swivel_duration = 0.2
swivel_P = .0035
max_led_power = 0.7

# Our list of known face encodings and a matching list of metadata about each face.
known_face_encodings = []
known_face_metadata = []


def running_on_jetson_nano():
    # To make the same code work on a laptop or on a Jetson Nano, we'll detect when we are running on the Nano
    # so that we can access the camera correctly in that case.
    # On a normal Intel laptop, platform.machine() will be "x86_64" instead of "aarch64"
    return platform.machine() == "aarch64"


def get_jetson_gstreamer_source(capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=5, flip_method=0): # was framerate=60
    """
    Return an OpenCV-compatible video source description that uses gstreamer to capture video from the camera on a Jetson Nano
    """
    return (
            f'nvarguscamerasrc ! video/x-raw(memory:NVMM), ' +
            f'width=(int){capture_width}, height=(int){capture_height}, ' +
            f'format=(string)NV12, framerate=(fraction){framerate}/1 ! ' +
            f'nvvidconv flip-method={flip_method} ! ' +
            f'video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! ' +
            'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
            )

def stop_motors_later(robot):
    time.sleep(swivel_duration)
    robot.stop()

def swivel_for_time(robot, speed, duration):
    #robot.set_motors(speed, speed)
    robot.left_motor.value = speed
    time.sleep(duration)
    robot.stop()
    
def toggle_led(robot, p):
    global led_power
    if (led_power == 0):
        led_power = p
    else:
        led_power = 0
    robot.right_motor.value = led_power

def toggle_red_led(robot):
    toggle_led(robot, max_led_power)

def toggle_green_led(robot):
    toggle_led(robot, -max_led_power)

def main_loop():
    global led_power
    led_power = max_led_power
    
    # Get access to the webcam. The method is different depending on if this is running on a laptop or a Jetson Nano.
    if running_on_jetson_nano():
        # Accessing the camera with OpenCV on a Jetson Nano requires gstreamer with a custom gstreamer source string
        video_capture = cv2.VideoCapture(get_jetson_gstreamer_source(), cv2.CAP_GSTREAMER)
    else:
        # Accessing the camera with OpenCV on a laptop just requires passing in the number of the webcam (usually 0)
        # Note: You can pass in a filename instead if you want to process a video file instead of a live camera stream
        video_capture = cv2.VideoCapture(0)

    robot = Robot()

    print ("beginning Jetbot face recognition loop")
    
    noface_count = 0

    try:
      while True:
        # Grab a single frame of video
        ret, frame = video_capture.read()

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Find all the face locations and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        #face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        # Grab the image of the the face from the current frame of video
        if (len(face_locations) > 0):
          noface_count = 0
          max_width = 0
          closest_face_left = 0
          closest_face_right = 0
          for location in face_locations:
            top, right, bottom, left = location
            width = right - left
            if (width > max_width):
              max_width = width
              closest_face_left = left
              closest_face_right = right
          
          error = closest_face_left + max_width / 2 - view_position_center
          if (abs(error) > position_tolerance):
            swivel_speed = abs(error) * swivel_P # was 0.2
            if (error > 0):
              x = threading.Thread(target=swivel_for_time, args=(robot,-swivel_speed,swivel_duration,)) # was robot.left(speed=swivel_speed)
            else:
              x = threading.Thread(target=swivel_for_time, args=(robot,swivel_speed,.1,)) # was robot.right(speed=swivel_speed)
            x.start()
            #time.sleep(.1)
          else:
            robot.stop()
          with open("/home/jetbot/jethead-stats.txt",'w',encoding = 'utf-8') as f:
            f.write("{} {} {}".format(closest_face_left, closest_face_right, error))
          toggle_green_led(robot) # show heartbeat with red led

        else:
          robot.stop()
          noface_count += 1
          with open("/home/jetbot/jethead-stats.txt",'w',encoding = 'utf-8') as f:
            f.write("{}".format(noface_count))
          toggle_red_led(robot) # show heartbeat with red led
            
    except:
      with open("/home/jetbot/jethead-stats.txt",'w',encoding = 'utf-8') as f:
        f.write("crashed")
      # Release handle to the webcam
      print ("cleaning up after jetbot")
      video_capture.release()
      cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
