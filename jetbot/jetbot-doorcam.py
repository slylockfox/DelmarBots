import face_recognition
import cv2
from datetime import datetime, timedelta
import numpy as np
import platform
import pickle

from jetbot import Robot

position_tolerance = 10
view_position_center = 125
swivel_speed = 0.2

# Our list of known face encodings and a matching list of metadata about each face.
known_face_encodings = []
known_face_metadata = []


def running_on_jetson_nano():
    # To make the same code work on a laptop or on a Jetson Nano, we'll detect when we are running on the Nano
    # so that we can access the camera correctly in that case.
    # On a normal Intel laptop, platform.machine() will be "x86_64" instead of "aarch64"
    return platform.machine() == "aarch64"


def get_jetson_gstreamer_source(capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=60, flip_method=0):
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

def main_loop():
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
          top, right, bottom, left = face_locations[0]
          face_image = small_frame[top:bottom, left:right]
          face_image = cv2.resize(face_image, (150, 150))
          #face_label = f"Left: {left}"
          
          error = left - view_position_center
          if (abs(error) > position_tolerance):
            if (error > 0):
              robot.left(speed=swivel_speed)
            else:
              robot.right(speed=swivel_speed)
          else:
            robot.stop()
          print (left, error)
        else:
          print ("no faces")

    except:
      # Release handle to the webcam
      print ("cleaning up after jetbot")
      video_capture.release()
      cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
