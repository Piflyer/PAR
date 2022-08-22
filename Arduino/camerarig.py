import subprocess
from asyncore import write
from token import NUMBER
import serial
import time
import gphoto2 as gp 
from mainpackage import *

#Max images for Camera rig is 20, max for turntable is 100
NUMBER_OF_IMAGES = 75
#Define output path for images
PATH = ""
#Define serial port for Arduino
arduino = serial.Serial(port="", baudrate=9600)
i = 0
#Get Camera information
def init_camera():
    camera = gp.Camera()
    camera.init()
    return camera
camera = init_camera()
time.sleep(10)
print("Starting up...")
while True:
    #Get image from camera
    arduino.write(str.encode(str(NUMBER_OF_IMAGES)))
    trigger = arduino.readline()
    if trigger:
        file_path = camera.capture(gp.GP_CAPTURE_IMAGE)
        if i < 10:
            target = PATH + "/photo00" + str(i) + ".jpg"
            camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
        if i < 100 and i >= 10:
            target = PATH + "/photo0" + str(i) + ".jpg"
            camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
        if i >= 100:
            target = PATH + "/photo" + str(i) + ".jpg"
            camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
        print(f"Photo {i + 1} taken")
        i += 1
        if i == NUMBER_OF_IMAGES:
            time.sleep(0.5)
            print("The End")
            break