from asyncore import write
from token import NUMBER
import serial
from mainpackage import *
import time
import gphoto2 as gp 

#Stops for middle third of rail
NUMBER_OF_STOPS = 2

#Stops for turntable
NUMBER_OF_TURNT_STOPS = 2

RAIL_OUTPUT = NUMBER_OF_STOPS - 1
#Define output path for images
PATH = ""
# Set serial port for both motorized rail and turntable
camerarig = serial.Serial(port="", baudrate=9600)
turntable = serial.Serial(port="", baudrate=9600)
i = 0
j = 0
k = 0
def init_camera():
    camera = gp.Camera()
    camera.init()
    return camera
camera = init_camera()
print("Camera initialized")
turnstops = 14336/NUMBER_OF_TURNT_STOPS
time.sleep(10)
for i in range(NUMBER_OF_TURNT_STOPS):
    camerarig.write(str.encode(str(1300 + RAIL_OUTPUT)))
    print("Rail Command")
    for j in range(NUMBER_OF_STOPS):
        trigger = camerarig.readline()
        if trigger:
            file_path = camera.capture(gp.GP_CAPTURE_IMAGE)
            if i < 10:
                target = PATH + "/photo00" + str(k) + ".jpg"
                camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
            if i < 100 and i >= 10:
                target = PATH + "/photo0" + str(k) + ".jpg"
                camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
            if i >= 100:
                target = PATH + "/photo" + str(k) + ".jpg"
                camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL).save(target)
            print("Photo taken")
            print(j)
            j += 1
            k += 1
            # if i == NUMBER_OF_STOPS:
            #     time.sleep(0.5)
            #     i = 0
            #     print("The End")
            #     pass
    turntable.write(str.encode(str(turnstops)))
    print("Turntable Command")
    print(i)
