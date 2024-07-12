#!/usr/bin/env python3
import sys
import serial
import array
import math
import cv2
from struct import unpack
from PIL import Image
import numpy as np

IMAGE_WIDTH=320
IMAGE_HEIGHT=240

def remapDegrees(minAngle, maxAngle, degrees):
    delta = maxAngle - minAngle;
    if (maxAngle < minAngle):
        delta += 360
    if (delta < 0):
        return
    for cnt in range(16):
        degrees[cnt] = int(minAngle + (delta * cnt / 15))
        if (degrees[cnt] >= 360):
            degrees[cnt] -= 360

def plotDistanceMap(degrees, distances, pointcloud, image):
    black = 0
    white = 255
    for i in range(16):
        deg = degrees[i]
        dis = distances[i]
        pos = (int(pointcloud[deg][1]), int(pointcloud[deg][0]))
        if pos[0] < IMAGE_HEIGHT and pos[1] < IMAGE_WIDTH:
            image.putpixel(pos, black)
        if (dis < 10000):
            x = int( math.cos((1.0 * math.pi * deg) / 180.0) * (dis / 50.0) + 160)
            y = int( math.sin((1.0 * math.pi * deg) / 180.0) * (dis / 50.0) + 120)
            if y < IMAGE_HEIGHT and x < IMAGE_WIDTH:
                image.putpixel((x,y), white)
            pointcloud[deg][1] = x
            pointcloud[deg][0] = y

def main():
    print("ESC key to exit")
    header = bytearray([0x55, 0xaa, 0x23, 0x10])
    state = "STATE_WAIT_HEADER"
    counter = 0
    payload = bytearray([0]*64)
    pointcloud = np.zeros((360, 2))
    image = Image.new("L", (IMAGE_WIDTH,IMAGE_HEIGHT), color=0)
    with serial.Serial('/dev/ttyUSB0', 230400) as ser:
        while True:
            data = ser.read(1)
            if state == "STATE_WAIT_HEADER":
                if (data[0] == header[0]):
                    counter += 1
                    payload[0] = data[0]
                    state = "STATE_READ_HEADER"
                else:
                    #waitfor next byte
                    pass

            elif state == "STATE_READ_HEADER":
                if (data[0] == header[counter]):
                    payload[counter] = data[0]
                    counter += 1
                    if (counter == len(header)):
                        state = "STATE_READ_PAYLOAD"
                else:
                    counter = 0
                    state = "STATE_WAIT_HEADER"

            elif state == "STATE_READ_PAYLOAD":
                payload[counter] = data[0]
                counter += 1
                if (counter == 60):
                    state = "STATE_READ_DONE"
            elif state == "STATE_READ_DONE":
                (pac_header0,
                pac_header1,
                pac_header2,
                pac_header3,
                pac_rotation_speed,
                pac_angle_begin,
                pac_distance_0, pac_reserved_0,
                pac_distance_1, pac_reserved_1,
                pac_distance_2, pac_reserved_2,
                pac_distance_3, pac_reserved_3,
                pac_distance_4, pac_reserved_4,
                pac_distance_5, pac_reserved_5,
                pac_distance_6, pac_reserved_6,
                pac_distance_7, pac_reserved_7,
                pac_distance_8, pac_reserved_8,
                pac_distance_9, pac_reserved_9,
                pac_distance_10, pac_reserved_10,
                pac_distance_11, pac_reserved_11,
                pac_distance_12, pac_reserved_12,
                pac_distance_13, pac_reserved_13,
                pac_distance_14, pac_reserved_14,
                pac_distance_15, pac_reserved_15,
                pac_angle_end,
                pac_crc,) = unpack("=BBBBHHHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHH", payload[0:60])

                degree_begin = int((pac_angle_begin - 40960) / 64)
                degree_end = int((pac_angle_end - 40960) / 64)
                if ((degree_begin < 360) and (degree_end < 360)):
                    degrees = [0]*16
                    distances = [
                        pac_distance_0,
                        pac_distance_1,
                        pac_distance_2,
                        pac_distance_3,
                        pac_distance_4,
                        pac_distance_5,
                        pac_distance_6,
                        pac_distance_7,
                        pac_distance_8,
                        pac_distance_9,
                        pac_distance_10,
                        pac_distance_11,
                        pac_distance_12,
                        pac_distance_13,
                        pac_distance_14,
                        pac_distance_15]
                    remapDegrees(degree_begin, degree_end, degrees)
                    plotDistanceMap(degrees, distances, pointcloud, image)
                    cv_image = np.array(image, dtype=np.uint8)
                    cv2.imshow('lidar', cv_image)
                    key = cv2.waitKey(1)
                    if key == 27: # wait for ESC key to exit
                        cv2.destroyAllWindows()
                        return
                state = "STATE_WAIT_HEADER"
                counter = 0

if __name__ == '__main__':
    sys.exit(main())
