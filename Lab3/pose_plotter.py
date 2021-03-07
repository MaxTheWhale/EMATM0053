from turtle import *;
import serial

port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=3.0)

scale = 0.02

mode("standard")
home()
radians()
speed(0)
delay(0)
while True:
    data = port.readline()
    values = str(data).split(",")
    setpos(float(values[0][2:]) * scale, float(values[1]) * scale)
    setheading(float(values[2][:-5]))
end_fill()
done()
