from turtle import *
import serial
import time

'''
skapa en turtle scen
While true
* Samla id och range
* Loopa över all id

* Uppdatera en turtle med fix vinkel och radiellt avstånd baserat på range
* 

'''
#
scale = 50
big_radius = 10
small_radius = 1
tags = {}
turtle_tags = {}
direction = [-1,1]
def serial_receive(port: str):
    # init_fifo(FIFO)
    setup(1100,1100)
    screensize 
    turtle1 = Turtle()
    turtle2 = Turtle()
    turtle1.speed(0)
    turtle2.speed(0)
    begin_fill()
    turtle1.pen(pencolor='red',pensize=2)
    turtle1.penup()
    turtle1.goto(0,-small_radius*scale)
    turtle1.pendown()
    turtle1.circle(small_radius*scale)
    turtle1.hideturtle()
    turtle2.penup()
    turtle2.goto(0,-big_radius*scale)
    turtle2.pendown()
    turtle2.circle(big_radius*scale)
    turtle2.hideturtle()
    turtle2.position
    hideturtle()

    turtle3 = Turtle(shape="circle")
    i = 0

    while True:
        with serial.Serial(port, 115200, timeout=2) as ser:
            turtle3.penup()
            line = ser.readline().decode("utf-8")
            id = line.split("\t")[0].strip("ID: ")
            id = "  " + str(id) + "  "
            distance = float(line.split("\t")[1].strip("Distance: "))
            i+=1
            if i==10:
                turtle3.clear()
                turtle3.write(str(id), move=False, align='right', font=('Comic sans', 12, 'normal'))
            elif i==20:
                turtle3.clear()
                turtle3.write(str(id), move=False, align='right', font=('Comic sans', 12, 'normal'))
                i = 0
            turtle3.sety(100*distance)




serial_receive("/dev/ttyUSB0")