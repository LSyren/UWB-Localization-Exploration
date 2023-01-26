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

    while True:
        with serial.Serial(port, 115200, timeout=2) as ser:
            line = ser.readline().decode("utf-8")
            if line != '':
                (id, distance) = line.split(":")
                tags[id] = float(distance)
                if id not in turtle_tags.keys():
                    nr_turtles = len(turtle_tags)
                    t = Turtle(shape="circle")
                    t.penup()
                    # t.setheading(len(turtle_tags)*180)
                    t.setposition(direction[nr_turtles]
                              * float(distance)*scale, 0)
                    turtle_tags[id] = (t, direction[nr_turtles])
                for key in tags.keys():
                    (turtle3, d) = turtle_tags[key]
                    distance = tags[key]
                    turtle3.clear()
                    turtle3.write('  '+key+'  ', move=False, align='right',
                              font=('Comic sans', 12, 'normal'))
                    turtle3.setx(d*scale*distance)
                    if (distance < (0.4)):
                        turtle3.goto(0, 0)



serial_receive("/dev/ttyUSB0")
