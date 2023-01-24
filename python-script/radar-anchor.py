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
scale = 100
tags = {}
turtle_tags = {}
def serial_receive(port: str):
    # init_fifo(FIFO)
    setup(1100,1100)
    turtle1 = Turtle()
    turtle2 = Turtle()
    turtle1.speed(0)
    turtle2.speed(0)
    begin_fill()
    turtle1.pen(pencolor='red',pensize=2)
    turtle1.penup()
    turtle1.goto(0,-5*10)
    turtle1.pendown()
    turtle1.circle(5*10)
    turtle1.hideturtle()
    turtle2.penup()
    turtle2.goto(0,-5*100)
    turtle2.pendown()
    turtle2.circle(5*100)
    turtle2.hideturtle()
    hideturtle()
    i = 0
    '''
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
    '''
    while True:
        with serial.Serial(port, 115200, timeout=2) as ser:
            line = ser.readline().decode("utf-8")
            print(line)
            id = line.split("\t")[0].strip("ID: ")
            distance = float(line.split("\t")[1].strip("Distance: "))
            tags[id] = float(distance)
            if id not in turtle_tags.keys():
                t = Turtle(shape="circle")
                t.penup()
                t.setheading(len(turtle_tags)*45)
                turtle_tags[id] = t
            if len(turtle_tags) >= 0:
                for key in tags.keys():
                    turtle3 = turtle_tags[key]
                    distance = tags[key]
                    if i == 10:
                        turtle3.clear()
                        turtle3.write('  '+key+'  ', move=False, align='right',
                                    font=('Comic sans', 12, 'normal'))
                    elif i == 20:
                        turtle3.clear()
                        turtle3.write('  '+key+'  ', move=False, align='right',
                                    font=('Comic sans', 12, 'normal'))
                        i = 0
                        turtle3.sety(100*distance)
            i += 1



serial_receive("/dev/ttyUSB0")
