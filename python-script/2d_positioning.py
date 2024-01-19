import kivy
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Rectangle, Color, Ellipse
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
import numpy as np

import threading
import queue
import random
import time
import sys
import os
import collections

from uart_source import serial_receive


q = queue.Queue()

SPACE_SIZE_X = 4.5
SPACE_SIZE_Y = 5.0

class MyLayout(BoxLayout):
    def __init__(self, data_source=None, **kwargs):
        super().__init__(**kwargs)

        self.ids['lbl_data_per_sec'].text = str(0)

    def update(self, *args):
        self.update_view()

    def update_view(self):
        pass


class Agent(Widget):
    def __init__(self, pos, size, **kwargs):
        super().__init__(**kwargs)
        self.w = size[0]
        self.h = size[1]
        x = pos[0] - self.width/2
        y = pos[1] - self.height/2
        self.circle = Ellipse(pos=(x, y), size=(self.w, self.h))

        with self.canvas:
            Color(0.8, 0.2, 0.2, 0.7, mode="rgba")
            self.canvas.add(self.circle)

    def move_to(self, pos):
        # TODO(michalc): I'm moving the sub widget of the Agent... not sure
        # if that's good.
        self.circle.pos = (pos[0] - (self.w / 2), pos[1] - (self.h / 2))


class CanvasWidget(RelativeLayout):
    global q

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.agent = Agent(pos=self.pos, size=(30, 30))
        self.add_widget(self.agent)

        self.samples_count = 5

        self.buf_x = collections.deque(maxlen=self.samples_count)
        self.buf_y = collections.deque(maxlen=self.samples_count)

        Clock.schedule_interval(self.update, 1.0/60.0)

    def update(self, *args):
        data_avg = [0] * 3
        try:
            while not q.empty():
                data = q.get(block=False)
                if len(data) == 3:
                    self.buf_x.append(data[0])
                    self.buf_y.append(data[1])

                    data_avg[0] = sum(self.buf_x) / self.samples_count
                    data_avg[1] = sum(self.buf_y) / self.samples_count
                    x = (data_avg[0] / SPACE_SIZE_X) * self.size[0]
                    y = (data_avg[1] / SPACE_SIZE_Y) * self.size[1]

                    self.agent.move_to((x, y))
        except queue.Empty:
            pass


class MainApp(App):
    global q

    def build(self):
        return MyLayout(q)

    def update(self, *args):
        pass


def dummy_source(q):
    max_x = SPACE_SIZE_X
    max_y = SPACE_SIZE_Y
    max_z = 0

    cur_pos = [SPACE_SIZE_X / 2, SPACE_SIZE_Y / 2, 0]
    new_pos = [0] * 3
    counter = 0
    sign = -1

    while True:
        counter += 1
        if counter % 24 == 0:
            sign *= -1

        if sign == 1:
            delta_pos = [random.uniform(-0.04, 0.0),
                         random.uniform(-0.04, 0.0),
                         random.uniform(-0.04, 0.0)]
        elif sign == -1:
            delta_pos = [random.uniform(0.0, 0.04),
                         random.uniform(0.0, 0.04),
                         random.uniform(0.0, 0.04)]

        new_pos = [cur_pos[0] + delta_pos[0],
                   cur_pos[1] + delta_pos[1],
                   cur_pos[2] + delta_pos[2]]
        # pos = (random.uniform(0, max_x),
        #        random.uniform(0, max_y),
        #        random.uniform(0, max_z))
        # pos = (SPACE_SIZE_X, 0.8, 0)
        q.put(new_pos)
        cur_pos = new_pos
        time.sleep(0.01)


if len(sys.argv) <= 1:
    print("")
    print("Please put the path to the serial port as an argument:")
    print("")
    print(f"\tpython3 {os.path.basename(__file__)} /dev/ttyUSB0")
    sys.exit(1)


PORT = sys.argv[1]
thr_recv = threading.Thread(target=serial_receive, args=(PORT, q,), daemon=True)
thr_recv.start()

MainApp().run()
