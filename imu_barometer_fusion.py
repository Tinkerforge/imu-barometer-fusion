#!/usr/bin/env python
# -*- coding: utf-8 -*-  

# This is an example program doing sensor fusion between the IMU Brick and
# the Barometer Bricklet.
# We try to estimate a more reliable altitude by combining air pressure
# measurements from the Barometer Bricklet with the acceleration measurements 
# and the quaternion from the IMU Brick.
#
# The result is drawn on a live graph with Qwt.
#
# There is still some drift in the Barometer data that is not removed. Perhaps
# we can improve that by imcorporating GPS data in the future! Also there is
# likely room for improvement by simply twiddling with the velocity, position
# and integral gain (KP1, KP2 and KI).

HOST = "localhost"
PORT = 4223
UID_IMU = "6wVE7W"    # <---- Change to your UID
UID_BAROMETER = "etG" # <---- Change to your UID


from PyQt4.QtGui import QApplication, QMainWindow, QVBoxLayout, QPen, QWidget, QPushButton
from PyQt4.QtCore import Qt, QTimer
import PyQt4.Qwt5 as Qwt

import sys
import time
import math

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu import IMU
from tinkerforge.bricklet_barometer import Barometer

class Plot(Qwt.QwtPlot):
    def __init__(self, y_axis, plot_list, *args):
        Qwt.QwtPlot.__init__(self, *args)
     
        self.setAxisTitle(Qwt.QwtPlot.xBottom, 'Time [s]')
        self.setAxisTitle(Qwt.QwtPlot.yLeft, y_axis)

        c = self.canvas()
        c.setPaintAttribute(Qwt.QwtPlotCanvas.PaintCached, False)
        c.setPaintAttribute(Qwt.QwtPlotCanvas.PaintPacked, True)
        
        self.has_legend = plot_list[0][0] != ''
        
        if self.has_legend: 
            legend = Qwt.QwtLegend()
            legend.setItemMode(Qwt.QwtLegend.CheckableItem)
            self.insertLegend(legend, Qwt.QwtPlot.RightLegend)
        
        self.setAutoReplot(True)
        
        self.curve = []
        
        self.data_x = []
        self.data_y = []
        self.last_max_y = -1000000
        self.last_min_y = 1000000
        
        for x in plot_list:
            c = Qwt.QwtPlotCurve(x[0])
            c.setRenderHint(Qwt.QwtPlotItem.RenderAntialiased)
            self.curve.append(c)
            self.data_x.append([])
            self.data_y.append([])
        
            c.attach(self)
            c.setPen(x[1])
            self.show_curve(c, True)
        
        if self.has_legend: 
            self.legendChecked.connect(self.show_curve)
        
    def show_curve(self, item, on):
        item.setVisible(on)
        if self.has_legend:
            widget = self.legend().find(item)
            if isinstance(widget, Qwt.QwtLegendItem):
                widget.setChecked(on)
        self.replot()
           

    def add_data(self, i, data_x, data_y):
        self.data_x[i].append(data_x)
        self.data_y[i].append(data_y)
        if len(self.data_x[i]) > 100: # 2 minutes
            self.data_x[i] = self.data_x[i][1:]
            self.data_y[i] = self.data_y[i][1:]


        max_y = max(self.data_y[i])
        min_y = min(self.data_y[i])
        s = abs(max_y-min_y)
        if s < 1.5:
            max_y += (1.5 - s)/2
            min_y -= (1.5 - s)/2

#        if abs(self.last_max_y - max_y) < 0.5 and abs(self.last_min_y - min_y) < 0.5:
#            max_y = self.last_max_y
#            min_y = self.last_min_y
#        else:
#            self.last_max_y = max_y
#            self.last_min_y = min_y

           
        self.setAxisScale(Qwt.QwtPlot.yLeft, min_y, max_y)
        self.setAxisScale(Qwt.QwtPlot.xBottom, self.data_x[i][0], self.data_x[i][-1])
        self.curve[i].setData(self.data_x[i], self.data_y[i])
        
    def clear_graph(self):
        for i in range(len(self.data_x)):
            self.data_x[i] = []
            self.data_y[i] = []
            
class PlotWidget(QWidget):
    def __init__(self, y_axis, plot_list, clear_button = None, parent = None):
        QWidget.__init__(self, parent)
        
        self.stop = True
        
        self.plot = Plot(y_axis, plot_list)

        if clear_button is None:
            self.clear_button = QPushButton('Clear Graph')
        else:
            self.clear_button = clear_button

        self.clear_button.pressed.connect(self.clear_pressed)
        
        layout = QVBoxLayout(self)
        layout.addWidget(self.plot)

        if clear_button is None:
            layout.addWidget(self.clear_button)
        
        self.counter = 0
        self.update_func = []
        
        for pl in plot_list:
            self.update_func.append(pl[2])
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)
        
    def update(self):
        if self.stop:
            return
        
        for i in range(len(self.update_func)):
            value = self.update_func[i]()

            if value is not None:
                self.plot.add_data(i, self.counter/10.0, value)
            
        self.counter += 1
            
    def clear_pressed(self):
        self.plot.clear_graph()
        self.counter = 0

class IMUBarometerFusion(QMainWindow):
    FACTOR = 1
    KP1 = 0.55*FACTOR # PI observer velocity gain 
    KP2 = 1.0*FACTOR  # PI observer position gain
    KI = 0.001/FACTOR # PI observer integral gain (bias cancellation)

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)

        self.initialized = False
        self.altitude_error_i = 0
        self.acc_scale  = 0.0
        self.start_time = time.time()
        self.altitude_error = 0
        self.inst_acceleration = 0.0
        self.delta = 0
        self.estimated_velocity = 0.0
        self.estimated_altitude = 0.0
        self.last_time = time.time()

        self.last_orig_altitude = 0
        self.last_estimated_altitude = 0

        ipcon = IPConnection() 
        self.imu = IMU(UID_IMU, ipcon) 
        self.barometer = Barometer(UID_BAROMETER, ipcon)
        ipcon.connect(HOST, PORT) 

        # Turn leds and orientation calculation off, to save calculation time
        # for the IMU Brick. This makes sure that the measurements are taken
        # in equidistant 2ms intervals
        self.imu.leds_off()
        self.imu.orientation_calculation_off()

        # Turn averaging of in the Barometer Bricklet to make sure that
        # the data is without delay
        self.barometer.set_averaging(0, 0, 0)

        red_pen = QPen(Qt.red)
        red_pen.setWidth(5)

        plot_list = [#['', Qt.blue, self.get_orig_value],
                     ['', red_pen, self.get_estimated_value]
                    ]
        self.plot_widget = PlotWidget('Height [m]', plot_list)
        self.plot_widget.stop = False
        self.setCentralWidget(self.plot_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(6)

    def get_orig_value(self):
        return self.last_orig_altitude

    def get_estimated_value(self):
        return self.last_estimated_altitude

    # Update measurements and compute new altitude every 6ms.
    def update(self):
        q = self.imu.get_quaternion()
        acc = self.imu.get_acceleration()
        alt = self.barometer.get_altitude()/100.0

        compensated_acc_q = self.compute_compensated_acc(q, acc)
        compensated_acc_q_earth = self.compute_dynamic_acceleration_vector(q, compensated_acc_q)

        self.last_orig_altitude = alt
        self.last_estimated_altitude = self.compute_altitude(compensated_acc_q_earth[2], alt)

    # Remove gravity from accelerometer measurements
    def compute_compensated_acc(self, q, a):
        g = (2*(q.x*q.z - q.w*q.y),
             2*(q.w*q.x + q.y*q.z),
             q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

        return (a[0]/1000.0 - g[0], 
                a[1]/1000.0 - g[1], 
                a[2]/1000.0 - g[2],
                0.0)

    # Rotate dynamic acceleration vector from sensor frame to earth frame
    def compute_dynamic_acceleration_vector(self, q, compensated_acc_q):
        def q_conj(q):
            return -q[0], -q[1], -q[2], q[3]

        def q_mult(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

            return x, y, z, w

        tmp = q_mult(q, compensated_acc_q)
        return q_mult(tmp, q_conj(q))


    # Computes estimation of altitude based on barometer and accelerometer measurements
    # Code is based on blog post from Fabio Varesano: http://www.varesano.net/blog/fabio/complementary-filtering-high-res-barometer-and-accelerometer-reliable-altitude-estimation
    # He seems to have got the idea from the MultiWii project
    def compute_altitude(self, compensated_acceleration, altitude):
        self.current_time = time.time()
    
        # Initialization
        if not self.initialized:
            self.initialized = True
            self.estimated_altitude = altitude 
            self.estimated_velocity = 0
            self.altitude_error_i = 0

        # Estimation Error
        self.altitude_error = altitude - self.estimated_altitude
        self.altitude_error_i = self.altitude_error_i + self.altitude_error
        self.altitude_error_i = min(2500.0, max(-2500.0, self.altitude_error_i))

        self.inst_acceleration = compensated_acceleration * 9.80665 + self.altitude_error_i * self.KI
        dt = self.current_time - self.last_time

        # Integrators
        self.delta = self.inst_acceleration * dt + (self.KP1 * dt) * self.altitude_error
        self.estimated_altitude += (self.estimated_velocity/5.0 + self.delta) * (dt / 2) + (self.KP2 * dt) * self.altitude_error
        self.estimated_velocity += self.delta*10.0 
    
        self.last_time = self.current_time
    
        return self.estimated_altitude

if __name__ == "__main__":
    test = QApplication(sys.argv)
    main = IMUBarometerFusion()
    main.show()
    sys.exit(test.exec_())
