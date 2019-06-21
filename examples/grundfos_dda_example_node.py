#!/usr/bin/env python
#
# MIT License

# Copyright (c) 2019 Goncalo Cabrita

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Goncalo Cabrita, goncabrita@gmail.com
# Date: 21 Jun 2019
#


import rospy
from std_msgs.msg import Float32, Bool
from modbus_rs485.modbus_rs485_client import ModbusRS485Client
import sys

class DDANode(object):
    def __init__(self):
        rospy.init_node('grundfos_dda_example_node', anonymous=True)
        rospy.loginfo("Grundfos DDA driver for ROS")

        # Load parameters
        self.address = rospy.get_param('~address', 100)
        self.bus = rospy.get_param('~namespace', 'modbus_rs485')
        self.min_reference = rospy.get_param('~min_reference', 0.02)

        self.client = ModbusRS485Client(slave, network)

        rospy.loginfo("[Grundfos DDA] Connecting to pump %d on bus %s...", self.address, self.bus)
        try:
            self.client.connect()
            self.stop_pump()
            self.set_control_mode(0)
        except:
            rospy.logfatal("[Grundfos DDA] Failed to connect to dosing pump %d on bus %s.", self.address, self.bus)
            sys.exit("failed to connect to pump")

        rospy.loginfo("[Grundfos DDA] Connected to pump %d on bus %s.", self.address, self.bus)

        self.external_stop = False

        self.pub = rospy.Publisher('flow', Float32, queue_size=10)
        rospy.Subscriber("flow_reference", Float32, self.reference_callback)
        rospy.Subscriber("external_stop", Bool, self.external_stop_callback)
    

    # Pump specific functions
    def set_control_mode(self, mode):
        result = self.client.write_register(101, mode)

    def stop_pump(self):
        result = self.client.write_register(100, 0x0001)
    
    def start_pump(self):
        result = self.client.write_register(100, 0x0003)

    def write_flow_reference(self, reference):
        result = self.client.write_register(103, reference)

    def read_flow(self):
        result = self.client.read_holding_registers(301, 1)
        return result.registers[0]


    # Callbacks
    def reference_callback(self, msg):
        rospy.logdebug("[Grundfos DDA] Setting reference to %.1f", msg.data*1000.0)
        self.write_flow_reference(msg.data*1000.0) # Convert from l to ml
        if msg.data <= self.min_reference:
            self.stop_pump()
        elif not self.external_stop:
            self.start_pump()
    
    def external_stop_callback(self, msg):
        self.external_stop = msg.data
        if msg.data:
            rospy.logdebug("[Grundfos DDA] Turning pump ON")
            self.start_pump()
        else:
            rospy.logdebug("[Grundfos DDA] Turning pump OFF")
            self.stop_pump()


    # Main loop
    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():

            flow = self.read_flow()

            msg = Float32()
            msg.data = flow/1000.0 # Convert from ml to l
            self.pub.publish(msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        dda_node = DDANode()
        dda_node.spin()
    except rospy.ROSInterruptException:
        pass
