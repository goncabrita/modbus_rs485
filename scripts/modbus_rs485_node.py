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

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import sys
from modbus_rs485.srv import *
import rospy
import struct
import uuid
import Queue


class ModbusRS485Request(object):
    def __init__(self, service_request, function_code, result_tag=None, output_value=None):
        self.slave = service_request.slave
        self.address = service_request.address
        if hasattr(service_request, 'count'):
            self.count = service_request.count
        else:
            self.count = 0
        self.function_code = function_code
        self.result_tag = result_tag
        self.output_value = output_value


class ModbusRS485Server(object):
    READ_FUNCTIONS = [cst.READ_COILS,
                      cst.READ_DISCRETE_INPUTS,
                      cst.READ_HOLDING_REGISTERS,
                      cst.READ_INPUT_REGISTERS]
    WRITE_FUNCTIONS = [cst.WRITE_SINGLE_COIL,
                       cst.WRITE_SINGLE_REGISTER,
                       cst.WRITE_MULTIPLE_COILS,
                       cst.WRITE_MULTIPLE_REGISTERS]

    def __init__(self):
        rospy.init_node('modbus_rs485_server')
        rospy.loginfo("Modbus RS485 abstraction layer for ROS")

        self.requests_queue = Queue.Queue()
        self.results = {}

        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 19200)
        self.bytesize = rospy.get_param("~byte_size", 8)
        self.stopbits = rospy.get_param("~stop_bits", 1)
        self.parity = rospy.get_param("~parity", 'N')
        self.retries = rospy.get_param("~retries", 3)
        self.timeout = rospy.get_param("~timeout", 0.05)
        self.interval = rospy.get_param("~interval", 0.01)

        self.client = modbus_rtu.RtuMaster(serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=self.bytesize, parity=self.parity, stopbits=self.stopbits))
        self.client.set_timeout(self.timeout)

        self.namespace = rospy.get_param("~namespace", "modbus_rs485")

        read_coils_srv = rospy.Service('/'+self.namespace+'/read_coils', ReadBits, self.handle_read_coils)
        read_discrete_inputs_srv = rospy.Service('/'+self.namespace+'/read_discrete_inputs', ReadRegisters, self.handle_read_discrete_inputs)
        read_holding_registers_srv = rospy.Service('/'+self.namespace+'/read_holding_registers', ReadRegisters, self.handle_read_holding_registers)
        read_input_registers_srv = rospy.Service('/'+self.namespace+'/read_input_registers', ReadRegisters, self.handle_read_input_registers)
        write_coil_srv = rospy.Service('/'+self.namespace+'/write_coil', WriteBits, self.handle_write_coil)
        write_register_srv = rospy.Service('/'+self.namespace+'/write_register', WriteRegisters, self.handle_write_register)
        write_coils_srv = rospy.Service('/'+self.namespace+'/write_coils', WriteBits, self.handle_write_coils)
        write_registers_srv = rospy.Service('/'+self.namespace+'/write_registers', WriteRegisters, self.handle_write_registers)

    def spin(self):
        rospy.loginfo("[CoolFarm Modbus RTU] Running Modbus RTU server...")
        while not rospy.is_shutdown():
            if not self.requests_queue.empty():
                exception = None
                request = self.requests_queue.get()
                for _ in range(self.retries):
                    try:
                        if exception is not None:
                            rospy.logdebug("[CoolFarm Modbus RTU]   > Retrying to reach %d for address %d...", request.slave, request.address)
                        if request.function_code in ModbusRS485Server.READ_FUNCTIONS:
                            result = self.client.execute(request.slave, request.function_code, request.address, request.count)
                        elif request.function_code in ModbusRS485Server.WRITE_FUNCTIONS:
                            result = self.client.execute(request.slave, request.function_code, request.address, output_value=request.output_value)
                    except modbus_rtu.ModbusInvalidResponseError as e:
                        exception = e
                        rospy.sleep(self.interval)
                        continue
                    except modbus_tk.exceptions.ModbusInvalidResponseError as e:
                        exception = e
                        rospy.sleep(self.interval)
                        continue
                    break
                else:
                    self.results[request.result_tag] = exception
                if request.result_tag is not None:
                    self.results[request.result_tag] = result
            rospy.sleep(self.interval)
        
    def wait_for_reply(self, tag):
        c = 0
        while tag not in self.results:
            rospy.sleep(self.interval)
            c += 1
            if c == self.retries*7:
                return False
        return True

    def handle_read_coils(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d READ_COILS request for address %d", req.slave, req.address)
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.READ_COILS, result_tag=tag)
        self.requests_queue.put(request)
        result = ReadBitsResponse()
        if self.wait_for_reply(tag):
            result.bits = self.results[tag]
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > READ_COILS result %s, took %f seconds", str(result.bits), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on READ_COILS, took %f seconds", elapsed_time.to_sec())
        return result

    def handle_read_discrete_inputs(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d READ_DISCRETE_INPUTS request for address %d", req.slave, req.address)
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.READ_DISCRETE_INPUTS, result_tag=tag)
        self.requests_queue.put(request)
        result = ReadRegistersResponse()
        if self.wait_for_reply(tag):
            result.registers = self.results[tag]
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > READ_DISCRETE_INPUTS result %s, took %f seconds", str(result.registers), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on READ_DISCRETE_INPUTS, took %f seconds", elapsed_time.to_sec())
        return result

    def handle_read_holding_registers(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d READ_HOLDING_REGISTERS request for address %d", req.slave, req.address)
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.READ_HOLDING_REGISTERS, result_tag=tag)
        self.requests_queue.put(request)
        result = ReadRegistersResponse()
        if self.wait_for_reply(tag):
            result.registers = self.results[tag]
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > READ_HOLDING_REGISTERS result %s, took %f seconds", str(result.registers), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on READ_HOLDING_REGISTERS, took %f seconds", elapsed_time.to_sec())
        return result

    def handle_read_input_registers(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d READ_INPUT_REGISTERS request for address %d", req.slave, req.address)
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.READ_INPUT_REGISTERS, result_tag=tag)
        self.requests_queue.put(request)
        result = ReadRegistersResponse()
        if self.wait_for_reply(tag):
            result.registers = self.results[tag]
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > READ_INPUT_REGISTERS result %s, took %f seconds", str(result.registers), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on READ_INPUT_REGISTERS, took %f seconds", elapsed_time.to_sec())
        return result

    def handle_write_coil(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d WRITE_SINGLE_COIL request for address %d : %s", req.slave, req.address, str(req.bits[0]))
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.WRITE_SINGLE_COIL, result_tag=tag, output_value=req.bits[0])
        self.requests_queue.put(request)
        if self.wait_for_reply(tag):
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > WRITE_SINGLE_COIL result %s, took %f seconds", str(self.results[tag]), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on WRITE_SINGLE_COIL, took %f seconds", elapsed_time.to_sec())
        return WriteBitsResponse()

    def handle_write_register(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d WRITE_SINGLE_REGISTER request for address %d : %s", req.slave, req.address, str(req.registers[0]))
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.WRITE_SINGLE_REGISTER, result_tag=tag, output_value=req.registers[0])
        self.requests_queue.put(request)
        if self.wait_for_reply(tag):
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > WRITE_SINGLE_REGISTER result %s, took %f seconds", str(self.results[tag]), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on WRITE_SINGLE_REGISTER, took %f seconds", elapsed_time.to_sec())
        return WriteRegistersResponse()

    def handle_write_coils(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d WRITE_MULTIPLE_COILS request for address %d : %s", req.slave, req.address, str(output_value=req.bits))
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.WRITE_MULTIPLE_COILS, result_tag=tag, output_value=req.bits)
        self.requests_queue.put(request)
        if self.wait_for_reply(tag):
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > WRITE_MULTIPLE_COILS result %s, took %f seconds", str(self.results[tag]), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on WRITE_MULTIPLE_COILS, took %f seconds", elapsed_time.to_sec())
        return WriteBitsResponse()

    def handle_write_registers(self, req):
        tag = str(uuid.uuid1().hex)
        rospy.logdebug("[CoolFarm Modbus RTU] %d WRITE_MULTIPLE_REGISTERS request for address %d : %s", req.slave, req.address, str(req.registers))
        start_time = rospy.Time.now()
        request = ModbusRS485Request(req, cst.WRITE_MULTIPLE_REGISTERS, result_tag=tag, output_value=req.registers)
        self.requests_queue.put(request)
        if self.wait_for_reply(tag):
            elapsed_time = rospy.Time.now() - start_time
            rospy.logdebug("[CoolFarm Modbus RTU]   > WRITE_MULTIPLE_REGISTERS result %s, took %f seconds", str(self.results[tag]), elapsed_time.to_sec())
            del self.results[tag]
        else:
            elapsed_time = rospy.Time.now() - start_time
            rospy.logerr("[CoolFarm Modbus RTU]   > Timeout on WRITE_MULTIPLE_REGISTERS, took %f seconds", elapsed_time.to_sec())
        return WriteRegistersResponse()

if __name__ == '__main__':
    try:
        server = ModbusRS485Server()
        server.spin()
    except rospy.ROSInterruptException:
        pass
