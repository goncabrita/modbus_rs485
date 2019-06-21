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
from modbus_rs485.srv import *

class ModbusRS485Result:

    def __init__(self, bits=None, registers=None):
        if bits is not None:
            self.bits = bits
        if registers is not None:
            self.registers = registers

class ModbusRS485Client:

    def __init__(self, slave, namespace):
        self.slave = slave
        self.namespace = namespace

    def connect(self):
        rospy.wait_for_service('/'+self.namespace+'/read_coils')
        rospy.wait_for_service('/'+self.namespace+'/read_discrete_inputs')
        rospy.wait_for_service('/'+self.namespace+'/read_holding_registers')
        rospy.wait_for_service('/'+self.namespace+'/read_input_registers')
        rospy.wait_for_service('/'+self.namespace+'/write_coil')
        rospy.wait_for_service('/'+self.namespace+'/write_register')
        rospy.wait_for_service('/'+self.namespace+'/write_coils')
        rospy.wait_for_service('/'+self.namespace+'/write_registers')
        return True

    def read_coils(self, address, count):
        read_coils_srv = rospy.ServiceProxy('/'+self.namespace+'/read_coils', ReadBits)
        result = read_coils_srv(address, count, self.slave)
        return ModbusRS485Result(bits=result.bits)

    def read_discrete_inputs(self, address, count):
        read_discrete_inputs_srv = rospy.ServiceProxy('/'+self.namespace+'/read_discrete_inputs', ReadRegisters)
        result = read_discrete_inputs_srv(address, count, self.slave)
        return ModbusRS485Result(registers=result.registers)

    def read_holding_registers(self, address, count):
        read_holding_registers_srv = rospy.ServiceProxy('/'+self.namespace+'/read_holding_registers', ReadRegisters)
        result = read_holding_registers_srv(address, count, self.slave)
        return ModbusRS485Result(registers=result.registers)

    def read_input_registers(self, address, count):
        read_input_registers_srv = rospy.ServiceProxy('/'+self.namespace+'/read_input_registers', ReadRegisters)
        result = read_input_registers_srv(address, count, self.slave)
        return ModbusRS485Result(registers=result.registers)

    def write_coil(self, address, bit):
        write_coil_srv = rospy.ServiceProxy('/'+self.namespace+'/write_coil', WriteBits)
        write_coil_srv(address, [bit], self.slave)
        return True

    def write_register(self, address, register):
        write_register_srv = rospy.ServiceProxy('/'+self.namespace+'/write_register', WriteRegisters)
        write_register_srv(address, [register], self.slave)
        return True

    def write_coils(self, address, bits):
        write_coils_srv = rospy.ServiceProxy('/'+self.namespace+'/write_coils', WriteBits)
        write_coils_srv(address, bits, self.slave)
        return True

    def write_registers(self, address, registers):
        write_registers_srv = rospy.ServiceProxy('/'+self.namespace+'/write_registers', WriteRegisters)
        write_registers_srv(address, registers, self.slave)
        return True
