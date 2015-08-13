'''
Copyright 2015 Stefan Andrei Chelariu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''
import smbus
import math
from time import sleep

class BMP085:
	bus 	 = None
	cal_data = None
	DEV_ADDR = 0x77
	T_READ   = 0x2E
	P_READ   = 0x34
	factor   = None
	oss 	 = 0

	'''
	*Register Map
	'''
	AC1_MSB = 0xAA
	AC1_LSB = 0xAB
	AC2_MSB = 0xAC
	AC2_LSB = 0xAD
	AC3_MSB = 0xAE
	AC3_LSB = 0xAF
	AC4_MSB = 0xB0
	AC4_LSB = 0xB1
	AC5_MSB = 0xB2
	AC5_LSB = 0xB3
	AC6_MSB = 0xB4
	AC6_LSB = 0xB5
	B1_MSB  = 0xB6
	B1_LSB  = 0xB7
	B2_MSB  = 0xB8
	B2_LSB  = 0xB9
	MB_MSB  = 0xBA
	MB_LSB  = 0xBB
	MC_MSB  = 0xBC
	MC_LSB  = 0xBD
	MD_MSB  = 0xBE
	MD_LSB  = 0xBF
	BUF_RD  = 0xF4
	BUF_MSB = 0xF6
	BUF_LSB = 0xF7
	BUF_XLSB= 0xF8

	def _format_data(self, num):
		tmp = (2**16) - 1
		if num&(1<<15):
			return num | ~tmp
		else:
			return num & tmp
		
	def _read_cal_data(self):
		return {
			'ac1' : self.bus.read_byte_data(self.DEV_ADDR, self.AC1_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC1_LSB),
			'ac2' : self.bus.read_byte_data(self.DEV_ADDR, self.AC2_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC2_LSB),
			'ac3' : self.bus.read_byte_data(self.DEV_ADDR, self.AC3_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC3_LSB),
			'ac4' : self.bus.read_byte_data(self.DEV_ADDR, self.AC4_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC4_LSB),
			'ac5' : self.bus.read_byte_data(self.DEV_ADDR, self.AC5_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC5_LSB),
			'ac6' : self.bus.read_byte_data(self.DEV_ADDR, self.AC6_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.AC6_LSB),
			'b1'  : self.bus.read_byte_data(self.DEV_ADDR,  self.B1_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.B1_LSB),
			'b2'  : self.bus.read_byte_data(self.DEV_ADDR,  self.B2_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.B2_LSB),
			'mb'  : self.bus.read_byte_data(self.DEV_ADDR,  self.MB_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.MB_LSB),
			'mc'  : self.bus.read_byte_data(self.DEV_ADDR,  self.MC_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.MC_LSB),
			'md'  : self.bus.read_byte_data(self.DEV_ADDR,  self.MD_MSB) << 8 | self.bus.read_byte_data(self.DEV_ADDR, self.MD_LSB)
			}

	def _get_raw_temp(self):
		self.bus.write_byte_data(self.DEV_ADDR, self.BUF_RD, self.T_READ)
		sleep(0.005)
		return self.bus.read_byte_data(self.DEV_ADDR, self.BUF_MSB) <<8 | self.bus.read_byte_data(self.DEV_ADDR, self.BUF_LSB)
		
	def _get_raw_pres(self):
		self.bus.write_byte_data(self.DEV_ADDR, self.BUF_RD, self.P_READ + (self.oss<<6))	
		sleep(0.005)
		return (self.bus.read_byte_data(self.DEV_ADDR, self.BUF_MSB) << 16 | self.bus.read_byte_data(self.DEV_ADDR, self.BUF_LSB) <<8 | self.bus.read_byte_data(self.DEV_ADDR, self.BUF_XLSB)) >> (8 - self.oss)

	def get_temperature(self):
		_a = (self._get_raw_temp() - self.cal_data['ac6'])*self.cal_data['ac5']/math.pow(2,15)
		_b = self.cal_data['mc']*math.pow(2,11)/(_a + self.cal_data['md'])
		self.factor = _a + _b
		return ((_a + _b + 8)/math.pow(2,4))/10

	def get_pressure(self):
		_k1 = self.factor -4000
		_a  = (self.cal_data['b2']*math.pow(_k1,2)/math.pow(2,12))/math.pow(2,11)
		_b  = self.cal_data['ac2']*_k1/math.pow(2,11)
		_c  = _a + _b
		_k2 = ((self.cal_data['ac1']*4 + _c) + 2)/4
		_a  = self.cal_data['ac3']*_k1/math.pow(2,13)
		_b  = (self.cal_data['b1']*(math.pow(_k1,2)/math.pow(2,12)))/math.pow(2,16)
		_c  = ((_a + _b) + 2)/math.pow(2,2)
		_k3 = self.cal_data['ac4']*(int((_c + 32768))&0xffffffff)/math.pow(2,15)
		_k4 = ((self._get_raw_pres()&0xffffffff) - _k2)*50000
		if _k4 < 0x80000000:
			_p = (_k4*2)/_k3
		else:
			_p = (_k4/_k3)*2
		_a  = math.pow((_p/math.pow(2,8)),2)
		_a  = (_a*3038)/math.pow(2,16)
		_b  = (-7357*_p)/math.pow(2,16)
		_p  = _p + (_a + _b + 3791)/math.pow(2,4)
		return _p				 
	
	def __init__(self, i2c_module):
		self.bus = smbus.SMBus(i2c_module)
		self.cal_data = self._read_cal_data()
		self.cal_data['ac1'] = self._format_data(self.cal_data['ac1'])
                self.cal_data['ac2'] = self._format_data(self.cal_data['ac2'])
                self.cal_data['ac3'] = self._format_data(self.cal_data['ac3'])
                self.cal_data['b1']  = self._format_data(self.cal_data['b1'])
                self.cal_data['b2']  = self._format_data(self.cal_data['b2'])
                self.cal_data['mb']  = self._format_data(self.cal_data['mb'])
                self.cal_data['mc']  = self._format_data(self.cal_data['mc'])
                self.cal_data['md']  = self._format_data(self.cal_data['md'])
		#TODO: check if cal_data is valid
