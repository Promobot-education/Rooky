#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
"""Модуль для работы с манипулятором Rooky."""

__author__ = "Promobot"
__license__ = "Apache License, Version 2.0"
__status__ = "Production"
__url__ = "https://git.promo-bot.ru"
__version__ = "0.1.0"


import bus_handler
import time
import Servo
import Servo_ppm

_right_arm_servos = [21, 22, 23, 24, 25]
_left_arm_servos = [31, 32, 33, 34, 35]
_right_arm_ppm_addr = 26
_left_arm_ppm_addr = 36

joint_ratio = {}
joint_limits = {}

joint_ratio['joint_1'] = 3
joint_ratio['joint_2'] = 1.9
joint_ratio['joint_3'] = 1.9
joint_ratio['joint_4'] = 1.2
joint_ratio['joint_5'] = 1.2
joint_ratio['joint_6'] = 1.7
joint_ratio['joint_7'] = 0.37

joint_limits['joint_1'] = [-25, 134]
joint_limits['joint_2'] = [0, 83]
joint_limits['joint_3'] = [-90, 83]
joint_limits['joint_4'] = [0, 80]
joint_limits['joint_5'] = [-86, 86]
joint_limits['joint_6'] = [-20, 31]
joint_limits['joint_7'] = [0, 74]

_rooky_parts = ['left', 'right']

class Rooky():
	"""Класс для работы с манипулятором.

	Args:
		* port (str): имя посдеовательного порта манипулятора.
		* side (str): Типа манипулятора. Левый или правый.
		* debug (bool): Вывод отладочной информации в консоль.
	
	"""
	def __init__(self, port, side = "left", debug = False):
		self._debug = debug
		self._rooky_side = side
		self._arm_servos = []
		self._arm_ppm_servos = []
		self._servos_id_list = []
		self._cur_setp = {}
		self._cur_setp['joint_2'] = 0
		self._cur_setp['joint_3'] = 0
		self._cur_setp['joint_4'] = 0
		self._cur_setp['joint_5'] = 0

		if side in _rooky_parts:

			self.master = bus_handler.Bus(port = port, baudrate = 460800, debug = debug, timeout = 1.0)

			if side == 'right':
				self._servos_id_list = _right_arm_servos
				self._ppm_addr = _right_arm_ppm_addr
				self._ppm_start_pos = (0, 90)

			if side == 'left':
				self._servos_id_list = _left_arm_servos
				self._ppm_addr = _left_arm_ppm_addr   
				self._ppm_start_pos = (0, -90)


			for i in self._servos_id_list:
				self._arm_servos.append(Servo.Servo(i,self.master.bus))

			self._arm_ppm_servos.append(Servo_ppm.Servo_ppm(self._ppm_addr, self.master.bus, 1, self._ppm_start_pos[0]))
			self._arm_ppm_servos.append(Servo_ppm.Servo_ppm(self._ppm_addr, self.master.bus, 2, self._ppm_start_pos[1]))

			for servo in self._arm_servos:	
				servo.set_command(0x0)
				servo.set_speed(10)
				servo.set_torque(True)
				servo.set_point(0)

			self._untouched_values = self._arm_ppm_servos[0].read_touch()
		time.sleep(3)		


	def _degrees_to_ticks(self,deg):
		return deg / 0.0219


	def _check_limits(self,name,position):
		if joint_limits[name][0] <= position <= joint_limits[name][1]:
			return 1
		else:
			print("ERROR! %s: invalid setpoint: %d" % (name,position)) 

	def _move_simple_joint(self,name,speed,position):
		if (self._check_limits(name,position)):

			setpoint = int(self._degrees_to_ticks(position) * joint_ratio[name])

			self._arm_servos[0].set_torque(True)

			self._arm_servos[0].set_speed(speed)

			self._arm_servos[0].set_point(setpoint)
		else:
			return -1

	def _move_diff_joint(self,name,speed):
		if name == 'joint_2' or name == 'joint_3':
			self._arm_servos[1].set_speed(speed)
			self._arm_servos[2].set_speed(speed)

			self._arm_servos[1].set_torque(True)
			self._arm_servos[2].set_torque(True)

			self._arm_servos[1].set_point(self._cur_setp['joint_2'] + self._cur_setp['joint_3'])
			self._arm_servos[2].set_point(self._cur_setp['joint_2'] - self._cur_setp['joint_3'])
		if name == 'joint_4' or name == 'joint_5':
			self._arm_servos[3].set_speed(speed)
			self._arm_servos[4].set_speed(speed)

			self._arm_servos[3].set_torque(True)
			self._arm_servos[4].set_torque(True)

			self._arm_servos[3].set_point(self._cur_setp['joint_4'] + self._cur_setp['joint_5'])
			self._arm_servos[4].set_point(self._cur_setp['joint_4'] - self._cur_setp['joint_5'])


	def _set_diff_joint(self,name,speed,position):
		if (self._check_limits(name,position)):
			self._cur_setp[name] = int(self._degrees_to_ticks(position) * joint_ratio[name])
			self._move_diff_joint(name,speed)
		else:
			return -1

	def _move_servo_joint(self, name, position):
		if name == 'joint_6':
			self._arm_ppm_servos[0].set_point(position * joint_ratio[name])
		elif name == 'joint_7' and self._rooky_side is "left":
			self._arm_ppm_servos[1].set_point(self._ppm_start_pos[1] + position * joint_ratio[name])
		elif name == 'joint_7' and self._rooky_side is "right":
			self._arm_ppm_servos[1].set_point(self._ppm_start_pos[1] - position * joint_ratio[name])

	def _check_erros(self,data):
		if len(data["Errors"]) > 0:
			print("ERROR! Servo: %d: in troubles: %s" % (data["ID"],data["Errors"])) 

	def move_joint(self,joint_name,speed = 10,position = 0):
		"""Перемещение сустава в желаемую позицию.

		Args:
			* joint_name (str):имя сустава.
			* speed (float): Скорость перемещения сустава.
			* position (float): Желаемая позиция сустава в градусах.
		
		"""
		if joint_name == 'joint_1':
			self._move_simple_joint(joint_name, speed, position)
		if joint_name == 'joint_2':
			self._set_diff_joint(joint_name, speed, position)
		if joint_name == 'joint_3':
			self._set_diff_joint(joint_name, speed, position) 
		if joint_name == 'joint_4':
			self._set_diff_joint(joint_name, speed, position)       
		if joint_name == 'joint_5':
			self._set_diff_joint(joint_name, speed, position)
		if joint_name == 'joint_6':
			self._move_servo_joint(joint_name, position)
		if joint_name == 'joint_7':
			self._move_servo_joint(joint_name, position)

	def read_servos(self):
		"""Чтение доступных данных с сервоприводов
		Args:
			None
		Returns:
			* список словарей с информацией от сервоприводов.
			См Servo.get_data(), а также Servo_ppm.Servo_ppm.get_data()
		Raises:
			None
		"""
		_servo_data = [None] * len(self._arm_servos)
		i = 0
		for servo in self._arm_servos:
			_servo_data[i] = servo.get_data()
			if _servo_data[i]:
				self._check_erros(_servo_data[i])
			else:
				print(self._servos_id_list)
				print("ERROR! Cant read servo: %d" % (self._servos_id_list[i])) 
			i = i + 1
		for ppm in self._arm_ppm_servos:
			_servo_data.append(ppm.get_data())
		return _servo_data

	def read_touch(self):
		"""Чтение данных с датчиков касания

        Args:
            None
        Returns:
            * Словарь с ключами:
                | "Touch_1"
                | "Touch_2"
				или пустой, при ошибке

        Raises:
            None

        """
		return self._arm_ppm_servos[0].read_touch()

	def is_touched(self):
		"""Есть ли факт касания датчика касания

        Args:
            None
        Returns: 
			* True - есть касание датчика
			  False - нет касания датчика

        """
		vals = self._arm_ppm_servos[0].read_touch()
		if vals["Touch_1"] < self._untouched_values["Touch_1"] * 0.95 or vals["Touch_2"] < self._untouched_values["Touch_2"] * 0.95:
			return True
		else:
			return False

	def relax(self,state):
		"""Полностью (включая тормоз) отключить питание двигателей всех суставов.

		Args:
			* state (bool):True - отключить питание. False - включить питание.
		
		"""
		if state:
			for servo in self._arm_servos:
				servo.set_command(0xAAAA)
			self._arm_ppm_servos[0].set_torque(0)
		else:
			for servo in self._arm_servos:
				servo.set_command(0x0)
			self._arm_ppm_servos[0] = Servo_ppm.Servo_ppm(self._ppm_addr,self.master.bus, 1, self._ppm_start_pos[0])
			self._arm_ppm_servos[1] = Servo_ppm.Servo_ppm(self._ppm_addr,self.master.bus, 2, self._ppm_start_pos[1])


