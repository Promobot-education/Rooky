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

_right_arm_servos = [21, 22, 23, 24, 25]
_left_arm_servos = [31, 32, 33, 34, 35]


_right_arm_servos_ppm = [26]
_left_arm_servos_ppm = [36]


joint_ratio = {}
joint_limints = {}


joint_ratio['joint_1'] = 3
joint_ratio['joint_2'] = 1.9
joint_ratio['joint_3'] = 1.9
joint_ratio['joint_4'] = 1.25
joint_ratio['joint_5'] = 1.25

joint_limints['joint_1'] = [-10, 90]
joint_limints['joint_2'] = [-10, 90]
joint_limints['joint_3'] = [-90, 90]
joint_limints['joint_4'] = [-10, 90]
joint_limints['joint_5'] = [-90, 90]



_rooky_parts = ['left', 'right']

class Rooky():
	"""Класс для работы с манипулятором.

	Args:
		* port (str): имя посдеовательного порта манипулятора.
		* side (str): Типа манипулятора. Левый или правый.
	
	"""
	def __init__(self,port,side = "left",debug = False):

		self._arm_servos = []

		self._servos_id_list = []

		self._debug = debug

		self._cur_setp = {}
		self._cur_setp['joint_2'] = 0
		self._cur_setp['joint_3'] = 0
		self._cur_setp['joint_4'] = 0
		self._cur_setp['joint_5'] = 0


		if side in _rooky_parts: 


			self.master = bus_handler.Bus(port = port, baudrate = 460800, debug = debug, timeout = 1.0)

			if side == 'right':
				self._servos_id_list = _right_arm_servos
				self.servos_ppm = _right_arm_servos_ppm

			if side == 'left':
				self._servos_id_list = _left_arm_servos
				self.servos_ppm = _left_arm_servos_ppm   


			for i in self._servos_id_list:
				self._arm_servos.append(Servo.Servo(i,self.master.bus))


			for servo in self._arm_servos:	

				servo.set_command(0x0)
				servo.set_speed_limit(10)
				servo.set_torque(True)
				servo.set_point(0)



		time.sleep(5)		


	def _degrees_to_ticks(self,deg):

		return deg / 0.0219



	def _check_limits(self,name,position):

		if position > joint_limints[name][0] and position < joint_limints[name][1]:
			return 1
		else:
			print("ERROR! %s: invalid setpoint: %d" % (name,position)) 




	def _move_simple_joint(self,name,speed,position):

		if (self._check_limits(name,position)):

			setpoint = int(self._degrees_to_ticks(position) * joint_ratio[name])

			self._arm_servos[0].set_torque(True)

			self._arm_servos[0].set_speed_limit(speed)

			self._arm_servos[0].set_point(setpoint)

		else:
			return -1



	def _move_diff_joint(self,name,speed):

	
		if name == 'joint_2' or name == 'joint_3':


			self._arm_servos[1].set_speed_limit(speed)
			self._arm_servos[2].set_speed_limit(speed)

			self._arm_servos[1].set_torque(True)
			self._arm_servos[2].set_torque(True)

			self._arm_servos[1].set_point(self._cur_setp['joint_2'] + self._cur_setp['joint_3'])
			self._arm_servos[2].set_point(self._cur_setp['joint_2'] - self._cur_setp['joint_3'])


		if name == 'joint_4' or name == 'joint_5':


			self._arm_servos[3].set_speed_limit(speed)
			self._arm_servos[4].set_speed_limit(speed)


			self._arm_servos[3].set_torque(True)
			self._arm_servos[4].set_torque(True)


			print(self._cur_setp['joint_4'] + self._cur_setp['joint_5'])
			print(self._cur_setp['joint_4'] - self._cur_setp['joint_5'])

			self._arm_servos[3].set_point(self._cur_setp['joint_4'] + self._cur_setp['joint_5'])
			self._arm_servos[4].set_point(self._cur_setp['joint_4'] - self._cur_setp['joint_5'])



	def _set_diff_joint(self,name,speed,position):


		if (self._check_limits(name,position)):

			self._cur_setp[name] = int(self._degrees_to_ticks(position) * joint_ratio[name])
			self._move_diff_joint(name,speed)

		else:
			return -1



	def _check_erros(self,data):

		if len(data["Errors"]) > 0:
			print("ERROR! Servo: %d: in troubles: %s" % (data["ID"],data["Errors"])) 



	def move_joint(self,joint_name,speed = 10,position = 0):
		"""Перемещение сустава в желаемую позицию.

		Args:
			* joint_name (str):имя сустава.
			* speed (int): Скорость перемещения сустава.
			* position (int): Желаемая позиция сустава в градусах.
		
		"""
		if joint_name == 'joint_1':
			self._move_simple_joint(joint_name,speed,position)

		if joint_name == 'joint_2':
			self._set_diff_joint(joint_name,speed,position)

		if joint_name == 'joint_3':
			self._set_diff_joint(joint_name,speed,position) 

		if joint_name == 'joint_4':
			self._set_diff_joint(joint_name,speed,position)       

		if joint_name == 'joint_5':
			self._set_diff_joint(joint_name,speed,position) 




	def relax(self,state):
		"""Полностью (включая тормоз) отключить питание двигателей всех суставов.

		Args:
			* state (bool):True - отключить питание. False - включить питание.
		
		"""
		if state:
			for servo in self._arm_servos:
				servo.set_command(0xAAAA)
		else:
			for servo in self._arm_servos:
				servo.set_command(0x0)



	def read_servos(self):
		"""Чтение данных.

		Args:
			* state (bool):True - отключить питание. False - включить питание.
		
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

		return _servo_data

