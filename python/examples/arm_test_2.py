#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
"""Пример работы с манипулятором Rooky."""

__author__ = "Promobot"
__license__ = "Apache License, Version 2.0"
__status__ = "Production"
__url__ = "https://git.promo-bot.ru"
__version__ = "0.1.0"

# Импортируем необходимые библиотеки
# Библиотека для работы с Rooky версии 2
import Rooky2

# Библиотека для работы с временными задержками
import time

# Укажем тип Rooky left или right
side = "right"
# Создадим объект Rooky в соответствии с его типом: левая или правая
# '/dev/RS_485' - последовательный порт, для ubuntu по умолчанию - '/dev/RS_485'.
arm = Rooky2.Rooky('/dev/RS_485', side)

# Выведем сообщение, чтобы пользователь коснулся датчика
print('Touch sensor to start action')

while True:
	# Ждем касания датчика
	if not arm.is_touched():
		continue

	# Касание произошло, приступим к очереди движений
	arm.move_joints([{
		# Имя сустава должно выглядеть подобным образом
		# right_arm_1_joint или left_arm_1_joint
		'name':'{0}_arm_1_joint'.format(side),
		'degree': 100
	},{
		'name':'{0}_arm_2_joint'.format(side),
		'degree': 83
	},{
		'name':'{0}_arm_3_joint'.format(side),
		'degree': 10
	},{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 40
	},{
		'name':'{0}_arm_5_joint'.format(side),
		'degree': -10
	},{
		'name':'{0}_arm_6_joint'.format(side),
		'degree': -20
	},{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 70
		# 2.5 - это время в секундах
		# указывает как долго Rooky будет достигать заданной цели
	},],2.5)

	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 0
	},],0.8)
	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 80
	},],0.8)
	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 0
	},],0.8)
	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 80
	},],0.8)
	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 0
	},{
		'name':'{0}_arm_6_joint'.format(side),
		'degree': -20
	}],0.8)
	arm.move_joints([{
		'name':'{0}_arm_4_joint'.format(side),
		'degree': 80
	},],0.8)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 0
	},],0.3)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 74
	},],0.3)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 0
	},],0.3)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 74
	},],0.3)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 0
	},],0.3)
	arm.move_joints([{
		'name':'{0}_arm_7_joint'.format(side),
		'degree': 74
	},],0.3)
	time.sleep(2)

	# Вернем все суставы в начальное положение
	arm.reset_joints()

	# Получим информацию о состоянии сервоприводов
	for i in arm.read_servos_data():
		print(i)

	# Выведем информацию о необходимости касания для повторения действий
	print('Touch sensor to start action')
