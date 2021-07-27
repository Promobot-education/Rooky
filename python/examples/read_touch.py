#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
"""Пример работы с датчиком касания Rooky."""

__author__ = "Promobot"
__license__ = "Apache License, Version 2.0"
__status__ = "Production"
__url__ = "https://git.promo-bot.ru"
__version__ = "0.1.0"

# Импортируем необходимые библиотеки
# Библиотека для работы с Rooky
import Rooky

# Библиотека для работы с временными задержками
import time

# Создадим объект Rooky в соответствии с его типом: левая или правая
# '/dev/RS_485' - последовательный порт, для ubuntu по умолчанию - '/dev/RS_485'.
# 'right' - тип Rooky (left или right)
arm = Rooky.Rooky('/dev/RS_485','right')

while True:
	# Для наглядности будем выводить безразмерные показания с платы
	# Управляющая плата имеет возможность подключения двух датчиков касания
	# Но задействован только один
	vals = arm.read_touch()

	# Если было зарегистрировано касание датчика, выведем информацию в консоль
	if arm.is_touched():
		print("Touched")
	print(vals)
	time.sleep(1)
