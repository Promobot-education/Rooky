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


import Rooky

import time

arm = Rooky.Rooky('/dev/RS_485','left')

while True:
	arm.move_joint('join_1',6,30)
	arm.read_servos()
	time.sleep(5)
	arm.move_joint('join_1',6,0)
	time.sleep(5)
