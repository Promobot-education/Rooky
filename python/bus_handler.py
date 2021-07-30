#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Модуль для работы с датчиком расстояния Promobot по шине Modbus RTU."""

__author__ = "Promobot"
__license__ = "Apache License, Version 2.0"
__status__ = "Production"
__url__ = "https://git.promo-bot.ru"
__version__ = "0.1.0"


import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


class Bus():

    """Класс для работы с шиной данных

    Args:
        * port (str): Имя последовательног порта шины данных, например ``/dev/RS485``.
        * debug (bool): Вывод отладочной информации
        * baudrate (int): Скорость соединения. По умолчанию 460800 Бод
        * tieout (float): Таймаут соединения
        
    """

    def __init__(self, port, baudrate = 460800, debug = False, timeout = 1.0, port_forward = False):

        self.bus = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity='N', stopbits=1, xonxoff=0, rtscts=port_forward, dsrdtr=port_forward)) 

        self.bus.set_verbose(debug)

        self.bus.set_timeout(timeout)




