#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
"""Модуль для работы с сервоприводом Promobot по шине Modbus RTU."""

__author__ = "Promobot"
__license__ = "Apache License, Version 2.0"
__status__ = "Production"
__url__ = "https://git.promo-bot.ru"
__version__ = "0.1.0"

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import struct, sys

# Servo registers
_START_ANGLE_1_REG = 4
_START_ANGLE_2_REG = 5
_OVERCURRENT_TIME_REG = 8
_MAX_CURRENT_REG = 9

_COMMAND_REG = 40
_TORQUE_REG = 41
_SERVO_1_ANGLE_REG = 43
_SERVO_2_ANGLE_REG = 44
_ERRORS_REG = 45
_CURRENT_REG = 46
_VOLTAGE_REG = 47
_TOUCH_1_REG = 50
_TOUCH_2_REG = 51

# Allowed commands
_PASS_COMMANDS = [0xDEAD,
                  0xAAAA]

# Errors list
_ERRORS_LIST = ["COMMUNICATION",
                "HALL_BOARD",
                "WRONG_DIRECTION",
                "OVERCURRENT",
                "MAGNET_ERROR",
                "ENCODER",
                "DRV_ERR",
                "DISABLED",
                "REBOOTED"]


class Servo_ppm():
    """Класс для работы с сервоприводом.

    Args:
        * master (ModbusRTU): объект посдеовательного порта`.
        * addr (int): Адрес устройства. 1-250
    
    """

    def __init__(self, addr, master, servo_num, start_pos=0):
        if master is None:
            raise ValueError('Master is Null')
        self.master = master
        self.addr = addr
        self.logger = modbus_tk.utils.create_logger("console")
        self.angle_reg = _SERVO_1_ANGLE_REG if servo_num is 1 else _SERVO_2_ANGLE_REG
        self._init_settings(start_pos)
        self.set_torque(1)
        self.set_point(start_pos)

    #######PRIVATE FUNCTIONS#######

    def except_decorator(fn):
        def wrapped(self, *args):
            try:
                return fn(self, *args)
            except Exception as e:
                # self.logger.error("%s", e)
                return False

        return wrapped

    def _bytes_to_float(self, high, low):
        raw = struct.pack('>HH', low, high)
        fl = struct.unpack('>f', raw)[0]
        return round(float(fl), 2)

    def _float_to_bytes(self, value):
        val = float(value)
        ba = bytearray(struct.pack("f", val))
        vals = [0, 0]
        vals[0] = ba[1] << 8 | ba[0]
        vals[1] = ba[3] << 8 | ba[2]
        return vals

    def _getSignedNumber(self, number, bitLength):
        mask = (2 ** bitLength) - 1
        if number & (1 << (bitLength - 1)):
            return number | ~mask
        else:
            return number & mask

    def _read_errors(self, value):
        out_list = []
        error_list = bin(value)[2:].zfill(16)
        i = 0
        for e in reversed(error_list):
            if int(e) == 1:
                out_list.append(_ERRORS_LIST[i])
            i = i + 1
        return out_list

    def _init_settings(self, start_pos=0):
        try:
            self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, _TORQUE_REG, 1)
        except:
            print('ERROR! Cant init servo:{0}'.format(self.addr))
            return False

        try:
            self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _MAX_CURRENT_REG, output_value=2500)
            print('Servo:{0} Inited!'.format(self.addr))
            return True
        except:
            print('ERROR! Cant init servo:{0}'.format(self.addr))
            return False
            ############################

    @except_decorator
    def set_torque(self, state):
        """Включение(отключение) питания обмоток двигателя

        Args:       
            * state (int): 
                | 0 - отключение питания обмоток двигателя, обмотки замкнуты, двигатель в торможении. 
                | 1 - включение питания обмоток двигателя. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            ValueError

        """
        if state == 1 or state == 0:
            return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _TORQUE_REG, output_value=state)
        else:
            raise ValueError("Wrong value for torque_register!")

    @except_decorator
    def set_command(self, command):
        """Write command to servo.

        Args:
            * command (int): one of available commands. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            ValueError

        """
        if command in _PASS_COMMANDS:
            return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _COMMAND_REG, output_value=command)
        else:
            raise ValueError("Wrong command for command_register!")

    @except_decorator
    def set_point(self, value):
        """Установка задачи. (Положение, скорость, ШИМ в зависимости от режима работы)

        Args:
            * value (int): Требуемое значение. от -32000 to 32000.
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None

        """
        print(value)
        return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, self.angle_reg, output_value=value)

    @except_decorator
    def get_data(self):
        """Чтение данных с сервопривода

        Args:
            None
        Returns:
            * Словарь с ключами:
                | "Command"
                | "Torque"
                | "Angle"
                | "Errors"
                | "Current"
                | "Voltage"
        Raises:
            None

        """
        data = {}
        try:
            values = self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, 0, 51)
            pos = values[self.angle_reg].to_bytes(2, sys.byteorder, signed=False)
        except Exception:
            return data

        if values:
            data["Command"] = values[_COMMAND_REG]
            data["Torque"] = values[_TORQUE_REG]
            data["Angle"] = int.from_bytes(pos, sys.byteorder, signed=True)
            data["Errors"] = self._read_errors(_ERRORS_REG)
            data["Current"] = values[_CURRENT_REG]
            data["Voltage"] = values[_VOLTAGE_REG]

        return data

    def read_touch(self):
        """Чтение данных с датчика

        Args:
            None
        Returns:
            Словарь c ключами: 
                | "Touch_1"
                | "Touch_2"
        Raises:
            None

        """
        data = {}
        try:
            touch_1 = self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, _TOUCH_1_REG, 1)
            touch_2 = self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, _TOUCH_2_REG, 1)
        except Exception:
            return data
        data["Touch_1"] = touch_1[0]
        data["Touch_2"] = touch_2[0]

        return data
