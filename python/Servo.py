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

import struct


# Servo registers
_TORQUE_REG = 41
_SETPOINT_REG = 42
_POS_REG = 47
_SPEED_REG = 48

_MODE_1_REG = 3
_MODE_2_REG = 4
_COMMAND_REG = 40
_ERRORS_REG = 45
_CURRENT_REG = 49

_SPEED_LIMIT_REG = 22

_PID_SPEED_P_REG = 10
_PID_SPEED_I_REG = 12
_PID_SPEED_D_REG = 14

_PID_POS_P_REG = 16
_PID_POS_I_REG = 18
_PID_POS_D_REG = 20


# Allowed commands
_PASS_COMMANDS = [0x0, 0xDEAD,
                  0xAAAA]

# PID max and min values
_PID_MAX_VAL = 20.0
_PID_MIN_VAL = 0.0

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


class Servo():
    """Класс для работы с сервоприводом.

    Args:
        * addr (int): Адрес устройства. 1-250
        * master (ModbusRTU): объект посдеовательного порта`.
        * init (bool): Инициализация
    """

    def __init__(self, addr, master, init=True):

        self.master = master
        self.addr = addr
        self.logger = modbus_tk.utils.create_logger("console")
        if (init):
            self._init_settings()

    #######PRIVATE FUNCTIONS#######

    def except_decorator(fn):
        def wrapped(self, *args):
            try:
                return fn(self, *args)
            except Exception as e:
                self.logger.error("%s", e)
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
    
    @except_decorator
    def _check_PID_val(self, value):
        if  _PID_MIN_VAL <= value <= _PID_MAX_VAL:
            return 1
        else:
            raise ValueError("Wrong value for PID register!")


    def _init_settings(self):
        try:
            mode = self.master.execute(
                self.addr, cst.READ_HOLDING_REGISTERS, _MODE_2_REG, 1)
        except:
            print('ERROR! Cant init servo:{0}'.format(self.addr))
            return False

        mode_new = mode[0] & ~(1 << 0)
        try:
            self.master.execute(
                self.addr, cst.WRITE_SINGLE_REGISTER, _MODE_2_REG, output_value=mode_new)
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
            None
        """
        if state == 1 or state == 0:
            return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _TORQUE_REG, output_value=state)
        else:
            raise ValueError("Wrong value for torque_register!")

    @except_decorator
    def set_speed(self, speed):
        """Установка ограничения скорости сервопривода, об/мин

        Args:
            * speed (float): speed limit. от 0.0 до 100.0
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if 0 <= speed <= 100:
            val = self._float_to_bytes(speed)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _SPEED_LIMIT_REG, output_value=val)
        else:
            raise ValueError("Wrong speed value!")

    @except_decorator
    def set_command(self, command):
        """Отправка команды в сервопривод.

        Args:
            * command (int(hex)): одна из доступных команд. 
              Разрешенные комманды: 0xDEAD, 0xAAAA
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None

        """
        if command in _PASS_COMMANDS:
            return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _COMMAND_REG, output_value=command)
        else:
            raise ValueError("Wrong command for command_register!")

    @except_decorator
    def set_point(self, value):
        """Установка задачи. (Положение, скорость, ШИМ в зависимости от режима работы)

        Args:
            * value (int): Требуемое значение. от -32768 до 32767.
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None

        """
        return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _SETPOINT_REG, output_value=value)

    @except_decorator
    def set_Pos_PID_P(self, value):
        """Запись коэф. P в ПИД регулятор по положению.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None

        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_POS_P_REG, output_value=val)

    @except_decorator
    def set_Pos_PID_I(self, value):
        """Запись коэф. I в ПИД регулятор по положению.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None

        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_POS_I_REG, output_value=val)

    @except_decorator
    def set_Pos_PID_D(self, value):
        """Запись коэф. D в ПИД регулятор по положению.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_POS_D_REG, output_value=val)

    @except_decorator
    def set_Speed_PID_P(self, value):
        """Запись коэф. P в ПИД регулятор по скорости.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_SPEED_P_REG, output_value=val)

    @except_decorator
    def set_Speed_PID_I(self, value):
        """Запись коэф. I в ПИД регулятор по скорости.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_SPEED_I_REG, output_value=val)

    @except_decorator
    def set_Speed_PID_D(self, value):
        """Запись коэф. D в ПИД регулятор по скорости.

        Args:
            * value (float): Желаемое значение в диапазоне от 0.0 до 20.0. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if (self._check_PID_val(value)):
            val = self._float_to_bytes(value)
            return self.master.execute(self.addr, cst.WRITE_MULTIPLE_REGISTERS, _PID_SPEED_D_REG, output_value=val)

    @except_decorator
    def set_PID_Mode(self, value):
        """Высталвение режима работы ПИД регуляторов сервопривода.

        Args:
            * value (str): Режим работы.
                | "NORMAL" - каскадный режим ПИД ругялтора по скорости и положению.
                | "PWM" - отключение всех ПИД регуляторов и прямое управление скважностью ШИМ.
                | "SPEED" - включение только ПИД регулятора по сокрости.                  
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        mode = self.master.execute(
            self.addr, cst.READ_HOLDING_REGISTERS, _MODE_1_REG, 1)
        if value == "NORMAL":
            mode_new = mode[0] | (1 << 1)
            mode_new = mode_new | (1 << 2)
        if value == "PWM":
            mode_new = mode[0] & ~(1 << 1)
            mode_new = mode_new & ~(1 << 2)
        if value == "SPEED":
            mode_new = mode[0] | (1 << 1)
            mode_new = mode_new & ~(1 << 2)
        return self.master.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _MODE_1_REG, output_value=mode_new)

    def set_PID(self, mode, val_P, val_I, val_D):
        """Высталвение режима работы ПИД регуляторов c заданными коэффициентами.
        
        Args:
            * mode (str): Режим работы.
                | "position" - каскадный режим ПИД ругялтора по скорости и положению.
                | "speed" - включение только ПИД регулятора по сокрости.
            * val_P - пропорциональный коэффициент
            * val_I - интегральный коэффициент
            * val_D - дифференциальный коэффициент
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка
        Raises:
            None
        """
        if mode == "speed":
            self.set_PID_Mode("SPEED")
            command_list = (
                [self.set_Speed_PID_P, val_P],
                [self.set_Speed_PID_I, val_I],
                [self.set_Speed_PID_D, val_D])
        elif mode == "position":
            self.set_PID_Mode("NORMAL")
            command_list = (
                [self.set_Pos_PID_P, val_P],
                [self.set_Pos_PID_I, val_I],
                [self.set_Pos_PID_D, val_D])
        else:
            return False
        for c, p in command_list:
            if not c(p):
                return False
        return True

    def get_data(self):
        """Чтение данных с сервопривода

        Args:
            None
        Returns:
            * Словарь с ключами: 
                | "ID", 
                | "Torque", 
                | "Setpoint" 
                | "Position" 
                | "Speed" 
                | "Command" 
                | "Current" 
                | "Pos_PID_P" 
                | "Pos_PID_I" 
                | "Pos_PID_D" 
                | "Speed_PID_P" 
                | "Speed_PID_I" 
                | "Speed_PID_D" 
                | "Errors"
                или пустой, при ошибке
        Raises:
            None
        """
        data = {}
        try: 
            values = self.master.execute(self.addr, cst.READ_HOLDING_REGISTERS, 0, 50)
        except Exception:
            return data
        if values:
            data["ID"] = values[0]
            data["Torque"] = values[_TORQUE_REG]
            data["Setpoint"] = self._getSignedNumber(values[_SETPOINT_REG], 16)
            data["Position"] = self._getSignedNumber(values[_POS_REG], 16)
            data["Speed"] = self._getSignedNumber(values[_SPEED_REG], 16)
            data["Command"] = values[_COMMAND_REG]
            data["Current"] = self._getSignedNumber(values[_CURRENT_REG], 16)
            data["Pos_PID_P"] = self._bytes_to_float(values[_PID_POS_P_REG], values[_PID_POS_P_REG + 1])
            data["Pos_PID_I"] = self._bytes_to_float(values[_PID_POS_I_REG], values[_PID_POS_I_REG + 1])
            data["Pos_PID_D"] = self._bytes_to_float(values[_PID_POS_D_REG], values[_PID_POS_D_REG + 1])
            data["Speed_PID_P"] = self._bytes_to_float(values[_PID_SPEED_P_REG], values[_PID_SPEED_P_REG + 1])
            data["Speed_PID_I"] = self._bytes_to_float(values[_PID_SPEED_I_REG], values[_PID_SPEED_I_REG + 1])
            data["Speed_PID_D"] = self._bytes_to_float(values[_PID_SPEED_D_REG], values[_PID_SPEED_D_REG + 1])
            data["Errors"] = self._read_errors(values[_ERRORS_REG])
        return data
