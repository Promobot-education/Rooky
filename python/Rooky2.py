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
import copy
from threading import Thread, Event, Lock

_right_arm_servos = [21, 22, 23, 24, 25]
_left_arm_servos = [31, 32, 33, 34, 35]

_right_arm_ppm_addr = 26
_left_arm_ppm_addr = 36

joint_ratio = {}
joint_ratio['_arm_1_joint'] = 3
joint_ratio['_arm_2_joint'] = 1.9
joint_ratio['_arm_3_joint'] = 1.9
joint_ratio['_arm_4_joint'] = 1.2
joint_ratio['_arm_5_joint'] = 1.2
joint_ratio['_arm_6_joint'] = 1.7
joint_ratio['_arm_7_joint'] = 0.37

joint_limits = {}
joint_limits['_arm_1_joint'] = [-25, 134]
joint_limits['_arm_2_joint'] = [0, 83]
joint_limits['_arm_3_joint'] = [-90, 83]
joint_limits['_arm_4_joint'] = [0, 80]
joint_limits['_arm_5_joint'] = [-86, 86]
joint_limits['_arm_6_joint'] = [-20, 31]
joint_limits['_arm_7_joint'] = [0, 74]

ROOKY_PARTS = ['left', 'right']
TRAJ_HZ = 60
SERVO_HZ = 40
SAFE_SERVO_SPEED = 25
UNLIMIT_SERVO_SPEED = 100
gl_servo_speed = {} # список словарей с желаемыми скоростями для серв
gl_servo_setp = {} # список словарей с желаемыми позициями для серв
gl_servos_data = {} # список словарей с информацией от серв
gl_touch_values = [] # список значений от датчиков касания из двух возможных регистров
servo_setp_lock = Lock()        # синхронизация потоков


def _degrees_to_ticks(deg):
    return deg / 0.0219

def _tics_to_degrees(tics):
    return tics * 0.0219

def _check_erros(data):
    if len(data["Errors"]) > 0:
        print("ERROR! Servo: %d: in troubles: %s" %
              (data["ID"], data["Errors"]))



class Rooky:
    """Класс для работы с манипулятором.

    Args:
        * port (str): Имя посдеовательного порта манипулятора.
        * side (str): Типа манипулятора. Левый или правый.
        * debug (bool): Вывод отладочной информации в консоль.
    """

    def __init__(self, port, side="left", debug=False):
        global gl_servo_setp
        self._rooky_side = side
        self._arm_servos = []
        self._debug = debug
        self._touch_k = 0.95
        self._cur_setp = {  self._make_joint_name(1): 0, 
                            self._make_joint_name(2): 0, 
                            self._make_joint_name(3): 0, 
                            self._make_joint_name(4): 0, 
                            self._make_joint_name(5): 0, 
                            self._make_joint_name(6): 0, 
                            self._make_joint_name(7): 0
                        }

        if side not in ROOKY_PARTS:
            print ('Wrong side - {0}'.format(side))
            return False

        self._master = bus_handler.Bus(port=port, baudrate=460800, debug=debug, timeout=1.0)

        self._servos_id_list = []
        if side == 'right':
            self._servos_id_list = _right_arm_servos
            self._ppm_addr = _right_arm_ppm_addr
            self._ppm_start_pos = (0, 90)
        elif side == 'left':
            self._servos_id_list = _left_arm_servos
            self._ppm_addr = _left_arm_ppm_addr
            self._ppm_start_pos = (0, -90)

        gl_servo_setp = copy.deepcopy(self._convert_pos_to_raw(self._cur_setp))
        
        for i in range(1,6):
            gl_servo_speed["servo_{0}".format(i)] = SAFE_SERVO_SPEED

        self._read_flag = Event()
        self._reset_flag = Event()
        self._get_sensor_value_flag = Event()
        thread = PortController(self._master, self._servos_id_list, self._ppm_addr, self._ppm_start_pos, self._read_flag, self._reset_flag, self._get_sensor_value_flag)
        thread.daemon = True
        thread.start()
        self._untouch_value = self.get_sensor_value()
        time.sleep(3)
        print('Initialisation is complete')

    def _make_joint_name(self, joint_number):
        if self._rooky_side in ["left", "right"]:
            return "{0}_arm_{1}_joint".format(self._rooky_side, joint_number)
        else:
            return ""

    def _check_limits(self, name, position):
        common_name = name[4:] if self._rooky_side == 'left' else name[5:]
        if joint_limits[common_name][0] <= position <= joint_limits[common_name][1]:
            return True
        else:
            print("ERROR! %s: invalid setpoint: %d" % (name, position))
            return False

    # Конвертация позиций суставов в тики для сервоприводов
    def _convert_pos_to_raw(self, input_pos):
        positions = copy.copy(input_pos)
        dict = {}
        for i in range (1, 8):
            if self._make_joint_name(i) not in positions.keys():
                return dict
        
        # корректировка направления движения джоинта
        positions[self._make_joint_name(5)] = positions[self._make_joint_name(5)] * -1
        if self._rooky_side == "left":
            positions[self._make_joint_name(6)] = positions[self._make_joint_name(6)] * -1
        elif self._rooky_side == "right":
            positions[self._make_joint_name(7)] = positions[self._make_joint_name(7)] * -1
        
        dict['servo_1'] = int(_degrees_to_ticks(positions[self._make_joint_name(1)]) * joint_ratio['_arm_1_joint'])

        joint_2_tick = _degrees_to_ticks(positions[self._make_joint_name(2)]) * joint_ratio['_arm_2_joint']
        joint_3_tick = _degrees_to_ticks(positions[self._make_joint_name(3)]) * joint_ratio['_arm_3_joint']
        dict['servo_2'] = int(joint_2_tick + joint_3_tick)
        dict['servo_3'] = int(joint_2_tick - joint_3_tick)
        joint_4_tick = _degrees_to_ticks(positions[self._make_joint_name(4)]) * joint_ratio['_arm_4_joint']
        joint_5_tick = _degrees_to_ticks(positions[self._make_joint_name(5)]) * joint_ratio['_arm_5_joint']
        dict['servo_4'] = int(joint_4_tick + joint_5_tick)
        dict['servo_5'] = int(joint_4_tick - joint_5_tick)
        dict['servo_6'] = int(positions[self._make_joint_name(6)] * joint_ratio['_arm_6_joint'])
        dict['servo_7'] = int(self._ppm_start_pos[1] + positions[self._make_joint_name(7)] * joint_ratio['_arm_7_joint'])
        return dict

    def _create_trajectory(self, joints_dict, move_time):
        tm = 1 / TRAJ_HZ
        if move_time <= tm:
            self._move_several_joints(joints_dict, SAFE_SERVO_SPEED)
            return

        curr_pos_dict = copy.deepcopy(self._cur_setp)
        start_time = time.perf_counter()
        end_time = start_time + move_time
        iter_time = 0
        while (time.perf_counter() <= end_time):
            if time.perf_counter() >= iter_time + tm:
                iter_time = time.perf_counter()
                progress_k = (iter_time - start_time)/(end_time - start_time)
                pos_dict = self._calc_intermediate_position(curr_pos_dict, joints_dict, progress_k)
                self._move_several_joints(pos_dict, UNLIMIT_SERVO_SPEED)
        self._move_several_joints(joints_dict, UNLIMIT_SERVO_SPEED)
    
    def _move_several_joints(self, joints_dict, speed):
        global gl_servo_setp, gl_servo_speed
        for key in joints_dict:
            if not self._check_limits(key, joints_dict[key]):
                return
        for key in joints_dict:
            self._cur_setp[key] = joints_dict[key]
                
        servo_setp_lock.acquire()
        for key in gl_servo_speed:
            gl_servo_speed[key] = speed
        gl_servo_setp = copy.deepcopy(self._convert_pos_to_raw(self._cur_setp))
        servo_setp_lock.release()


    def _calc_intermediate_position(self, start_pos_dict, end_pos_dict, progress_k):
        if progress_k > 1:
            progress_k = 1
        intermediate_pos_dict = {}
        for key in end_pos_dict:
            intermediate_pos_dict[key] = start_pos_dict[key] + (end_pos_dict[key] - start_pos_dict[key])*progress_k
        return intermediate_pos_dict

    def move_joints(self, joints, move_time):
        """Установка указанных суставов на указанные углы поворота
        Args:
            * joints - list(dict('name': , 'degree': )) список словарей с ключами:
                    name - имя сустава, например для правой Rooky 'name':'right_arm_1_joint'
                    degree - абсолютный угол(градусы) - угол относительно нулевого положения, 
                             на который нужно установить сустав
            * move_time - время, за которое суставы должны оказаться в итоговом месте
        Returns:
            None
        Raises:
            None
        """
        joints_dict = {}
        for joint in joints:
            joints_dict[joint['name']] = joint['degree']
        self._create_trajectory(joints_dict, move_time)

    def read_servos_data(self):
        """Чтение доступных данных с сервоприводов
        Args:
            None
        Returns:
            * список словарей с информацией от сервоприводов.
            См Servo.get_data(), а также Servo_ppm.Main_ppm.get_data()
        Raises:
            None
        """
        global gl_servos_data
        self._read_flag.set()

        while self._read_flag.is_set():
            time.sleep(0.001)
        return gl_servos_data

    def get_sensor_value(self):
        """Чтение данных с датчика касания
        Args:
            None
        Returns:
            * value (int) - безразмерная величина, при касании уменьшается
        Raises:
            None
        """
        self._get_sensor_value_flag.set()
        while self._get_sensor_value_flag.is_set():
            time.sleep(0.001)
        return gl_touch_values['Touch_2']

    def is_touched(self):
        """Есть ли факт касания датчика касания
        Args:
            None
        Returns:
            * True - есть касание датчика
              False - нет касания датчика
        """
        val = self.get_sensor_value()
        if val < self._untouch_value * self._touch_k:
            return True
        else:
            return False

    def reset_joints(self):
        """Сброс всех суставов в нулевое положение
        Args:
            None
        """
        global gl_servo_setp
        for key in self._cur_setp:
            self._cur_setp[key] = 0
        servo_setp_lock.acquire()
        gl_servo_setp = copy.deepcopy(self._convert_pos_to_raw(self._cur_setp))
        servo_setp_lock.release()

        self._reset_flag.set()
        while self._reset_flag.is_set():
            time.sleep(0.001)

    def get_servo_angle(self, joint):
        """Получить положение сустава
        Args:
            * joint - имя сустава, например "right_arm_1_joint", 
                    допустимо передать номер сустава, например - 7
        Returns:
            * angle (float) - угол в котором сейчас находится сустав
        """
        global gl_servos_data
        self._read_flag.set()

        while self._read_flag.is_set():
            time.sleep(0.001)

        if isinstance(joint, int):
            joint = self._make_joint_name(joint)
      
        if "_arm_1_joint" in joint:
            data = gl_servos_data[0]
            angle = _tics_to_degrees(data['Position'])/joint_ratio['_arm_1_joint']
        elif "_arm_2_joint" in joint:
            servo2_pos = _tics_to_degrees(gl_servos_data[1]['Position'])
            servo3_pos = _tics_to_degrees(gl_servos_data[2]['Position'])
            angle = (servo2_pos/joint_ratio['_arm_2_joint'] + servo3_pos/joint_ratio['_arm_3_joint'])/2
        elif "_arm_3_joint" in joint:
            servo2_pos = _tics_to_degrees(gl_servos_data[1]['Position'])
            servo3_pos = _tics_to_degrees(gl_servos_data[2]['Position'])
            angle = (servo2_pos/joint_ratio['_arm_2_joint'] - servo3_pos/joint_ratio['_arm_3_joint'])/2
        elif "_arm_4_joint" in joint:
            servo4_pos = _tics_to_degrees(gl_servos_data[3]['Position'])
            servo5_pos = _tics_to_degrees(gl_servos_data[4]['Position'])
            angle = (servo4_pos/joint_ratio['_arm_4_joint'] + servo5_pos/joint_ratio['_arm_5_joint'])/2
        elif "_arm_5_joint" in joint:
            servo4_pos = _tics_to_degrees(gl_servos_data[3]['Position'])
            servo5_pos = _tics_to_degrees(gl_servos_data[4]['Position'])
            angle = (servo4_pos/joint_ratio['_arm_4_joint'] - servo5_pos/joint_ratio['_arm_5_joint'])/2
        elif "_arm_6_joint"in joint :
            data = gl_servos_data[5]
            angle = data['Angle_1']/joint_ratio['_arm_6_joint']
        elif "_arm_7_joint" in joint :
            data = gl_servos_data[5]
            if self._rooky_side == 'left':
                angle = (data['Angle_2'] - self._ppm_start_pos[1])/joint_ratio['_arm_7_joint']
            elif self._rooky_side == 'right':
                angle = (self._ppm_start_pos[1] - data['Angle_2'])/joint_ratio['_arm_7_joint']
        return angle

    def calibration(self):
        """Калибровка, в данном случае сброс суставов в начальное положение.
        Args:
            None
        """
        self.reset_joints()

    def set_touch_sensor_threshold(self, percent):
        """Установить чувствительность для определения факта касания
        Args:
            * percent - процент различий
                        чем меньше процент различий, 
                        тем выше программная чувствительность для регистрации касания
        """
        if percent > 100:
            percent = 100
        self._touch_k = 1-percent/100

class PortController(Thread):
    """Класс(поток), который постоянно пишет изменяющиеся данные в сервы, по запросу читает данные
    Args:
        master - класс для работы с портом 
        id_list - список id сервоприводов
        ppm_id - id управляющей ppm платы
        ppm_start_pos - (list) начальные позиции для двух сервоприводов 
        read_ev - событие на чтение из сервоприводов
        reset_ev - событие на сброс сервоприводов в ноль
        read_touch_ev - событие на чтение данных с датчика касания
    """
    def __init__(self, master, id_list, ppm_id, ppm_start_pos, read_ev, reset_ev, read_touch_ev):
        Thread.__init__(self)
        self.read_flag = read_ev
        self.reset_flag = reset_ev
        self.read_touch_flag = read_touch_ev
        self.master = master
        self.arm_servos = []
        self.current_servo_speed = {}
        self.desire_servo_speed = {}
        self.curr_servo_pos = {}
        for i, id in enumerate(id_list):
            servo = Servo.Servo(id, self.master.bus)
            servo.set_command(0x0)
            servo.set_torque(True)
            if servo.set_point(0):
                self._set_curr_servo_pos(i+1, 0) 
            if servo.set_speed(SAFE_SERVO_SPEED):
                self.current_servo_speed["servo_{0}".format(i+1)] = SAFE_SERVO_SPEED
            else:
                self.current_servo_speed["servo_{0}".format(i+1)] = -1
            self.arm_servos.append(servo)
        self._ppm_start_pos = ppm_start_pos
        self._arm_ppm_main = Servo_ppm.Main_ppm(ppm_id, self.master.bus, ppm_start_pos[0], ppm_start_pos[1])
        self._set_curr_servo_pos(6, ppm_start_pos[0])
        self._set_curr_servo_pos(7, ppm_start_pos[1])

    def _set_curr_servo_pos(self, j_num, raw_val):
        self.curr_servo_pos['servo_{0}'.format(j_num)] = raw_val
    
    def _get_curr_servo_pos(self, j_num):
        return self.curr_servo_pos['servo_{0}'.format(j_num)]

    def _write_to_servos(self):
        global gl_servo_speed, gl_servo_setp
        servo_setp_lock.acquire()
        for i in range(5):
            servo_name = 'servo_{0}'.format(i+1)
            curr_pos = self._get_curr_servo_pos(i+1)
            desire_pos = gl_servo_setp[servo_name]
            # Этап подбора скорости для движения
            if abs(desire_pos - curr_pos) > 1500:
                self.desire_servo_speed[servo_name] = SAFE_SERVO_SPEED
                print('Desired position ({1}) of servo_{0} is too far from current ({2}). Servo speed is limit to {3} rpm'.format(i+1, desire_pos, curr_pos, self.desire_servo_speed[servo_name]))
            else:
                self.desire_servo_speed[servo_name] = gl_servo_speed[servo_name]
            # этап записи в серву
            if curr_pos != desire_pos:
                # скорость
                if self.desire_servo_speed[servo_name] != self.current_servo_speed[servo_name]:
                    if self.arm_servos[i].set_speed(self.desire_servo_speed[servo_name]):
                        self.current_servo_speed[servo_name] = self.desire_servo_speed[servo_name]
                # позиция
                if self.arm_servos[i].set_point(desire_pos):
                    self._set_curr_servo_pos(i+1, desire_pos)
        
        # Исключили двойное обращение к плате управления
        if self.curr_servo_pos['servo_6'] != gl_servo_setp['servo_6'] or self.curr_servo_pos['servo_7'] != gl_servo_setp['servo_7']:
            if self._arm_ppm_main.set_points(gl_servo_setp['servo_6'], gl_servo_setp['servo_7']):
                self._set_curr_servo_pos(6, gl_servo_setp['servo_6'])
                self._set_curr_servo_pos(7, gl_servo_setp['servo_7'])
        servo_setp_lock.release()

    def _read_servos(self):
        global gl_servos_data
        _servo_data = [None] * len(self.arm_servos)
        i = 0
        for servo in self.arm_servos:
            _servo_data[i] = servo.get_data()
            if _servo_data[i]:
                _check_erros(_servo_data[i])
            else:
                print(self._servos_id_list)
                print("ERROR! Cant read servo: %d" % (self._servos_id_list[i]))
            i = i + 1
        _servo_data.append(self._arm_ppm_main.get_data())
        gl_servos_data = _servo_data

    def _read_touch(self):
        global gl_touch_values
        gl_touch_values = self._arm_ppm_main.read_touch()

    def _reset_servos(self):
        if self._arm_ppm_main.set_points(self._ppm_start_pos[0], self._ppm_start_pos[1]):
            self._set_curr_servo_pos(6, self._ppm_start_pos[0])
            self._set_curr_servo_pos(7, self._ppm_start_pos[1])
        continue_flag = True
        temp_speed_limit = 10
        while continue_flag:
            continue_flag = False
            for i, servo in enumerate(self.arm_servos):
                if self.current_servo_speed["servo_{0}".format(i+1)] != temp_speed_limit:
                    if servo.set_speed(temp_speed_limit):
                        self.current_servo_speed["servo_{0}".format(i+1)] = temp_speed_limit
                desire_pos = 0
                if servo.set_point(desire_pos):
                    self._set_curr_servo_pos(i+1, desire_pos)
                val = servo.get_data()
                if not val:
                    continue
                _check_erros(val)
                if not (-30 <= val['Position'] <= 30):
                    continue_flag = True
        

    def run(self):
        last_time = 0
        while True:
            if self.read_flag.is_set():
                self._read_servos()
                self.read_flag.clear()
            if self.read_touch_flag.is_set():
                self._read_touch()
                self.read_touch_flag.clear()
            if self.reset_flag.is_set():
                self._reset_servos()
                self.reset_flag.clear()
            if time.perf_counter() > last_time + (1/SERVO_HZ):
                self._write_to_servos()
                last_time = time.perf_counter()