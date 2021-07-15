#pragma once

#include <map>
#include <string>
#include <vector>
#include "Servo.h"

#define SERVOS_TYPE_1_COUNT 5

enum RookySide
{
    LEFT,
    RIGHT,
    NONE
};

bool initRooky(std::string port, RookySide side = RookySide::NONE, bool debug = false);

void moveJoint(const std::string &name, float speed = 10, float position = 0);

void relax(bool flag);

bool readServos(std::vector <ServoData> &servo_datas);

bool isTouched();