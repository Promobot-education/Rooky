#include "Rooky.h"

#define DEBUG(f_, ...) if (_debug) {printf("DEBUG, %s. ", __func__); printf((f_), ##__VA_ARGS__);}
#define ERROR(f_, ...) {printf("ERROR, %s. ", __func__); printf((f_), ##__VA_ARGS__);}

enum PpmServo
{
    SERVO_1 = 1,
    SERVO_2
};

struct Limit
{
    int min;
    int max;
};

std::vector<int> _servos;

int _ppmServo = 0;
bool _debug = false;

static RookySide _rookySide = RookySide::NONE;

const std::vector<int> LeftArmServos = {31, 32, 33, 34, 35};
const std::vector<int> RightArmServos = {21, 22, 23, 24, 25};

std::map<std::string, int> _set_points = {
    {"joint_2", 0},
    {"joint_3", 0},
    {"joint_4", 0},
    {"joint_5", 0},
};

const std::map<std::string, float> JointsRatio = {
    {"joint_1", 3.0},
    {"joint_2", 1.9},
    {"joint_3", 1.9},
    {"joint_4", 1.2},
    {"joint_5", 1.2},
    {"joint_6", 1.7},
    {"joint_7", 0.3},
};

const std::map<std::string, Limit> JointsLimits = {
    {"joint_1", {-10, 90}},
    {"joint_2", {-10, 90}},
    {"joint_3", {-90, 90}},
    {"joint_4", {-10, 90}},
    {"joint_5", {-90, 90}},
    {"joint_6", {-20, 35}},
    {"joint_7", {0, 74}},
};

const int _rightArmServoPpm = 26;
const int _leftArmPpmServo = 36;

std::vector<int> _untouchedValues;

bool initRooky(std::string port, RookySide side, bool debug)
{
    std::vector<int> servosAddresses;
    switch (side)
    {
    case LEFT:
        servosAddresses = LeftArmServos;
        _ppmServo = _leftArmPpmServo;
        break;
    case RIGHT:
        servosAddresses = RightArmServos;
        _ppmServo = _rightArmServoPpm;
        break;
    default:
        _ppmServo = 0;
        ERROR("Initialization failed. Incorrect Rooky side\n");
        return false;
    }

    _rookySide = side;

    if (!servosAddresses.size())
        return false;

    if (init_port(port.c_str(), debug) < 0)
        return false;

    for (auto addr : servosAddresses)
    {
        initServo(addr);
        //Стандартный ре-им работы сервопривода
        set_PID_mode(addr, NORMAL);

        //Включение питания обмоток двигателя
        set_torque(addr, true);
        _servos.push_back(addr);
    }
    initServoPpm(_ppmServo);

    if (!read_touch(_ppmServo, _untouchedValues))
        return false;
    _debug = debug;
    return true;
}

int _degreesToTick(float deg)
{
    return static_cast<int>(deg / 0.0219);
}

bool _checkLimits(const std::string &name, float position)
{
    auto it = JointsLimits.find(name);

    if (it == JointsLimits.end())
        return false;

    Limit limits = it->second;

    if (position > limits.max || position < limits.min)
    {
        ERROR("Invalid setpoint: %f\n", position);
        return false;
    }
    return true;
}

bool _moveSimpleJoint(const std::string &name, float speed, float position)
{
    if (!_checkLimits(name, position))
        return false;

    auto it = JointsRatio.find(name);

    if (it == JointsRatio.end())
        return false;

    float ratio = it->second;

    int setpoint = _degreesToTick(position * ratio);
    set_torque(_servos[0], true);
    set_speed_limit(_servos[0], speed);
    set_point(_servos[0], static_cast<int16_t>(setpoint));
    return true;
}

bool _moveDiffJoint(const std::string &name, float speed)
{
    if (name == "joint_2" || name == "joint_3")
    {
        set_speed_limit(_servos[1], speed);
        set_speed_limit(_servos[2], speed);
        set_torque(_servos[1], true);
        set_torque(_servos[2], true);
        set_point(_servos[1], _set_points["joint_2"] + _set_points["joint_3"]);
        set_point(_servos[2], _set_points["joint_2"] - _set_points["joint_3"]);
        return true;
    }

    if (name == "joint_4" || name == "joint_5")
    {
        set_speed_limit(_servos[3], speed);
        set_speed_limit(_servos[4], speed);
        set_torque(_servos[3], true);
        set_torque(_servos[4], true);
        set_point(_servos[3], _set_points["joint_4"] + _set_points["joint_5"]);
        set_point(_servos[4], _set_points["joint_4"] - _set_points["joint_5"]);
        return true;
    }
    DEBUG("exit\n");
    return false;
}

bool _moveServoJiont(const std::string &name, PpmServo servoNum, float position)
{
    DEBUG("Name - %s, position - %f\n", name.c_str(), position);
    if (!_checkLimits(name, position))
        return false;

    auto it = JointsRatio.find(name);

    if (it == JointsRatio.end())
        return false;

    float ratio = it->second;

    int angle;
    switch (servoNum)
    {
    case PpmServo::SERVO_1:
        angle = (position * ratio) * -1;
        break;
    case PpmServo::SERVO_2:
        if (_rookySide == RookySide::LEFT)
            angle = -90 + (position * ratio);
        else if (_rookySide == RookySide::RIGHT)
            angle = -35 + (position * ratio);
        else
            return false;
        break;
    }
    DEBUG("Raw servo angle - %d\n", angle);
    set_ppm_angle(_ppmServo, servoNum, static_cast<int16_t>(angle));
    return true;
}

bool _setDiffJoint(const std::string &name, float speed, float position)
{
    if (!_checkLimits(name, position))
        return false;

    auto it = JointsRatio.find(name);

    if (it == JointsRatio.end())
        return false;

    float ratio = it->second;

    _set_points[name] = static_cast<int>(_degreesToTick(position) * ratio);
    _moveDiffJoint(name, speed);
    return true;
}

void _checkErrors(const ServoData &data)
{
    for (auto err : data.errors)
    {
        if (err != nullptr)
            ERROR("Servo: %d: in troubles: %s \n", data.ID, err);
    }
}

void moveJoint(const std::string &name, float speed, float position)
{
    if (name == "joint_1")
        _moveSimpleJoint(name, speed, position);
    else if (name == "joint_2")
        _setDiffJoint(name, speed, position);
    else if (name == "joint_3")
        _setDiffJoint(name, speed, position);
    else if (name == "joint_4")
        _setDiffJoint(name, speed, position);
    else if (name == "joint_5")
        _setDiffJoint(name, speed, position);
    else if (name == "joint_6")
        _moveServoJiont(name, PpmServo::SERVO_1, position);
    else if (name == "joint_7")
        _moveServoJiont(name, PpmServo::SERVO_2, position);
    else
        ERROR("Wrong joint name: %s\n", name.c_str());
}

void relax(bool flag)
{
    DEBUG("Relax - %s\n", flag ? "true" : "false");
    if (flag)
    {
        for (auto addr : _servos)
            set_command(addr, 0xAAAA);
        set_torque(_ppmServo, false);
    }
    else
    {
        for (auto addr : _servos)
            set_command(addr, 0x0);
        initServoPpm(_ppmServo);
    }
}

bool readServos(std::vector <ServoData> &servo_datas)
{
    for (auto addr : _servos)
    {
        ServoData data = {};
        if (!get_servo_data(addr, data))
            return false;
        _checkErrors(data);
        servo_datas.push_back(data);
    }
    return true;
}

bool isTouched()
{
    std::vector<int>vals;
    if (!read_touch(_ppmServo, vals))
        return false;
    
    DEBUG("vals %d, %d\n", vals[0], vals[1]);
    DEBUG("untouched %d, %d\n", _untouchedValues[0], _untouchedValues[1]);
    if (!vals.size() || !_untouchedValues.size())
        return false;

    if (vals[0] < _untouchedValues[0] * 0.95 || vals[1] < _untouchedValues[1] * 0.95)
        return true;
    return false;
}
