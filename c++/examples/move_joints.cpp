#include "Rooky.h"
#include <unistd.h>

int main()
{
    //Инициализация последовательного порта для работы с датчиком. По умолчанию /dev/RS_485
    if (!initRooky("/dev/RS_485", RookySide::LEFT, false))
    {
        return -1;
    }
    relax(false);

    struct Action
    {
        std::string name;
        float speed;
        float position;
    };

    std::vector<Action> actionSequence = {
        {"joint_1", 20, 20},
        {"joint_1", 20, 0},
        {"joint_2", 20, 20},
        {"joint_2", 20, 0},
        {"joint_3", 20, 20},
        {"joint_3", 20, 0},
        {"joint_4", 20, 20},
        {"joint_4", 20, 0},
        {"joint_5", 20, 20},
        {"joint_5", 20, 0},
        {"joint_6", 20, 30},
        {"joint_6", 20, 0},
        {"joint_7", 20, 70},
        {"joint_7", 20, 0},
        {"joint_7", 20, 70},
        {"joint_7", 20, 0},
    };
	
    for (auto action : actionSequence)
    {
        moveJoint(action.name, action.speed, action.position);
        sleep(1);
    }
    relax(true);
    return 0;
}
