#include "Rooky.h"
#include <unistd.h>

int main()
{
	// Инициализация Rooky.
	// По умолчанию для ubuntu - /dev/RS_485
	if (!initRooky("/dev/RS_485", RookySide::LEFT, false))
		return -1;

	// Выключим расслабление руки если оно происходило до этого
	relax(false);

	// Создадим структуру для описания движения сустава
	struct Action
	{
		std::string name; 	// имя сустава
		float speed; 	 	// предельная скорость сервоприводов
		float position;		// угол, на который следует повернуть сустав
	};

	// Создадим вектор из действий
	std::vector<Action> actionSequence = // @suppress("Invalid arguments")
	{
		{ "joint_1", 5, 30 },
		{ "joint_1", 20, 0 },
		{ "joint_2", 5, 30 },
		{ "joint_2", 10, 0 },
		{ "joint_3", 20, 40 },
		{ "joint_3", 20, 0 },
		{ "joint_4", 10, 40 },
		{ "joint_4", 20, 0 },
		{ "joint_5", 20, 40 },
		{ "joint_5", 20, 0 },
		{ "joint_6", 20, 30 },
		{ "joint_6", 20, 0 },
		{ "joint_7", 20, 70 },
		{ "joint_7", 20, 0 },
		{ "joint_7", 20, 70 },
		{ "joint_7", 20, 0 } };

	// Выполним каждое действие с паузой в 1с между ними
	for (auto action : actionSequence)
	{
		INFO("Move joint - %s, speed - %.2f, to position - %.2f\n", action.name.c_str(), action.speed, action.position);
		moveJoint(action.name, action.speed, action.position);
		sleep(1);
	}

	// Расслабим руку
	relax(true);
	return 0;
}
