#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <string>
#include <vector>

#define JOINTS_COUNT 7

enum RookySide
{
	LEFT = 0,
	RIGHT
};

int main(int argc, char **argv)
{
	// Определим тип Rooky
	RookySide rooky_side = LEFT;

	// Инициализируем ROS с указанием названия узла
	ros::init(argc, argv, "move_joints");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	// В зависимости от типа Rooky определим наименование топика
	// в который необходимо отправлять сообщение
	std::string controller_name;
	if (rooky_side == RIGHT)
		controller_name = "right_arm_controller/command";
	else
		controller_name = "left_arm_controller/command";

	// Определим publisher с очередью в 1 сообщение
	ros::Publisher msg = nh.advertise<trajectory_msgs::JointTrajectory>(controller_name.c_str(), 1);

	// Создадим сообщение которое будем отправлять в топик
	// Данное сообщение будет содержать положение сразу всех суставов
	trajectory_msgs::JointTrajectory joints_state;
	joints_state.joint_names.resize(JOINTS_COUNT);
	for (int i = 1; i < JOINTS_COUNT + 1; i++)
	{
		// Построим имя сустава, 
		// например строка с именем должна выглядеть так:
		// "left_arm_1_joint"
		std::string joint_name;
		if (rooky_side == RIGHT)
			joint_name += "right";
		else
			joint_name += "left";

		joint_name += "_arm_";

		// Переведем int в ASCII цифру
		joint_name += boost::lexical_cast<std::string>(i); 
		joint_name += "_joint";

		// Добавим имя сустава в сообщение
		joints_state.joint_names[i - 1] = joint_name;
	}

	// Создадим описание положений - Точки
	std::vector<trajectory_msgs::JointTrajectoryPoint> points(1);
	while (ros::ok())
	{
		// Не будем задавать скорость движения
		points[0].velocities.resize(0);

		// Задаим время, которое должна двигаться Rooky до указанного положения
		points[0].time_from_start.sec = 1;

		points[0].positions.resize(JOINTS_COUNT);

		// Последовательно запишем позиции для каждого узла
		for (int i = 0; i < JOINTS_COUNT; i++)
		{
			// Вначале все суставы двигаем на 1 радиан
			// Потом все суставы возвращаем в 0
			if (points[0].positions[i] != 0)
				points[0].positions[i] = 0;
			else
				points[0].positions[i] = 1;
		}

		joints_state.points = points;

		// Опубликуем сообщение в топик
		msg.publish(joints_state);

		// Сделаем паузу на 5 секунд
		ros::Duration(5).sleep();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
