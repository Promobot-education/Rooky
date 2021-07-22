#include "ros/ros.h"
#include "promobot_msgs/ServoSmallState.h"

/**
 * Функция которая будет вызвана, когда получим сообщение из топика
 */
void servoSmallCallBack(const promobot_msgs::ServoSmallState::ConstPtr &msg)
{
	// Определим статичную переменную,
	// будем считать что это значение с датчика касания, когда его никто не касается
	// с ней будем сравнивать значения с датчика в другой раз
	static int16_t calibrated_value = 0;
	static int i = 0;
	if (i == 0)
	{
		i++;

		// Запомним значение
		calibrated_value = msg->capSensorVal;
		ROS_INFO("First value as untouched: [%d]", calibrated_value);
	}

	bool isTouched = false;

	// При касании датчика, его значения уменьшаются
	// если изменение произошло сильнее чем на 5%, значит есть касание датчика
	if (msg->capSensorVal < calibrated_value * 0.95)
		isTouched = true;

	ROS_INFO("Current touch value: [%d]. Untouched value: [%d]", msg->capSensorVal, calibrated_value);
	ROS_INFO("Touch state: [%s]", isTouched ? "true" : "false");
	ros::Duration(1).sleep();
}

int main(int argc, char **argv)
{
	// Инициализируем ROS с указанием названия узла
	ros::init(argc, argv, "read_touch");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);

	// Подпишемся на топик
	ros::Subscriber sb = nh.subscribe("promobot_servos/small", 1, servoSmallCallBack);

	ros::spin();

	return 0;
}
