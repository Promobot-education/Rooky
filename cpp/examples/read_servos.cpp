#include "Rooky.h"
#include <vector>
#ifdef __unix__
#include <unistd.h>
#define _msleep(x) usleep(x * 1000)
#else
#include <windows.h>
#define _msleep(x) Sleep(x)
#endif

// Создадим вектор адресов сервоприводов для левой руки
const std::vector<int> LEFT_ARM_SERVOS = { 31, 32, 33, 34, 35, 36 };

// Создадим вектор адресов сервоприводов для правой руки
const std::vector<int> RIGHT_ARM_SERVOS = { 21, 22, 23, 24, 25, 26 };

int main()
{

	RookySide side = RookySide::LEFT;

	// Инициализация Rooky.
	// По умолчанию для ubuntu - /dev/RS_485
	if (!initRooky("/dev/RS_485", side, false))
		return -1;

	// В зависимости от типа Rooky выберем соответствующие адреса сервоприводов
	std::vector<int> servos;
	if (side == RookySide::RIGHT)
		servos = RIGHT_ARM_SERVOS;
	else
		servos = LEFT_ARM_SERVOS;

	// Создадим вектор структур, куда будем записывать данные с сервоприводов 1 типа
	std::vector<ServoData> datas;

	// Создадим структуру в которую будем записывать данны с сервоприводов 2 типа
	PpmServoData ppm_data;
	while (true)
	{
		// Чтобы данные считались корректно, включим  сервоприводы
		for (int srv_adr : servos)
			set_torque(srv_adr, true);

		// Считаем данные с сервоприводов
		if (!readServos(datas))
			return -1;

		// Считаем данные с сервоприводов
		if (!readPpmServo(ppm_data))
			return -1;

// @formatter:off
        printf("========================\n");
        printf("id             - %04d  %04d  %04d  %04d  %04d\n", datas[0].ID, datas[1].ID, datas[2].ID, datas[3].ID, datas[4].ID);
        printf("torque         - %04d  %04d  %04d  %04d  %04d\n", datas[0].torque, datas[1].torque, datas[2].torque, datas[3].torque, datas[4].torque);
        printf("command        - %02x  %02x  %02x  %02x  %02x\n", datas[0].command, datas[1].command,datas[2].command,datas[3].command, datas[4].command);
        printf("current        - %04d  %04d  %04d  %04d  %04d\n", datas[0].current, datas[1].current, datas[2].current, datas[3].current, datas[4].current);
        printf("setpoint       - %04d  %04d  %04d  %04d  %04d\n", datas[0].setpoint, datas[1].setpoint,datas[2].setpoint,datas[3].setpoint,datas[4].setpoint);
        printf("position       - %04d  %04d  %04d  %04d  %04d\n", datas[0].position, datas[1].position, datas[2].position, datas[3].position, datas[4].position);
        printf("pos_PID_P      - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_P, datas[1].pos_PID_P, datas[2].pos_PID_P, datas[3].pos_PID_P, datas[4].pos_PID_P);
        printf("pos_PID_I      - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_I, datas[1].pos_PID_I, datas[2].pos_PID_I, datas[3].pos_PID_I, datas[4].pos_PID_I);
        printf("pos_PID_D      - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_D, datas[1].pos_PID_D, datas[2].pos_PID_D, datas[3].pos_PID_D, datas[4].pos_PID_D);
        printf("speed_PID_P    - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_P, datas[1].speed_PID_P, datas[2].speed_PID_P, datas[3].speed_PID_P, datas[4].speed_PID_P);
        printf("speed_PID_I    - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_I, datas[1].speed_PID_I, datas[2].speed_PID_I, datas[3].speed_PID_I, datas[4].speed_PID_I);
        printf("speed_PID_D    - %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_D, datas[1].speed_PID_D, datas[2].speed_PID_D, datas[3].speed_PID_D, datas[4].speed_PID_D);
        printf("speed          - %04d  %04d  %04d  %04d  %04d\n", datas[0].speed, datas[1].speed,datas[2].speed,datas[3].speed,datas[4].speed);
        printf("++++++++++++++++++++++++\n");
		printf("id             - %d\n", ppm_data.ID);
		printf("torque         - %d\n", ppm_data.torque);
		printf("command        - %02x\n", ppm_data.command);
		printf("current        - %d\n", ppm_data.current);
		printf("voltage        - %d\n", ppm_data.voltage);
		printf("servo 1 angle  - %d\n", ppm_data.angle_1);
		printf("servo 2 angle  - %d\n", ppm_data.angle_2);
		printf("touch sensor 1 - %d\n", ppm_data.touch_1);
		printf("touch sensor 2 - %d\n", ppm_data.touch_2);
        printf("isTouched 	   - %s\n", isTouched() ? "true" : "false");
// @formatter:on

		datas.clear();

		// Установим задержку в 100мс
		_msleep(100);
	}
	return 0;
}
