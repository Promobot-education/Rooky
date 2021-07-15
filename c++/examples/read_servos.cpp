#include "Rooky.h"
#include <unistd.h>

int main()
{
    //Инициализация последовательного порта для работы с датчиком. По умолчанию /dev/RS_485
    if (!initRooky("/dev/RS_485", RookySide::LEFT, false))
    {
        return -1;
    }
    std::vector<ServoData> datas;
    while (true)
    {
        for(int i = 31; i < 36; i++)
            set_torque(i, true);

        if (!readServos(datas))
            return -1;

        printf("========================\n");
        printf("id -            %04d  %04d  %04d  %04d  %04d\n", datas[0].ID, datas[1].ID, datas[2].ID, datas[3].ID, datas[4].ID);
        printf("torque -        %04d  %04d  %04d  %04d  %04d\n", datas[0].torque, datas[1].torque, datas[2].torque, datas[3].torque, datas[4].torque);
        printf("command -       %02x  %02x  %02x  %02x  %02x\n", datas[0].command, datas[1].command,datas[2].command,datas[3].command, datas[4].command);
        printf("current -       %04d  %04d  %04d  %04d  %04d\n", datas[0].current, datas[1].current, datas[2].current, datas[3].current, datas[4].current);
        printf("setpoint -      %04d  %04d  %04d  %04d  %04d\n", datas[0].setpoint, datas[1].setpoint,datas[2].setpoint,datas[3].setpoint,datas[4].setpoint);
        printf("position -      %04d  %04d  %04d  %04d  %04d\n", datas[0].position, datas[1].position, datas[2].position, datas[3].position, datas[4].position);
        printf("pos_PID_P -     %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_P, datas[1].pos_PID_P, datas[2].pos_PID_P, datas[3].pos_PID_P, datas[4].pos_PID_P);
        printf("pos_PID_I -     %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_I, datas[1].pos_PID_I, datas[2].pos_PID_I, datas[3].pos_PID_I, datas[4].pos_PID_I);
        printf("pos_PID_D -     %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].pos_PID_D, datas[1].pos_PID_D, datas[2].pos_PID_D, datas[3].pos_PID_D, datas[4].pos_PID_D);
        printf("speed_PID_P -   %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_P, datas[1].speed_PID_P, datas[2].speed_PID_P, datas[3].speed_PID_P, datas[4].speed_PID_P);
        printf("speed_PID_I -   %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_I, datas[1].speed_PID_I, datas[2].speed_PID_I, datas[3].speed_PID_I, datas[4].speed_PID_I);
        printf("speed_PID_D -   %.2f  %.2f  %.2f  %.2f  %.2f\n", datas[0].speed_PID_D, datas[1].speed_PID_D, datas[2].speed_PID_D, datas[3].speed_PID_D, datas[4].speed_PID_D);
        printf("speed -         %04d  %04d  %04d  %04d  %04d\n", datas[0].speed, datas[1].speed,datas[2].speed,datas[3].speed,datas[4].speed);
        printf("isTouched - %s\n", isTouched() ? "true" : "false");

        datas.clear();
        usleep(100000);
    }
    return 0;
}