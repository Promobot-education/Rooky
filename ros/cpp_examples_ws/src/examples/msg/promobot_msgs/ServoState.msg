int32 id 		# ID сервы
string name             # Имя сервы
uint16 command		# Регистр команд
bool torque        	# Разрешение работы двигателя
int32 setPoint      	# Установка задачи
uint16 fault_code	# Регистр ошибок
string fault_code_str	# Регистр ошибок строка
int32 encoderValue	# Значение енкодера
int32 position		# Положение оси
int32 velocity		# Скорость
int32 motorCurrent	# Ток на двигателе
HallSensors sensors	# Датчики крайних положений
