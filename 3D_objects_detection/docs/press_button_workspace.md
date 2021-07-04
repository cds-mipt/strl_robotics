Для запуска манипулятора надо запустить файл press_rtde.py  
Он находится в Repos/manipulator/catkin_ws/src/manipulator/src/  
Предварительно надо выполнить команду `source devel/setup.bash` в Repos/manipulator/catkin_ws, чтобы скрипт узнал об кастомном сервисе, который используется в проекте.
Проверяйте, что координаты в переменной button_loсation_l в коде press_rtde.py не вписаны вручную после вызова сервиса SOLO.
