Для запуска манипулятора надо запустить файл press_button.py  
Он находится в Repos/manipulator/catkin_ws/src/manipulator/src/  
Предварительно надо выполнить команду `source devel/setup.bash` в Repos/manipulator/catkin_ws, чтобы скрипт узнал об кастомном сервисе, который используется в проекте.
Проверяйте, что button_loacation_l не переопределен после вызова сервиса (что координаты в коде не вписаны вручную).
