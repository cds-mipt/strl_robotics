
# Описание

Этот модуль обеспечивает высокоуровневый контроллер для мобильного робота с дифференциальным приводом. Он содержит два метода управления роботом - отслеживание пути (Path Following) и отслеживание траектории (Trajectory Tracking). Модуль управления реализован в ROS. Пакеты управления принимают одометрию робота и путь планировщика в качестве входных данных и выдают закон управления линейной и угловой скоростями в качестве выходных данных.

## Системные требования

* Linux 18 и выше
* ROS Melodic 

## Конфигурация
Настройка параметров алгоритмов осуществляется в файлах config_path_following.yaml и config_trajectory_tracking.yaml. Эти параметры:
* max_v: максимальный модуль линейной скорости.
* min_v: минимальный модуль линейной скорость (пока всегда нуль).
* acc_v: ускорение продольного движения. Исползуется при необходимости увелечения линейиой скорости робота.
* max_w: максимальный модуль угловой скорости.

* control_method: указывает на выбранный метод решения задачи слежения за референсной траекторией (Trajectory tracking problem):

    (1)- первый нелинейный метод по Ляпунову. Исползуется по умолчению. Показывает найлучшие результаты.
    Из статьи: Kanayama, Y. Kimura, F. Miyazaki and T. Noguchi, "A stable tracking control method for an autonomous mobile robot," Proceedings., IEEE International  Conference on Robotics and Automation, Cincinnati, OH, USA, 1990, pp. 384-389 vol.1.
    
    (2)- второй нелинейный метод по Ляпунову.
    Из статьи: Felipe N. Martins, Wanderley C. Celeste, Ricardo Carelli, Mário Sarcinelli-Filho, Teodiano F. Bastos-Filho, "An adaptive dynamic controller for autonomous mobile robot trajectory tracking", Control Engineering Practice, Volume 16, Issue 11,2008,Pages 1354-1363.
    
    (3)- линейный метод. Из книги Gregor Klancar Andrej Zdesar Saso Blazic Igor Skrjanc, "Wheeled Mobile Robotics", Elsevier, 2017. Страница номер 101.
    
 * path_topic - топик, в котором публикуется путь от планировщика. 
 
 * cmd_topic - топик, в котором модуль управления (control) публикует команды управления.
 
## Запуск

Для тестировать движение с алгоритмом Path Following нужно написать ROS команду
* roslaunch path_following path_following.launch 

Для тестировать движение с алгоритмом Trajectory Tracking нужно написать ROS команду
* roslaunch trajectory_tracking trajectory_tracking.launch 
