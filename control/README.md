
Узел управления принимает сообщению позиции от одометрии и путь от планировщика как вход и выдает линейная и угловая скорости. Узел управления проверит все пути от планировшика и параллельно отслеживает последний обновленный путь. Пропорциональный регулятор был использован и только точки x и y от планировщика. Также учитывали ограничения на скорости и ускорения.

запустить:
roslaunch husky_mobile02 control.launch 