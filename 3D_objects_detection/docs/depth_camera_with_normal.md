Инструкция как поднять узел из двух сетей соло, триангулирующих позицию объекта.

Вход: SensorImage
Выход: Pose

image_l (SensorImage)    
solo_1 (ObjectsArray)      
zed_camera.py(Pose в топике /zed_node/button_Pose_from_Dsensor) 


Шаги для запуска **(организованы в виде скриптов в bash_scripts, запускаются из Repos/solov2_pytorch1.7 ./bash_scripts/i_terminal)**: 
- (терминал 1) 
`cd Repos/solov2_pytorch1.7`  
`./docker/latest/start_devel.sh`  
`./docker/latest/into.sh`  
[подключаться к докеру можно /docker/latest/into.sh сколько угодно раз, к каждому заново надо делать source]
- (терминал 2) 
`cd Repos/solov2_utils`  
`docker cp camera_objects_msgs solo:/home/docker_solo/catkin_ws/src/`  
`docker cp camera_objects_visualizer solo:/home/docker_solo/catkin_ws/src/ (для визуализации)`  
- (терминал 1) 
`source /opt/ros/melodic/setup.bash`  
`cd catkin_ws`  
`catkin_make`  
`source devel/setup.bash`  
`cd src/solov2/solo_ros/launch/`  
`roslaunch zed_depth_test.launch`    
- (терминал 2)
`cd ../solov2_pytorch1.7`  
`./docker/latest/into.sh`  
`cd catkin_ws`  
`source devel/setup.bash`  
`cd src/solov2/solo_ros/scripts/`  
`python zed_camera.py`  
- (терминал 3 для визуализации)
`cd Repos/solov2_pytorch1.7`  
`./docker/latest/into.sh`  
`cd catkin_ws`  
`source devel/setup.bash`  
`cd src/camera_objects_visualizer/launch`  
`roslaunch zed_depth_test_visual.launch` 
(теперь можно открыть rqtи посмотреть на топик /zed_node/left/objects_image
- (терминал 4 для визуализации)
`cd Repos/solov2_utils`  
`rosrun rviz rviz -d normal.rviz`  
или найдите в недавних
