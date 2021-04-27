Инструкция как поднять узел из двух сетей соло, триангулирующих позицию объекта.

Вход: SensorImage
Выход: Pose

image_l, image_r (SensorImage)    
solo_1, solo_2 (ObjectsArray)      
xyz_with_filter.py(Pose) 

Шаги для запуска: 
- cd Repos/solov2_pytorch1.7 (терминал 1)
- ./docker/latest/start_devel.sh
- ./docker/latest/into.sh 
- cd Repos/solov2_utils (терминал 2)
- docker cp camera_objects_msgs solo:/home/docker_solo/catkin_ws/src/
- source /opt/ros/melodic/setup.bash (терминал 1)
- cd catkin_ws
- catkin_make 
- source devel/setup.bash;
- [подключаться к докеру можно /docker/latest/into.sh, к каждому заново надо делать source]
- cd src/solov2/solo_ros/launch/
- roslaunch test.launch
          



