# Модуль Place Recognition

Данный модуль решает задачу Place Recognition.

Модуль является ROS-пакетом из нескольких узлов для решения разных подзадач - визуального, лидарного или многомодального распознавания места.

## Установка и запуск

Работа узлов осуществляется в изолированном Docker-контейнере.

Конфигурация контейнера:
- Ubuntu 20.04
- CUDA 11.1.1
- ROS Noetic
- PyTorch 1.9.1

### Docker

Сборка образа осуществляется командой:

```
bash docker/build.sh
```

Запуск контейнера:

```
bash start.sh [DATABASE_DIR]
```

Где `DATABASE_DIR` - директория [базы данных](#база-данных).

Переход к терминалу внутри контейнера:

```
bash docker/into.sh
```

Содержимое директории `/home/docker_place_recognition` в контейнере:

```
.
|-- catkin_ws
|   `-- src
|       `-- place_recognition
|           |-- CMakeLists.txt
|           |-- config   
|           |-- launch   
|           |-- package.xml
|           |-- scripts
|           |-- src
|           `-- weights
`-- reference_db
    `-- . . .
```

### База данных

```
.
|-- database.csv    # хранит соответствие image_timestamp, lidar_timestamp и координат места
|-- database.index  # faiss-index дескрипторов изображений из директории images
|-- images
|   |-- 1657129500013542235.png
|   |-- . . .
|   `-- 1657129745128927814.png
`-- lidar
|   |-- 1657129500054874330.bin
|   |-- . . .
|   `-- 1657129745136591629.bin
```

Директории `images` и `lidar` задействуются только на этапе препроцессинга данных, и, опционально, для визуализации найденных совпадений.

Пример содержимого `database.csv`:

```
,image_ts,lidar_ts,t_x,t_y,t_z,q_x,q_y,q_z,q_w
0,1657129500013542235,1657129500054874330,-28.36226581060896,26.92416916605968,0.0,0.0,-0.0,-0.6605275365584498,-0.7508018203547631
1,1657129502210146681,1657129502173871888,-28.36226581060896,26.92416916605968,0.0,0.0,-0.0,-0.6605275365584498,-0.7508018203547631
2,1657129504215803631,1657129504193848415,-28.30277507809432,27.38685757711791,0.0,0.0,-0.0,-0.6615791675271582,-0.7498753263670385
```

#### Подготовка БД

todo

### Запуск узла

Перед запуском необходимо выполнить сборку пакета:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

И загрузить веса моделей:

```
cd ~/catkin_ws/src/place_recognition/scripts
bash load_weights.sh
```

#### Visual node

Запуск с помощью launch-файла [visual_pr.launch](./place_recognition/launch/visual_pr.launch):

```
roslaunch place_recognition visual_pr.launch
```

Аргументы:

- `config_path`: путь к yaml-конфигу алгоритма. Значение по умолчанию: `/home/docker_place_recognition/catkin_ws/src/place_recognition/config/netvlad_config.yaml`.
- `namespace`: пространство имен публикуемых узлом топиков (т.е. "приставка" в начале названия топика). Значение по умолчанию: `/visual_pr`.
- `image_topic`: имя топика входящий query-изображений. Значение по умолчанию: `/pr_query_image/compressed`.
- `pose_topic`: имя топика, в который будут публиковаться данные о найденной reference-позе. Значение по умолчанию: `/estimated_pose`. Полное имя топика будет `namespace` + `pose_topic`.

## Описание узлов

- Visual node: распознавание места по данным с камеры робота;
- LiDAR node: распознавание места по данным лидара;
- Multimodal node: распознавание места по комплексированным данным с разных сенсоров.

### Visual node

На данный момент интегрированы следующие алгоритмы:

- NetVLAD: [netvlad_config.yaml](./place_recognition/config/netvlad_config.yaml) ([arxiv](https://arxiv.org/abs/1511.07247), [github.com/Relja/netvlad](https://github.com/Relja/netvlad))

TODO:

- Patch-NetVLAD: [arxiv](https://arxiv.org/abs/2103.01486v1), [github.com/QVPR/Patch-NetVLAD](https://github.com/QVPR/Patch-NetVLAD)
- CosPlace: [arxiv](https://arxiv.org/abs/2204.02287), [github.com/gmberton/CosPlace](https://github.com/gmberton/CosPlace)

Узел подписывается на топики:
-  `/pr_query_image/compressed`: изображение-запрос, тип `sensor_msgs/CompressedImage`.

Узел публикует сообщения в топики:
- `/visual_pr/estimated_pose`: поза наиболее похожего изображения-места из БД, тип `geometry_msgs/PoseStamped`;
- `/visual_pr/ref_image_ts`: timestamp наиболее похожего изображения-места из БД (может быть полезно для визуализации и отладки), тип `std_msgs/String`.
