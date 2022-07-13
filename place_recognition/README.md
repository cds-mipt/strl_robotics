# Модуль Place Recognition

Данный модуль решает задачу Place Recognition.

Модуль является ROS-пакетом из нескольких узлов для решения разных подзадач -- визуального, лидарного или многомодального распознавания места.

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
bash docker/start.sh
```

Переход к терминалу внутри контейнера:

```
bash docker/into.sh
```

### Запуск узла

todo

## Описание узлов

- Visual node: распознавание места по данным с камеры робота;
- LiDAR node: распознавание места по данным лидара;
- Multimodal node: распознавание места по комплексированным данным с разных сенсоров.

### Visual node

Планируется интегрировать следующие алгоритмы:
- NetVLAD -- [arxiv](https://arxiv.org/abs/1511.07247), [github.com/Relja/netvlad](https://github.com/Relja/netvlad)
- Patch-NetVLAD -- [arxiv](https://arxiv.org/abs/2103.01486v1), [github.com/QVPR/Patch-NetVLAD](https://github.com/QVPR/Patch-NetVLAD)
- CosPlace -- [arxiv](https://arxiv.org/abs/2204.02287), [github.com/gmberton/CosPlace](https://github.com/gmberton/CosPlace)
