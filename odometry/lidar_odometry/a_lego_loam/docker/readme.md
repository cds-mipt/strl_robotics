Создание образа из докер файла:
```
./build
```

Запуск конейнера:
```
./start.sh <путь к папке src, в которой лежит репозиторий>
```

Сборка компонента для отладки:
```
cd catkin_ws 
catkin_make -j 3
source devel/setup.bash
```

Сборка компонента для релиза:
```
cd catkin_ws 
catkin_make install -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
rm -R build devel
```
