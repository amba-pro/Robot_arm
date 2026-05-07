# Финальный прогон перед сдачей (команды по порядку)

## Стендовый чек-лист (10-15 минут)

### A) Быстрый прогон без железа (mock, рекомендовано)

1. Терминал 1:
```bash
cd ~/Robot\ ARM4
ARM4_MOCK=1 python3 angle_reader.py
```
2. Терминал 2:
```bash
cd ~/Robot\ ARM4
ARM4_MOCK=1 python3 robot_tcp_server.py
```
3. Терминал 3:
```bash
cd ~/Robot\ ARM4/ros_ws
colcon build
source install/setup.bash
ros2 launch arm4_bringup arm4_bringup.launch.py rviz:=true rqt:=true
```
4. Терминал 4 (проверка топиков):
```bash
cd ~/Robot\ ARM4/ros_ws
source install/setup.bash
ros2 topic echo /arm4/angles
```
5. Терминал 5 (проверка топиков):
```bash
cd ~/Robot\ ARM4/ros_ws
source install/setup.bash
ros2 topic echo /joint_states
```
6. В `rqt_plot` добавить `/arm4/angles/data[0..4]`, убедиться, что линии "живые".

### B) Если есть железо (реальный стенд)

Запусти те же шаги, но без `ARM4_MOCK=1`:
- `python3 angle_reader.py`
- `python3 robot_tcp_server.py`

### C) Docker (если успеваешь, +2-3 минуты)

```bash
cd ~/Robot\ ARM4
docker build -t arm4:latest .
docker run -it --rm --network host --privileged -v /dev:/dev -v $(pwd):/workspace arm4:latest
```

### D) Что заскринить сразу по ходу

1. `angle_reader.py` (mock или real);
2. `robot_tcp_server.py` (mock или real);
3. `ros2 launch ... rviz:=true rqt:=true`;
4. `rqt_plot` с `/arm4/angles/data[0..4]`;
5. `ros2 topic echo /arm4/angles` и `/joint_states`;
6. `docker build`/`docker run` (если делал Docker).

## 1) Подготовка окружения (Linux)

```bash
cd ~/Robot\ ARM4
chmod +x scripts/install_dependencies.sh scripts/setup_ssh.sh
./scripts/install_dependencies.sh
sudo ./scripts/setup_ssh.sh
```

## 2) Запуск основной системы

Терминал 1:
```bash
cd ~/Robot\ ARM4
python3 angle_reader.py
```

Терминал 2:
```bash
cd ~/Robot\ ARM4
python3 robot_tcp_server.py
```

Терминал 3 (клиент, опционально):
```bash
cd ~/Robot\ ARM4
python3 mock-client-multi.py
```

### Вариант без реального робота (заглушка)

Если стенд/железо недоступны, включите mock-режим:

Терминал 1:
```bash
cd ~/Robot\ ARM4
ARM4_MOCK=1 python3 angle_reader.py
```

Терминал 2:
```bash
cd ~/Robot\ ARM4
ARM4_MOCK=1 python3 robot_tcp_server.py
```

В этом режиме `angle_reader.py` генерирует тестовые A0-A4, а `robot_tcp_server.py` стартует без serial-портов.

## 3) ROS 2

Терминал 4:
```bash
cd ~/Robot\ ARM4/ros_ws
colcon build
source install/setup.bash
ros2 launch arm4_bringup arm4_bringup.launch.py rviz:=true rqt:=true
```

Проверка топиков (новый терминал):
```bash
cd ~/Robot\ ARM4/ros_ws
source install/setup.bash
ros2 topic echo /arm4/angles
ros2 topic echo /joint_states
```

## 4) Визуализация датчика

В `rqt_plot` добавить:
- `/arm4/angles/data[0]`
- `/arm4/angles/data[1]`
- `/arm4/angles/data[2]`
- `/arm4/angles/data[3]`
- `/arm4/angles/data[4]`

## 5) Docker (опционально, но желательно для оценки)

```bash
cd ~/Robot\ ARM4
docker build -t arm4:latest .
docker run -it --rm --network host --privileged -v /dev:/dev -v $(pwd):/workspace arm4:latest
```

## 6) Скриншоты для отчета

Сделать минимум 5 скриншотов:
1. `angle_reader.py` в работе;
2. `robot_tcp_server.py` в работе;
3. запуск ROS launch;
4. `rqt_plot` с живыми данными A0-A4;
5. `docker build`/`docker run` (если делал Docker-часть).
