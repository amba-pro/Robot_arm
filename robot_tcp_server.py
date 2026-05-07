#!/usr/bin/env python3
"""
TCP сервер для управления роботом через GUI клиент (mock-client-multi.py)
Принимает команды от GUI и управляет моторами через MotorController из ym1.py
"""

import socket
import threading
import time
import sys
import os
import inspect
import glob
import serial
import serial.tools.list_ports
import platform
from typing import Optional, Dict, List


# === ЛОГИРОВАНИЕ В ФАЙЛ + КОНСОЛЬ ===
LOG_FILE = "robot_tcp_server.log"
MOCK_MODE = os.getenv("ARM4_MOCK", "0").lower() in ("1", "true", "yes", "on")


class TeeStream:
    """Поток, который дублирует вывод в несколько потоков (консоль + файл)"""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            try:
                s.write(data)
                s.flush()
            except Exception:
                # Не падаем, если один из потоков недоступен
                pass

    def flush(self):
        for s in self.streams:
            try:
                s.flush()
            except Exception:
                pass


class MockMotorController:
    """Заглушка контроллера для запуска без физического оборудования."""

    def __init__(self):
        self.mks_motors_enabled = {}

    def __getattr__(self, name):
        def _mock_method(*args, **kwargs):
            if name.startswith("read_"):
                return {}
            if name.startswith("is_"):
                return False
            if name.startswith("enable_") or name.startswith("disable_"):
                return True
            return None

        return _mock_method


try:
    # Открываем файл логов в режиме добавления
    _log_file = open(LOG_FILE, "a", encoding="utf-8")
    # Дублируем stdout и stderr в файл
    sys.stdout = TeeStream(sys.__stdout__, _log_file)
    sys.stderr = TeeStream(sys.__stderr__, _log_file)
    print(f"📝 Логирование включено, файл: {LOG_FILE}")
except Exception as _e:
    # Если не удалось открыть файл, продолжаем только с консолью
    print(f"⚠️ Не удалось инициализировать лог-файл {LOG_FILE}: {_e}")

# Добавляем текущую директорию в путь для импорта
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# Импортируем из ym1.py - пробуем разные способы
import importlib.util

# Ищем файл ym1.py в текущей директории
ym1_file = os.path.join(current_dir, "ym1.py")
if not os.path.exists(ym1_file):
    # Пробуем найти файл с похожим именем
    for filename in os.listdir(current_dir):
        if filename.startswith("ym") and filename.endswith(".py"):
            ym1_file = os.path.join(current_dir, filename)
            print(f"📁 Найден файл: {filename}")
            break

if not os.path.exists(ym1_file):
    raise FileNotFoundError(
        f"❌ Файл ym1.py не найден в директории {current_dir}\n"
        f"   Убедитесь, что файл ym1.py находится в той же директории, что и robot_tcp_server.py"
    )

# Загружаем модуль напрямую из файла
spec = importlib.util.spec_from_file_location("ym1", ym1_file)
if spec is None or spec.loader is None:
    raise ImportError(f"Не удалось загрузить модуль из {ym1_file}")

ym1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ym1)

# Импортируем нужные классы и константы
SerialManager = ym1.SerialManager
MotorController = ym1.MotorController
check_port_availability = ym1.check_port_availability
auto_detect_port = ym1.auto_detect_port
ARDUINO_USB_VENDOR_ID = ym1.ARDUINO_USB_VENDOR_ID
ARDUINO_USB_PRODUCT_ID = ym1.ARDUINO_USB_PRODUCT_ID
CH340_USB_VENDOR_ID = ym1.CH340_USB_VENDOR_ID
CH340_USB_PRODUCT_ID = ym1.CH340_USB_PRODUCT_ID
BAUDRATE = ym1.BAUDRATE
TIMEOUT = ym1.TIMEOUT
MOTOR_ANGLE_LIMITS = ym1.MOTOR_ANGLE_LIMITS
ENCODER_CHANNEL = ym1.ENCODER_CHANNEL
MOTOR_ADDR = ym1.MOTOR_ADDR
INVERT_DIRECTION = ym1.INVERT_DIRECTION
BASE_SPEEDS = ym1.BASE_SPEEDS
MIN_SPEED = ym1.MIN_SPEED
MAX_SPEED = ym1.MAX_SPEED

# === НАСТРОЙКИ СЕТИ ===
UDP_PORT = 50000
TCP_PORT = 50001
DISCOVERY_MSG = b"DISCOVER_ROBOHAND"
RESPONSE_PREFIX = "FOUND_ROBOHAND:"
TURNOFF_MSG = b"TURNOFF_ROBOHAND"

# === МАППИНГ: 8 байт от GUI → 6 моторов ===
# GUI отправляет 8 байт (индексы 0-7)
# Ползунки 0-5 соответствуют моторам 1-6 в том же порядке
GUI_TO_MOTOR_MAPPING = {
    0: 1,  # GUI слайдер 0 → Мотор 1 (Клешня - сервопривод MG996R)
    1: 2,  # GUI слайдер 1 → Мотор 2 (Поворот кисти - концевик A0)
    2: 3,  # GUI слайдер 2 → Мотор 3 (Плечо 2 - энкодер A1)
    3: 4,  # GUI слайдер 3 → Мотор 4 (Плечо 1 - энкодер A2)
    4: 5,  # GUI слайдер 4 → Мотор 5 (Плечо 0, Nema 34 - энкодер A3)
    5: 6,  # GUI слайдер 5 → Мотор 6 (Поворот основания - энкодер A4)
    # 6, 7 - не используются
}

# Режим работы: 'direct' - прямое управление скоростью, 'position' - позиционирование
# Для высокой отзывчивости оставляем прямой режим (position можно включить позже)
CONTROL_MODE = 'direct'

# Допуски и параметры калибровки (можно настраивать по моторам)
CALIBRATION_TOLERANCE = {
    3: 2.0,   # плечо 2
    4: 2.0,   # плечо 1
    5: 4.0,   # мотор 5 (плечо 0, Nema 34) трудно попадает точно в 90°, даем больший допуск
    6: 2.5    # поворот основания
}
CALIBRATION_MAX_ATTEMPTS = 150

# === ПАРАМЕТРЫ КАЛИБРОВКИ МОТОРА 2 (загружаются из файла) ===
MOTOR2_CALIBRATION_FILE = "motor2_calibration.txt"
MOTOR2_STARTING_ANGLE = 90.0  # Стартовая позиция (всегда 90°) - мотор устанавливается вручную
MOTOR2_FULL_ROTATION_ANGLE = 180.0  # Полный угол поворота (градусы)
MOTOR2_FULL_ROTATION_TIME = 4.28  # Время полного оборота (сек) - из калибровки
MOTOR2_ROTATION_SPEED = 42.07  # Скорость вращения (градусы/сек) - из калибровки


def load_motor2_calibration() -> Dict[str, float]:
    """Загружает параметры калибровки мотора 2 из файла"""
    calibration = {
        'starting_angle': MOTOR2_STARTING_ANGLE,
        'full_rotation_angle': MOTOR2_FULL_ROTATION_ANGLE,
        'full_rotation_time': MOTOR2_FULL_ROTATION_TIME,
        'rotation_speed': MOTOR2_ROTATION_SPEED
    }
    
    calib_file = os.path.join(current_dir, MOTOR2_CALIBRATION_FILE)
    if os.path.exists(calib_file):
        try:
            with open(calib_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('#') or not line:
                        continue
                    
                    # Парсим строки вида "PARAMETER = value"
                    if '=' in line:
                        key, value = line.split('=', 1)
                        key = key.strip()
                        value = value.strip()
                        
                        # Убираем комментарии из значения
                        if '#' in value:
                            value = value.split('#')[0].strip()
                        
                        # Преобразуем в число
                        try:
                            num_value = float(value)
                            if 'STARTING_ANGLE' in key:
                                calibration['starting_angle'] = num_value
                            elif 'FULL_ROTATION_ANGLE' in key:
                                calibration['full_rotation_angle'] = num_value
                            elif 'FULL_ROTATION_TIME' in key:
                                calibration['full_rotation_time'] = num_value
                            elif 'ROTATION_SPEED' in key:
                                calibration['rotation_speed'] = num_value
                        except ValueError:
                            pass
            
            print(f"✅ Загружены параметры калибровки мотора 2:")
            print(f"   Стартовая позиция: {calibration['starting_angle']:.1f}°")
            print(f"   Полный угол поворота: {calibration['full_rotation_angle']:.1f}°")
            print(f"   Время полного оборота: {calibration['full_rotation_time']:.2f} сек")
            print(f"   Скорость вращения: {calibration['rotation_speed']:.2f} °/сек")
        except Exception as e:
            print(f"⚠️ Ошибка загрузки параметров калибровки мотора 2: {e}")
            print(f"   Используются значения по умолчанию")
    else:
        print(f"⚠️ Файл калибровки {MOTOR2_CALIBRATION_FILE} не найден")
        print(f"   Используются значения по умолчанию")
    
    return calibration


# Загружаем параметры калибровки мотора 2 при импорте
MOTOR2_CALIB = load_motor2_calibration()


class RobotTCPServer:
    """TCP сервер для управления роботом через GUI"""
    
    def __init__(self, motor_controller: MotorController):
        self.controller = motor_controller
        self.udp_socket: Optional[socket.socket] = None
        self.tcp_server: Optional[socket.socket] = None
        self.tcp_client: Optional[socket.socket] = None
        self.running = False
        self.target_angles: Dict[int, int] = {}  # Целевые углы для каждого мотора
        self.last_angle_update: Dict[int, float] = {}  # Время последнего обновления угла
        self.angle_update_timeout = 0.1  # Если угол не обновлялся 0.1 сек - останавливаем мотор
        self.last_angle_read_time = 0  # Время последнего чтения углов
        self.cached_angles: Optional[Dict[str, float]] = None  # Кэш углов
        self.angle_read_interval = 0.005  # Читаем углы каждые 5 мс для максимально быстрой реакции
        self.last_direction: Dict[int, bool] = {}  # Последнее направление для каждого мотора (для предотвращения колебаний)
        self.direction_change_time: Dict[int, float] = {}  # Время последнего изменения направления
        self.is_calibrating = False  # Флаг калибровки
        self.motor_targets: Dict[int, float] = {}
        self.motor_threads: Dict[int, threading.Thread] = {}
        self.motor_stop_events: Dict[int, threading.Event] = {}
        self.motor_thread_lock = threading.Lock()
        
        # Параметры калибровки мотора 2 (программное отслеживание без концевика)
        self.motor2_calib = MOTOR2_CALIB.copy()
        self.motor2_starting_angle = self.motor2_calib.get('starting_angle', 90.0)  # Стартовая позиция (90°)
        self.motor2_current_angle = self.motor2_starting_angle  # Текущий угол мотора 2 (0-180°)
        self.motor2_rotation_start_angle = None  # Угол в момент начала текущего вращения
        self.motor2_last_rotation_start = None  # Время начала последнего вращения
        self.motor2_last_rotation_direction = None  # Направление последнего вращения
        self.motor2_is_rotating = False  # Флаг вращения мотора 2
        self.motor2_last_update_time = time.time()  # Время последнего обновления угла мотора 2
        self.motor2_initialized = False  # Флаг инициализации мотора 2
        
        # Поток для чтения углов (отдельный от основного цикла)
        self.angle_reader_thread: Optional[threading.Thread] = None
        self.angle_reader_running = False
        self.angle_read_lock = threading.Lock()
        
        try:
            sig = inspect.signature(MotorController.move_to_angle)
            self.supports_stop_event = 'stop_event' in sig.parameters
        except (ValueError, TypeError):
            self.supports_stop_event = False
        if not self.supports_stop_event:
            print("ℹ️ MotorController.move_to_angle() без stop_event — позиционный режим работает без прерывания.")
        
    def get_local_ip(self) -> str:
        """Получает локальный IP адрес"""
        try:
            # Подключаемся к внешнему адресу, чтобы узнать свой IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"
    
    def start_udp_discovery(self):
        """Запускает UDP сервер для обнаружения"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socket.bind(("", UDP_PORT))
        self.udp_socket.settimeout(0.1)
        
        print(f"✅ UDP сервер запущен на порту {UDP_PORT}")
        
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                if data == DISCOVERY_MSG:
                    local_ip = self.get_local_ip()
                    response = f"{RESPONSE_PREFIX}{local_ip}"
                    self.udp_socket.sendto(response.encode(), addr)
                    print(f"📡 Отправлен ответ на обнаружение: {response} → {addr[0]}")
                elif data == TURNOFF_MSG:
                    print("🔌 Получена команда выключения")
                    # Останавливаем все моторы
                    self.controller.stop_all()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"⚠️ Ошибка UDP: {e}")
    
    def apply_angles_direct(self, angles: Dict[int, int]):
        """
        Прямое управление моторами на основе целевых углов
        Вычисляет направление и скорость для каждого мотора
        """
        current_time = time.time()
        
        # Отладочный вывод (только первые несколько раз или при изменении углов)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        
        # Увеличиваем счетчик только если углы не изменились
        # Если углы изменились, счетчик будет сброшен в handle_tcp_client
        
        # КРИТИЧНО: Не читаем углы здесь - это блокирует обработку команд!
        # Углы будут читаться только когда действительно нужны (для моторов 3-6)
        # Команды отправляются СРАЗУ без ожидания чтения углов
        
        # Убрать отладочный вывод для максимальной скорости
        
        for gui_idx, motor_num in GUI_TO_MOTOR_MAPPING.items():
            if gui_idx not in angles:
                if self._debug_counter < 3:
                    print(f"⚠️ GUI индекс {gui_idx} не найден в углах")
                continue
            
            target_angle = angles[gui_idx]
            
            # Обработка мотора 1 (клешня) - сервопривод MG996R
            if motor_num == 1:
                # Управление сервоприводом MG996R (мгновенно, без задержек)
                angle = int(target_angle)
                if angle < 0:
                    angle = 0
                elif angle > 180:
                    angle = 180
                
                # Отправляем команду только если угол изменился
                try:
                    # Сохраняем последний отправленный угол
                    if not hasattr(self, '_last_gripper_angle'):
                        self._last_gripper_angle = None
                    if not hasattr(self, '_last_gripper_time'):
                        self._last_gripper_time = 0
                    
                    current_time = time.time()
                    # Отправляем команду только если:
                    # 1. Угол изменился
                    # 2. Прошло достаточно времени с последней отправки (не чаще 10 раз в секунду)
                    if self._last_gripper_angle != angle and (current_time - self._last_gripper_time) >= 0.1:
                        # Используем метод контроллера для отправки команды
                        self.controller.set_gripper_angle(angle)
                        self._last_gripper_angle = angle
                        self._last_gripper_time = current_time
                        if not hasattr(self, '_gripper_msg_count'):
                            self._gripper_msg_count = 0
                        if self._gripper_msg_count < 3:
                            print(f"🦾 Мотор 1 (клешня): {angle}°")
                            self._gripper_msg_count += 1
                except Exception as e:
                    if not hasattr(self, '_gripper_error_count'):
                        self._gripper_error_count = 0
                    if self._gripper_error_count < 3:
                        print(f"❌ Ошибка управления мотором 1: {e}")
                        self._gripper_error_count += 1
                continue
            
            # КРИТИЧНО: Всегда сохраняем целевой угол при каждом обновлении
            # Это гарантирует реакцию на изменения ползунков
            old_target = self.target_angles.get(motor_num)
            self.target_angles[motor_num] = target_angle
            self.last_angle_update[motor_num] = current_time
            
            # Если целевой угол изменился - сбрасываем счетчик остановки
            if old_target is not None and abs(old_target - target_angle) > 0.5:
                # Целевой угол изменился - мотор должен начать движение
                if hasattr(self, '_stop_msg_count'):
                    self._stop_msg_count.pop(motor_num, None)
            
            # Мотор 2 (поворот кисти) - управление по времени
            if motor_num == 2:
                self.handle_motor2_with_calibration(target_angle, current_time)
                continue
            
            # Моторы 3-6 - управление через энкодеры
            channel = ENCODER_CHANNEL.get(motor_num)
            if not channel:
                continue
            
            # Получаем текущий угол из кэша
            current_angle = None
            cache_age = current_time - self.last_angle_read_time
            
            # КРИТИЧНО: Если кэш устарел (более 5.0 сек) или пуст - останавливаем мотор для безопасности
            # Увеличиваем порог до 5.0 сек, чтобы моторы могли работать даже при временных ошибках чтения
            if cache_age > 5.0 or not self.cached_angles:
                # Кэш устарел - останавливаем мотор для безопасности
                try:
                    if motor_num == 5:
                        self.controller.stop_arduino_motor()
                    else:
                        addr = MOTOR_ADDR.get(motor_num)
                        if addr:
                            self.controller.stop_mks_motor(addr)
                except:
                    pass
                # Добавляем отладочное сообщение
                if not hasattr(self, '_cache_warning_count'):
                    self._cache_warning_count = {}
                count = self._cache_warning_count.get(motor_num, 0)
                if count < 3:
                    print(f"⚠️ Мотор {motor_num}: кэш устарел (возраст: {cache_age:.2f}с) или пуст, пропуск команды")
                    self._cache_warning_count[motor_num] = count + 1
                continue
            
            if channel in self.cached_angles:
                current_angle_raw = self.cached_angles[channel]
                # Коррекция для зеркальных моторов
                if motor_num in [3, 4]:
                    current_angle = 180 - current_angle_raw
                else:
                    current_angle = current_angle_raw
            
            # Если угол неизвестен - останавливаем мотор для безопасности
            if current_angle is None:
                try:
                    if motor_num == 5:
                        self.controller.stop_arduino_motor()
                    else:
                        addr = MOTOR_ADDR.get(motor_num)
                        if addr:
                            self.controller.stop_mks_motor(addr)
                except:
                    pass
                # Добавляем отладочное сообщение
                if not hasattr(self, '_angle_missing_count'):
                    self._angle_missing_count = {}
                count = self._angle_missing_count.get(motor_num, 0)
                if count < 3:
                    print(f"⚠️ Мотор {motor_num}: угол {channel} не найден в кэше, пропуск команды")
                    self._angle_missing_count[motor_num] = count + 1
                continue
            
            # Проверяем достижение цели
            diff = abs(current_angle - target_angle)
            
            # Пороги остановки для каждого мотора
            if motor_num in [5, 6]:
                stop_threshold = 3.0
            else:
                stop_threshold = 2.0
            
            # КРИТИЧНО: Если уже близко к цели - останавливаем мотор и пропускаем
            if diff < stop_threshold:
                # Останавливаем мотор, если он еще не остановлен
                try:
                    if motor_num == 5:
                        self.controller.stop_arduino_motor()
                    else:
                        addr = MOTOR_ADDR.get(motor_num)
                        if addr:
                            self.controller.stop_mks_motor(addr)
                except:
                    pass
                continue
            
            # Определяем направление
            if motor_num == 6:
                # Для мотора 6 - кратчайший путь
                error_cw = (current_angle - target_angle) % 360
                error_ccw = (target_angle - current_angle) % 360
                forward = error_ccw < error_cw
                forward = not forward  # Инверсия для мотора 6
            else:
                need_increase = current_angle < target_angle
                forward = not need_increase if motor_num in INVERT_DIRECTION else need_increase
            
            # Базовая скорость
            speed = BASE_SPEEDS.get(motor_num, 2)
            
            # Отправляем команду
            try:
                if motor_num == 5:
                    direction = 'b' if forward else 'f'
                    cmd = f"{direction}{int(speed * 100)}"
                    self.controller.arduino_serial.send_command(cmd, wait_response=False)
                else:
                    addr = MOTOR_ADDR.get(motor_num)
                    if addr:
                        self.controller.send_mks_speed(addr, speed, forward)
                
                # Отладочное сообщение для моторов 3-6 (только первые несколько раз)
                if not hasattr(self, '_motor_move_msg_count'):
                    self._motor_move_msg_count = {}
                count = self._motor_move_msg_count.get(motor_num, 0)
                if count < 5:
                    direction_str = "вперед" if forward else "назад"
                    print(f"🔄 Мотор {motor_num}: {direction_str} → {target_angle}° (текущий: {current_angle:.1f}°, diff: {diff:.1f}°)")
                    self._motor_move_msg_count[motor_num] = count + 1
            except Exception as e:
                if not hasattr(self, '_error_count'):
                    self._error_count = {}
                self._error_count[motor_num] = self._error_count.get(motor_num, 0) + 1
                if self._error_count[motor_num] <= 3:
                    print(f"❌ Ошибка мотора {motor_num}: {e}")
    
    def apply_angles_position(self, angles: Dict[int, int]):
        """
        Управление моторами в режиме позиционирования (через move_to_angle)
        """
        if not angles:
            return
        
        for gui_idx, motor_num in GUI_TO_MOTOR_MAPPING.items():
            if motor_num == 1:
                continue
            
            if gui_idx not in angles:
                continue
            
            target_angle = angles[gui_idx]
            min_angle, max_angle = MOTOR_ANGLE_LIMITS.get(motor_num, (0, 180))
            target_angle = max(min_angle, min(max_angle, target_angle))
            
            last_target = self.motor_targets.get(motor_num)
            if last_target is not None and abs(last_target - target_angle) < 0.5:
                continue
            
            self.motor_targets[motor_num] = target_angle
            self._start_position_move(motor_num, target_angle)
    
    def _start_position_move(self, motor_num: int, target_angle: float):
        """Запускает отдельный поток для перемещения мотора в целевой угол"""
        with self.motor_thread_lock:
            existing_thread = self.motor_threads.get(motor_num)
            if existing_thread and existing_thread.is_alive():
                stop_event = self.motor_stop_events.get(motor_num)
                if stop_event:
                    stop_event.set()
                existing_thread.join(timeout=0.5)
            
            stop_event = threading.Event()
            self.motor_stop_events[motor_num] = stop_event
            
            thread = threading.Thread(
                target=self._motor_move_worker,
                args=(motor_num, target_angle, stop_event),
                daemon=True
            )
            self.motor_threads[motor_num] = thread
            thread.start()
    
    def _motor_move_worker(self, motor_num: int, target_angle: float, stop_event: threading.Event):
        """Фоновый поток для перемещения мотора в указанный угол"""
        try:
            print(f"🚀 Мотор {motor_num}: перемещение к {target_angle:.1f}° (позиционный режим)")
            if self.supports_stop_event:
                success = self.controller.move_to_angle(
                    motor_num,
                    target_angle,
                    tolerance=1.0,
                    verbose=False,
                    stop_event=stop_event
                )
            else:
                success = self.controller.move_to_angle(
                    motor_num,
                    target_angle,
                    tolerance=1.0,
                    verbose=False
                )
            if success:
                print(f"✅ Мотор {motor_num}: достиг {target_angle:.1f}°")
            else:
                if stop_event.is_set() and self.supports_stop_event:
                    print(f"⏹️ Мотор {motor_num}: движение отменено")
                else:
                    print(f"⚠️ Мотор {motor_num}: не удалось достигнуть {target_angle:.1f}°")
        except Exception as e:
            print(f"❌ Ошибка позиционирования мотора {motor_num}: {e}")
        finally:
            with self.motor_thread_lock:
                self.motor_threads.pop(motor_num, None)
                self.motor_stop_events.pop(motor_num, None)
    
    def _stop_all_position_moves(self):
        """Останавливает все фоновые потоки позиционирования"""
        with self.motor_thread_lock:
            for event in self.motor_stop_events.values():
                event.set()
            for thread in self.motor_threads.values():
                thread.join(timeout=0.5)
            self.motor_threads.clear()
            self.motor_stop_events.clear()
    
    def handle_motor2_with_calibration(self, target_angle: int, current_time: float):
        """
        Управление мотором 2 на основе параметров калибровки и времени вращения
        
        Мотор 2 (0xE0) устанавливается ВРУЧНУЮ в стартовую позицию 90°,
        а угол поворота вычисляется АВТОМАТИЧЕСКИ на основе времени вращения
        согласно параметрам из motor2_calibration.txt.
        
        БЕЗ концевика - только программное отслеживание угла от стартовой позиции 90°
        
        Args:
            target_angle: Целевой угол (0-180°)
            current_time: Текущее время
        """
        # Инициализация должна быть выполнена при подключении клиента
        # Здесь просто проверяем, что инициализация была выполнена
        
        # Ограничиваем целевой угол диапазоном калибровки
        max_angle = self.motor2_calib.get('full_rotation_angle', 180.0)
        target_angle = max(0, min(int(max_angle), target_angle))
        rotation_speed = self.motor2_calib.get('rotation_speed', 42.07)  # °/сек из калибровки
        
        # Вычисляем разницу между текущим и целевым углом
        angle_diff = target_angle - self.motor2_current_angle
        
        # Если угол очень близко к целевому - останавливаем (если мотор вращается)
        # Увеличен допуск для более быстрой работы
        if abs(angle_diff) < 3.0:
            if self.motor2_is_rotating:
                addr = MOTOR_ADDR.get(2)
                if addr:
                    self.controller.stop_mks_motor(addr)
                self.motor2_is_rotating = False
                self.motor2_current_angle = target_angle
                self.motor2_last_update_time = current_time
                # Убираем избыточный вывод
            return
        
        # Вычисляем необходимое время вращения для достижения целевого угла
        rotation_time = abs(angle_diff) / rotation_speed if rotation_speed > 0 else 0  # секунды
        
        # Определяем направление вращения
        forward = angle_diff > 0
        
        # Если меняется направление или мотор не вращается - начинаем новое вращение
        if (not self.motor2_is_rotating or 
            self.motor2_last_rotation_direction != forward):
            addr = MOTOR_ADDR.get(2)
            if not addr:
                print("  ❌ Мотор 2: адрес не найден!")
                return
            
            # Сохраняем стартовую позицию для текущего вращения
            self.motor2_rotation_start_angle = self.motor2_current_angle
            
            speed = BASE_SPEEDS.get(2, 3)
            # Минимальный вывод - только первые несколько раз
            if not hasattr(self, '_motor2_rotation_count'):
                self._motor2_rotation_count = 0
            if self._motor2_rotation_count < 5:
                print(f"🔄 Мотор 2: {'вперед' if forward else 'назад'} → {target_angle}° (текущий: {self.motor2_current_angle:.1f}°)")
                self._motor2_rotation_count += 1
            
            # Отправляем команду вращения СРАЗУ (без задержек и проверок)
            self.controller.send_mks_speed(addr, speed, forward=forward)
            self.motor2_is_rotating = True
            self.motor2_last_rotation_start = current_time
            self.motor2_last_rotation_direction = forward
        
        # Обновляем текущий угол на основе времени вращения (автоматический расчет)
        if self.motor2_is_rotating and self.motor2_last_rotation_start and self.motor2_rotation_start_angle is not None:
            elapsed_time = current_time - self.motor2_last_rotation_start
            
            # Вычисляем угол, который мотор прошел с момента начала вращения
            angle_moved = rotation_speed * elapsed_time
            
            # Вычисляем новый угол от стартовой позиции текущего вращения
            if forward:
                # Вращение вперед - увеличиваем угол
                new_angle = self.motor2_rotation_start_angle + angle_moved
                # Ограничиваем максимальным углом и целевым углом
                new_angle = min(new_angle, max_angle, target_angle)
            else:
                # Вращение назад - уменьшаем угол
                new_angle = self.motor2_rotation_start_angle - angle_moved
                # Ограничиваем минимальным углом (0) и целевым углом
                new_angle = max(new_angle, 0.0, target_angle)
            
            # Ограничиваем угол диапазоном калибровки (0 - max_angle)
            new_angle = max(0.0, min(max_angle, new_angle))
            
            # Упрощенный расчет - без сглаживания для мгновенной реакции
            # Обновляем текущий угол напрямую (без истории)
            self.motor2_current_angle = new_angle
            self.motor2_last_update_time = current_time
            
            # Проверяем, достигли ли цели (реалистичный допуск 1.5°)
            remaining_diff = abs(self.motor2_current_angle - target_angle)
            if remaining_diff < 1.5:
                addr = MOTOR_ADDR.get(2)
                if addr:
                    self.controller.stop_mks_motor(addr)
                self.motor2_is_rotating = False
                self.motor2_current_angle = target_angle  # Фиксируем точное значение
                self.motor2_rotation_start_angle = None
                self.motor2_last_update_time = current_time
                if not hasattr(self, '_motor2_stop_count'):
                    self._motor2_stop_count = 0
                if self._motor2_stop_count < 5:
                    print(f"✅ Мотор 2: достиг {target_angle}°")
                    self._motor2_stop_count += 1
    
    def stop_idle_motors(self):
        """Останавливает моторы, которые не получали обновлений углов"""
        current_time = time.time()
        motors_to_stop = []
        
        for motor_num, last_update in self.last_angle_update.items():
            if current_time - last_update > self.angle_update_timeout:
                motors_to_stop.append(motor_num)
        
        # Останавливаем только если действительно прошло время (не останавливаем сразу после обновления)
        for motor_num in motors_to_stop:
            if motor_num == 5:
                self.controller.stop_arduino_motor()
            else:
                addr = MOTOR_ADDR.get(motor_num)
                if addr:
                    self.controller.stop_mks_motor(addr)
            # КРИТИЧНО: НЕ удаляем target_angles - оставляем его для реакции на новые команды
            # self.target_angles.pop(motor_num, None)  # НЕ удаляем
            self.last_angle_update.pop(motor_num, None)
        
        # Останавливаем мотор 2 если он не получал обновлений
        if self.motor2_is_rotating:
            current_time = time.time()
            if self.motor2_last_rotation_start:
                elapsed = current_time - self.motor2_last_rotation_start
                # Если мотор вращается слишком долго без обновлений - останавливаем
                if elapsed > 10.0:  # 10 секунд без обновлений
                    addr = MOTOR_ADDR.get(2)
                    if addr:
                        self.controller.stop_mks_motor(addr)
                    self.motor2_is_rotating = False
    
    def handle_tcp_client(self, client_socket: socket.socket, addr):
        """Обрабатывает TCP клиента"""
        print(f"✅ TCP клиент подключен: {addr[0]}:{addr[1]}")
        client_socket.settimeout(0.1)  # Увеличиваем таймаут до 100 мс для стабильности
        
        buffer = b""  # Буфер для накопления данных
        
        # Инициализируем мотор 2 при подключении (без калибровки, мгновенная реакция)
        if not self.motor2_initialized:
            self.motor2_current_angle = self.motor2_starting_angle
            self.motor2_initialized = True
            print(f"📍 Мотор 2: инициализирован, стартовая позиция {self.motor2_starting_angle}° (мгновенная реакция)")
        
        # КРИТИЧНО: Читаем реальные углы моторов ПЕРЕД началом обработки команд
        # Это позволяет правильно определить направление движения
        print("📊 Чтение текущих углов моторов при подключении...")
        try:
            initial_angles = self.controller.read_all_angles()
            if initial_angles:
                self.cached_angles = initial_angles
                self.last_angle_read_time = time.time()
                print("✅ Углы моторов прочитаны для кэша")
            else:
                print("⚠️ Не удалось прочитать начальные углы, будет задержка при первом движении")
        except Exception as e:
            print(f"⚠️ Не удалось прочитать начальные углы: {e}")
            print("   Моторы могут случайно вращаться при подключении")
        
        print("✅ Готов к управлению через GUI")
        
        try:
            while self.running:
                try:
                    # Читаем данные
                    chunk = client_socket.recv(8 - len(buffer))
                    if len(chunk) == 0:
                        break
                    
                    buffer += chunk
                    
                    # Если накопили 8 байт - обрабатываем
                    if len(buffer) >= 8:
                        data = buffer[:8]
                        buffer = buffer[8:]  # Оставляем остаток в буфере
                        # Преобразуем байты в углы
                        angles = {}
                        for i, angle_byte in enumerate(data):
                            if i in GUI_TO_MOTOR_MAPPING:
                                angles[i] = int(angle_byte)
                        
                        # Отладочное сообщение (только при изменении углов)
                        if not hasattr(self, '_last_angles'):
                            self._last_angles = {}
                        
                        # Выводим только если углы изменились
                        if angles != self._last_angles:
                            print(f"📥 Команда: {angles}")
                            self._last_angles = angles.copy()
                        
                        # КРИТИЧНО: Применяем углы СРАЗУ при каждом изменении
                        # Это гарантирует реакцию на изменения ползунков
                        try:
                            if CONTROL_MODE == 'direct':
                                self.apply_angles_direct(angles)
                            else:
                                self.apply_angles_position(angles)
                        except Exception as e:
                            print(f"❌ Ошибка применения углов: {e}")
                            import traceback
                            traceback.print_exc()
                        
                        # Эхо-ответ (как в оригинальном ESP коде)
                        try:
                            client_socket.send(data)
                        except:
                            pass
                    
                    # НЕ останавливаем моторы слишком часто - только если действительно нет обновлений
                    # Это позволяет моторам продолжать движение даже при небольших задержках
                    # self.stop_idle_motors()  # Отключено для более плавного управления
                    
                except socket.timeout:
                    # Проверяем, не нужно ли остановить моторы
                    self.stop_idle_motors()
                    continue
                except Exception as e:
                    print(f"⚠️ Ошибка обработки TCP: {e}")
                    break
        
        except Exception as e:
            print(f"⚠️ Ошибка TCP соединения: {e}")
        finally:
            client_socket.close()
            print(f"🔌 TCP клиент отключен: {addr[0]}:{addr[1]}")
            # Останавливаем все моторы при отключении
            try:
                self._stop_all_position_moves()
                self.controller.stop_all()
            except Exception as e:
                # Игнорируем ошибки при остановке (порт может быть недоступен)
                pass
    
    def start_angle_reader_thread(self):
        """Запускает отдельный поток для чтения углов и остановки моторов"""
        if self.angle_reader_running:
            return
        
        def angle_reader():
            """Фоновый поток для чтения углов и остановки моторов"""
            self.angle_reader_running = True
            print("✅ Поток чтения углов запущен")
            error_count = 0
            max_errors = 5
            
            while self.running and self.angle_reader_running:
                try:
                    # КРИТИЧНО: Постоянно читаем углы с энкодеров
                    # Делаем несколько попыток для надежности
                    new_angles = None
                    for read_attempt in range(2):  # 2 попытки чтения
                        new_angles = self.controller.read_all_angles()
                        if new_angles:
                            break
                        if read_attempt < 1:
                            time.sleep(0.1)  # Небольшая задержка между попытками
                    
                    if new_angles:
                        # Обновляем кэш с блокировкой
                        with self.angle_read_lock:
                            self.cached_angles = new_angles
                            self.last_angle_read_time = time.time()
                        error_count = 0  # Сбрасываем счетчик при успешном чтении
                    else:
                        error_count += 1
                        # КРИТИЧНО: НЕ очищаем кэш при временных ошибках - используем последние известные углы
                        # Очищаем кэш только если ошибок слишком много подряд
                        # Увеличиваем порог до 30 ошибок для большей устойчивости
                        if error_count >= max_errors * 6:  # 30 ошибок подряд
                            print(f"⚠️ Поток чтения углов: слишком много ошибок ({error_count}), очищаем кэш и останавливаем моторы")
                            with self.angle_read_lock:
                                self.cached_angles = None  # Очищаем кэш только при критических ошибках
                            try:
                                # Останавливаем все MKS моторы
                                for motor_num in [2, 3, 4, 6]:
                                    addr = MOTOR_ADDR.get(motor_num)
                                    if addr:
                                        self.controller.stop_mks_motor(addr)
                                # Останавливаем мотор 5
                                self.controller.stop_arduino_motor()
                            except:
                                pass
                            error_count = 0  # Сбрасываем счетчик
                        # При временных ошибках продолжаем использовать старый кэш
                        # Это позволяет моторам продолжать работать даже при кратковременных проблемах чтения
                    
                    # КРИТИЧНО: Останавливаем моторы, достигшие цели (на основе актуальных углов)
                    self.stop_motors_at_target()
                    
                    time.sleep(0.03)  # 30 мс - более частое чтение для быстрой реакции
                    
                except Exception as e:
                    error_count += 1
                    if error_count <= 3:
                        print(f"⚠️ Ошибка чтения углов: {e}")
                    # Если много ошибок - останавливаем моторы, но НЕ очищаем кэш сразу
                    # Увеличиваем порог до 20 ошибок для большей устойчивости
                    if error_count >= max_errors * 4:  # 20 ошибок подряд
                        print(f"⚠️ Критическая ошибка чтения углов, останавливаем моторы")
                        try:
                            for motor_num in [2, 3, 4, 6]:
                                addr = MOTOR_ADDR.get(motor_num)
                                if addr:
                                    self.controller.stop_mks_motor(addr)
                            self.controller.stop_arduino_motor()
                        except:
                            pass
                        error_count = 0
                    time.sleep(0.1)
        
        self.angle_reader_thread = threading.Thread(target=angle_reader, daemon=True)
        self.angle_reader_thread.start()
        print("✅ Поток чтения углов запущен")
    
    def stop_motors_at_target(self):
        """Останавливает моторы, достигшие целевых углов (вызывается из потока чтения углов)"""
        if not self.cached_angles or not self.target_angles:
            return
        
        current_time = time.time()
        cache_age = current_time - self.last_angle_read_time
        
        # КРИТИЧНО: Используем только актуальный кэш (менее 0.2 сек)
        if cache_age > 0.2:
            return  # Кэш устарел, не останавливаем моторы
        
        motors_to_stop = []
        
        for motor_num, target_angle in list(self.target_angles.items()):
            # Моторы 1 и 2 обрабатываются отдельно
            if motor_num in [1, 2]:
                continue
            
            channel = ENCODER_CHANNEL.get(motor_num)
            if not channel or channel not in self.cached_angles:
                continue
            
            current_angle_raw = self.cached_angles[channel]
            
            # Коррекция угла для зеркальных моторов (только 3-4, мотор 2 работает программно)
            if motor_num in [3, 4]:
                current_angle = 180 - current_angle_raw
            else:
                current_angle = current_angle_raw
            
            # Проверка достижения цели
            diff = abs(current_angle - target_angle)
            
            # Пороги остановки для каждого мотора
            if motor_num == 5 or motor_num == 6:
                stop_threshold = 3.0
            elif motor_num == 4:
                stop_threshold = 2.5
            else:
                stop_threshold = 2.0
            
            # КРИТИЧНО: Останавливаем только если реально достигли цели
            if diff < stop_threshold:
                motors_to_stop.append((motor_num, diff))
        
        # Останавливаем моторы
        for motor_num, diff in motors_to_stop:
            try:
                if motor_num == 5:
                    self.controller.stop_arduino_motor()
                elif motor_num in MOTOR_ADDR:
                    addr = MOTOR_ADDR[motor_num]
                    self.controller.stop_mks_motor(addr)
                
                # КРИТИЧНО: НЕ удаляем target_angles - оставляем его для реакции на новые команды
                # Это позволяет моторам реагировать на изменения ползунков
                self.last_angle_update.pop(motor_num, None)
                
                # Очищаем направление для этого мотора
                self.last_direction.pop(motor_num, None)
                
                if not hasattr(self, '_stop_msg_count'):
                    self._stop_msg_count = {}
                count = self._stop_msg_count.get(motor_num, 0)
                if count < 3:
                    print(f"✅ Мотор {motor_num}: достиг цели (diff={diff:.1f}°)")
                    self._stop_msg_count[motor_num] = count + 1
            except Exception as e:
                pass  # Игнорируем ошибки остановки
    
    def start_tcp_server(self):
        """Запускает TCP сервер"""
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.tcp_server.bind(("", TCP_PORT))
        except OSError as e:
            if e.errno == 98:  # Address already in use
                print(f"⚠️ Порт {TCP_PORT} уже занят.")
                print(f"   Попробуйте:")
                print(f"   • Закрыть другой процесс, использующий порт {TCP_PORT}")
                print(f"   • Или подождать несколько секунд")
                print(f"   • Или выполнить: sudo lsof -ti:{TCP_PORT} | xargs sudo kill -9")
                # Закрываем сокет перед выходом
                self.tcp_server.close()
                raise
            else:
                raise
        self.tcp_server.listen(1)
        self.tcp_server.settimeout(1.0)
        
        local_ip = self.get_local_ip()
        print(f"✅ TCP сервер запущен на {local_ip}:{TCP_PORT}")
        
        # Запускаем поток чтения углов при старте сервера
        if not self.angle_reader_running:
            self.start_angle_reader_thread()
        
        while self.running:
            try:
                client_socket, addr = self.tcp_server.accept()
                # Обрабатываем только одного клиента за раз
                if self.tcp_client:
                    client_socket.close()
                    continue
                
                self.tcp_client = client_socket
                # Запускаем обработку в отдельном потоке
                client_thread = threading.Thread(
                    target=self.handle_tcp_client,
                    args=(client_socket, addr),
                    daemon=True
                )
                client_thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"⚠️ Ошибка TCP сервера: {e}")
    
    def start(self):
        """Запускает сервер"""
        self.running = True
        
        # UDP поток для обнаружения
        udp_thread = threading.Thread(target=self.start_udp_discovery, daemon=True)
        udp_thread.start()
        
        # TCP сервер в основном потоке
        self.start_tcp_server()
    
    def stop(self):
        """Останавливает сервер"""
        print("\n🛑 Остановка сервера...")
        self.running = False
        self.angle_reader_running = False
        
        # Ждем завершения потока чтения углов
        if self.angle_reader_thread and self.angle_reader_thread.is_alive():
            self.angle_reader_thread.join(timeout=1.0)
        
        if self.tcp_client:
            try:
                self.tcp_client.close()
            except:
                pass
        
        if self.tcp_server:
            try:
                self.tcp_server.close()
            except:
                pass
        
        if self.udp_socket:
            try:
                self.udp_socket.close()
            except:
                pass
        
        # Останавливаем все моторы
        try:
            self._stop_all_position_moves()
            self.controller.stop_all()
        except Exception as e:
            # Игнорируем ошибки при остановке (порт может быть недоступен)
            print(f"⚠️ Ошибка при остановке моторов (порт может быть недоступен): {e}")
        print("✅ Сервер остановлен")


def get_usb_device_id(port_path: str) -> Optional[str]:
    """
    Получает USB ID устройства для указанного порта
    
    Args:
        port_path: Путь к порту (например, '/dev/ttyUSB0')
    
    Returns:
        USB ID в формате 'vendor:product' или None
    """
    try:
        # Извлекаем имя устройства (например, 'ttyUSB0' из '/dev/ttyUSB0')
        device_name = os.path.basename(port_path)
        
        # Путь к информации об устройстве в sysfs
        sysfs_path = f'/sys/class/tty/{device_name}/device'
        
        if not os.path.exists(sysfs_path):
            return None
        
        # Переходим к USB устройству через симлинки
        # Структура: /sys/class/tty/ttyUSB0/device -> ../../ttyUSB0/tty/ttyUSB0
        # Нужно найти путь к USB устройству
        device_real_path = os.path.realpath(sysfs_path)
        
        # Ищем путь к USB устройству (идем вверх по дереву)
        current_path = device_real_path
        for _ in range(10):  # Максимум 10 уровней вверх
            try:
                parent = os.path.dirname(current_path)
                if not os.path.exists(parent):
                    break
                
                # Проверяем наличие файлов idVendor и idProduct
                vendor_path = os.path.join(parent, 'idVendor')
                product_path = os.path.join(parent, 'idProduct')
                
                if os.path.exists(vendor_path) and os.path.exists(product_path):
                    # Нашли USB устройство
                    with open(vendor_path, 'r') as f:
                        vendor = f.read().strip()
                    with open(product_path, 'r') as f:
                        product = f.read().strip()
                    return f"{vendor}:{product}"
            except (OSError, PermissionError):
                # Пропускаем, если нет доступа
                pass
            
            current_path = parent
            if current_path == '/' or current_path == parent:
                break
        
        return None
    except Exception as e:
        # В случае ошибки просто возвращаем None
        return None


def find_port_by_usb_id(vendor_id: str, product_id: str, exclude_port: Optional[str] = None) -> Optional[str]:
    """
    Находит serial порт по USB ID устройства
    
    Args:
        vendor_id: Vendor ID в hex формате (например, '10c4')
        product_id: Product ID в hex формате (например, 'ea60')
        exclude_port: Порт для исключения из поиска (например, уже найденный порт)
    
    Returns:
        Путь к найденному порту или None
    """
    target_vid = int(vendor_id, 16)
    target_pid = int(product_id, 16)
    print(f"🔍 Поиск устройства с USB ID: {vendor_id}:{product_id}...")
    
    # Используем serial.tools.list_ports для кроссплатформенного поиска
    ports = serial.tools.list_ports.comports()
    
    for port_info in ports:
        port = port_info.device
        
        # Пропускаем исключенный порт
        if exclude_port and port == exclude_port:
            continue
        
        # Проверяем доступность порта
        if not check_port_availability(port):
            continue
        
        # Проверяем USB ID
        if port_info.vid == target_vid and port_info.pid == target_pid:
            print(f"   ✅ Найдено устройство {vendor_id}:{product_id} на порту {port}")
            return port
        elif port_info.vid is not None and port_info.pid is not None:
            # Показываем найденные устройства для отладки
            found_id = f"{port_info.vid:04x}:{port_info.pid:04x}"
            print(f"   Проверка {port}: USB ID = {found_id}")
    
    # На Linux также проверяем через sysfs (если доступно)
    if platform.system() == 'Linux':
        port_patterns = [
            '/dev/ttyUSB*',  # USB-to-Serial адаптеры
            '/dev/ttyACM*',  # USB CDC устройства
        ]
        
        for pattern in port_patterns:
            ports = glob.glob(pattern)
            for port in sorted(ports):
                # Пропускаем исключенный порт
                if exclude_port and port == exclude_port:
                    continue
                
                if not check_port_availability(port):
                    continue
                
                device_id = get_usb_device_id(port)
                if device_id:
                    print(f"   Проверка {port}: USB ID = {device_id}")
                    if device_id == f"{vendor_id.lower()}:{product_id.lower()}":
                        print(f"   ✅ Найдено устройство {vendor_id}:{product_id} на порту {port}")
                        return port
    
    return None


def find_available_serial_ports() -> List[str]:
    """
    Автоматически находит доступные serial порты (кроссплатформенная версия)
    
    Returns:
        Список доступных портов
    """
    available_ports = []
    
    print("🔍 Поиск доступных serial портов...")
    
    # Используем serial.tools.list_ports для кроссплатформенного поиска
    ports = serial.tools.list_ports.comports()
    
    for port_info in ports:
        port = port_info.device
        if check_port_availability(port):
            available_ports.append(port)
            # Показываем информацию о порте
            if port_info.vid is not None and port_info.pid is not None:
                usb_id = f"{port_info.vid:04x}:{port_info.pid:04x}"
                desc = port_info.description or "Unknown"
                print(f"   ✅ Найден порт: {port} (USB ID: {usb_id}, {desc})")
            else:
                print(f"   ✅ Найден порт: {port}")
    
    # На Linux также проверяем через glob (для совместимости)
    if platform.system() == 'Linux':
        port_patterns = [
            '/dev/ttyUSB*',  # USB-to-Serial адаптеры
            '/dev/ttyACM*',  # USB CDC устройства (Arduino Uno, etc.)
            '/dev/ttyS*',    # Стандартные serial порты
        ]
        
        for pattern in port_patterns:
            ports = glob.glob(pattern)
            for port in sorted(ports):
                if port not in available_ports and check_port_availability(port):
                    available_ports.append(port)
                    usb_id = get_usb_device_id(port)
                    if usb_id:
                        print(f"   ✅ Найден порт: {port} (USB ID: {usb_id})")
                    else:
                        print(f"   ✅ Найден порт: {port}")
    
    return available_ports


def auto_detect_port(preferred_port: str = None, usb_vendor_id: str = None, usb_product_id: str = None, exclude_port: Optional[str] = None) -> Optional[str]:
    """
    Автоматически определяет подходящий serial порт
    
    Args:
        preferred_port: Предпочтительный порт (если указан и доступен, будет использован)
        usb_vendor_id: USB Vendor ID для поиска конкретного устройства (например, '10c4')
        usb_product_id: USB Product ID для поиска конкретного устройства (например, 'ea60')
        exclude_port: Порт для исключения из поиска (например, уже найденный порт)
    
    Returns:
        Путь к найденному порту или None
    """
    # Приоритет 1: Поиск по USB ID (если указан)
    if usb_vendor_id and usb_product_id:
        port_by_usb = find_port_by_usb_id(usb_vendor_id, usb_product_id, exclude_port=exclude_port)
        if port_by_usb:
            print(f"✅ Найдено устройство по USB ID {usb_vendor_id}:{usb_product_id} на порту {port_by_usb}")
            return port_by_usb
        else:
            print(f"⚠️ Устройство с USB ID {usb_vendor_id}:{usb_product_id} не найдено, продолжаем поиск...")
    
    # Приоритет 2: Проверяем предпочтительный порт
    if preferred_port and check_port_availability(preferred_port):
        print(f"✅ Используется предпочтительный порт: {preferred_port}")
        return preferred_port
    
    # Приоритет 3: Ищем все доступные порты (исключая уже найденный)
    available_ports = find_available_serial_ports()
    
    # Исключаем порт, если он указан
    if exclude_port and exclude_port in available_ports:
        available_ports.remove(exclude_port)
        print(f"   Исключен порт: {exclude_port}")
    
    if not available_ports:
        print("❌ Не найдено доступных serial портов!")
        return None
    
    # Если найден только один порт - используем его
    if len(available_ports) == 1:
        selected_port = available_ports[0]
        print(f"✅ Автоматически выбран порт: {selected_port}")
        return selected_port
    
    # Если найдено несколько портов - выбираем первый доступный
    # Приоритет: COM (Windows) или ttyUSB > ttyACM > ttyS (Linux)
    if platform.system() == 'Windows':
        # На Windows приоритет COM портам
        for port in available_ports:
            if 'COM' in port.upper():
                print(f"✅ Автоматически выбран порт: {port}")
                return port
    else:
        # На Linux приоритет: ttyUSB > ttyACM > ttyS
        priority_order = ['ttyUSB', 'ttyACM', 'ttyS']
        for priority in priority_order:
            for port in available_ports:
                if priority in port:
                    print(f"✅ Автоматически выбран порт: {port} (приоритет: {priority})")
                    return port
    
    # Если не нашли по приоритету - берем первый
    selected_port = available_ports[0]
    print(f"✅ Автоматически выбран порт: {selected_port}")
    return selected_port


def main():
    """Основная функция"""
    print("\n" + "="*70)
    print("     TCP СЕРВЕР ДЛЯ УПРАВЛЕНИЯ РОБОТОМ")
    print("="*70)
    print("📌 Функции:")
    print("  • UDP обнаружение (порт 50000)")
    print("  • TCP сервер для приема команд (порт 50001)")
    print("  • Управление моторами через MotorController")
    print("  • Совместимость с mock-client-multi.py")
    print("="*70)
    
    # Автоматическое определение портов
    print("\n🔍 Поиск портов...")
    
    if MOCK_MODE:
        print("🧪 Режим заглушки включен (ARM4_MOCK=1): запуск без serial-устройств")
        controller = MockMotorController()
        server = RobotTCPServer(controller)
        try:
            server.start()
        except KeyboardInterrupt:
            pass
        finally:
            server.stop()
            print("\n👋 До свидания!")
        return

    # Поиск Arduino (FT232)
    arduino_port = auto_detect_port(
        usb_vendor_id=ARDUINO_USB_VENDOR_ID,
        usb_product_id=ARDUINO_USB_PRODUCT_ID
    )
    
    if not arduino_port:
        print("\n❌ ОШИБКА: Не удалось найти Arduino (FT232) порт!")
        print(f"   Ожидаемый USB ID: {ARDUINO_USB_VENDOR_ID}:{ARDUINO_USB_PRODUCT_ID}")
        print("\n📋 Доступные порты:")
        all_ports = serial.tools.list_ports.comports()
        found_any = False
        for port_info in all_ports:
            if port_info.vid is not None and port_info.pid is not None:
                usb_id = f"{port_info.vid:04x}:{port_info.pid:04x}"
                desc = port_info.description or "Unknown"
                print(f"   • {port_info.device}: USB ID = {usb_id} ({desc})")
                found_any = True
        if not found_any:
            print("   (Нет портов с USB ID)")
        print("\n   Проверьте:")
        print("   • Подключено ли устройство Arduino")
        print("   • Правильно ли установлены драйверы")
        print("   • Есть ли права доступа к порту")
        print("\n⚠️ ВНИМАНИЕ: Продолжение без Arduino порта может привести к ошибкам!")
        response = input("   Продолжить в любом случае? (yes/no): ").strip().lower()
        if response not in ['yes', 'y', 'да', 'д']:
            sys.exit(1)
        # Если пользователь согласился, используем первый доступный порт
        all_ports = serial.tools.list_ports.comports()
        available_ports = [p.device for p in all_ports if check_port_availability(p.device)]
        if available_ports:
            arduino_port = available_ports[0]
            print(f"⚠️ Используется первый доступный порт для Arduino: {arduino_port}")
        else:
            print("❌ Нет доступных портов!")
            sys.exit(1)
    
    print(f"✅ Найден Arduino порт: {arduino_port}")
    
    # Поиск CH340 (исключаем уже найденный Arduino порт)
    ch340_port = auto_detect_port(
        usb_vendor_id=CH340_USB_VENDOR_ID,
        usb_product_id=CH340_USB_PRODUCT_ID,
        exclude_port=arduino_port
    )
    
    if not ch340_port:
        print("\n❌ ОШИБКА: Не удалось найти CH340 порт!")
        print(f"   Ожидаемый USB ID: {CH340_USB_VENDOR_ID}:{CH340_USB_PRODUCT_ID}")
        print(f"   (Arduino порт {arduino_port} исключен из поиска)")
        print(f"\n📋 Доступные порты (кроме {arduino_port}):")
        all_ports = serial.tools.list_ports.comports()
        for port_info in all_ports:
            if port_info.device != arduino_port:
                if port_info.vid is not None and port_info.pid is not None:
                    usb_id = f"{port_info.vid:04x}:{port_info.pid:04x}"
                    desc = port_info.description or "Unknown"
                    print(f"   • {port_info.device}: USB ID = {usb_id} ({desc})")
        print("\n   Проверьте:")
        print("   • Подключено ли устройство CH340")
        print("   • Правильно ли установлены драйверы")
        print("   • Есть ли права доступа к порту")
        sys.exit(1)
    
    print(f"✅ Найден CH340 порт: {ch340_port}")
    
    # Инициализация
    try:
        arduino_serial = SerialManager(arduino_port, BAUDRATE, TIMEOUT)
        ch340_serial = SerialManager(ch340_port, BAUDRATE, TIMEOUT)
        controller = MotorController(arduino_serial, ch340_serial)
        print(f"\n✅ Подключено:")
        print(f"   Arduino: {arduino_port} (энкодеры, мотор 1, мотор 5)")
        print(f"   CH340: {ch340_port} (моторы MKS 2, 3, 4, 6)")
        
        # Быстрый тест связи и чтения углов
        print("\n🔧 Быстрый тест связи...")
        if hasattr(controller, 'quick_test'):
            if controller.quick_test():
                print("🎉 ТЕСТ ПРОЙДЕН УСПЕШНО! Система готова к работе.")
            else:
                print("⚠️ ТЕСТ НЕ ПРОЙДЕН! Проверьте подключение Arduino.")
        else:
            # Если метод quick_test не доступен, используем test_angle_reading
            print("📊 Тестирование чтения углов...")
            try:
                controller.test_angle_reading()
            except Exception as e:
                print(f"⚠️ Ошибка теста: {e}")
        
        # Автоматическое включение всех MKS моторов при старте
        print("\n⚡ Включение всех MKS моторов...")
        for motor_num, addr in MOTOR_ADDR.items():
            try:
                # Проверяем текущий статус
                status = controller.read_mks_status(addr)
                is_enabled = status.get('enabled', False) if status else False
                
                if not is_enabled:
                    print(f"  → Включение мотора {motor_num} (0x{addr:02X})...")
                    if controller.enable_mks_motor(addr, True):
                        print(f"  ✅ Мотор {motor_num} включен")
                        controller.mks_motors_enabled[motor_num] = True
                    else:
                        print(f"  ⚠️ Не удалось включить мотор {motor_num}")
                else:
                    print(f"  ✅ Мотор {motor_num} уже включен")
                    controller.mks_motors_enabled[motor_num] = True
                
                time.sleep(0.05)  # Небольшая задержка между моторами
            except Exception as e:
                print(f"  ⚠️ Ошибка включения мотора {motor_num}: {e}")
                # Пытаемся включить в любом случае
                try:
                    controller.enable_mks_motor(addr, True)
                except:
                    pass
        
        print("✅ Все MKS моторы готовы к работе\n")
    except Exception as e:
        print(f"\n❌ ОШИБКА инициализации: {e}")
        print(f"   Arduino порт: {arduino_port}")
        print(f"   CH340 порт: {ch340_port}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    # Проверка наличия angle_reader.py
    angle_reader_file = os.path.join(current_dir, "angle_reader.py")
    angle_cache_file = os.path.join(current_dir, "angles_cache.json")
    
    if os.path.exists(angle_reader_file):
        print("=" * 70)
        print("⚠️  ВАЖНО: Обнаружен angle_reader.py")
        print("=" * 70)
        print("📌 Для работы системы необходимо запустить angle_reader.py отдельно:")
        print(f"   python3 {angle_reader_file}")
        print()
        print("📁 Файл кэша углов будет создан автоматически:")
        print(f"   {angle_cache_file}")
        print()
        
        # Проверяем, обновляется ли файл кэша
        if os.path.exists(angle_cache_file):
            file_time = os.path.getmtime(angle_cache_file)
            age = time.time() - file_time
            if age < 2.0:
                print("✅ Файл кэша углов свежий (обновляется)")
            else:
                print(f"⚠️  Файл кэша углов устарел ({age:.1f} сек)")
                print("   Убедитесь, что angle_reader.py запущен!")
        else:
            print("⚠️  Файл кэша углов не найден")
            print("   Запустите angle_reader.py для создания файла")
        print("=" * 70)
        print()
    else:
        print("⚠️  angle_reader.py не найден - система будет работать в режиме прямого чтения")
        print("   (рекомендуется использовать angle_reader.py для стабильной работы)")
        print()
    
    # Создание и запуск сервера
    server = RobotTCPServer(controller)
    
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        arduino_serial.close()
        ch340_serial.close()
        print("\n👋 До свидания!")


if __name__ == "__main__":
    main()

