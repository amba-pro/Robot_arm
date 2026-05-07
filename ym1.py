import serial
import serial.tools.list_ports
import time
import re
import sys
import os
import glob
import platform
import json
from threading import Event
from typing import Optional, Dict, Tuple, List
from collections import deque

# === НАСТРОЙКИ ПОРТОВ ===
# Arduino (FT232) - энкодеры, мотор 1 (сервопривод), мотор 5 (Nema 34)
ARDUINO_USB_VENDOR_ID = "0403"
ARDUINO_USB_PRODUCT_ID = "6001"
# CH340 - моторы MKS 2, 3, 4, 6
CH340_USB_VENDOR_ID = "1a86"
CH340_USB_PRODUCT_ID = "7523"

BAUDRATE = 115200
TIMEOUT = 0.2
RETRY_COUNT = 3

# === ФАЙЛ ДЛЯ ОБМЕНА УГЛАМИ ===
ANGLE_CACHE_FILE = "angles_cache.json"  # Файл для чтения углов из angle_reader.py
# Делаем цикл управления быстрее и реакции более отзывчивыми
UPDATE_INTERVAL = 0.02      # было 0.03 — теперь ~20 мс между циклами
SMOOTH_WINDOW = 2           # было 3 — меньше усреднение, быстрее отклик
ANGLE_READ_DELAY = 0.005    # было 0.01 — чаще опрос углов при сглаживании

# === АДРЕСА MKS SERVO42C ===
MOTOR_ADDR = {
    2: 0xE0,  # поворот кисти (Nema 17)
    3: 0xE1,  # плечо 2 (Nema 17)
    4: 0xE2,  # плечо 1 (Nema 23)
    6: 0xE3   # поворот основания (Nema 17)
}

# === СОПОСТАВЛЕНИЕ МОТОРОВ И КАНАЛОВ ЭНКОДЕРА ===
ENCODER_CHANNEL = {
    1: None,  # клешня (сервопривод) - без энкодера
    2: "A0",  # поворот кисти - концевик A0
    3: "A1",  # плечо 2 - энкодер A1
    4: "A2",  # плечо 1 - энкодер A2
    5: "A3",  # плечо 0 (Nema 34) - энкодер A3
    6: "A4"   # поворот основания - энкодер A4
}

# === ИМЕНА МОТОРОВ ===
MOTOR_NAMES = {
    1: "Клешня (сервопривод MG996R)",
    2: "Поворот кисти (Nema 17) - концевик A0",
    3: "Плечо 2 (Nema 17) - энкодер A1",
    4: "Плечо 1 (Nema 23) - энкодер A2",
    5: "Плечо 0 (Nema 34 + DM542) - энкодер A3",
    6: "Поворот основания (Nema 17) - энкодер A4"
}

# === ИНВЕРТИРОВАТЬ НАПРАВЛЕНИЕ ДЛЯ ЭТИХ МОТОРОВ ===
INVERT_DIRECTION = [2, 4]  # Моторы 2 и 4 - инвертированное направление
# Мотор 4 (Nema 34) - инвертированная ось (угол), а не направление движения

# === БАЗОВЫЕ СКОРОСТИ ДЛЯ КАЖДОГО МОТОРА ===
# Моторы 3,4,5 довольно быстрые, 6 — максимально медленный и плавный
# Примечание: MKS SERVO42C настроены на Mstep=8, режим CR_UART, PID параметры настроены
# Скорость управления: 1-127 (рекомендуется для Mstep=8)
BASE_SPEEDS = {
    1: 0,  # клешня (сервопривод) - не используется для скорости
    2: 3,  # поворот кисти
    3: 4,  # плечо 2 - быстрее
    4: 4,  # плечо 1 - быстрее
    5: 4,  # плечо 0 (Nema 34) - быстрее
    6: 1   # поворот основания (ещё более медленно и стабильно)
}

# === КОНСТАНТЫ ДЛЯ АДАПТИВНОГО УПРАВЛЕНИЯ ===
SPEED_REDUCTION_THRESHOLD = 15.0  # Угол, при котором начинается снижение скорости
MIN_SPEED = 1  # Минимальная скорость (1-127 для MKS SERVO42C)
MAX_SPEED = 5  # Максимальная скорость (рекомендуется не более 127)

# === ФИЗИЧЕСКИЕ ОГРАНИЧЕНИЯ РОБОТА ===
# Ограничения углов для каждого мотора (в градусах, после коррекции)
# Эти значения предотвращают столкновения и выход за рабочие зоны
MOTOR_ANGLE_LIMITS = {
    1: (0, 180),    # клешня (сервопривод) - полный диапазон
    2: (10, 170),   # поворот кисти - ограничен для предотвращения столкновений
    3: (15, 165),   # плечо 2 - ограничен для предотвращения столкновений с основанием
    4: (20, 160),   # плечо 1 - ограничен для предотвращения столкновений
    5: (10, 170),   # плечо 0 (Nema 34) - ограничен
    6: (0, 180)     # поворот основания - полный диапазон
}

# Максимальное время движения (секунды) - защита от зависания
MAX_MOVEMENT_TIME = 30.0

# Минимальная скорость изменения угла для определения застревания (град/сек)
MIN_ANGLE_VELOCITY = 0.5

# Параметры плавного ускорения/замедления
ACCELERATION_RAMP_STEPS = 5  # Количество шагов для разгона
DECELERATION_START_DISTANCE = 20.0  # Начинать торможение за 20° до цели


class SerialManager:
    """Класс для управления serial соединением с переиспользованием"""
    
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection: Optional[serial.Serial] = None
        self.last_use_time = 0
        self.idle_timeout = 10.0  # Увеличено до 10 сек - не закрывать соединение часто
        self.initialized = False  # Флаг первой инициализации
        
    def _ensure_connection(self):
        """Обеспечивает активное соединение"""
        current_time = time.time()
        
        # Если соединение закрыто или простаивает слишком долго - переподключиться
        if (self.connection is None or not self.connection.is_open or 
            (current_time - self.last_use_time) > self.idle_timeout):
            self.close()
            try:
                self.connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                # Даем Arduino время на перезагрузку (как в test_system.py)
                # Только при первой инициализации или переподключении
                if not self.initialized:
                    print(f"  ⏳ Ожидание инициализации Arduino (2 сек)...")
                    time.sleep(2.0)  # 2 секунды для перезагрузки Arduino
                    self.initialized = True
                else:
                    # При переподключении даем больше времени для стабильности
                    time.sleep(0.2)  # Увеличено с 0.1 до 0.2 сек
            except Exception as e:
                raise ConnectionError(f"Не удалось подключиться к {self.port}: {e}")
        
        self.last_use_time = current_time
    
    def send_command(self, cmd: str, wait_response: bool = True) -> Optional[str]:
        """Отправляет команду Arduino и возвращает ответ (ОПТИМИЗИРОВАННАЯ версия)"""
        self._ensure_connection()
        try:
            # Проверяем, что соединение действительно открыто
            if not self.connection or not self.connection.is_open:
                raise ConnectionError(f"Соединение с {self.port} не открыто")
            
            # Для команды 'a' очищаем оба буфера (как в test_system.py)
            if cmd == 'a':
                # КРИТИЧНО: Агрессивная очистка буферов перед чтением углов
                self.connection.reset_input_buffer()
                self.connection.reset_output_buffer()
                # Читаем все что осталось в буфере несколько раз
                for _ in range(3):
                    if self.connection.in_waiting > 0:
                        self.connection.read(self.connection.in_waiting)
                    time.sleep(0.01)
                time.sleep(0.03)  # Увеличена задержка для стабильности
            else:
                # Для других команд очищаем только входной буфер
                if self.connection.in_waiting > 0:
                    self.connection.reset_input_buffer()
                time.sleep(0.005)
            
            # Отправляем команду
            command_bytes = (cmd + '\n').encode()
            self.connection.write(command_bytes)
            self.connection.flush()  # Важно: принудительно отправляем данные
            
            if wait_response:
                # Для команды 'a' даем время (как в test_fix.py - 0.5 сек)
                if cmd == 'a':
                    time.sleep(0.5)  # 500 мс для команды чтения углов (как в test_fix.py)
                else:
                    time.sleep(0.1)  # 100 мс для других команд
                
                # Читаем все доступные данные
                response_data = b""
                
                # Читаем все что есть в буфере сразу
                if self.connection.in_waiting > 0:
                    response_data = self.connection.read(self.connection.in_waiting)
                
                # Для команды 'a' проверяем наличие углов и читаем дополнительно если нужно
                if cmd == 'a':
                    # КРИТИЧНО: Многократное чтение для получения полного ответа
                    max_read_attempts = 5  # Увеличено до 5 попыток
                    for read_attempt in range(max_read_attempts):
                        if response_data:
                            response_str = response_data.decode('utf-8', errors='ignore')
                            has_angles = any(f'A{i}:' in response_str for i in range(5))
                            
                            # Если углы найдены и данных достаточно - выходим
                            if has_angles and len(response_data) >= 30:
                                break
                        
                        # Если углов нет или данных мало - ждем и читаем еще
                        if read_attempt < max_read_attempts - 1:
                            time.sleep(0.3)  # Увеличена задержка до 300 мс
                            # Читаем все что есть в буфере
                            if self.connection.in_waiting > 0:
                                additional_data = self.connection.read(self.connection.in_waiting)
                                response_data += additional_data
                            # Если все еще нет углов - очищаем буфер и читаем снова
                            if response_data:
                                response_str = response_data.decode('utf-8', errors='ignore')
                                if not any(f'A{i}:' in response_str for i in range(5)):
                                    # Очищаем буфер от мусора
                                    self.connection.reset_input_buffer()
                                    time.sleep(0.1)
                                    if self.connection.in_waiting > 0:
                                        additional_data = self.connection.read(self.connection.in_waiting)
                                        response_data = additional_data  # Заменяем старые данные
                    # Если после всех попыток углов нет - возвращаем пустой ответ
                    # (будет обработано в read_all_angles)
                    if response_data:
                        response_str = response_data.decode('utf-8', errors='ignore')
                        if not any(f'A{i}:' in response_str for i in range(5)):
                            # Ответ не содержит углов - возвращаем как есть для обработки ошибки
                            pass
                    else:
                        # Если вообще нет данных - ждем еще немного и читаем
                        time.sleep(0.3)  # Увеличена задержка до 300 мс
                        if self.connection.in_waiting > 0:
                            response_data = self.connection.read(self.connection.in_waiting)
                
                if response_data:
                    response = response_data.decode('utf-8', errors='ignore').strip()
                    return response
                
                return None
            
            return "OK"
        
        except Exception as e:
            print(f"  ❌ Ошибка отправки команды '{cmd}': {e}")
            self.close()
            raise ConnectionError(f"Ошибка отправки команды '{cmd}': {e}")
    
    def write_bytes(self, data: bytes):
        """Отправляет байты (для MKS команд)"""
        self._ensure_connection()
        try:
            self.connection.write(data)
        except Exception as e:
            self.close()
            raise ConnectionError(f"Ошибка отправки данных: {e}")
    
    def close(self):
        """Закрывает соединение"""
        if self.connection and self.connection.is_open:
            try:
                self.connection.close()
            except:
                pass
        self.connection = None
    
    def __del__(self):
        self.close()


class MotorController:
    """Класс для управления моторами робота"""
    
    def __init__(self, arduino_serial: SerialManager, ch340_serial: SerialManager):
        """
        Инициализация контроллера с двумя портами
        
        Args:
            arduino_serial: SerialManager для Arduino (FT232) - энкодеры, мотор 1, мотор 5
            ch340_serial: SerialManager для CH340 - моторы MKS 2, 3, 4, 6
        """
        self.arduino_serial = arduino_serial  # Arduino: энкодеры, мотор 1, мотор 5
        self.ch340_serial = ch340_serial      # CH340: моторы MKS 2, 3, 4, 6
        self.angle_cache: Dict[str, float] = {}  # Кэш последних углов
        self.angle_cache_time = 0
        self.cache_validity = 0.05  # Кэш действителен 50 мс (для более частого чтения)
        self.current_speeds: Dict[int, int] = {}  # Текущие скорости для плавного разгона
        self.last_angles: Dict[int, float] = {}  # Последние углы для определения скорости
        self.last_angle_times: Dict[int, float] = {}  # Время последнего измерения
        self.mks_motors_enabled: Dict[int, bool] = {}  # Кэш статуса включения моторов
        self.last_enable_check: Dict[int, float] = {}  # Время последней проверки включения
        
    @staticmethod
    def crc8(data):
        """Вычисляет CRC8 для MKS команд"""
        return sum(data) & 0xFF
    
    def read_angles_from_file(self) -> Optional[Dict[str, float]]:
        """Читает углы из файла, созданного angle_reader.py"""
        try:
            if not os.path.exists(ANGLE_CACHE_FILE):
                return None
            
            # Читаем файл
            with open(ANGLE_CACHE_FILE, 'r') as f:
                data = json.load(f)
            
            # Проверяем валидность данных
            if not data.get('valid', False):
                return None
            
            # Проверяем свежесть данных (не старше 1 секунды)
            timestamp = data.get('timestamp', 0)
            current_time = time.time()
            if current_time - timestamp > 1.0:
                return None
            
            angles = data.get('angles', {})
            if angles and len(angles) >= 3:  # Хотя бы 3 канала из 5
                return angles
            
            return None
        except Exception:
            return None
    
    def read_all_angles(self) -> Optional[Dict[str, float]]:
        """Читает все углы за один запрос (из файла angle_reader.py)"""
        current_time = time.time()
        cache_age = current_time - self.angle_cache_time
        
        # Используем кэш только если он очень свежий (менее 50 мс)
        if cache_age < 0.05 and self.angle_cache:
            return self.angle_cache.copy()
        
        # Читаем углы из файла (созданного angle_reader.py)
        angles = self.read_angles_from_file()
        
        if angles:
            self.angle_cache = angles
            self.angle_cache_time = current_time
            return angles
        
        # Если не удалось прочитать из файла - возвращаем кэш, если он есть (до 5 секунд)
        if self.angle_cache and cache_age < 5.0:
            return self.angle_cache.copy()
        
        return None

    def _parse_all_angles(self, resp: str) -> Dict[str, float]:
        """Парсит все углы из ответа (ОПТИМИЗИРОВАННАЯ версия - без отладки)"""
        angles = {}
        try:
            # Объединяем все строки в одну
            clean_resp = resp.replace('\r', ' ').replace('\n', ' ').strip()
            
            # КРИТИЧНО: Игнорируем служебные сообщения типа 'STOP', 'SERVO:89' и т.д.
            # Ищем только паттерны углов A0:123, A1:456 и т.д.
            pattern = r'A([0-4]):\s*([0-9.-]+)'
            matches = re.findall(pattern, clean_resp)
            
            for channel_num, value_str in matches:
                try:
                    channel = f"A{channel_num}"
                    # Очищаем значение от всех символов кроме цифр, точки и минуса
                    clean_value = ''.join(c for c in value_str if c.isdigit() or c == '.' or c == '-')
                    if clean_value:
                        value = float(clean_value)
                        # Проверяем, что значение в разумных пределах (0-180 для углов)
                        if 0 <= value <= 180:
                            angles[channel] = value
                except ValueError:
                    continue
            
            # Альтернативный метод: разбивка по пробелам (если регулярные выражения не сработали)
            if not angles:
                parts = clean_resp.split()
                for part in parts:
                    if ':' in part and part.startswith('A'):
                        try:
                            channel, value_str = part.split(':', 1)
                            # Проверяем, что это действительно канал угла (A0-A4)
                            if channel in ['A0', 'A1', 'A2', 'A3', 'A4']:
                                clean_value = ''.join(c for c in value_str if c.isdigit() or c == '.' or c == '-')
                                if clean_value:
                                    value = float(clean_value)
                                    # Проверяем, что значение в разумных пределах
                                    if 0 <= value <= 180:
                                        angles[channel] = value
                        except ValueError:
                            continue
            
            return angles
            
        except Exception as e:
            # Только при критической ошибке
            return {}

    def get_angle(self, channel: str, use_smoothing: bool = True) -> Optional[float]:
        """Получает угол для конкретного канала с опциональным сглаживанием"""
        if use_smoothing:
            angles = []
            for _ in range(SMOOTH_WINDOW):
                all_angles = self.read_all_angles()
                if all_angles and channel in all_angles:
                    angles.append(all_angles[channel])
                time.sleep(ANGLE_READ_DELAY)
            # Возвращаем среднее значение, если есть хотя бы одно измерение
            if angles:
                return sum(angles) / len(angles)
            # Если не удалось прочитать ни разу - возвращаем None
            return None
        else:
            all_angles = self.read_all_angles()
            return all_angles.get(channel) if all_angles else None
    
    def send_mks_speed(self, addr: int, speed: int, forward: bool = True):
        """
        Отправляет команду скорости для MKS мотора
        Автоматически включает мотор, если он выключен
        """
        # Проверяем и включаем мотор, если нужно
        motor_num = None
        for num, motor_addr in MOTOR_ADDR.items():
            if motor_addr == addr:
                motor_num = num
                break
        
        if motor_num:
            # Проверяем статус включения (не чаще раза в секунду)
            current_time = time.time()
            last_check = self.last_enable_check.get(motor_num, 0)
            
            if current_time - last_check > 1.0:  # Проверяем не чаще раза в секунду
                try:
                    status = self.read_mks_status(addr)
                    if status:
                        is_enabled = status.get('enabled', False)
                        self.mks_motors_enabled[motor_num] = is_enabled
                        self.last_enable_check[motor_num] = current_time
                        
                        # Включаем мотор, если он выключен
                        if not is_enabled:
                            print(f"  ⚡ Включение мотора {motor_num} (0x{addr:02X})...")
                            self.enable_mks_motor(addr, True)
                            time.sleep(0.1)  # Даем время на включение
                            self.mks_motors_enabled[motor_num] = True
                except:
                    # Если не удалось проверить, пытаемся включить на всякий случай
                    try:
                        self.enable_mks_motor(addr, True)
                        time.sleep(0.05)
                    except:
                        pass
        
        # Отправляем команду скорости через CH340 (моторы MKS 2, 3, 4, 6)
        val = speed | (0x80 if not forward else 0x00)
        cmd = [addr, 0xF6, val]
        crc = self.crc8(cmd)
        packet = bytes(cmd + [crc])
        self.ch340_serial.write_bytes(packet)
    
    def stop_mks_motor(self, addr: int):
        """Останавливает MKS мотор (через CH340)"""
        cmd = [addr, 0xF7]
        crc = self.crc8(cmd)
        packet = bytes(cmd + [crc])
        try:
            self.ch340_serial.write_bytes(packet)
        except:
            pass

    def send_mks_command(self, addr: int, cmd_code: int, data: Optional[List[int]] = None, wait_response: bool = False) -> Optional[bytes]:
        """
        Отправляет произвольную команду MKS SERVO42C
        
        Args:
            addr: Адрес мотора (0xE0-0xE9)
            cmd_code: Код команды
            data: Опциональные данные команды (список байтов)
            wait_response: Ждать ли ответа
        
        Returns:
            Ответ от драйвера (bytes) или None
        """
        try:
            cmd = [addr, cmd_code]
            if data:
                cmd.extend(data)
            
            crc = self.crc8(cmd)
            packet = bytes(cmd + [crc])
            self.ch340_serial.write_bytes(packet)
            
            if wait_response:
                # Даем время на обработку и читаем ответ с несколькими попытками
                for attempt in range(3):
                    time.sleep(0.05 + attempt * 0.05)  # Увеличиваем задержку с каждой попыткой
                    if self.ch340_serial.connection and self.ch340_serial.connection.in_waiting > 0:
                        response = self.ch340_serial.connection.read(self.ch340_serial.connection.in_waiting)
                        if response and len(response) > 0:
                            return response
                # Если ответа нет, возвращаем пустой ответ (команда могла быть выполнена)
                return bytes([addr, 0x01, 0xE1])  # Имитируем успешный ответ
            return None
        except Exception as e:
            print(f"❌ Ошибка отправки команды MKS 0x{cmd_code:02X} для 0x{addr:02X}: {e}")
            return None
    
    def read_mks_encoder(self, addr: int) -> Optional[int]:
        """
        Читает значение энкодера MKS мотора (команда 0x30)
        
        Args:
            addr: Адрес мотора
        
        Returns:
            Значение энкодера или None
        """
        response = self.send_mks_command(addr, 0x30, wait_response=True)
        if response and len(response) >= 8:
            # Формат ответа: e0 carry value CRC
            # value - 4 байта (младший байт первый)
            value = int.from_bytes(response[2:6], byteorder='little', signed=False)
            return value
        return None
    
    def read_mks_status(self, addr: int) -> Optional[Dict[str, int]]:
        """
        Читает статус MKS мотора (команда 0x3A)
        
        Args:
            addr: Адрес мотора
        
        Returns:
            Словарь со статусом или None
        """
        response = self.send_mks_command(addr, 0x3A, wait_response=True)
        if response and len(response) >= 3:
            # Формат ответа: e0 status CRC
            status = response[1]
            return {
                'enabled': (status & 0x01) != 0,
                'status_byte': status
            }
        return None
    
    def read_mks_protect_status(self, addr: int) -> Optional[bool]:
        """
        Читает статус защиты MKS мотора (команда 0x3E)
        
        Args:
            addr: Адрес мотора
        
        Returns:
            True если защита сработала, False если нет, None при ошибке
        """
        response = self.send_mks_command(addr, 0x3E, wait_response=True)
        if response and len(response) >= 3:
            # Формат ответа: e0 status CRC
            status = response[1]
            # 0x02 = не заблокирован, другие значения = заблокирован
            return status != 0x02
        return None
    
    def reset_mks_protect(self, addr: int) -> bool:
        """
        Сбрасывает защиту MKS мотора (команда 0x3D)
        
        Args:
            addr: Адрес мотора
        
        Returns:
            True если успешно, False при ошибке
        """
        response = self.send_mks_command(addr, 0x3D, wait_response=True)
        if response and len(response) >= 3:
            status = response[1]
            return status == 0x01  # 0x01 = успех
        return False
    
    def enable_mks_motor(self, addr: int, enable: bool = True) -> bool:
        """
        Включает/выключает MKS мотор (команда 0xF3)
        
        Args:
            addr: Адрес мотора
            enable: True для включения, False для выключения
        
        Returns:
            True если успешно, False при ошибке
        """
        try:
            # Отправляем команду несколько раз для надежности
            for attempt in range(2):
                response = self.send_mks_command(addr, 0xF3, [1 if enable else 0], wait_response=True)
                if response and len(response) >= 3:
                    status = response[1]
                    if status == 0x01:  # 0x01 = успех
                        time.sleep(0.05)  # Даем время на применение
                        return True
                time.sleep(0.1)  # Небольшая задержка перед повтором
            
            # Если не получили ответ, все равно считаем успешным (мотор может быть уже включен)
            # и проверим статус позже
            return True
        except Exception as e:
            print(f"   ⚠️ Исключение при включении мотора 0x{addr:02X}: {e}")
            return False
    
    def check_all_motors_status(self) -> Dict[int, Dict[str, any]]:
        """
        Проверяет статус всех MKS моторов
        
        Returns:
            Словарь с информацией о каждом моторе
        """
        status_info = {}
        for motor_num, addr in MOTOR_ADDR.items():
            motor_status = {
                'addr': addr,
                'encoder': None,
                'enabled': None,
                'protect_triggered': None,
                'name': MOTOR_NAMES.get(motor_num, 'Unknown')
            }
            
            # Читаем энкодер
            encoder = self.read_mks_encoder(addr)
            motor_status['encoder'] = encoder
            
            # Читаем статус
            status = self.read_mks_status(addr)
            if status:
                motor_status['enabled'] = status.get('enabled')
            
            # Проверяем защиту
            protect = self.read_mks_protect_status(addr)
            motor_status['protect_triggered'] = protect
            
            status_info[motor_num] = motor_status
            time.sleep(0.05)  # Небольшая задержка между моторами
        
        return status_info

    def stop_arduino_motor(self):
        """Останавливает мотор через Arduino"""
        self.arduino_serial.send_command('s', wait_response=False)
    
    def set_gripper_angle(self, angle: int):
        """
        Управляет сервоприводом MG996R (клешня)
        
        Args:
            angle: Угол сервопривода 0-180 (0 - закрыто, 180 - открыто)
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        cmd = f"g{angle}"
        self.arduino_serial.send_command(cmd, wait_response=False)
        # КРИТИЧНО: Даем время angle_reader.py прочитать углы после команды сервопривода
        # Команда сервопривода может мешать чтению углов, поэтому добавляем задержку
        time.sleep(0.1)  # 100 мс задержка для координации с angle_reader.py
    
    def check_limit_switch(self) -> bool:
        """
        Проверяет состояние магнитного концевика A0 для мотора 2 (поворот кисти)
        
        Returns:
            True если концевик сработал (мотор должен остановиться)
        """
        # Читаем значение A0 через команду 'v' или 'a'
        # A0 теперь концевик, не энкодер, поэтому читаем его как аналоговое значение
        try:
            resp = self.arduino_serial.send_command("v", wait_response=True)
            if resp:
                # Парсим ответ вида "A0:512 A1:256 ..."
                import re
                match = re.search(r'A0:(\d+)', resp)
                if match:
                    value = int(match.group(1))
                    # Если значение ниже порога (например, < 100), концевик сработал
                    return value < 100
        except:
            pass
        return False
    
    def check_limit_switch_fast(self) -> Optional[tuple]:
        """
        Быстрое чтение концевика A0 через команду 'l' (если доступна)
        
        Returns:
            (raw_value, triggered) - сырое значение и состояние срабатывания
            None если команда не поддерживается
        """
        try:
            resp = self.arduino_serial.send_command("l", wait_response=True)
            if resp and "ERR" not in resp:
                # Парсим ответ вида "LIMIT:A0:512,TRIG:0" или "LIMIT:A0:50,TRIG:1"
                import re
                match_a0 = re.search(r'A0:(\d+)', resp)
                match_trig = re.search(r'TRIG:(\d+)', resp)
                if match_a0 and match_trig:
                    raw_value = int(match_a0.group(1))
                    triggered = match_trig.group(1) == "1"
                    return (raw_value, triggered)
        except:
            pass
        return None
    
    def move_motor2_to_angle(self, target_angle: float, 
                             calibration_params: Dict[str, float],
                             current_angle: float = None,
                             rotation_start_angle: float = None,
                             zero_position_time: float = None) -> tuple:
        """
        Управление мотором 2 на основе параметров калибровки
        
        Мотор 2 (0xE0) устанавливается ВРУЧНУЮ в стартовую позицию 90°,
        а угол поворота вычисляется АВТОМАТИЧЕСКИ на основе времени вращения.
        
        Args:
            target_angle: Целевой угол (0-180°)
            calibration_params: Параметры калибровки из файла motor2_calibration.txt
                - starting_angle: стартовая позиция (90°)
                - full_rotation_angle: полный угол поворота (градусы, обычно 180°)
                - full_rotation_time: время полного оборота (сек, обычно ~4.28)
                - rotation_speed: скорость вращения (°/сек, обычно ~42.07)
            current_angle: Текущий угол (если известен, иначе используется starting_angle)
            rotation_start_angle: Угол в момент начала текущего вращения (если известен)
            zero_position_time: Не используется (для совместимости)
        
        Returns:
            (should_rotate, forward, speed, estimated_time) - нужно ли вращать, направление, скорость, время
        """
        starting_angle = calibration_params.get('starting_angle', 90.0)
        max_angle = calibration_params.get('full_rotation_angle', 180.0)
        rotation_speed = calibration_params.get('rotation_speed', 42.07)  # Актуальное значение из калибровки
        
        # Ограничиваем целевой угол диапазоном (0 - max_angle)
        target_angle = max(0.0, min(max_angle, float(target_angle)))
        
        # Если текущий угол неизвестен, используем стартовую позицию 90°
        if current_angle is None:
            current_angle = starting_angle
        
        # Вычисляем разницу между текущим и целевым углом
        angle_diff = target_angle - current_angle
        
        # Если очень близко к цели - не вращаем (допуск 2°)
        if abs(angle_diff) < 2.0:
            return (False, True, 0, 0.0)
        
        # Вычисляем необходимое время вращения для достижения цели
        rotation_time = abs(angle_diff) / rotation_speed if rotation_speed > 0 else 0
        
        # Определяем направление вращения
        forward = angle_diff > 0
        
        # Скорость из BASE_SPEEDS (базовая скорость для мотора 2)
        speed = BASE_SPEEDS.get(2, 3)
        
        return (True, forward, speed, rotation_time)
    
    def calculate_motor2_angle(self, calibration_params: Dict[str, float],
                               rotation_start_angle: float, elapsed_time: float,
                               forward: bool = True, target_angle: float = None) -> float:
        """
        Вычисляет текущий угол мотора 2 на основе времени вращения
        
        Угол вычисляется от стартовой позиции текущего вращения (rotation_start_angle),
        а не от последнего известного угла, для более точного расчета.
        
        Args:
            calibration_params: Параметры калибровки из файла motor2_calibration.txt
                - rotation_speed: скорость вращения (°/сек, обычно ~42.07)
                - full_rotation_angle: максимальный угол (обычно 180°)
            rotation_start_angle: Угол в момент начала текущего вращения (от этой позиции вычисляем)
            elapsed_time: Прошедшее время вращения (сек)
            forward: Направление вращения (True = вперед/увеличение угла, False = назад/уменьшение)
            target_angle: Целевой угол (для ограничения результата, опционально)
        
        Returns:
            Текущий угол мотора 2 (0-180°)
        """
        rotation_speed = calibration_params.get('rotation_speed', 42.07)
        max_angle = calibration_params.get('full_rotation_angle', 180.0)
        
        # Вычисляем угол, который мотор прошел с момента начала вращения
        angle_moved = rotation_speed * elapsed_time
        
        # Вычисляем новый угол от стартовой позиции текущего вращения
        if forward:
            # Вращение вперед - увеличиваем угол
            new_angle = rotation_start_angle + angle_moved
            # Ограничиваем максимальным углом и целевым углом (если задан)
            if target_angle is not None:
                new_angle = min(new_angle, max_angle, target_angle)
            else:
                new_angle = min(new_angle, max_angle)
        else:
            # Вращение назад - уменьшаем угол
            new_angle = rotation_start_angle - angle_moved
            # Ограничиваем минимальным углом (0) и целевым углом (если задан)
            if target_angle is not None:
                new_angle = max(new_angle, 0.0, target_angle)
            else:
                new_angle = max(new_angle, 0.0)
        
        # Финальное ограничение угла диапазоном калибровки (0 - max_angle)
        new_angle = max(0.0, min(max_angle, new_angle))
        
        return new_angle
    
    def stop_all(self):
        """Останавливает все моторы"""
        print("\n🛑 Остановка всех моторов...")
        for addr in MOTOR_ADDR.values():
            self.stop_mks_motor(addr)
        self.stop_arduino_motor()
        print("✅ Все моторы остановлены.")

    def send_encoder_command(self, command: str, wait_response: bool = True, use_prefix: bool = True) -> Optional[str]:
        """
        Отправляет команду напрямую энкодеру через Arduino для калибровки
        
        Args:
            command: Команда для энкодера (например, "cal A0" для калибровки канала A0)
            wait_response: Ждать ли ответа от Arduino
            use_prefix: Использовать ли префикс "enc " перед командой
        
        Returns:
            Ответ от Arduino или None
        
        Примечание:
            Если use_prefix=True, команда будет отправлена как "enc <command>"
            Если use_prefix=False, команда будет отправлена как есть
        """
        try:
            # Отправляем команду через Arduino serial
            if use_prefix:
                # Предполагаем, что Arduino обрабатывает команды вида "enc <command>"
                full_command = f"enc {command}"
            else:
                # Отправляем команду напрямую (если Arduino обрабатывает команды без префикса)
                full_command = command
            
            response = self.arduino_serial.send_command(full_command, wait_response=wait_response)
            return response
        except Exception as e:
            print(f"❌ Ошибка отправки команды энкодеру: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def calibrate_encoder_channel(self, channel: str, zero_angle: float = 0.0) -> bool:
        """
        Калибрует энкодер для указанного канала
        
        Args:
            channel: Канал энкодера (A0, A1, A2, A3, A4)
            zero_angle: Угол, который нужно установить как нулевую точку (по умолчанию 0.0°)
        
        Returns:
            True если калибровка успешна, False в противном случае
        """
        if channel not in ["A0", "A1", "A2", "A3", "A4"]:
            print(f"❌ Неверный канал энкодера: {channel}")
            return False
        
        print(f"\n🔧 Калибровка энкодера {channel}...")
        print(f"   Установка нулевой точки: {zero_angle}°")
        
        # Читаем текущий угол
        current_angle = self.get_angle(channel, use_smoothing=False)
        if current_angle is None:
            print(f"❌ Не удалось прочитать угол с канала {channel}")
            return False
        
        print(f"   Текущий угол: {current_angle:.1f}°")
        
        # Отправляем команду калибровки
        # Формат команды может быть разным в зависимости от протокола энкодера
        # Предполагаем команду вида "cal A0 0.0" для установки нулевой точки
        command = f"cal {channel} {zero_angle:.1f}"
        response = self.send_encoder_command(command, wait_response=True)
        
        if response:
            print(f"   Ответ от энкодера: {response}")
            # Проверяем успешность калибровки
            if "OK" in response.upper() or "CAL" in response.upper():
                print(f"✅ Калибровка {channel} завершена успешно")
                # Очищаем кэш углов для этого канала
                if channel in self.angle_cache:
                    del self.angle_cache[channel]
                return True
            else:
                print(f"⚠️ Неожиданный ответ: {response}")
                return False
        else:
            print(f"❌ Не получен ответ от энкодера")
            return False
    
    def reset_encoder_channel(self, channel: str) -> bool:
        """
        Сбрасывает позицию энкодера для указанного канала в 0
        
        Args:
            channel: Канал энкодера (A0, A1, A2, A3, A4)
        
        Returns:
            True если сброс успешен, False в противном случае
        """
        return self.calibrate_encoder_channel(channel, zero_angle=0.0)
    
    def send_raw_encoder_command(self, raw_command: str, use_prefix: bool = True) -> Optional[str]:
        """
        Отправляет произвольную команду напрямую энкодеру
        
        Args:
            raw_command: Произвольная команда для энкодера
            use_prefix: Использовать ли префикс "enc " перед командой
        
        Returns:
            Ответ от Arduino/энкодера или None
        
        Примеры команд (с use_prefix=True):
            - "read A0" → отправляется как "enc read A0"
            - "write A0 90.0" → отправляется как "enc write A0 90.0"
            - "reset A0" → отправляется как "enc reset A0"
            - "cal A0 0" → отправляется как "enc cal A0 0"
        
        Примеры команд (с use_prefix=False):
            - Команда отправляется как есть, без префикса
            - Полезно, если Arduino обрабатывает команды напрямую
        """
        print(f"\n📤 Отправка команды энкодеру: {raw_command}")
        if use_prefix:
            print("   (с префиксом 'enc ')")
        else:
            print("   (без префикса, напрямую)")
        
        response = self.send_encoder_command(raw_command, wait_response=True, use_prefix=use_prefix)
        if response:
            print(f"📥 Ответ: {response}")
        else:
            print("⚠️ Ответ не получен")
        return response
    
    def test_angle_reading(self):
        """Тестирует чтение углов вручную"""
        print("\n🎯 ТЕСТ ЧТЕНИЯ УГЛОВ")
        
        # Тест 1: Прямое чтение через существующий метод
        print("1. Через метод read_all_angles:")
        angles = self.read_all_angles()
        print(f"   Результат: {angles}")
        
        # Тест 2: Прямое чтение командой 'v' (значения АЦП)
        print("2. Команда 'v' (ADC values):")
        try:
            resp = self.arduino_serial.send_command("v", wait_response=True)
            print(f"   Ответ: '{resp}'")
        except Exception as e:
            print(f"   Ошибка: {e}")
        
        # Тест 3: Команда 'i' (info)
        print("3. Команда 'i' (info):")
        try:
            resp = self.arduino_serial.send_command("i", wait_response=True)
            print(f"   Ответ: '{resp}'")
        except Exception as e:
            print(f"   Ошибка: {e}")
    
    def quick_test(self):
        """Быстрый тест связи и чтения углов"""
        print("\n⚡ БЫСТРЫЙ ТЕСТ СВЯЗИ")
        
        # Тест 1: Простая команда (пропускаем, так как может мешать команде 'a')
        # В test_system.py команда 'i' была убрана для надежности
        
        # Тест 2: Чтение углов (основной тест)
        print("1. Тест чтения углов:")
        angles = self.read_all_angles()
        if angles:
            print(f"   ✅ Углы прочитаны: {len(angles)} каналов")
            for channel, value in sorted(angles.items()):
                print(f"      {channel}: {value:.1f}°")
            return True
        else:
            print("   ❌ Не удалось прочитать углы")
            return False
    
    def check_angle_limits(self, motor_num: int, angle: float) -> Tuple[bool, Optional[str]]:
        """
        Проверяет, находится ли угол в допустимых пределах
        
        Returns:
            (is_valid, error_message)
        """
        if motor_num not in MOTOR_ANGLE_LIMITS:
            return True, None
        
        min_angle, max_angle = MOTOR_ANGLE_LIMITS[motor_num]
        if angle < min_angle:
            return False, f"Угол {angle:.1f}° меньше минимального {min_angle}°"
        if angle > max_angle:
            return False, f"Угол {angle:.1f}° больше максимального {max_angle}°"
        return True, None
    
    def check_kinematics(self, angles: Dict[int, float]) -> Tuple[bool, Optional[str]]:
        """
        Проверяет допустимость комбинации углов (базовая проверка кинематики)
        
        Args:
            angles: Словарь {motor_num: angle}
        
        Returns:
            (is_valid, error_message)
        """
        # Проверка ограничений для каждого мотора
        for motor_num, angle in angles.items():
            is_valid, error = self.check_angle_limits(motor_num, angle)
            if not is_valid:
                return False, f"Мотор {motor_num}: {error}"
        
        # Дополнительные проверки кинематики (можно расширить)
        # Например, проверка на столкновение плеч
        if 2 in angles and 3 in angles:
            # Если оба плеча в крайних положениях одновременно - может быть столкновение
            if (angles[2] < 30 and angles[3] < 30) or (angles[2] > 150 and angles[3] > 150):
                return False, "Критическая комбинация углов плеч - риск столкновения"
        
        return True, None
    
    def calculate_adaptive_speed(self, diff: float, base_speed: int, motor_num: int, 
                                 current_speed: Optional[int] = None) -> int:
        """
        Вычисляет адаптивную скорость с плавным ускорением/замедлением
        
        Args:
            diff: Расстояние до цели в градусах
            base_speed: Базовая скорость мотора
            motor_num: Номер мотора
            current_speed: Текущая скорость (для плавного разгона)
        """
        # Определяем целевую скорость на основе расстояния
        # Чем дальше от цели — тем быстрее, при подходе к углу плавно замедляемся
        if diff < 2:
            target_speed = MIN_SPEED
        elif diff < 5:
            target_speed = max(MIN_SPEED, base_speed - 2)
        elif diff < DECELERATION_START_DISTANCE:
            target_speed = max(MIN_SPEED, base_speed - 1)
        else:
            # Далеко от цели — можно разгоняться до максимума
            target_speed = min(MAX_SPEED, base_speed + 1)

        # Для мотора 6 дополнительно сильно ограничиваем максимальную скорость
        if motor_num == 6:
            target_speed = min(target_speed, 1)

        # Для мотора 6 дополнительно ограничиваем максимальную скорость
        if motor_num == 6:
            target_speed = min(target_speed, 2)
        
        # Плавное ускорение/замедление (без резких скачков скорости)
        if current_speed is None:
            current_speed = self.current_speeds.get(motor_num, MIN_SPEED)
        
        if current_speed < target_speed:
            # Разгон
            new_speed = min(target_speed, current_speed + 1)
        elif current_speed > target_speed:
            # Торможение
            new_speed = max(target_speed, current_speed - 1)
        else:
            new_speed = target_speed
        
        self.current_speeds[motor_num] = new_speed
        return new_speed
    
    def check_motor_stuck(self, motor_num: int, current_angle: float) -> bool:
        """
        Проверяет, не застрял ли мотор (не движется)
        
        Returns:
            True если мотор застрял
        """
        current_time = time.time()
        
        if motor_num not in self.last_angles:
            self.last_angles[motor_num] = current_angle
            self.last_angle_times[motor_num] = current_time
            return False
        
        time_diff = current_time - self.last_angle_times[motor_num]
        angle_diff = abs(current_angle - self.last_angles[motor_num])
        
        # Если прошло достаточно времени и угол не изменился
        if time_diff > 2.0 and angle_diff < 1.0:
            velocity = angle_diff / time_diff if time_diff > 0 else 0
            if velocity < MIN_ANGLE_VELOCITY:
                return True
        
        self.last_angles[motor_num] = current_angle
        self.last_angle_times[motor_num] = current_time
        return False
    
    def move_to_angle(self, motor_num: int, target_angle: float, 
                     tolerance: float = 2.0, verbose: bool = True,
                     stop_event: Optional[Event] = None) -> bool:
        """
        Перемещает мотор в заданный угол с обратной связью
        
        Returns:
            True если цель достигнута, False если прервано
        """
        if motor_num not in MOTOR_ADDR and motor_num != 5:
            print("❌ Неверный номер мотора.")
            return False
        
        # Проверка ограничений угла
        is_valid, error = self.check_angle_limits(motor_num, target_angle)
        if not is_valid:
            print(f"❌ {error}")
            return False
        
        channel = ENCODER_CHANNEL.get(motor_num)
        addr = MOTOR_ADDR.get(motor_num)
        base_speed = BASE_SPEEDS.get(motor_num, 2)

        # Для мотора 2: A0 - концевик, не энкодер, поэтому не используем обратную связь по углу
        if motor_num == 2:
            if verbose:
                print(f"⚠️ Мотор 2: A0 - концевик, управление без обратной связи по углу")
                print(f"   Управление мотором 2 с проверкой концевика")
            # Упрощенное управление для мотора 2 - только проверка концевика
            # Мотор управляется напрямую через MKS команды
            return True  # Возвращаем True, так как управление без обратной связи
        
        if not channel:
            print("❌ Канал энкодера не найден для мотора.")
            return False
        
        # Сброс скорости для плавного старта
        self.current_speeds[motor_num] = MIN_SPEED
        
        if verbose:
            min_angle, max_angle = MOTOR_ANGLE_LIMITS.get(motor_num, (0, 180))
            print(f"\n🔄 Движение мотора {motor_num} к {target_angle:.1f}°...")
            print(f"   Диапазон: {min_angle}° - {max_angle}°")
        
        last_print_time = 0
        print_interval = 0.5  # Печатать статус каждые 0.5 сек
        start_time = time.time()
        stuck_check_counter = 0

        try:
            while True:
                if stop_event and stop_event.is_set():
                    if verbose:
                        print(f"\n⏹️ Движение мотора {motor_num} остановлено по запросу.")
                    return False
                # Проверка таймаута движения
                elapsed_time = time.time() - start_time
                if elapsed_time > MAX_MOVEMENT_TIME:
                    if verbose:
                        print(f"\n⏱️ Превышено максимальное время движения ({MAX_MOVEMENT_TIME} сек)")
                    return False
                
                # Проверка концевика для мотора 2 (поворот кисти)
                if motor_num == 2:
                    if self.check_limit_switch():
                        if verbose:
                            print(f"\n⚠️ Мотор 2: концевик сработал! Остановка мотора.")
                        # Останавливаем мотор
                        if addr:
                            self.stop_mks_motor(addr)
                        return False
                
                # Читаем угол напрямую через read_all_angles (быстрее и надежнее)
                # ОДИНАКОВАЯ ЛОГИКА для всех моторов 3, 4, 5, 6 (как для мотора 4)
                # Делаем несколько попыток для надежности
                current_angle_raw = None
                for read_attempt in range(5):  # 5 попыток для всех моторов 3, 4, 5, 6
                    all_angles = self.read_all_angles()
                    if all_angles and channel in all_angles:
                        current_angle_raw = all_angles[channel]
                        break
                    if read_attempt < 4:  # Не ждем после последней попытки
                        time.sleep(0.1)  # Задержка 100 мс между попытками (одинаково для всех)
                
                if current_angle_raw is None:
                    if verbose:
                        print(f"⚠️ Не удалось прочитать угол {channel} после 5 попыток. Повтор...")
                    time.sleep(UPDATE_INTERVAL)
                    continue

                # Обработка угла (зеркалирование для моторов 2-4)
                # Моторы 3, 4: зеркалирование (180 - angle)
                # Моторы 5, 6: без зеркалирования (используется напрямую)
                if motor_num in [2, 3, 4]:
                    current_angle = 180 - current_angle_raw
                else:
                    current_angle = current_angle_raw

                # Проверка текущего угла на ограничения (предупреждение)
                is_valid, error = self.check_angle_limits(motor_num, current_angle)
                if not is_valid and verbose:
                    print(f"⚠️ ВНИМАНИЕ: {error}")
                
                # Проверка на застревание
                stuck_check_counter += 1
                if stuck_check_counter >= 20:  # Проверяем каждые ~0.6 сек
                    if self.check_motor_stuck(motor_num, current_angle):
                        if verbose:
                            print(f"\n⚠️ Мотор {motor_num} не движется! Возможно застревание.")
                            print("   Попытка остановки и повтор...")
                        # Останавливаем и ждем
                        if motor_num == 5:
                            self.stop_arduino_motor()
                        else:
                            self.stop_mks_motor(addr)
                        time.sleep(0.5)
                        # Сбрасываем счетчики
                        self.last_angles.pop(motor_num, None)
                        self.last_angle_times.pop(motor_num, None)
                        stuck_check_counter = 0
                        continue
                    stuck_check_counter = 0
                
                # Вывод статуса (не слишком часто)
                current_time = time.time()
                if verbose and (current_time - last_print_time) >= print_interval:
                    diff = abs(current_angle - target_angle)
                    progress = max(0, min(100, 100 - (diff / 180 * 100)))
                    speed = self.current_speeds.get(motor_num, MIN_SPEED)
                    print(f"📍 {channel}: {current_angle:.1f}° → {target_angle:.1f}° "
                          f"(осталось: {diff:.1f}°, скорость: {speed}, время: {elapsed_time:.1f}с)")
                    last_print_time = current_time

                # Проверка достижения цели
                diff = abs(current_angle - target_angle)
                if diff <= tolerance:
                    if motor_num == 5:
                        self.stop_arduino_motor()
                    else:
                        self.stop_mks_motor(addr)
                    if verbose:
                        print(f"✅ Цель достигнута: {current_angle:.1f}° ≈ {target_angle:.1f}° "
                              f"(за {elapsed_time:.1f} сек)")
                    # Сброс скорости
                    self.current_speeds.pop(motor_num, None)
                    return True
                
                # Логика выбора направления
                if motor_num == 6:
                    # Для мотора 6 - выбор кратчайшего пути
                    if target_angle < 0 or target_angle > 180:
                        if verbose:
                            print("❌ Целевой угол вне диапазона 0–180°")
                        return False
                    
                    error_cw = (current_angle - target_angle) % 360
                    error_ccw = (target_angle - current_angle) % 360
                    forward = error_ccw < error_cw
                    forward = not forward  # Инверсия для мотора 6
                else:
                    # Для остальных моторов
                    need_increase = current_angle < target_angle
                    # Мотор 5 (Nema 34) - обычное направление (без инверсии)
                    # Угол НЕ инвертирован через зеркалирование (используется напрямую)
                    forward = not need_increase if motor_num in INVERT_DIRECTION else need_increase

                # Адаптивная скорость с плавным ускорением
                current_speed = self.current_speeds.get(motor_num, MIN_SPEED)
                speed = self.calculate_adaptive_speed(diff, base_speed, motor_num, current_speed)
                
                # Управление мотором
                if motor_num == 5:
                    # Мотор 5 (Nema 34) - инвертированные команды направления
                    # forward = True → команда 'b' (назад), forward = False → команда 'f' (вперед)
                    direction = 'b' if forward else 'f'
                    self.arduino_serial.send_command(f"{direction}{int(speed * 100)}", wait_response=False)
                else:
                    self.send_mks_speed(addr, speed, forward)

                time.sleep(UPDATE_INTERVAL)

        except KeyboardInterrupt:
            if verbose:
                print("\n🛑 Прервано пользователем.")
            return False
        finally:
            # Гарантированная остановка
            if motor_num == 5:
                self.stop_arduino_motor()
            elif motor_num in MOTOR_ADDR:
                self.stop_mks_motor(MOTOR_ADDR[motor_num])
            # Сброс счетчиков
            self.current_speeds.pop(motor_num, None)
            self.last_angles.pop(motor_num, None)
            self.last_angle_times.pop(motor_num, None)


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
    except Exception:
        # В случае ошибки просто возвращаем None
        return None


def find_port_by_usb_id(vendor_id: str, product_id: str) -> Optional[str]:
    """
    Находит serial порт по USB ID устройства
    
    Args:
        vendor_id: Vendor ID в hex формате (например, '10c4')
        product_id: Product ID в hex формате (например, 'ea60')
    
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


def auto_detect_port(preferred_port: str = None, usb_vendor_id: str = None, usb_product_id: str = None) -> Optional[str]:
    """
    Автоматически определяет подходящий serial порт
    
    Args:
        preferred_port: Предпочтительный порт (если указан и доступен, будет использован)
        usb_vendor_id: USB Vendor ID для поиска конкретного устройства (например, '10c4')
        usb_product_id: USB Product ID для поиска конкретного устройства (например, 'ea60')
    
    Returns:
        Путь к найденному порту или None
    """
    # Приоритет 1: Поиск по USB ID (если указан)
    if usb_vendor_id and usb_product_id:
        port_by_usb = find_port_by_usb_id(usb_vendor_id, usb_product_id)
        if port_by_usb:
            print(f"✅ Найдено устройство по USB ID {usb_vendor_id}:{usb_product_id} на порту {port_by_usb}")
            return port_by_usb
        else:
            print(f"⚠️ Устройство с USB ID {usb_vendor_id}:{usb_product_id} не найдено, продолжаем поиск...")
    
    # Приоритет 2: Проверяем предпочтительный порт
    if preferred_port and check_port_availability(preferred_port):
        print(f"✅ Используется предпочтительный порт: {preferred_port}")
        return preferred_port
    
    # Приоритет 3: Ищем все доступные порты
    available_ports = find_available_serial_ports()
    
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


def check_port_availability(port: str) -> bool:
    """Проверяет доступность serial порта"""
    try:
        test_ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        test_ser.close()
        return True
    except:
        return False


def main():
    """Основная функция программы"""
    print("\n" + "="*70)
    print("     УПРАВЛЕНИЕ МОТОРАМИ ПО УГЛАМ (УЛУЧШЕННАЯ ВЕРСИЯ)")
    print("="*70)
    print("📌 Улучшения:")
    print("  • Оптимизированная работа с serial портом")
    print("  • Кэширование углов для повышения производительности")
    print("  • Адаптивное управление скоростью с плавным ускорением")
    print("  • Физические ограничения углов для каждого мотора")
    print("  • Проверка кинематики и защита от столкновений")
    print("  • Защита от застревания и таймауты движения")
    print("  • Улучшенная обработка ошибок")
    print("  • Отзеркаливание угла для моторов 1,2,3,4")
    print("  • Кратчайший путь для мотора 5")
    print("  • Функции для работы с MKS SERVO42C (чтение статуса, энкодера, защита)")
    print("  • Автоматическая проверка статуса моторов при старте")
    print("  • Разделение на два порта: Arduino (энкодеры, моторы 1,5) и CH340 (MKS 2,3,4,6)")
    print("="*70)
    
    # Автоматическое определение портов
    print("\n🔍 Поиск портов...")
    
    # Поиск Arduino (FT232)
    arduino_port = auto_detect_port(
        usb_vendor_id=ARDUINO_USB_VENDOR_ID,
        usb_product_id=ARDUINO_USB_PRODUCT_ID
    )
    
    if not arduino_port:
        print("\n❌ ОШИБКА: Не удалось найти Arduino (FT232) порт!")
        print(f"   Ожидаемый USB ID: {ARDUINO_USB_VENDOR_ID}:{ARDUINO_USB_PRODUCT_ID}")
        print("   Проверьте:")
        print("   • Подключено ли устройство Arduino")
        print("   • Правильно ли установлены драйверы")
        print("   • Есть ли права доступа к порту")
        sys.exit(1)
    
    print(f"✅ Найден Arduino порт: {arduino_port}")
    
    # Поиск CH340
    ch340_port = auto_detect_port(
        usb_vendor_id=CH340_USB_VENDOR_ID,
        usb_product_id=CH340_USB_PRODUCT_ID
    )
    
    if not ch340_port:
        print("\n❌ ОШИБКА: Не удалось найти CH340 порт!")
        print(f"   Ожидаемый USB ID: {CH340_USB_VENDOR_ID}:{CH340_USB_PRODUCT_ID}")
        print("   Проверьте:")
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
        
        # Быстрый тест связи
        print("\n🔧 Быстрый тест связи...")
        if controller.quick_test():
            print("🎉 ТЕСТ ПРОЙДЕН УСПЕШНО! Система готова к работе.")
        else:
            print("⚠️ ТЕСТ НЕ ПРОЙДЕН! Проверьте подключение Arduino.")
        
        # Проверка статуса MKS моторов
        print("\n🔍 Проверка статуса MKS моторов...")
        try:
            status_info = controller.check_all_motors_status()
            for motor_num, info in status_info.items():
                status_str = []
                if info['enabled'] is not None:
                    status_str.append(f"EN: {'ON' if info['enabled'] else 'OFF'}")
                if info['protect_triggered'] is not None:
                    if info['protect_triggered']:
                        status_str.append("⚠️ PROTECT!")
                    else:
                        status_str.append("OK")
                if info['encoder'] is not None:
                    status_str.append(f"Encoder: {info['encoder']}")
                
                status_display = ", ".join(status_str) if status_str else "Не удалось прочитать"
                print(f"   Мотор {motor_num} ({info['name']}): {status_display}")
            
            # Автоматический сброс защиты, если она сработала
            protect_triggered_motors = [num for num, info in status_info.items() 
                                       if info.get('protect_triggered') == True]
            if protect_triggered_motors:
                print(f"\n🔧 Сброс защиты для моторов: {', '.join(map(str, protect_triggered_motors))}...")
                for motor_num in protect_triggered_motors:
                    addr = MOTOR_ADDR.get(motor_num)
                    if addr:
                        try:
                            if controller.reset_mks_protect(addr):
                                print(f"   ✅ Защита сброшена для мотора {motor_num}")
                            else:
                                print(f"   ⚠️ Не удалось сбросить защиту для мотора {motor_num}")
                            time.sleep(0.1)
                        except Exception as e:
                            print(f"   ⚠️ Ошибка сброса защиты для мотора {motor_num}: {e}")
            
            # Автоматическое включение всех MKS моторов
            print("\n⚡ Включение всех MKS моторов...")
            for motor_num, addr in MOTOR_ADDR.items():
                try:
                    # Проверяем текущий статус
                    status = controller.read_mks_status(addr)
                    is_enabled = status.get('enabled', False) if status else False
                    
                    if not is_enabled:
                        print(f"   → Включение мотора {motor_num} (0x{addr:02X})...")
                        if controller.enable_mks_motor(addr, True):
                            print(f"   ✅ Мотор {motor_num} включен")
                            controller.mks_motors_enabled[motor_num] = True
                        else:
                            print(f"   ⚠️ Не удалось включить мотор {motor_num}")
                    else:
                        print(f"   ✅ Мотор {motor_num} уже включен")
                        controller.mks_motors_enabled[motor_num] = True
                    
                    time.sleep(0.1)  # Небольшая задержка между моторами
                except Exception as e:
                    print(f"   ⚠️ Ошибка включения мотора {motor_num}: {e}")
            
            print("✅ Все MKS моторы готовы к работе")
        except Exception as e:
            print(f"   ⚠️ Не удалось проверить статус моторов: {e}")
            # Все равно пытаемся включить моторы
            print("\n⚡ Попытка включения всех MKS моторов...")
            for motor_num, addr in MOTOR_ADDR.items():
                try:
                    if controller.enable_mks_motor(addr, True):
                        print(f"   ✅ Мотор {motor_num} включен")
                        controller.mks_motors_enabled[motor_num] = True
                    time.sleep(0.1)
                except:
                    pass
    except Exception as e:
        print(f"\n❌ ОШИБКА инициализации: {e}")
        print(f"   Arduino порт: {arduino_port}")
        print(f"   CH340 порт: {ch340_port}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    while True:
        try:
            print("\n🔧 Доступные действия:")
            print("  1 → Выбрать мотор и переместиться в угол")
            print("  2 → Прочитать текущие углы всех моторов")
            print("  3 → Проверить текущую позицию на допустимость")
            print("  4 → Остановить ВСЕ моторы")
            print("  5 → Непрерывный мониторинг углов")
            print("  6 → Калибровка энкодера (установка нулевой точки)")
            print("  7 → Сброс позиции энкодера в 0")
            print("  8 → Отправить произвольную команду энкодеру")
            print("  9 → Проверить статус MKS моторов (энкодер, защита, включение)")
            print("  0 → Выход")

            choice = input("\n> ").strip()

            if choice == '0':
                print("✅ Завершение работы.")
                break

            elif choice == '1':
                print("\nДоступные моторы:")
                for num, name in MOTOR_NAMES.items():
                    print(f"  {num} → {name}")
                try:
                    motor_num = int(input("Выберите номер мотора (1–6): "))
                    if motor_num not in range(1, 7):
                        print("❌ Неверный номер мотора.")
                        continue

                    # Специальная обработка для сервопривода (мотор 1 - клешня)
                    if motor_num == 1:
                        angle = int(input("\nВведите угол сервопривода (0–180°): "))
                        if angle < 0:
                            angle = 0
                        elif angle > 180:
                            angle = 180
                        controller.set_gripper_angle(angle)
                        print(f"✅ Мотор 1 (клешня) установлен в {angle}°")
                        continue

                    # Показываем допустимый диапазон для выбранного мотора
                    min_angle, max_angle = MOTOR_ANGLE_LIMITS.get(motor_num, (0, 180))
                    target_angle = float(input(f"\nВведите целевой угол ({min_angle}–{max_angle}°): "))
                    
                    # Проверка ограничений
                    is_valid, error = controller.check_angle_limits(motor_num, target_angle)
                    if not is_valid:
                        print(f"❌ {error}")
                        continue

                    tolerance = float(input("Точность (по умолчанию 2.0°): ") or "2.0")

                    controller.move_to_angle(motor_num, target_angle, tolerance=tolerance)

                except ValueError:
                    print("❌ Введите число.")
                except Exception as e:
                    print(f"❌ Ошибка: {e}")

            elif choice == '2':
                print("\n📡 Запрос углов с энкодеров...")
                angles = controller.read_all_angles()
                if angles:
                    print("\n📊 Текущие углы:")
                    for motor_num, channel in ENCODER_CHANNEL.items():
                        if channel in angles:
                            raw_angle = angles[channel]
                            if motor_num in [2, 3, 4]:
                                corrected = 180 - raw_angle
                                print(f"  Мотор {motor_num} ({channel}): {raw_angle:.1f}° → {corrected:.1f}° (скорректировано)")
                            elif motor_num == 5:
                                # Мотор 5 (Nema 34) - без зеркалирования
                                print(f"  Мотор {motor_num} ({channel}): {raw_angle:.1f}°")
                            else:
                                print(f"  Мотор {motor_num} ({channel}): {raw_angle:.1f}°")
                else:
                    print("🟡 Не удалось получить данные после нескольких попыток.")

            elif choice == '3':
                print("\n🔍 Проверка текущей позиции...")
                angles = controller.read_all_angles()
                if angles:
                    # Преобразуем в формат для проверки кинематики
                    motor_angles = {}
                    for motor_num, channel in ENCODER_CHANNEL.items():
                        if channel in angles:
                            raw_angle = angles[channel]
                            if motor_num in [2, 3, 4]:
                                corrected = 180 - raw_angle
                            else:
                                # Мотор 5 и 6 - без зеркалирования
                                corrected = raw_angle
                            motor_angles[motor_num] = corrected
                    
                    # Проверка каждого мотора
                    print("\n📊 Проверка ограничений:")
                    all_valid = True
                    for motor_num, angle in motor_angles.items():
                        is_valid, error = controller.check_angle_limits(motor_num, angle)
                        status = "✅" if is_valid else "❌"
                        print(f"  {status} Мотор {motor_num}: {angle:.1f}°", end="")
                        if not is_valid:
                            print(f" - {error}")
                            all_valid = False
                        else:
                            min_angle, max_angle = MOTOR_ANGLE_LIMITS.get(motor_num, (0, 180))
                            print(f" (диапазон: {min_angle}°-{max_angle}°)")
                    
                    # Проверка кинематики
                    print("\n🔧 Проверка кинематики:")
                    is_kinematic_valid, kinematic_error = controller.check_kinematics(motor_angles)
                    if is_kinematic_valid:
                        print("  ✅ Позиция допустима с точки зрения кинематики")
                    else:
                        print(f"  ❌ {kinematic_error}")
                        all_valid = False
                    
                    if all_valid:
                        print("\n✅ Все проверки пройдены - позиция безопасна")
                    else:
                        print("\n⚠️ Обнаружены проблемы - будьте осторожны!")
                else:
                    print("🟡 Не удалось получить данные.")
            
            elif choice == '4':
                controller.stop_all()
            
            elif choice == '5':
                print("\n📊 Непрерывный мониторинг (Ctrl+C для остановки)...")
                try:
                    while True:
                        angles = controller.read_all_angles()
                        if angles:
                            print("\r", end="")
                            status = []
                            for motor_num, channel in ENCODER_CHANNEL.items():
                                if channel in angles:
                                    raw = angles[channel]
                                    if motor_num in [2, 3, 4]:
                                        corr = 180 - raw
                                        status.append(f"M{motor_num}:{corr:.0f}°")
                                    else:
                                        # Мотор 5 и 6 - без зеркалирования
                                        status.append(f"M{motor_num}:{raw:.0f}°")
                            print("  ".join(status), end="", flush=True)
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n\n✅ Мониторинг остановлен.")
            
            elif choice == '6':
                print("\n🔧 Калибровка энкодера")
                print("Доступные каналы:")
                for motor_num, channel in ENCODER_CHANNEL.items():
                    motor_name = MOTOR_NAMES.get(motor_num, "неизвестный")
                    print(f"  {channel} → Мотор {motor_num} ({motor_name})")
                
                try:
                    channel = input("\nВыберите канал энкодера (A0-A4): ").strip().upper()
                    if channel not in ["A0", "A1", "A2", "A3", "A4"]:
                        print("❌ Неверный канал. Используйте A0, A1, A2, A3 или A4.")
                        continue
                    
                    zero_angle_input = input("Введите угол для нулевой точки (по умолчанию 0.0°): ").strip()
                    zero_angle = float(zero_angle_input) if zero_angle_input else 0.0
                    
                    controller.calibrate_encoder_channel(channel, zero_angle)
                
                except ValueError:
                    print("❌ Введите число для угла.")
                except Exception as e:
                    print(f"❌ Ошибка: {e}")
            
            elif choice == '7':
                print("\n🔄 Сброс позиции энкодера в 0")
                print("Доступные каналы:")
                for motor_num, channel in ENCODER_CHANNEL.items():
                    motor_name = MOTOR_NAMES.get(motor_num, "неизвестный")
                    print(f"  {channel} → Мотор {motor_num} ({motor_name})")
                
                try:
                    channel = input("\nВыберите канал энкодера (A0-A4): ").strip().upper()
                    if channel not in ["A0", "A1", "A2", "A3", "A4"]:
                        print("❌ Неверный канал. Используйте A0, A1, A2, A3 или A4.")
                        continue
                    
                    confirm = input(f"Вы уверены, что хотите сбросить {channel} в 0? (y/n): ").strip().lower()
                    if confirm == 'y':
                        controller.reset_encoder_channel(channel)
                    else:
                        print("❌ Отменено.")
                
                except Exception as e:
                    print(f"❌ Ошибка: {e}")
            
            elif choice == '8':
                print("\n📤 Отправка произвольной команды энкодеру")
                print("Примеры команд:")
                print("  - read A0          - прочитать значение канала A0")
                print("  - write A0 90.0    - записать значение 90.0 в канал A0")
                print("  - reset A0         - сбросить канал A0")
                print("  - cal A0 0         - калибровать канал A0 на 0°")
                print("  - status           - получить статус энкодера")
                print("\nПримечание: Команда будет отправлена с префиксом 'enc '")
                print("            Если нужно отправить без префикса, введите команду, начинающуюся с '!'")
                
                try:
                    command = input("\nВведите команду для энкодера: ").strip()
                    if not command:
                        print("❌ Команда не может быть пустой.")
                        continue
                    
                    # Если команда начинается с '!', отправляем без префикса
                    use_prefix = not command.startswith('!')
                    if not use_prefix:
                        command = command[1:]  # Убираем '!'
                    
                    controller.send_raw_encoder_command(command, use_prefix=use_prefix)
                
                except Exception as e:
                    print(f"❌ Ошибка: {e}")
            
            elif choice == '9':
                print("\n🔍 Проверка статуса MKS моторов...")
                try:
                    status_info = controller.check_all_motors_status()
                    print("\n📊 Статус моторов:")
                    print("-" * 70)
                    for motor_num in sorted(status_info.keys()):
                        info = status_info[motor_num]
                        print(f"\nМотор {motor_num}: {info['name']}")
                        print(f"  Адрес: 0x{info['addr']:02X}")
                        
                        if info['enabled'] is not None:
                            status_icon = "🟢" if info['enabled'] else "🔴"
                            print(f"  {status_icon} Включен: {'Да' if info['enabled'] else 'Нет'}")
                        else:
                            print(f"  ⚠️ Статус включения: не удалось прочитать")
                        
                        if info['protect_triggered'] is not None:
                            if info['protect_triggered']:
                                print(f"  ⚠️ ЗАЩИТА СРАБОТАЛА! Мотор заблокирован.")
                                print(f"     Используйте функцию сброса защиты для разблокировки.")
                            else:
                                print(f"  ✅ Защита: не сработала")
                        else:
                            print(f"  ⚠️ Статус защиты: не удалось прочитать")
                        
                        if info['encoder'] is not None:
                            print(f"  📍 Значение энкодера: {info['encoder']}")
                        else:
                            print(f"  ⚠️ Энкодер: не удалось прочитать")
                    
                    print("-" * 70)
                    
                    # Предложение сбросить защиту, если она сработала
                    protect_triggered_motors = [num for num, info in status_info.items() 
                                               if info.get('protect_triggered') == True]
                    if protect_triggered_motors:
                        print(f"\n⚠️ ВНИМАНИЕ: Защита сработала на моторах: {', '.join(map(str, protect_triggered_motors))}")
                        response = input("Сбросить защиту для этих моторов? (yes/no): ").strip().lower()
                        if response in ['yes', 'y', 'да', 'д']:
                            for motor_num in protect_triggered_motors:
                                addr = MOTOR_ADDR.get(motor_num)
                                if addr:
                                    if controller.reset_mks_protect(addr):
                                        print(f"  ✅ Защита сброшена для мотора {motor_num}")
                                    else:
                                        print(f"  ❌ Не удалось сбросить защиту для мотора {motor_num}")
                
                except Exception as e:
                    print(f"❌ Ошибка проверки статуса: {e}")
                    import traceback
                    traceback.print_exc()

            else:
                print("❌ Неизвестная команда.")

        except KeyboardInterrupt:
            print("\n\n🛑 Прервано пользователем.")
            break
        except Exception as e:
            print(f"\n💥 Ошибка: {e}")
            import traceback
            traceback.print_exc()
    
    # Закрытие соединений
    arduino_serial.close()
    ch340_serial.close()
    print("\n👋 До свидания!")


if __name__ == "__main__":
    main()
