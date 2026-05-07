#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Скрипт для постоянного чтения углов с Arduino и сохранения в файл
Работает отдельно от основного скрипта управления моторами
"""

import serial
import serial.tools.list_ports
import time
import re
import json
import os
import sys
import math
from typing import Optional, Dict

# === НАСТРОЙКИ ===
ARDUINO_USB_VENDOR_ID = "0403"
ARDUINO_USB_PRODUCT_ID = "6001"
BAUDRATE = 115200
TIMEOUT = 0.2
ANGLE_FILE = "angles_cache.json"  # Файл для обмена данными
READ_INTERVAL = 0.05  # Читать углы каждые 50 мс
DEBUG_MODE = False  # Включить отладочный режим (показывать сырые ответы)
MOCK_MODE = os.getenv("ARM4_MOCK", "0").lower() in ("1", "true", "yes", "on")


def find_port_by_usb_id(vendor_id: str, product_id: str) -> Optional[str]:
    """Находит serial порт по USB ID устройства"""
    target_vid = int(vendor_id, 16)
    target_pid = int(product_id, 16)
    
    ports = serial.tools.list_ports.comports()
    
    for port_info in ports:
        if port_info.vid == target_vid and port_info.pid == target_pid:
            return port_info.device
    
    return None


def parse_angles(resp: str) -> Dict[str, float]:
    """Парсит углы из ответа Arduino"""
    angles = {}
    try:
        # Объединяем все строки в одну
        clean_resp = resp.replace('\r', ' ').replace('\n', ' ').strip()
        
        # Ищем паттерны углов A0:123, A1:456 и т.д.
        pattern = r'A([0-4]):\s*([0-9.-]+)'
        matches = re.findall(pattern, clean_resp)
        
        for channel_num, value_str in matches:
            try:
                channel = f"A{channel_num}"
                clean_value = ''.join(c for c in value_str if c.isdigit() or c == '.' or c == '-')
                if clean_value:
                    value = float(clean_value)
                    # Проверяем, что значение в разумных пределах (0-360 для углов)
                    # Arduino отправляет углы в диапазоне 0-360 градусов
                    if 0 <= value <= 360:
                        angles[channel] = value
            except ValueError:
                continue
        
        return angles
    except Exception:
        return {}


def read_angles_from_arduino(ser: serial.Serial) -> Optional[Dict[str, float]]:
    """Читает углы с Arduino (устойчивая версия с несколькими попытками)"""
    # Делаем до 3 попыток чтения для устойчивости
    for attempt in range(3):
        try:
            # Агрессивная очистка буферов перед чтением
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Читаем все что осталось в буфере
            for _ in range(3):
                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting)
                time.sleep(0.01)
            time.sleep(0.02)
            
            # Отправляем команду 'a'
            ser.write(b'a\n')
            ser.flush()
            
            # Ждем ответа (как в test_fix.py - 0.5 сек)
            time.sleep(0.5)
            
            # Читаем все доступные данные
            response_data = b""
            if ser.in_waiting > 0:
                response_data = ser.read(ser.in_waiting)
            
            # Дополнительные попытки чтения (если ответ неполный)
            for _ in range(3):
                time.sleep(0.1)
                if ser.in_waiting > 0:
                    response_data += ser.read(ser.in_waiting)
            
            if response_data:
                resp = response_data.decode('utf-8', errors='ignore')
                
                # Отладочный вывод сырого ответа
                if DEBUG_MODE and not hasattr(read_angles_from_arduino, '_raw_debug_shown'):
                    print(f"🔍 Сырой ответ Arduino: {repr(resp)}")
                    read_angles_from_arduino._raw_debug_shown = True
                
                angles = parse_angles(resp)
                # Проверяем, что прочитаны все 5 каналов
                if angles and len(angles) >= 3:  # Хотя бы 3 канала из 5
                    # Отладочный вывод: показываем какие каналы прочитаны
                    if len(angles) < 5:
                        missing = [f"A{i}" for i in range(5) if f"A{i}" not in angles]
                        if not hasattr(read_angles_from_arduino, '_debug_shown'):
                            print(f"⚠️ Не все каналы прочитаны. Отсутствуют: {', '.join(missing)}")
                            print(f"   Ответ Arduino: {resp[:100]}...")
                            read_angles_from_arduino._debug_shown = True
                    return angles
            
            # Если не получилось - пробуем еще раз с небольшой задержкой
            if attempt < 2:
                time.sleep(0.1)
                continue
            
        except serial.SerialException:
            # Ошибка порта - пробуем еще раз
            if attempt < 2:
                time.sleep(0.2)
                continue
            return None
        except Exception as e:
            # Другие ошибки - пробуем еще раз
            if attempt < 2:
                time.sleep(0.1)
                continue
            # Не выводим ошибку при каждой попытке - только в конце
            if attempt == 2:
                pass  # Тихая ошибка - не засоряем вывод
    
    return None


def save_angles_to_file(angles: Dict[str, float], timestamp: float):
    """Сохраняет углы в JSON файл"""
    try:
        data = {
            'angles': angles,
            'timestamp': timestamp,
            'valid': True
        }
        # Атомарная запись: сначала во временный файл, потом переименовываем
        temp_file = ANGLE_FILE + '.tmp'
        with open(temp_file, 'w') as f:
            json.dump(data, f)
        os.replace(temp_file, ANGLE_FILE)
    except Exception as e:
        print(f"❌ Ошибка сохранения углов: {e}")


def read_raw_adc_values(ser: serial.Serial) -> Optional[str]:
    """Читает сырые значения ADC с Arduino (команда 'v') для отладки"""
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.02)
        
        # Отправляем команду 'v' для чтения сырых значений ADC
        ser.write(b'v\n')
        ser.flush()
        time.sleep(0.3)
        
        response_data = b""
        if ser.in_waiting > 0:
            response_data = ser.read(ser.in_waiting)
        
        # Дополнительные попытки чтения
        for _ in range(2):
            time.sleep(0.1)
            if ser.in_waiting > 0:
                response_data += ser.read(ser.in_waiting)
        
        if response_data:
            return response_data.decode('utf-8', errors='ignore')
        return None
    except Exception:
        return None


def parse_raw_adc(resp: str) -> Dict[str, int]:
    """Парсит сырые значения ADC из ответа команды 'v'"""
    adc_values = {}
    try:
        clean_resp = resp.replace('\r', ' ').replace('\n', ' ').strip()
        pattern = r'A([0-4]):\s*([0-9]+)'
        matches = re.findall(pattern, clean_resp)
        
        for channel_num, value_str in matches:
            try:
                channel = f"A{channel_num}"
                value = int(value_str)
                if 0 <= value <= 1023:
                    adc_values[channel] = value
            except ValueError:
                continue
        
        return adc_values
    except Exception:
        return {}


def main():
    """Основной цикл чтения углов"""
    print("=" * 70)
    print("     ЧТЕНИЕ УГЛОВ С ARDUINO")
    print("=" * 70)
    print(f"📁 Файл кэша: {ANGLE_FILE}")
    print(f"⏱️  Интервал чтения: {READ_INTERVAL * 1000:.0f} мс")
    if MOCK_MODE:
        print("🧪 Режим заглушки включен (ARM4_MOCK=1)")
    if DEBUG_MODE:
        print("🔍 Отладочный режим включен")
    print()

    if MOCK_MODE:
        t0 = time.time()
        print("✅ Запуск генератора тестовых углов A0-A4")
        try:
            while True:
                t = time.time() - t0
                angles = {
                    "A0": 180.0 + 90.0 * math.sin(t * 0.8),
                    "A1": 180.0 + 80.0 * math.sin(t * 1.0 + 0.7),
                    "A2": 180.0 + 70.0 * math.sin(t * 1.2 + 1.4),
                    "A3": 180.0 + 60.0 * math.sin(t * 1.5 + 2.1),
                    "A4": 180.0 + 50.0 * math.sin(t * 1.8 + 2.8),
                }
                # Ограничиваем значения рабочим диапазоном 0..360
                angles = {k: max(0.0, min(360.0, float(v))) for k, v in angles.items()}
                now = time.time()
                save_angles_to_file(angles, now)
                if int(now) != getattr(main, "_last_print", 0):
                    print(
                        "🧪 MOCK angles: "
                        + ", ".join([f"{k}: {v:.1f}°" for k, v in angles.items()])
                    )
                    main._last_print = int(now)
                time.sleep(READ_INTERVAL)
        except KeyboardInterrupt:
            print("\n🛑 Остановка...")
            print("✅ Скрипт остановлен")
            return
    
    # Поиск Arduino порта
    print("🔍 Поиск Arduino порта...")
    arduino_port = find_port_by_usb_id(ARDUINO_USB_VENDOR_ID, ARDUINO_USB_PRODUCT_ID)
    
    if not arduino_port:
        print(f"❌ Arduino (USB ID {ARDUINO_USB_VENDOR_ID}:{ARDUINO_USB_PRODUCT_ID}) не найден!")
        print("📋 Доступные порты:")
        ports = serial.tools.list_ports.comports()
        for port_info in ports:
            vid = f"{port_info.vid:04x}" if port_info.vid else "N/A"
            pid = f"{port_info.pid:04x}" if port_info.pid else "N/A"
            print(f"   • {port_info.device}: USB ID = {vid}:{pid}")
        sys.exit(1)
    
    print(f"✅ Найден Arduino на порту: {arduino_port}")
    print()
    
    # Подключение к Arduino
    ser = None
    initialized = False
    last_successful_read = 0
    error_count = 0
    
    try:
        while True:
            try:
                # Подключение или переподключение
                if ser is None or not ser.is_open:
                    if ser:
                        ser.close()
                    
                    print("🔌 Подключение к Arduino...")
                    ser = serial.Serial(arduino_port, BAUDRATE, timeout=TIMEOUT)
                    
                    # При первой инициализации ждем 2 секунды для перезагрузки Arduino
                    if not initialized:
                        print("⏳ Ожидание инициализации Arduino (2 сек)...")
                        time.sleep(2.0)
                        initialized = True
                    else:
                        time.sleep(0.2)
                    
                    print("✅ Подключено")
                    error_count = 0
                    
                    # При первом подключении читаем сырые значения ADC для проверки
                    if not hasattr(main, '_adc_checked'):
                        print("🔍 Проверка сырых значений ADC...")
                        raw_adc = read_raw_adc_values(ser)
                        if raw_adc:
                            adc_vals = parse_raw_adc(raw_adc)
                            if adc_vals:
                                print(f"   Сырые значения ADC: {adc_vals}")
                                print(f"   💡 Для диагностики: двигайте мотор 5 и проверьте, меняется ли A3")
                            else:
                                print(f"   Сырой ответ: {raw_adc}")
                        else:
                            print("   ⚠️ Не удалось прочитать сырые значения ADC")
                        main._adc_checked = True
                        time.sleep(0.5)
                
                # Читаем углы
                angles = read_angles_from_arduino(ser)
                
                if angles:
                    timestamp = time.time()
                    
                    # Отслеживаем изменения углов
                    if not hasattr(main, '_last_angles'):
                        main._last_angles = {}
                    
                    # Проверяем, изменились ли углы
                    changed_channels = []
                    for ch in ['A0', 'A1', 'A2', 'A3', 'A4']:
                        if ch in angles:
                            last_val = main._last_angles.get(ch)
                            current_val = angles[ch]
                            if last_val is not None and abs(current_val - last_val) > 0.5:
                                changed_channels.append(f"{ch}: {last_val:.1f}°→{current_val:.1f}°")
                    
                    save_angles_to_file(angles, timestamp)
                    main._last_angles = angles.copy()
                    last_successful_read = timestamp
                    error_count = 0
                    
                    # Выводим статус раз в секунду
                    if int(timestamp) % 1 == 0 and int(timestamp) != getattr(main, '_last_print', 0):
                        # Форматируем вывод для всех 5 каналов
                        angle_strs = []
                        for ch in ['A0', 'A1', 'A2', 'A3', 'A4']:
                            if ch in angles:
                                angle_strs.append(f"{ch}: {angles[ch]:.1f}°")
                            else:
                                angle_strs.append(f"{ch}: N/A")
                        
                        status_msg = f"✅ Углы прочитаны: {len(angles)} каналов ({', '.join(angle_strs)})"
                        if changed_channels:
                            status_msg += f" | Изменения: {', '.join(changed_channels)}"
                        else:
                            # Если углы не меняются, периодически проверяем сырые значения ADC
                            if not hasattr(main, '_last_adc_check') or (timestamp - main._last_adc_check) > 5.0:
                                raw_adc = read_raw_adc_values(ser)
                                if raw_adc:
                                    adc_vals = parse_raw_adc(raw_adc)
                                    if adc_vals and 'A3' in adc_vals:
                                        status_msg += f" | 🔍 ADC A3: {adc_vals['A3']} (должен меняться при движении мотора 5)"
                                main._last_adc_check = timestamp
                            else:
                                status_msg += " | ⚠️ Углы не меняются!"
                        
                        print(status_msg)
                        main._last_print = int(timestamp)
                else:
                    error_count += 1
                    # Увеличиваем порог ошибок до 50 для большей устойчивости
                    # Временные ошибки из-за команд сервопривода не должны вызывать переподключение
                    if error_count > 50:
                        print("⚠️ Не удалось прочитать углы после 50 попыток, переподключение...")
                        ser.close()
                        ser = None
                        error_count = 0
                        time.sleep(1.0)
                    # При временных ошибках просто продолжаем, не переподключаясь
                
                # Ждем перед следующим чтением
                time.sleep(READ_INTERVAL)
                
            except serial.SerialException as e:
                print(f"❌ Ошибка Serial: {e}")
                if ser:
                    ser.close()
                ser = None
                time.sleep(1.0)
                
            except KeyboardInterrupt:
                print("\n🛑 Остановка...")
                break
                
            except Exception as e:
                print(f"❌ Неожиданная ошибка: {e}")
                if ser:
                    ser.close()
                ser = None
                time.sleep(1.0)
    
    finally:
        if ser and ser.is_open:
            ser.close()
        print("✅ Скрипт остановлен")


if __name__ == "__main__":
    main()

