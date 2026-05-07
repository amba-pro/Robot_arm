#!/usr/bin/env python3
"""
Скрипт проверки системы робота
Тестирует связь с Arduino и CH340, чтение углов, работу моторов
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
import platform
import re

# === USB ID УСТРОЙСТВ ===
ARDUINO_USB_VENDOR_ID = "0403"
ARDUINO_USB_PRODUCT_ID = "6001"
CH340_USB_VENDOR_ID = "1a86"
CH340_USB_PRODUCT_ID = "7523"

BAUDRATE = 115200
TIMEOUT = 1.0


def find_port_by_usb_id(vendor_id: str, product_id: str) -> tuple:
    """Находит порт по USB ID"""
    target_vid = int(vendor_id, 16)
    target_pid = int(product_id, 16)
    
    ports = serial.tools.list_ports.comports()
    
    for port_info in ports:
        if port_info.vid == target_vid and port_info.pid == target_pid:
            return port_info.device, port_info
    
    # На Linux также проверяем через sysfs
    if platform.system() == 'Linux':
        import glob
        port_patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
        
        for pattern in port_patterns:
            for port in glob.glob(pattern):
                try:
                    # Пробуем открыть порт для проверки
                    test_ser = serial.Serial(port, BAUDRATE, timeout=0.1)
                    test_ser.close()
                    # Здесь можно добавить проверку USB ID через sysfs
                    # но для простоты просто возвращаем порт
                except:
                    continue
    
    return None, None


def test_arduino_connection(port: str) -> dict:
    """Тестирует связь с Arduino"""
    result = {
        'success': False,
        'port': port,
        'angles': {},
        'raw_response': '',
        'error': None
    }
    
    try:
        print(f"\n🔍 Тест Arduino на порту {port}...")
        ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
        print(f"   ✅ Порт открыт")
        
        # Даем Arduino время на перезагрузку
        time.sleep(2)
        
        # Очищаем буфер
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.01)
        
        # Тест 1: Команда 'i' (info) - пропускаем, чтобы не мешать команде 'a'
        # В test_fix.py команда 'i' не используется, поэтому пропускаем её
        
        # Тест 2: Команда 'a' (углы) - основной тест
        print(f"   📤 Тест команды 'a' (чтение углов)...")
        
        # Очищаем буферы перед отправкой команды 'a' (как в test_fix.py)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.01)
        
        ser.write(b'a\n')
        ser.flush()
        
        # Ждем ответа (Arduino делает ADC сканирование) - как в test_fix.py
        time.sleep(0.5)
        
        # Читаем все доступные данные (точно как в test_fix.py)
        response_data = b""
        
        # Читаем все что есть в буфере сразу
        if ser.in_waiting > 0:
            response_data = ser.read(ser.in_waiting)
        
        # Проверяем, есть ли углы в ответе
        response_str = response_data.decode('utf-8', errors='ignore')
        has_angles = any(f'A{i}:' in response_str for i in range(5))
        
        # Если углов нет или данных мало, ждем еще и читаем снова
        if not has_angles or len(response_data) < 30:
            # Ждем еще немного и читаем снова (как в test_fix.py)
            time.sleep(0.2)
            if ser.in_waiting > 0:
                additional_data = ser.read(ser.in_waiting)
                response_data += additional_data
        
        if response_data:
            response = response_data.decode('utf-8', errors='ignore')
            result['raw_response'] = response
            print(f"   📥 Ответ 'a' ({len(response_data)} байт):")
            print(f"      '{response}'")
            
            # Парсим углы (используем регулярные выражения для надежности)
            clean_resp = response.replace('\r', ' ').replace('\n', ' ').strip()
            
            # Метод 1: Регулярные выражения
            pattern = r'A([0-4]):\s*([0-9.-]+)'
            matches = re.findall(pattern, clean_resp)
            
            for channel_num, value_str in matches:
                try:
                    channel = f"A{channel_num}"
                    clean_value = ''.join(c for c in value_str if c.isdigit() or c == '.' or c == '-')
                    if clean_value:
                        result['angles'][channel] = float(clean_value)
                except ValueError:
                    continue
            
            # Метод 2: Альтернативный парсинг (если регулярные выражения не сработали)
            if not result['angles']:
                parts = clean_resp.split()
                for part in parts:
                    if ':' in part and part.startswith('A'):
                        try:
                            channel, value_str = part.split(':', 1)
                            clean_value = ''.join(c for c in value_str if c.isdigit() or c == '.' or c == '-')
                            if clean_value:
                                result['angles'][channel] = float(clean_value)
                        except ValueError:
                            continue
            
            if result['angles']:
                print(f"   ✅ Углы распарсены: {result['angles']}")
                result['success'] = True
                
                # Проверяем наличие всех каналов
                expected = ['A0', 'A1', 'A2', 'A3', 'A4']
                missing = [ch for ch in expected if ch not in result['angles']]
                if missing:
                    print(f"   ⚠️ Отсутствуют каналы: {missing}")
                else:
                    print(f"   ✅ Все 5 каналов найдены")
            else:
                print(f"   ❌ Не удалось распарсить углы")
        else:
            print(f"   ❌ Нет ответа на команду 'a'")
            result['error'] = "Нет ответа от Arduino"
        
        ser.close()
        
    except Exception as e:
        print(f"   ❌ Ошибка: {e}")
        result['error'] = str(e)
    
    return result


def test_ch340_connection(port: str) -> dict:
    """Тестирует связь с CH340"""
    result = {
        'success': False,
        'port': port,
        'error': None
    }
    
    try:
        print(f"\n🔍 Тест CH340 на порту {port}...")
        ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
        print(f"   ✅ Порт открыт")
        
        # Даем время на инициализацию
        time.sleep(0.5)
        
        # Тест: Отправляем команду статуса для мотора 2 (0xE0)
        print(f"   📤 Тест команды статуса MKS мотора (0xE0, 0x3A)...")
        
        # Команда: [адрес, код_команды, CRC]
        addr = 0xE0
        cmd_code = 0x3A  # Чтение статуса
        cmd = [addr, cmd_code]
        crc = sum(cmd) & 0xFF
        packet = bytes(cmd + [crc])
        
        ser.write(packet)
        ser.flush()
        
        # Ждем ответа
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"   📥 Ответ получен ({len(response)} байт): {response.hex()}")
            
            if len(response) >= 3:
                status = response[1]
                is_enabled = (status & 0x01) != 0
                print(f"   ✅ Статус прочитан: включен={is_enabled}, статус_байт=0x{status:02X}")
                result['success'] = True
            else:
                print(f"   ⚠️ Неполный ответ от MKS мотора")
        else:
            print(f"   ⚠️ Нет ответа от MKS мотора (возможно, мотор не подключен)")
            result['error'] = "Нет ответа от MKS мотора"
        
        ser.close()
        
    except Exception as e:
        print(f"   ❌ Ошибка: {e}")
        result['error'] = str(e)
    
    return result


def main():
    """Основная функция проверки"""
    print("\n" + "="*70)
    print("     СИСТЕМА ПРОВЕРКИ РОБОТА")
    print("="*70)
    print("📌 Проверяет:")
    print("  • Подключение Arduino (FT232)")
    print("  • Подключение CH340")
    print("  • Чтение углов с энкодеров")
    print("  • Связь с MKS моторами")
    print("="*70)
    
    # Поиск портов
    print("\n🔍 Поиск устройств...")
    
    arduino_port, arduino_info = find_port_by_usb_id(ARDUINO_USB_VENDOR_ID, ARDUINO_USB_PRODUCT_ID)
    if arduino_port:
        print(f"✅ Найден Arduino (FT232): {arduino_port}")
        if arduino_info:
            print(f"   Описание: {arduino_info.description}")
    else:
        print(f"❌ Arduino (FT232) не найден")
        print(f"   Ожидаемый USB ID: {ARDUINO_USB_VENDOR_ID}:{ARDUINO_USB_PRODUCT_ID}")
    
    ch340_port, ch340_info = find_port_by_usb_id(CH340_USB_VENDOR_ID, CH340_USB_PRODUCT_ID)
    if ch340_port:
        print(f"✅ Найден CH340: {ch340_port}")
        if ch340_info:
            print(f"   Описание: {ch340_info.description}")
    else:
        print(f"❌ CH340 не найден")
        print(f"   Ожидаемый USB ID: {CH340_USB_VENDOR_ID}:{CH340_USB_PRODUCT_ID}")
    
    # Показываем все доступные порты
    print("\n📋 Все доступные порты:")
    all_ports = serial.tools.list_ports.comports()
    for port_info in all_ports:
        if port_info.vid is not None and port_info.pid is not None:
            usb_id = f"{port_info.vid:04x}:{port_info.pid:04x}"
            desc = port_info.description or "Unknown"
            print(f"   • {port_info.device}: USB ID = {usb_id} ({desc})")
    
    # Тестирование Arduino
    if arduino_port:
        arduino_result = test_arduino_connection(arduino_port)
        
        if arduino_result['success']:
            print(f"\n✅ Arduino: ТЕСТ ПРОЙДЕН")
            print(f"   Прочитано каналов: {len(arduino_result['angles'])}")
            print(f"   Углы: {arduino_result['angles']}")
        else:
            print(f"\n❌ Arduino: ТЕСТ НЕ ПРОЙДЕН")
            if arduino_result['error']:
                print(f"   Ошибка: {arduino_result['error']}")
    else:
        print(f"\n⚠️ Arduino: Порт не найден, тест пропущен")
    
    # Тестирование CH340
    if ch340_port:
        ch340_result = test_ch340_connection(ch340_port)
        
        if ch340_result['success']:
            print(f"\n✅ CH340: ТЕСТ ПРОЙДЕН")
        else:
            print(f"\n⚠️ CH340: ТЕСТ ЧАСТИЧНО ПРОЙДЕН (порт открыт, но нет ответа от моторов)")
            if ch340_result['error']:
                print(f"   Примечание: {ch340_result['error']}")
    else:
        print(f"\n⚠️ CH340: Порт не найден, тест пропущен")
    
    # Итоговый результат
    print("\n" + "="*70)
    print("     ИТОГОВЫЙ РЕЗУЛЬТАТ")
    print("="*70)
    
    all_ok = True
    
    if not arduino_port:
        print("❌ Arduino порт не найден")
        all_ok = False
    elif not arduino_result.get('success'):
        print("❌ Arduino: не удалось прочитать углы")
        all_ok = False
    else:
        print("✅ Arduino: работает корректно")
    
    if not ch340_port:
        print("❌ CH340 порт не найден")
        all_ok = False
    else:
        print("✅ CH340: порт найден и доступен")
    
    if all_ok and arduino_result.get('success'):
        print("\n🎉 ВСЕ ТЕСТЫ ПРОЙДЕНЫ! Система готова к работе.")
    else:
        print("\n⚠️ ЕСТЬ ПРОБЛЕМЫ! Проверьте подключение устройств.")
    
    print("="*70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Прервано пользователем.")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()

