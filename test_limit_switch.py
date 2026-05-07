#!/usr/bin/env python3
"""
Тестовый скрипт для диагностики концевика A0

Проверяет:
- Подключение концевика A0
- Значения аналогового сигнала
- Порог срабатывания
- Команды Arduino
"""

import sys
import os
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import ym1
from ym1 import SerialManager, PORT, BAUDRATE, TIMEOUT

LIMIT_SWITCH_THRESHOLD = 100


def test_limit_switch():
    """Тестирует концевик A0"""
    print("\n" + "="*70)
    print("     ТЕСТ КОНЦЕВИКА A0")
    print("="*70)
    
    # Автоматическое определение порта
    detected_port = ym1.auto_detect_port(
        preferred_port=PORT,
        usb_vendor_id="10c4",
        usb_product_id="ea60"
    )
    
    if not detected_port:
        print("\n❌ ОШИБКА: Не удалось найти доступный serial порт!")
        sys.exit(1)
    
    try:
        serial_mgr = SerialManager(detected_port, BAUDRATE, TIMEOUT)
        print(f"✅ Подключено к {detected_port}")
        
        print("\n📊 Тест 1: Чтение через команду 'v' (все аналоговые значения)")
        print("-" * 70)
        for i in range(10):
            resp = serial_mgr.send_command("v", wait_response=True)
            if resp:
                print(f"  Попытка {i+1}: {resp}")
                # Парсим A0
                import re
                match = re.search(r'A0:(\d+)', resp)
                if match:
                    value = int(match.group(1))
                    triggered = value < LIMIT_SWITCH_THRESHOLD
                    status = "🔴 СРАБОТАН" if triggered else "⚪ НЕ сработан"
                    print(f"    A0={value} → {status} (порог: <{LIMIT_SWITCH_THRESHOLD})")
            time.sleep(0.5)
        
        print("\n📊 Тест 2: Чтение через команду 'l' (быстрое чтение концевика)")
        print("-" * 70)
        for i in range(10):
            resp = serial_mgr.send_command("l", wait_response=True)
            if resp:
                print(f"  Попытка {i+1}: {resp}")
                # Парсим ответ вида "LIMIT:A0:512,TRIG:0"
                import re
                match_a0 = re.search(r'A0:(\d+)', resp)
                match_trig = re.search(r'TRIG:(\d+)', resp)
                if match_a0 and match_trig:
                    raw_value = int(match_a0.group(1))
                    triggered = match_trig.group(1) == "1"
                    status = "🔴 СРАБОТАН" if triggered else "⚪ НЕ сработан"
                    print(f"    A0={raw_value}, TRIG={triggered} → {status}")
            time.sleep(0.5)
        
        print("\n📊 Тест 3: Непрерывный мониторинг (10 секунд)")
        print("-" * 70)
        print("  Перемещайте концевик вручную для проверки реакции...")
        start_time = time.time()
        values = []
        
        while (time.time() - start_time) < 10.0:
            resp = serial_mgr.send_command("l", wait_response=True)
            if resp:
                import re
                match_a0 = re.search(r'A0:(\d+)', resp)
                match_trig = re.search(r'TRIG:(\d+)', resp)
                if match_a0:
                    raw_value = int(match_a0.group(1))
                    triggered = match_trig.group(1) == "1" if match_trig else False
                    values.append(raw_value)
                    status = "🔴 СРАБОТАН" if triggered else "⚪ НЕ сработан"
                    timestamp = time.time() - start_time
                    print(f"  [{timestamp:.1f}s] A0={raw_value:4d} → {status}")
            time.sleep(0.2)
        
        if values:
            min_val = min(values)
            max_val = max(values)
            avg_val = sum(values) / len(values)
            print(f"\n📊 Статистика:")
            print(f"  Минимальное значение: {min_val}")
            print(f"  Максимальное значение: {max_val}")
            print(f"  Среднее значение: {avg_val:.1f}")
            print(f"  Рекомендуемый порог: {(min_val + max_val) / 2:.0f}")
        
        print("\n✅ Тест завершен")
        
    except Exception as e:
        print(f"\n❌ ОШИБКА: {e}")
        import traceback
        traceback.print_exc()
    finally:
        serial_mgr.close()


if __name__ == "__main__":
    test_limit_switch()
