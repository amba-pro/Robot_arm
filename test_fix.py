import serial
import time

def test_arduino_direct():
    """Прямой тест связи с Arduino"""
    port = '/dev/ttyUSB3'  # Замените на ваш порт
    baudrate = 115200
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
        print(f"✅ Порт {port} открыт")
        
        # Даем Arduino время на перезагрузку
        time.sleep(2)
        
        # Очищаем буфер
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Тестируем команду 'a'
        print("📤 Отправка 'a'...")
        ser.write(b'a\n')
        ser.flush()
        
        # Ждем ответа
        time.sleep(0.5)
        
        # Читаем все доступные данные
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            response = data.decode('utf-8', errors='ignore')
            print(f"📥 Ответ: '{response}'")
            
            # Пробуем распарсить вручную
            parts = response.strip().split()
            angles = {}
            for part in parts:
                if ':' in part and part.startswith('A'):
                    try:
                        channel, value = part.split(':', 1)
                        angles[channel] = float(value)
                    except:
                        continue
            print(f"📊 Углы: {angles}")
        else:
            print("❌ Нет ответа")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")

if __name__ == "__main__":
    test_arduino_direct()
