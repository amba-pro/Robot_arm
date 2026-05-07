/*
 * Шаговый двигатель + АЦП + калибровка в градусах (0–359°)
 * 
 * 🔧 ИСПРАВЛЕНО: сохранение в EEPROM работает надёжно (магическое число 0xA5C3)
 * 
 * Калибровка:
 *   z       → текущее положение = 90°
 *   z<угол> → текущее положение = <угол> (0–359)
 *   w       → сохранить ВСЁ в EEPROM
 *   r       → перечитать из EEPROM (обычно не нужно — грузится при старте)
 * 
 * Команды:
 *   v       → A0:512 A1:256 ... (откалиброванные 0–1023)
 *   a       → A0:90 A1:45 ... (углы 0–359°)
 *   k       → показать СОДЕРЖИМОЕ EEPROM (для отладки)
 * 
 * UART: 115200
 */

#include <EEPROM.h>
#include <Servo.h>

// === ПИНЫ ===
const int STEP_PIN = 4;
const int DIR_PIN = 8;
const int EN_PIN = 10;
const int SERVO_PIN = 5;  // Сервопривод MG996R (клешня)
const int LIMIT_SWITCH_PIN = A0;  // Магнитный концевой выключатель для мотора 2 (поворот кисти)

// === EEPROM АДРЕСА ===
const int EEPROM_ADDR_MAX_SPEED       = 0;
const int EEPROM_ADDR_ACCELERATION    = 4;
const int EEPROM_ADDR_MOTOR_CHECKSUM  = 8;

// АЦП: 5 offsets + 5 angles + 2-байтное магическое число
const int EEPROM_ADDR_ADC_OFFSETS     = 16;  // 5 × int16_t = 10 байт
const int EEPROM_ADDR_ADC_ZERO_ANGLES = 26;  // 5 × int16_t = 10 байт
const int EEPROM_ADDR_ADC_MAGIC       = 36;  // 2 байта

const uint16_t ADC_CALIB_MAGIC = 0xA5C3;     // "подпись" валидных данных

// === ПАРАМЕТРЫ ДВИГАТЕЛЯ ===
struct MotorParams {
  unsigned int maxSpeed;
  unsigned int acceleration;
};

MotorParams params;
const unsigned int DEFAULT_MAX_SPEED = 1400;
const unsigned int DEFAULT_ACCELERATION = 5;

// === АЦП: калибровка (RAM) ===
int16_t adcOffsets[5] = {0};          // смещение (raw при калибровке)
int16_t adcZeroAngles[5] = {90,90,90,90,90}; // угол при калибровке

// === СЕРВОПРИВОД ===
Servo gripperServo;
int currentServoAngle = 90;  // Текущий угол сервопривода (0-180)

// === СОСТОЯНИЕ ДВИГАТЕЛЯ ===
enum MotorState { IDLE, ACCELERATING, CONSTANT_SPEED, DECELERATING };
MotorState currentState = IDLE;
boolean currentDirection = true;
int totalSteps = 0;
int stepsDone = 0;
int accelSteps = 0;
unsigned int currentDelay = 2000;
unsigned long lastStepTime = 0;

boolean newCommandPending = false;
int newCommandSteps = 0;
boolean newCommandForward = true;
boolean emergencyStopFlag = false;

// === UART ===
String inputString = "";
boolean stringComplete = false;

// === АЦП: буферы ===
uint16_t adcValuesAvg[5] = {0};

// === КОНЦЕВИК A0: обработка дребезга ===
bool lastLimitSwitchState = false;
bool currentLimitSwitchState = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;  // Время антидребезга (мс)

// ================================================================
// ======================= ВСПОМОГАТЕЛЬНЫЕ =========================
// ================================================================

byte calculateMotorChecksum() {
  return (byte)(params.maxSpeed + params.acceleration);
}

int calculateAccelerationSteps() {
  const unsigned int startDelay = 2000;
  int steps = 0;
  unsigned int delay = startDelay;
  while (delay > params.maxSpeed && steps < 1000) {
    delay -= params.acceleration;
    steps++;
  }
  return steps;
}

// ================================================================
// ======================= EEPROM =================================
// ================================================================

void loadParamsFromEEPROM() {
  byte checksum = EEPROM.read(EEPROM_ADDR_MOTOR_CHECKSUM);
  if (checksum == calculateMotorChecksum()) {
    EEPROM.get(EEPROM_ADDR_MAX_SPEED, params.maxSpeed);
    EEPROM.get(EEPROM_ADDR_ACCELERATION, params.acceleration);
  } else {
    params.maxSpeed = DEFAULT_MAX_SPEED;
    params.acceleration = DEFAULT_ACCELERATION;
  }
}

void saveParamsToEEPROM() {
  EEPROM.put(EEPROM_ADDR_MAX_SPEED, params.maxSpeed);
  EEPROM.put(EEPROM_ADDR_ACCELERATION, params.acceleration);
  EEPROM.write(EEPROM_ADDR_MOTOR_CHECKSUM, calculateMotorChecksum());
}

// === АЦП: НАДЁЖНАЯ ЗАГРУЗКА/СОХРАНЕНИЕ ===
void loadADCCalibrationFromEEPROM() {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_ADC_MAGIC, magic);
  
  if (magic == ADC_CALIB_MAGIC) {
    // Загружаем offsets
    for (int i = 0; i < 5; i++) {
      EEPROM.get(EEPROM_ADDR_ADC_OFFSETS + i * 2, adcOffsets[i]);
    }
    // Загружаем углы
    for (int i = 0; i < 5; i++) {
      EEPROM.get(EEPROM_ADDR_ADC_ZERO_ANGLES + i * 2, adcZeroAngles[i]);
      if (adcZeroAngles[i] < 0 || adcZeroAngles[i] > 359) {
        adcZeroAngles[i] = 90; // fallback
      }
    }
  } else {
    // Сброс калибровки
    for (int i = 0; i < 5; i++) {
      adcOffsets[i] = 0;
      adcZeroAngles[i] = 90;
    }
  }
}

void saveADCCalibrationToEEPROM() {
  // 1. offsets
  for (int i = 0; i < 5; i++) {
    EEPROM.put(EEPROM_ADDR_ADC_OFFSETS + i * 2, adcOffsets[i]);
  }
  // 2. углы
  for (int i = 0; i < 5; i++) {
    EEPROM.put(EEPROM_ADDR_ADC_ZERO_ANGLES + i * 2, adcZeroAngles[i]);
  }
  // 3. МАГИЧЕСКОЕ ЧИСЛО — В КОНЦЕ!
  EEPROM.put(EEPROM_ADDR_ADC_MAGIC, ADC_CALIB_MAGIC);
}

// ================================================================
// ======================= АЦП ====================================
// ================================================================

void initADC() {
  ADMUX = (1 << REFS0);  // AVCC, right-adjusted
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // /128
}

uint16_t readADCChannelAvg(uint8_t channel, uint8_t samples) {
  ADMUX = (ADMUX & 0xE0) | (channel & 0x07);

  // ХОЛОСТОЕ ПРЕОБРАЗОВАНИЕ — критично для стабильности!
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));

  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    sum += ADC;
  }
  return (uint16_t)(sum / samples);
}

void performFastADCScan() {
  uint8_t oldSREG = SREG;
  cli();
  uint8_t adcsra_saved = ADCSRA;
  ADCSRA = (adcsra_saved & 0xF8) | 0x05; // prescaler = 32
  const uint8_t SAMPLES = 12;
  for (int ch = 0; ch < 5; ch++) {
    adcValuesAvg[ch] = readADCChannelAvg(ch, SAMPLES);
  }
  ADCSRA = adcsra_saved;
  SREG = oldSREG;
}

// ================================================================
// ======================= ДВИЖЕНИЕ ===============================
// ================================================================

// Проверка концевика A0 (магнитный концевой выключатель для мотора 2 - поворот кисти)
// Используется обработка дребезга для надежности
boolean checkLimitSwitch() {
  // Читаем аналоговое значение концевика
  // Обычно магнитный концевик дает LOW (0) при срабатывании
  int limitValue = analogRead(LIMIT_SWITCH_PIN);
  bool reading = (limitValue < 100);  // Сработал, если значение < 100
  
  // Обработка дребезга контактов
  if (reading != lastLimitSwitchState) {
    // Состояние изменилось - сбрасываем таймер
    lastDebounceTime = millis();
  }
  
  // Если прошло достаточно времени с последнего изменения
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Состояние стабильно - обновляем текущее состояние
    if (reading != currentLimitSwitchState) {
      currentLimitSwitchState = reading;
    }
  }
  
  lastLimitSwitchState = reading;
  return currentLimitSwitchState;
}

// Быстрое чтение концевика без обработки дребезга (для команды 'l')
int readLimitSwitchRaw() {
  // Читаем аналоговое значение без обработки дребезга
  return analogRead(LIMIT_SWITCH_PIN);
}

void manageMotorMovement() {
  // Проверка концевика - если сработал, останавливаем мотор
  if (checkLimitSwitch() && currentState != IDLE) {
    currentState = IDLE;
    stepsDone = 0;
    totalSteps = 0;
    emergencyStopFlag = true;
    return;
  }
  
  if (emergencyStopFlag) {
    currentState = IDLE;
    stepsDone = 0;
    totalSteps = 0;
    emergencyStopFlag = false;
    return;
  }

  if (newCommandPending && currentState != IDLE) {
    currentState = IDLE;
    stepsDone = 0;
    totalSteps = 0;
  }

  if (currentState == IDLE && newCommandPending) {
    if (newCommandSteps > 0) {
      currentDirection = newCommandForward;
      digitalWrite(DIR_PIN, currentDirection ? HIGH : LOW);
      totalSteps = newCommandSteps;
      stepsDone = 0;
      accelSteps = calculateAccelerationSteps();
      if (accelSteps * 2 > totalSteps) accelSteps = totalSteps / 2;
      currentState = ACCELERATING;
      currentDelay = 2000;
      lastStepTime = micros();
    }
    newCommandPending = false;
    return;
  }

  if (currentState != IDLE) {
    unsigned long now = micros();
    if (now - lastStepTime >= currentDelay) {
      lastStepTime = now;

      digitalWrite(STEP_PIN, HIGH);
      __asm__ __volatile__ ("nop\n\t""nop\n\t");
      digitalWrite(STEP_PIN, LOW);

      stepsDone++;

      if (currentState == ACCELERATING) {
        if (stepsDone < accelSteps) {
          if (currentDelay > params.maxSpeed) {
            currentDelay -= params.acceleration;
            if (currentDelay < params.maxSpeed) currentDelay = params.maxSpeed;
          }
        } else {
          currentState = CONSTANT_SPEED;
          currentDelay = params.maxSpeed;
        }
      } else if (currentState == CONSTANT_SPEED) {
        int constSteps = totalSteps - 2 * accelSteps;
        if (stepsDone >= accelSteps + constSteps) {
          currentState = DECELERATING;
        }
      } else if (currentState == DECELERATING) {
        currentDelay += params.acceleration;
      }

      if (stepsDone >= totalSteps) {
        currentState = IDLE;
      }
    }
  }
}

// ================================================================
// ======================= UART ===================================
// ================================================================

void processCommand() {
  inputString.trim();
  if (inputString.length() == 0) return;
  char cmd = inputString.charAt(0);

  if (cmd == 'f' || cmd == 'b') {
    if (inputString.length() > 1) {
      int steps = inputString.substring(1).toInt();
      if (steps > 0) {
        newCommandSteps = steps;
        newCommandForward = (cmd == 'f');
        newCommandPending = true;
        Serial.print("OK");
      } else {
        Serial.print("ERR:STEPS<=0");
      }
    } else {
      Serial.print("ERR:NO_STEPS");
    }
  }
  else if (cmd == 's') {
    if (inputString.length() == 1) {
      emergencyStopFlag = true;
      newCommandPending = false;
      Serial.print("STOP");
    } else {
      unsigned int val = inputString.substring(1).toInt();
      if (val >= 100 && val <= 5000) {
        params.maxSpeed = val;
        Serial.print("SPEED:"); Serial.print(params.maxSpeed);
      } else {
        Serial.print("ERR:SPEED_RANGE");
      }
    }
  }
  else if (cmd == 'a' && inputString.length() > 1) {
    unsigned int val = inputString.substring(1).toInt();
    if (val >= 5 && val <= 500) {
      params.acceleration = val;
      Serial.print("ACCEL:"); Serial.print(params.acceleration);
    } else {
      Serial.print("ERR:ACCEL_RANGE");
    }
  }
  else if (cmd == 'e') {
    digitalWrite(EN_PIN, LOW);
    Serial.print("ENABLED");
  }
  else if (cmd == 'd') {
    digitalWrite(EN_PIN, HIGH);
    Serial.print("DISABLED");
  }
  else if (cmd == 'p') {
    Serial.print("SPEED:"); Serial.print(params.maxSpeed);
    Serial.print(",ACCEL:"); Serial.print(params.acceleration);
    Serial.print(",ACCEL_STEPS:"); Serial.print(calculateAccelerationSteps());
  }
  else if (cmd == 'w') {
    saveParamsToEEPROM();
    saveADCCalibrationToEEPROM();
    Serial.print("SAVED");
  }
  else if (cmd == 'r') {
    loadParamsFromEEPROM();
    loadADCCalibrationFromEEPROM();
    Serial.print("LOADED");
  }
  else if (cmd == 'i') {
    Serial.print("READY");
  }
  else if (cmd == 'v') {
    performFastADCScan();
    for (int i = 0; i < 5; i++) {
      if (i > 0) Serial.print(" ");
      int32_t cal = (int32_t)adcValuesAvg[i] - adcOffsets[i];
      if (cal < 0) cal = 0;
      else if (cal > 1023) cal = 1023;
      Serial.print("A"); Serial.print(i); Serial.print(":"); Serial.print(cal);
    }
  }
  else if (cmd == 'a' && inputString.length() == 1) {
    performFastADCScan();
    for (int i = 0; i < 5; i++) {
      if (i > 0) Serial.print(" ");
      int32_t diff = (int32_t)adcValuesAvg[i] - adcOffsets[i];
      int32_t angle = adcZeroAngles[i] + (diff * 360L) / 1024;
      angle %= 360;
      if (angle < 0) angle += 360;
      Serial.print("A"); Serial.print(i); Serial.print(":"); Serial.print(angle);
    }
  }
  else if (cmd == 't') {
    unsigned long t0 = micros();
    performFastADCScan();
    unsigned long dt = micros() - t0;
    Serial.print("ADC_TIME:"); Serial.print(dt); Serial.print(" us");
  }
  else if (cmd == 'z') {
    if (inputString.length() == 1) {
      performFastADCScan();
      for (int i = 0; i < 5; i++) {
        adcOffsets[i] = adcValuesAvg[i];
        adcZeroAngles[i] = 90;
      }
      Serial.print("ZERO@90_SET");
    } else {
      int angle = inputString.substring(1).toInt();
      if (angle >= 0 && angle <= 359) {
        performFastADCScan();
        for (int i = 0; i < 5; i++) {
          adcOffsets[i] = adcValuesAvg[i];
          adcZeroAngles[i] = angle;
        }
        Serial.print("ZERO@"); Serial.print(angle); Serial.print("_SET");
      } else {
        Serial.print("ERR:ANGLE_RANGE");
      }
    }
  }
  else if (cmd == 'k') { // ПОКАЗАТЬ СОДЕРЖИМОЕ EEPROM (не RAM!)
    uint16_t magic;
    EEPROM.get(EEPROM_ADDR_ADC_MAGIC, magic);
    Serial.print("EEPROM MAGIC: 0x");
    Serial.print(magic, HEX);
    if (magic == ADC_CALIB_MAGIC) {
      Serial.println(" (VALID)");
    } else {
      Serial.println(" (INVALID → defaults)");
    }

    for (int i = 0; i < 5; i++) {
      int16_t off, ang;
      EEPROM.get(EEPROM_ADDR_ADC_OFFSETS + i * 2, off);
      EEPROM.get(EEPROM_ADDR_ADC_ZERO_ANGLES + i * 2, ang);
      Serial.print("A"); Serial.print(i);
      Serial.print(": offset="); Serial.print(off);
      Serial.print(", zeroAngle="); Serial.print(ang);
      Serial.println();
    }
  }
  else if (cmd == 'g') { // Управление сервоприводом MG996R (клешня)
    if (inputString.length() > 1) {
      int angle = inputString.substring(1).toInt();
      if (angle >= 0 && angle <= 180) {
        currentServoAngle = angle;
        gripperServo.write(angle);
        Serial.print("SERVO:"); Serial.print(angle);
      } else {
        Serial.print("ERR:SERVO_RANGE");
      }
    } else {
      // Если команда без параметра - возвращаем текущий угол
      Serial.print("SERVO:"); Serial.print(currentServoAngle);
    }
  }
  else if (cmd == 'l') { // Быстрое чтение концевика A0 (без обработки дребезга)
    int rawValue = readLimitSwitchRaw();
    bool triggered = checkLimitSwitch();  // С обработкой дребезга
    Serial.print("LIMIT:A0:");
    Serial.print(rawValue);
    Serial.print(",TRIG:");
    Serial.print(triggered ? "1" : "0");
  }
  else {
    Serial.print("ERR:UNKNOWN_CMD");
  }
  Serial.println();
}

void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
    } else if (c != '\r') {
      inputString += c;
    }
  }
}

// ================================================================
// ======================= SETUP & LOOP ===========================
// ================================================================

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  
  // Инициализация сервопривода MG996R
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(90);  // Начальная позиция - среднее положение
  currentServoAngle = 90;
  
  // Настройка пина концевика (A0 как аналоговый вход)
  pinMode(LIMIT_SWITCH_PIN, INPUT);

  initADC();
  loadParamsFromEEPROM();
  loadADCCalibrationFromEEPROM();  // ← ЗАГРУЖАЕТСЯ АВТОМАТИЧЕСКИ ПРИ СТАРТЕ

  Serial.begin(115200);
  Serial.println("DM542 READY");
}

void loop() {
  if (stringComplete) {
    processCommand();
    inputString = "";
    stringComplete = false;
  }
  manageMotorMovement();
}
