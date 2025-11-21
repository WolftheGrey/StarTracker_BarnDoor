/*
 * Barndoor Star Tracker
 * Контроллер: M5Stack Atom Lite
 * Двигатель: 28BYJ-48 (биполярный) с редуктором 63.68395:1
 * Драйвер: DRV8825 с микрошагом 1/32
 * Шпилька: М6 (шаг 1мм)
 */

#include <M5Atom.h>

// ==================== КОНСТАНТЫ НАСТРОЕК ====================

// Пины управления драйвером DRV8825
#define PIN_STEP           33      // Пин STEP драйвера
#define PIN_DIR            23      // Пин DIR драйвера
#define PIN_MICROSTEP_EN   19      // Пин для включения микрошага (все джамперы MS1, MS2, MS3)

// Параметры механики
#define ARM_LENGTH_MM      200.0   // Длина плеча в мм (по умолчанию 200мм)
#define SCREW_PITCH_MM     1.0     // Шаг резьбы М6 в мм

// Параметры двигателя
#define MOTOR_STEPS_PER_REV    32          // Шагов на оборот двигателя (28BYJ-48)
#define GEAR_RATIO             63.68395     // Передаточное число редуктора
#define MICROSTEP_DIVISOR      32          // Делитель микрошага (1/32)
#define STEPS_PER_SCREW_REV    (MOTOR_STEPS_PER_REV * MICROSTEP_DIVISOR * GEAR_RATIO)  // ~65112 шагов на оборот шпильки

// Параметры парковки
#define PARKING_ANGLE_DEG      5.0         // Угол парковки в градусах (по умолчанию 5°)

// Астрономические константы
#define SIDEREAL_RATE_DEG_PER_HOUR    15.041  // Скорость вращения неба в градусах/час
#define SIDEREAL_RATE_DEG_PER_SEC     (SIDEREAL_RATE_DEG_PER_HOUR / 3600.0)  // ~0.004178°/сек

// Параметры кнопки
#define BUTTON_PIN           39      // Кнопка Atom Lite (GPIO39)
#define BUTTON_DEBOUNCE_MS   50      // Время антидребезга в мс
#define BUTTON_HOLD_MS       200     // Время удержания для ручного управления в мс

// Параметры индикации
#define LED_BLINK_INTERVAL_MS  500   // Интервал мигания светодиода в мс

// Параметры парковки (скорость)
#define PARKING_SPEED_DEG_PER_SEC    2.0   // Скорость парковки в градусах/сек (быстрее чем отслеживание)

// Заготовка для концевого датчика (пока не используется)
#define ENDSTOP_PIN           -1     // Пин концевого датчика (пока не подключен, -1 = отключен)
#define ENDSTOP_ACTIVE_LOW    true   // true если датчик активен на LOW (обычно для механических)

// ==================== ПЕРЕМЕННЫЕ СОСТОЯНИЯ ====================

enum SystemState {
  STATE_TRACKING,  // Отслеживание звезд
  STATE_PARKING,   // Возврат в нулевое положение
  STATE_PARKED     // Припаркован, ожидание
};

SystemState currentState = STATE_TRACKING;

// Позиция и движение
long currentStepPosition = 0;        // Текущая позиция в шагах (0 = начальное положение)
long parkingStepTarget = 0;         // Целевая позиция для парковки (всегда 0)
long maxTrackingSteps = 0;          // Максимальное количество шагов для отслеживания

// Таймеры для неблокирующей работы
unsigned long lastStepTime = 0;      // Время последнего шага
unsigned long stepInterval = 0;      // Интервал между шагами в микросекундах (рассчитывается)
unsigned long parkingStepInterval = 0; // Интервал между шагами при парковке
unsigned long lastButtonCheck = 0;   // Время последней проверки кнопки
unsigned long lastLEDToggle = 0;     // Время последнего переключения LED

// Состояние кнопки
bool buttonState = false;
bool buttonLastState = false;
bool buttonHeld = false;
bool buttonPressed = false;         // Флаг для обработки короткого нажатия
unsigned long buttonPressTime = 0;
SystemState stateBeforeManual = STATE_TRACKING;  // Состояние до ручного управления

// Состояние LED
bool ledState = false;

// ==================== ФУНКЦИИ РАСЧЕТОВ ====================

/**
 * Расчет угла поворота barndoor на основе перемещения шпильки
 * Для равнобедренного треугольника: tan(θ/2) = s/(2*L)
 * Для малых углов: θ ≈ s/L (в радианах)
 */
double calculateAngleFromSteps(long steps) {
  double screwRevolutions = (double)steps / STEPS_PER_SCREW_REV;
  double screwDisplacement = screwRevolutions * SCREW_PITCH_MM;  // Перемещение шпильки в мм
  double angleRad = atan(screwDisplacement / (2.0 * ARM_LENGTH_MM)) * 2.0;  // Точная формула
  return angleRad * 180.0 / PI;  // Конвертация в градусы
}

/**
 * Расчет количества шагов для заданного угла
 */
long calculateStepsFromAngle(double angleDeg) {
  double angleRad = angleDeg * PI / 180.0;
  double screwDisplacement = 2.0 * ARM_LENGTH_MM * tan(angleRad / 2.0);  // Точная формула
  double screwRevolutions = screwDisplacement / SCREW_PITCH_MM;
  return (long)(screwRevolutions * STEPS_PER_SCREW_REV);
}

/**
 * Расчет интервала между шагами для заданной угловой скорости
 */
unsigned long calculateStepInterval(double angleSpeedDegPerSec) {
  // Угловая скорость в градусах/сек -> скорость шпильки в мм/сек
  // Для малых углов: dθ/dt ≈ (1/L) * ds/dt
  // ds/dt = L * dθ/dt
  double screwSpeedMmPerSec = ARM_LENGTH_MM * (angleSpeedDegPerSec * PI / 180.0);
  double screwRevolutionsPerSec = screwSpeedMmPerSec / SCREW_PITCH_MM;
  double stepsPerSec = screwRevolutionsPerSec * STEPS_PER_SCREW_REV;
  
  if (stepsPerSec <= 0) return 0;
  return (unsigned long)(1000000.0 / stepsPerSec);  // Интервал в микросекундах
}

// ==================== ФУНКЦИИ УПРАВЛЕНИЯ ДВИГАТЕЛЕМ ====================

void setupMotorPins() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_MICROSTEP_EN, OUTPUT);
  
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_MICROSTEP_EN, HIGH);  // Включаем микрошаг 1/32 (все джамперы в HIGH)
}

void setMotorDirection(bool forward) {
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
}

void performStep() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(2);  // Минимальный импульс для DRV8825
  digitalWrite(PIN_STEP, LOW);
}

// ==================== ФУНКЦИИ УПРАВЛЕНИЯ КНОПКОЙ ====================

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonLastState = digitalRead(BUTTON_PIN);
}

void updateButtonState() {
  unsigned long currentTime = millis();
  
  // Читаем состояние кнопки (инвертируем, т.к. INPUT_PULLUP)
  bool currentButtonRead = !digitalRead(BUTTON_PIN);
  
  // Антидребезг
  if (currentButtonRead != buttonLastState) {
    lastButtonCheck = currentTime;
  }
  
  bool previousButtonState = buttonState;
  
  if ((currentTime - lastButtonCheck) > BUTTON_DEBOUNCE_MS) {
    buttonState = currentButtonRead;
  }
  
  buttonLastState = currentButtonRead;
  
  // Определение нажатия и удержания кнопки
  if (buttonState && !previousButtonState) {
    // Кнопка только что нажата
    buttonPressTime = currentTime;
    buttonPressed = false;
    buttonHeld = false;
  } else if (buttonState && previousButtonState) {
    // Кнопка удерживается
    if (!buttonHeld && (currentTime - buttonPressTime) > BUTTON_HOLD_MS) {
      // Переход в режим удержания
      buttonHeld = true;
      buttonPressed = false;  // Отменяем короткое нажатие
      stateBeforeManual = currentState;  // Сохраняем текущее состояние
      currentState = STATE_PARKING;      // Переходим в режим парковки
    }
  } else if (!buttonState && previousButtonState) {
    // Кнопка отпущена
    if (buttonHeld) {
      // Завершаем ручное управление
      buttonHeld = false;
      if (currentStepPosition <= 0) {
        // Если достигли нулевой позиции, переходим в PARKED
        currentStepPosition = 0;
        currentState = STATE_PARKED;
      } else {
        // Возвращаемся к предыдущему состоянию
        currentState = stateBeforeManual;
      }
    } else if (!buttonPressed && (currentTime - buttonPressTime) < BUTTON_HOLD_MS) {
      // Короткое нажатие (не удержание)
      buttonPressed = true;
      handleButtonClick();
    }
  }
}

void handleButtonClick() {
  switch (currentState) {
    case STATE_TRACKING:
      // Переход в режим парковки
      currentState = STATE_PARKING;
      break;
      
    case STATE_PARKED:
      // Начало отслеживания
      currentState = STATE_TRACKING;
      break;
      
    case STATE_PARKING:
      // Если уже паркуемся, короткое нажатие не делает ничего
      break;
  }
}

// ==================== ФУНКЦИИ ИНДИКАЦИИ ====================

void updateLED() {
  unsigned long currentTime = millis();
  
  switch (currentState) {
    case STATE_TRACKING:
      // Зеленый мигающий
      if ((currentTime - lastLEDToggle) > LED_BLINK_INTERVAL_MS) {
        ledState = !ledState;
        lastLEDToggle = currentTime;
        M5.dis.drawpix(0, ledState ? 0x00ff00 : 0x000000);  // Зеленый или выключен
      }
      break;
      
    case STATE_PARKING:
      // Красный мигающий
      if ((currentTime - lastLEDToggle) > LED_BLINK_INTERVAL_MS) {
        ledState = !ledState;
        lastLEDToggle = currentTime;
        M5.dis.drawpix(0, ledState ? 0xff0000 : 0x000000);  // Красный или выключен
      }
      break;
      
    case STATE_PARKED:
      // Желтый постоянный
      M5.dis.drawpix(0, 0xffff00);  // Желтый
      ledState = true;
      break;
  }
}

// ==================== ФУНКЦИИ РЕЖИМОВ РАБОТЫ ====================

void updateTracking() {
  unsigned long currentTime = micros();
  
  // Проверяем, нужно ли сделать шаг
  if ((currentTime - lastStepTime) >= stepInterval && stepInterval > 0) {
    // Проверяем, не достигли ли максимального угла
    if (currentStepPosition < maxTrackingSteps) {
      performStep();
      currentStepPosition++;
      lastStepTime = currentTime;
    } else {
      // Достигли максимального угла - останавливаемся
      // Можно перейти в PARKED или просто остановиться
      // Пока просто останавливаемся, оставаясь в TRACKING
    }
  }
}

void updateParking() {
  unsigned long currentTime = micros();
  unsigned long interval = buttonHeld ? parkingStepInterval : stepInterval;  // При удержании используем быструю скорость
  
  // Проверка концевого датчика (если подключен)
  if (ENDSTOP_PIN >= 0) {
    bool endstopTriggered = ENDSTOP_ACTIVE_LOW ? (digitalRead(ENDSTOP_PIN) == LOW) : (digitalRead(ENDSTOP_PIN) == HIGH);
    if (endstopTriggered) {
      // Достигли концевого датчика
      currentStepPosition = 0;  // Сбрасываем счетчик
      if (!buttonHeld) {
        currentState = STATE_PARKED;
      }
      return;
    }
  }
  
  // Движение в направлении парковки (уменьшение currentStepPosition)
  if ((currentTime - lastStepTime) >= interval && interval > 0) {
    if (currentStepPosition > parkingStepTarget) {
      setMotorDirection(false);  // Назад (к парковке)
      performStep();
      currentStepPosition--;
      lastStepTime = currentTime;
    } else {
      // Достигли позиции парковки
      currentStepPosition = 0;  // Сбрасываем счетчик
      if (!buttonHeld) {
        // Если не удерживаем кнопку, переходим в PARKED
        currentState = STATE_PARKED;
      }
    }
  }
}

void updateParked() {
  // В режиме ожидания ничего не делаем
  // Двигатель не работает
}

// ==================== ИНИЦИАЛИЗАЦИЯ ====================

void setup() {
  // Инициализация M5Atom
  M5.begin(true, false, true);  // Serial, I2C, Display
  delay(50);
  
  // Настройка пинов
  setupMotorPins();
  setupButton();
  
  // Инициализация расчетов
  parkingStepTarget = 0;
  maxTrackingSteps = calculateStepsFromAngle(90.0);  // Максимальный угол 90° (можно настроить)
  
  // Расчет интервала шагов для отслеживания
  stepInterval = calculateStepInterval(SIDEREAL_RATE_DEG_PER_SEC);
  
  // Расчет интервала шагов для парковки (быстрее)
  parkingStepInterval = calculateStepInterval(PARKING_SPEED_DEG_PER_SEC);
  
  // Установка начального направления (вперед для отслеживания)
  setMotorDirection(true);
  
  // Настройка концевого датчика (если подключен)
  if (ENDSTOP_PIN >= 0) {
    pinMode(ENDSTOP_PIN, ENDSTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  }
  
  // Начальное состояние LED
  M5.dis.drawpix(0, 0x00ff00);  // Зеленый
  
  // Инициализация таймеров
  lastStepTime = micros();
  lastButtonCheck = millis();
  lastLEDToggle = millis();
  
  // Начальное состояние: отслеживание
  currentState = STATE_TRACKING;
  
  Serial.begin(115200);
  Serial.println("Barndoor Star Tracker initialized");
  Serial.print("Step interval: ");
  Serial.print(stepInterval);
  Serial.println(" microseconds");
  Serial.print("Max tracking steps: ");
  Serial.println(maxTrackingSteps);
}

// ==================== ГЛАВНЫЙ ЦИКЛ ====================

void loop() {
  M5.update();  // Обновление M5Stack (кнопка и т.д.)
  
  // Обновление состояния кнопки
  updateButtonState();
  
  // Обновление LED индикации
  updateLED();
  
  // Обработка текущего режима
  switch (currentState) {
    case STATE_TRACKING:
      updateTracking();
      break;
      
    case STATE_PARKING:
      updateParking();
      break;
      
    case STATE_PARKED:
      updateParked();
      break;
  }
  
  // Небольшая задержка для стабильности (не критично, т.к. используем микросекунды для шагов)
  delay(1);
}

