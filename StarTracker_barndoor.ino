/*
 * Barndoor Star Tracker
 * Контроллер: M5Stack Atom Lite
 * Двигатель: 28BYJ-48 (биполярный) с редуктором 63.68395:1
 * Драйвер: DRV8825 с микрошагом 1/32
 * Шпилька: М6 (шаг 1мм)
 */

#include <M5Atom.h>
#include <Preferences.h>

// ==================== КОНСТАНТЫ НАСТРОЕК ====================

// Пины управления драйвером DRV8825
#define PIN_STEP           33      // Пин STEP драйвера
#define PIN_DIR            23      // Пин DIR драйвера
#define PIN_MICROSTEP_EN   22      // Пин для включения микрошага (все джамперы MS1, MS2, MS3)
#define PIN_ENDSTOP        25      // Пин концевого датчика
#define ENDSTOP_INVERT      true    // true = инвертировать логику (LOW = припаркованы), false = HIGH = припаркованы

// Параметры механики
#define ARM_LENGTH_MM      200.0   // Длина плеча в мм
#define SCREW_PITCH_MM     2.0     // Шаг резьбы в мм

// Параметры двигателя
#define MOTOR_STEPS_PER_REV    32          // Шагов на оборот двигателя (28BYJ-48)
#define GEAR_RATIO             63.68395     // Передаточное число редуктора
#define MICROSTEP_DIVISOR      32          // Делитель микрошага (1/32)
#define STEPS_PER_SCREW_REV    (MOTOR_STEPS_PER_REV * MICROSTEP_DIVISOR * GEAR_RATIO)  // ~65112 шагов на оборот шпильки (с микрошагом)
#define STEPS_PER_SCREW_REV_FULL_STEP    (MOTOR_STEPS_PER_REV * GEAR_RATIO)  // ~2035 шагов на оборот шпильки (без микрошага)

// Параметры углов
#define START_ANGLE_DEG        5.0         // Стартовый угол в градусах (при достижении концевого датчика)
#define MAX_ANGLE_DEG          60.0       // Максимальный угол раскрытия в градусах

// Астрономические константы
#define SIDEREAL_RATE_DEG_PER_HOUR    15.041  // Скорость вращения неба в градусах/час
#define SIDEREAL_RATE_DEG_PER_SEC     (SIDEREAL_RATE_DEG_PER_HOUR / 3600.0)  // ~0.004178°/сек

// Параметры кнопки
#define BUTTON_PIN           39      // Кнопка Atom Lite (GPIO39)
#define BUTTON_LONG_PRESS_MS  3000   // Время длинного нажатия в мс (3 секунды)
#define BUTTON_DEBOUNCE_MS    50     // Время антидребезга в мс

// Параметры индикации
#define LED_BLINK_INTERVAL_MS  500   // Интервал мигания светодиода в мс

// Параметры парковки (скорость)
// Ограничение: 28BYJ-48 может максимум ~1000-1500 шагов/сек без микрошага
// При наших параметрах это примерно 0.15-0.22°/сек
#define PARKING_SPEED_DEG_PER_SEC    0.2    // Скорость парковки в градусах/сек (без микрошага, ограничена возможностями мотора)
#define MAX_PARKING_STEPS_PER_SEC    500   // Максимальная скорость шагов для парковки (шагов/сек)

// Инверсия направления движения
#define INVERT_DIRECTION            false   // true = инвертировать направление (поменять местами вперед/назад)

// Параметры сохранения угла
#define SAVE_STEPS_WORKING    2048   // Сохранять каждые N шагов в рабочем режиме (с микрошагом)
#define SAVE_STEPS_PARKING    64     // Сохранять каждые N шагов в парковке (2048/32, без микрошага)

// ==================== ПЕРЕМЕННЫЕ СОСТОЯНИЯ ====================

enum SystemState {
  STATE_PAUSE,    // Пауза (стартовое состояние): мигающий желтый LED, мотор остановлен
  STATE_WORKING,  // Рабочий ход: вперед, микрошаг включен, максимально плавное движение
  STATE_PARKING   // Парковка: назад, микрошаг выключен, максимальная скорость
};

SystemState currentState = STATE_PAUSE;

// Позиция и движение
volatile long currentStepPosition = 0;     // Текущая позиция в шагах (0 = стартовое положение)
long lastSavedStepPosition = 0;            // Позиция последнего сохранения
long maxWorkingSteps = 0;                   // Максимальное количество шагов для рабочего режима
long lastScrewRevolutionSteps = 0;          // Позиция последнего оборота шпильки

// Угол
double currentAngleDeg = START_ANGLE_DEG;   // Текущий угол в градусах
bool isParked = false;                      // Флаг припаркованности (для определения, использовать ли сохраненный угол)

// Таймеры
unsigned long lastLEDToggle = 0;            // Время последнего переключения LED
bool ledOnState = false;                    // Текущее состояние LED (вкл/выкл)
unsigned long lastAngleSaveTime = 0;        // Время последнего сохранения угла

// Состояние кнопки (для прерывания)
volatile bool buttonPressedFlag = false;    // Флаг нажатия кнопки
volatile unsigned long buttonPressStartTime = 0;  // Время начала нажатия
volatile bool buttonLongPressDetected = false;     // Флаг длинного нажатия
volatile bool buttonShortPressDetected = false;    // Флаг короткого нажатия

// Концевой датчик
bool lastEndstopState = false;              // Предыдущее состояние концевого датчика

// Аппаратный таймер для генерации шагов
hw_timer_t * stepTimer = NULL;
volatile bool stepTimerFlag = false;        // Флаг для выполнения шага
volatile unsigned long stepIntervalUs = 0;  // Интервал между шагами в микросекундах
#define TIMER_FREQUENCY_HZ  1000000         // Частота таймера 1MHz (1 микросекунда на тик)

// Preferences для сохранения угла
Preferences preferences;

// ==================== ФУНКЦИИ РАСЧЕТОВ ====================

/**
 * Расчет угла поворота barndoor на основе перемещения шпильки
 * Для равнобедренного треугольника: tan(θ/2) = s/(2*L)
 */
double calculateAngleFromSteps(long steps, bool useMicrostep) {
  long stepsPerRev = useMicrostep ? STEPS_PER_SCREW_REV : STEPS_PER_SCREW_REV_FULL_STEP;
  double screwRevolutions = (double)steps / stepsPerRev;
  double screwDisplacement = screwRevolutions * SCREW_PITCH_MM;  // Перемещение шпильки в мм
  double angleRad = atan(screwDisplacement / (2.0 * ARM_LENGTH_MM)) * 2.0;  // Точная формула
  return angleRad * 180.0 / PI;  // Конвертация в градусы
}

/**
 * Расчет количества шагов для заданного угла
 */
long calculateStepsFromAngle(double angleDeg, bool useMicrostep) {
  double angleRad = angleDeg * PI / 180.0;
  double screwDisplacement = 2.0 * ARM_LENGTH_MM * tan(angleRad / 2.0);  // Точная формула
  double screwRevolutions = screwDisplacement / SCREW_PITCH_MM;
  long stepsPerRev = useMicrostep ? STEPS_PER_SCREW_REV : STEPS_PER_SCREW_REV_FULL_STEP;
  return (long)(screwRevolutions * stepsPerRev);
}

/**
 * Расчет интервала между шагами с компенсацией тангенциальной ошибки
 * Скорость пропорциональна cos²(θ/2)
 * @param angleDeg - текущий угол в градусах
 * @param useMicrostep - использовать микрошаг (true) или полный шаг (false)
 */
unsigned long calculateStepIntervalWithCompensation(double angleDeg, bool useMicrostep) {
  // Базовый интервал для sidereal rate
  double baseAngleSpeed = SIDEREAL_RATE_DEG_PER_SEC;
  
  // Компенсация тангенциальной ошибки: cos²(θ/2)
  double angleRad = angleDeg * PI / 180.0;
  double compensationFactor = cos(angleRad / 2.0);
  compensationFactor = compensationFactor * compensationFactor;  // cos²
  
  // Скорость с компенсацией
  double compensatedAngleSpeed = baseAngleSpeed / compensationFactor;
  
  // Угловая скорость в градусах/сек -> скорость шпильки в мм/сек
  double screwSpeedMmPerSec = ARM_LENGTH_MM * (compensatedAngleSpeed * PI / 180.0);
  double screwRevolutionsPerSec = screwSpeedMmPerSec / SCREW_PITCH_MM;
  long stepsPerRev = useMicrostep ? STEPS_PER_SCREW_REV : STEPS_PER_SCREW_REV_FULL_STEP;
  double stepsPerSec = screwRevolutionsPerSec * stepsPerRev;
  
  if (stepsPerSec <= 0) return 0;
  return (unsigned long)(1000000.0 / stepsPerSec);  // Интервал в микросекундах
}

/**
 * Расчет интервала между шагами для парковки (без компенсации, максимальная скорость)
 * Используется полный шаг (без микрошага) для максимальной скорости
 * Ограничено возможностями мотора 28BYJ-48 (~1000 шагов/сек максимум)
 */
unsigned long calculateParkingStepInterval() {
  // Угловая скорость в градусах/сек -> скорость шпильки в мм/сек
  // Для малых углов: dθ/dt ≈ (1/L) * ds/dt, поэтому ds/dt = L * dθ/dt
  double angleSpeedRadPerSec = PARKING_SPEED_DEG_PER_SEC * PI / 180.0;
  double screwSpeedMmPerSec = ARM_LENGTH_MM * angleSpeedRadPerSec;
  
  // Скорость шпильки -> обороты шпильки/сек
  double screwRevolutionsPerSec = screwSpeedMmPerSec / SCREW_PITCH_MM;
  
  // Обороты -> шаги/сек (используем полный шаг, без микрошага)
  double stepsPerSec = screwRevolutionsPerSec * STEPS_PER_SCREW_REV_FULL_STEP;
  
  // Ограничиваем скорость возможностями мотора
  if (stepsPerSec > MAX_PARKING_STEPS_PER_SEC) {
    stepsPerSec = MAX_PARKING_STEPS_PER_SEC;
  }
  
  if (stepsPerSec <= 0) return 0;
  
  unsigned long intervalUs = (unsigned long)(1000000.0 / stepsPerSec);
  
  // Минимальный интервал для шагового мотора (обычно не менее 500-1000 мкс для надежности)
  // 28BYJ-48 требует минимум ~1000 мкс (1 мс) между шагами для стабильной работы
  if (intervalUs < 1000) {
    intervalUs = 1000;  // Минимум 1000 мкс (1 мс) между шагами
  }
  
  return intervalUs;
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
  // Применяем инверсию направления, если задана
  bool actualDirection = INVERT_DIRECTION ? !forward : forward;
  digitalWrite(PIN_DIR, actualDirection ? HIGH : LOW);
}

void setMicrostep(bool enable) {
  // HIGH = микрошаг включен (1/32), LOW = полный шаг
  digitalWrite(PIN_MICROSTEP_EN, enable ? HIGH : LOW);
}

void performStep() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(2);  // Минимальный импульс для DRV8825
  digitalWrite(PIN_STEP, LOW);
}

// ==================== ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ====================

// Обработчик прерывания кнопки
void IRAM_ATTR buttonISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  // Антидребезг
  if (interruptTime - lastInterruptTime > BUTTON_DEBOUNCE_MS) {
    if (!buttonPressedFlag) {
      // Кнопка нажата
      buttonPressedFlag = true;
      buttonPressStartTime = interruptTime;
      buttonLongPressDetected = false;
      buttonShortPressDetected = false;
    }
  }
  lastInterruptTime = interruptTime;
}

// Обработчик аппаратного таймера для генерации шагов
void IRAM_ATTR onStepTimer() {
  stepTimerFlag = true;
}

// ==================== ФУНКЦИИ УПРАВЛЕНИЯ КНОПКОЙ ====================

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
}

void processButtonEvents() {
  if (buttonPressedFlag) {
    unsigned long currentTime = millis();
    unsigned long pressDuration = currentTime - buttonPressStartTime;
    
    // Проверяем, отпущена ли кнопка
    if (digitalRead(BUTTON_PIN) == HIGH) {
      // Кнопка отпущена
      buttonPressedFlag = false;
      
      if (pressDuration >= BUTTON_LONG_PRESS_MS) {
        // Длинное нажатие
        buttonLongPressDetected = true;
      } else if (pressDuration > BUTTON_DEBOUNCE_MS) {
        // Короткое нажатие
        buttonShortPressDetected = true;
      }
    } else {
      // Кнопка все еще нажата - проверяем длинное нажатие
      if (pressDuration >= BUTTON_LONG_PRESS_MS && !buttonLongPressDetected) {
        buttonLongPressDetected = true;
        handleLongPress();
        // Сбрасываем флаг, чтобы не обработать повторно после отпускания
        buttonLongPressDetected = false;
      }
    }
  }
  
  // Обработка длинного нажатия (после отпускания кнопки, если не было обработано ранее)
  if (buttonLongPressDetected) {
    buttonLongPressDetected = false;
    handleLongPress();
  }
  
  // Обработка короткого нажатия
  if (buttonShortPressDetected) {
    buttonShortPressDetected = false;
    handleShortPress();
  }
}

void handleShortPress() {
  switch (currentState) {
    case STATE_PAUSE:
      // Переход в рабочий режим
      enterWorkingMode();
      break;
      
    case STATE_WORKING:
      // Переход в паузу
      enterPauseMode();
      break;
      
    case STATE_PARKING:
      // Прерывание парковки, переход в паузу
      enterPauseMode();
      break;
  }
}

void handleLongPress() {
  if (currentState != STATE_PARKING) {
    // Запуск парковки
    enterParkingMode();
  }
}

// ==================== ФУНКЦИИ РЕЖИМОВ ====================

void enterPauseMode() {
  currentState = STATE_PAUSE;
  stopStepTimer();
  saveAngleToMemory();  // Сохраняем угол при смене режима
}

void enterWorkingMode() {
  currentState = STATE_WORKING;
  setMicrostep(true);   // Включаем микрошаг
  setMotorDirection(true);  // Вперед
  updateStepInterval();  // Обновляем интервал с учетом текущего угла
  startStepTimer();
  saveAngleToMemory();  // Сохраняем угол при смене режима
}

void enterParkingMode() {
  currentState = STATE_PARKING;
  // Сначала останавливаем таймер, если он работал
  stopStepTimer();
  // Устанавливаем направление назад (LOW)
  setMotorDirection(false);
  // Выключаем микрошаг (LOW) - это важно для максимальной скорости
  setMicrostep(false);
  // Небольшая задержка для установки сигналов
  delayMicroseconds(100);
  // Рассчитываем интервал для парковки
  stepIntervalUs = calculateParkingStepInterval();
  // Запускаем таймер
  if (stepIntervalUs > 0) {
    startStepTimer();
  }
  saveAngleToMemory();  // Сохраняем угол при смене режима
}

// ==================== ФУНКЦИИ ТАЙМЕРА ====================

void setupStepTimer() {
  // Создаем аппаратный таймер с частотой 1MHz (1 микросекунда на тик)
  stepTimer = timerBegin(TIMER_FREQUENCY_HZ);
  // Прикрепляем обработчик прерывания
  timerAttachInterrupt(stepTimer, &onStepTimer);
  // Останавливаем таймер (запустим позже)
  timerStop(stepTimer);
}

void startStepTimer() {
  if (stepIntervalUs > 0 && stepTimer != NULL) {
    // Устанавливаем будильник: alarm_value в тиках таймера, autoreload=true, reload_count=0 (не используется)
    timerAlarm(stepTimer, stepIntervalUs, true, 0);
    // Запускаем таймер
    timerStart(stepTimer);
  }
}

void stopStepTimer() {
  if (stepTimer != NULL) {
    timerStop(stepTimer);
  }
}

void updateStepInterval() {
  // Пересчитываем интервал с учетом текущего угла и компенсации
  stepIntervalUs = calculateStepIntervalWithCompensation(currentAngleDeg, true);
  // Если таймер работает, перезапускаем с новым интервалом
  if (currentState == STATE_WORKING && stepTimer != NULL) {
    timerStop(stepTimer);
    if (stepIntervalUs > 0) {
      timerAlarm(stepTimer, stepIntervalUs, true, 0);
      timerStart(stepTimer);
    }
  }
}

// ==================== ФУНКЦИИ ОБРАБОТКИ ШАГОВ ====================

void processStep() {
  // Проверяем флаг от аппаратного таймера
  if (!stepTimerFlag) return;
  stepTimerFlag = false;
  
  switch (currentState) {
    case STATE_WORKING:
      processWorkingStep();
      break;
      
    case STATE_PARKING:
      processParkingStep();
      break;
      
    default:
      break;
  }
}

void processWorkingStep() {
  // Проверяем максимальный угол
  if (currentStepPosition >= maxWorkingSteps) {
    // Достигли максимального угла - переходим в паузу
    enterPauseMode();
    return;
  }
  
  // Выполняем шаг
  performStep();
  currentStepPosition++;
  
  // Обновляем угол
  updateAngle(true);  // true = микрошаг
  
  // Пересчитываем интервал с учетом нового угла (компенсация тангенциальной ошибки)
  updateStepInterval();
  
  // Проверяем необходимость сохранения
  checkAndSaveAngle(true);  // true = микрошаг
}

void processParkingStep() {
  // Проверяем концевой датчик
  bool endstopRaw = digitalRead(PIN_ENDSTOP);
  bool endstopState = ENDSTOP_INVERT ? !endstopRaw : endstopRaw;
  
  // Проверяем, достигли ли концевого датчика (показывает "припаркованы")
  if (endstopState) {
    // Достигли концевого датчика - переходим в паузу
    // Обновляем состояние для отслеживания перехода
    lastEndstopState = true;
    enterPauseMode();
    return;
  }
  
  // Обновляем состояние концевика
  bool previousEndstopState = lastEndstopState;
  lastEndstopState = endstopState;
  
  // Проверяем переход из "припаркованы" в "не припаркованы" (выход из концевого датчика)
  if (previousEndstopState && !endstopState) {
    // Сброс угла на стартовый
    currentStepPosition = 0;
    currentAngleDeg = START_ANGLE_DEG;
    isParked = true;
    enterPauseMode();
    saveAngleToMemory();
    return;
  }
  
  // Выполняем шаг назад
  performStep();
  currentStepPosition--;
  
  // Обновляем угол
  updateAngle(false);  // false = полный шаг
  
  // Проверяем необходимость сохранения
  checkAndSaveAngle(false);  // false = полный шаг
}

// ==================== ФУНКЦИИ ОТСЛЕЖИВАНИЯ УГЛА ====================

void updateAngle(bool useMicrostep) {
  // Вычисляем угол из текущей позиции (используем абсолютное значение)
  double calculatedAngle = calculateAngleFromSteps(abs(currentStepPosition), useMicrostep);
  
  // Если позиция отрицательная (движение назад), уменьшаем угол от стартового
  if (currentStepPosition < 0) {
    currentAngleDeg = START_ANGLE_DEG - calculatedAngle;
    // Не позволяем углу быть меньше 0
    if (currentAngleDeg < 0) {
      currentAngleDeg = 0;
    }
  } else {
    // Если позиция положительная (движение вперед), увеличиваем угол от стартового
    currentAngleDeg = START_ANGLE_DEG + calculatedAngle;
  }
}

void checkAndSaveAngle(bool useMicrostep) {
  long saveInterval = useMicrostep ? SAVE_STEPS_WORKING : SAVE_STEPS_PARKING;
  long stepsSinceSave = abs(currentStepPosition - lastSavedStepPosition);
  
  // Проверяем, прошло ли достаточно шагов
  if (stepsSinceSave >= saveInterval) {
    saveAngleToMemory();
    lastSavedStepPosition = currentStepPosition;
  }
  
  // Проверяем, прошел ли полный оборот шпильки
  long stepsPerRev = useMicrostep ? STEPS_PER_SCREW_REV : STEPS_PER_SCREW_REV_FULL_STEP;
  long stepsSinceRev = abs(currentStepPosition - lastScrewRevolutionSteps);
  if (stepsSinceRev >= stepsPerRev) {
    saveAngleToMemory();
    lastScrewRevolutionSteps = currentStepPosition;
  }
}

// ==================== ФУНКЦИИ СОХРАНЕНИЯ/ЗАГРУЗКИ УГЛА ====================

void saveAngleToMemory() {
  preferences.begin("tracker", false);
  preferences.putDouble("angle", currentAngleDeg);
  preferences.putLong("steps", currentStepPosition);
  // isParked НЕ сохраняем - определяется только по концевому датчику
  preferences.end();
}

void loadAngleFromMemory() {
  preferences.begin("tracker", false);
  
  if (preferences.isKey("angle") && preferences.isKey("steps")) {
    // Загружаем сохраненные значения для компенсации тангенциальной ошибки
    double savedAngle = preferences.getDouble("angle", START_ANGLE_DEG);
    long savedSteps = preferences.getLong("steps", 0);
    
    currentAngleDeg = savedAngle;
    currentStepPosition = savedSteps;
    lastSavedStepPosition = savedSteps;
    lastScrewRevolutionSteps = savedSteps;
  } else {
    // Первый запуск - используем стартовый угол
    currentAngleDeg = START_ANGLE_DEG;
    currentStepPosition = 0;
    lastSavedStepPosition = 0;
    lastScrewRevolutionSteps = 0;
  }
  
  preferences.end();
  
  // isParked НЕ загружается из памяти - определяется только по концевому датчику
}

// ==================== ФУНКЦИИ ИНДИКАЦИИ ====================

void updateLED() {
  unsigned long currentTime = millis();
  
  switch (currentState) {
    case STATE_PAUSE:
      // Постоянно светящийся желтый
      M5.dis.drawpix(0, 0xffff00);  // Желтый постоянно включен
      break;
      
    case STATE_WORKING:
      // Зеленый мигающий (мигание не влияет на плавность, т.к. шаги генерируются аппаратным таймером)
      if ((currentTime - lastLEDToggle) > LED_BLINK_INTERVAL_MS) {
        lastLEDToggle = currentTime;
        ledOnState = !ledOnState;
        M5.dis.drawpix(0, ledOnState ? 0x00ff00 : 0x000000);  // Зеленый или выключен
      }
      break;
      
    case STATE_PARKING:
      // Мигающий красный
      if ((currentTime - lastLEDToggle) > LED_BLINK_INTERVAL_MS) {
        lastLEDToggle = currentTime;
        ledOnState = !ledOnState;
        M5.dis.drawpix(0, ledOnState ? 0xff0000 : 0x000000);  // Красный или выключен
      }
      break;
  }
}

// ==================== ФУНКЦИИ КОНЦЕВОГО ДАТЧИКА ====================

void checkEndstop() {
  // Проверяем концевой датчик только если не в режиме парковки
  // (в парковке проверка выполняется в processParkingStep)
  if (currentState == STATE_PARKING) {
    return;
  }
  
  bool endstopRaw = digitalRead(PIN_ENDSTOP);
  bool endstopState = ENDSTOP_INVERT ? !endstopRaw : endstopRaw;
  
  // Проверяем переход из "припаркованы" в "не припаркованы" (выход из концевого датчика)
  if (lastEndstopState && !endstopState) {
    // Сброс угла на стартовый
    currentStepPosition = 0;
    currentAngleDeg = START_ANGLE_DEG;
    isParked = true;
    saveAngleToMemory();
  }
  
  lastEndstopState = endstopState;
}

// ==================== ИНИЦИАЛИЗАЦИЯ ====================

void setup() {
  // Инициализация M5Atom
  M5.begin(true, false, true);  // Serial, I2C, Display
  delay(50);
  
  // Инициализация Serial ПЕРВОЙ для отладки
  Serial.begin(115200);
  delay(100);  // Даем время Serial инициализироваться
  
  // Настройка пинов
  setupMotorPins();
  setupButton();
  
  // Настройка концевого датчика
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);
  delay(10);  // Небольшая задержка для стабилизации пина
  
  // Загрузка угла из памяти (для компенсации тангенциальной ошибки)
  loadAngleFromMemory();
  
  // Определяем состояние парковки ТОЛЬКО по концевому датчику
  // Читаем несколько раз для надежности
  bool endstopState1 = digitalRead(PIN_ENDSTOP);
  delay(5);
  bool endstopState2 = digitalRead(PIN_ENDSTOP);
  delay(5);
  bool endstopState3 = digitalRead(PIN_ENDSTOP);
  // Используем значение, которое встречается минимум 2 раза из 3
  bool endstopRaw = (endstopState1 && endstopState2) || (endstopState2 && endstopState3) || (endstopState1 && endstopState3);
  // Инвертируем логику, если нужно
  bool endstopState = ENDSTOP_INVERT ? !endstopRaw : endstopRaw;
  
  Serial.print("Endstop readings (raw): ");
  Serial.print(endstopState1);
  Serial.print(" ");
  Serial.print(endstopState2);
  Serial.print(" ");
  Serial.print(endstopState3);
  Serial.print(" -> raw: ");
  Serial.print(endstopRaw);
  Serial.print(", inverted: ");
  Serial.println(endstopState);
  
  if (endstopState) {
    // Концевик показывает "припаркованы" - мы в стартовом положении
    // Сбрасываем позицию и угол на стартовые значения
    currentStepPosition = 0;
    currentAngleDeg = START_ANGLE_DEG;
    isParked = true;
    Serial.println("Endstop -> PARKED (reset to 0)");
    saveAngleToMemory();
  } else {
    // Концевик показывает "не припаркованы"
    // Используем загруженные из памяти значения угла и позиции для компенсации тангенциальной ошибки
    isParked = false;
    Serial.println("Endstop -> NOT PARKED (using angle/position from memory for compensation)");
    // Сохраняем текущее состояние
    saveAngleToMemory();
  }
  lastEndstopState = endstopState;
  
  // Инициализация расчетов
  maxWorkingSteps = calculateStepsFromAngle(MAX_ANGLE_DEG, true);
  
  // Настройка таймера шагов
  setupStepTimer();
  
  // Установка начального направления
  setMotorDirection(true);
  
  // Начальное состояние: пауза
  currentState = STATE_PAUSE;
  
  // Начальная индикация
  M5.dis.drawpix(0, 0xffff00);  // Желтый
  
  Serial.println("Barndoor Star Tracker initialized");
  Serial.print("Current angle: ");
  Serial.print(currentAngleDeg);
  Serial.println(" degrees");
  Serial.print("Current steps: ");
  Serial.println(currentStepPosition);
  Serial.print("Max working steps: ");
  Serial.println(maxWorkingSteps);
  Serial.print("Is parked: ");
  Serial.println(isParked ? "Yes" : "No");
}

// ==================== ГЛАВНЫЙ ЦИКЛ ====================

void loop() {
  M5.update();  // Обновление M5Stack
  
  // Обработка событий кнопки
  processButtonEvents();
  
  // Обработка шагов от таймера
  processStep();
  
  // Обновление LED индикации
  updateLED();
  
  // Проверка концевого датчика
  checkEndstop();
  
  // Небольшая задержка для стабильности
  delay(1);
}
