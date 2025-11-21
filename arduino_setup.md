# Настройка Arduino IDE для M5Stack Atom Lite

## Шаг 1: Установка поддержки ESP32

1. Откройте Arduino IDE
2. Перейдите в **File → Preferences**
3. В поле **Additional Boards Manager URLs** добавьте:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Нажмите **OK**

## Шаг 2: Установка платы ESP32

1. Перейдите в **Tools → Board → Boards Manager**
2. В поиске введите **"ESP32"**
3. Найдите **"esp32 by Espressif Systems"**
4. Нажмите **Install** (выберите последнюю стабильную версию)
5. Дождитесь завершения установки

## Шаг 3: Установка библиотеки M5Atom

1. Перейдите в **Sketch → Include Library → Manage Libraries**
2. В поиске введите **"M5Atom"**
3. Найдите **"M5Atom"** от M5Stack
4. Нажмите **Install**
5. Дождитесь завершения установки

## Шаг 4: Настройка платы

1. Подключите M5Stack Atom Lite к компьютеру через USB
2. В Arduino IDE:
   - **Tools → Board → ESP32 Arduino → M5Stack-ATOM**
   - **Tools → Port →** выберите порт вашего устройства (обычно `/dev/tty.usbserial-*` или `/dev/cu.usbserial-*` на Mac)
   - **Tools → Upload Speed → 115200** (или выше, если поддерживается)
   - **Tools → CPU Frequency → 240MHz (WiFi/BT)**
   - **Tools → Flash Frequency → 80MHz**
   - **Tools → Flash Size → 4MB (32Mb)**
   - **Tools → Partition Scheme → Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)**

## Шаг 5: Компиляция и загрузка

1. Откройте файл `StarTracker_barndoor.ino`
2. Нажмите **Verify** (✓) для проверки компиляции
3. Если компиляция успешна, нажмите **Upload** (→) для загрузки в устройство
4. Дождитесь завершения загрузки

## Проверка работы

После загрузки откройте **Serial Monitor** (Tools → Serial Monitor):
- Скорость: **115200 baud**
- Должны появиться сообщения о инициализации трекера

## Возможные проблемы

### Порт не определяется
- Проверьте USB кабель (должен поддерживать передачу данных)
- Попробуйте другой USB порт
- На Mac может потребоваться драйвер CH340 или CP2102

### Ошибка при загрузке
- Удерживайте кнопку на Atom Lite при подключении USB
- Попробуйте снизить Upload Speed до 115200
- Проверьте, что выбран правильный порт

### Ошибка компиляции
- Убедитесь, что установлена библиотека M5Atom
- Проверьте версию ESP32 Board Manager (должна быть актуальной)

