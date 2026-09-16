/*
 * Система управления машиной - Arduino Nano
 * Файл: machine2.ino
 * 
 * Логика:
 * - Управление щетками (мотор), воздухом и клапаном.
 * - Мониторинг АКБ с гистерезисом и индикацией.
 * - Глобальная блокировка (низкий заряд, зарядка или выкл ПММ).
 * - Все кнопки моментальные, активный HIGH. Для входов на A6 и A7 
 *   обязательно нужен внешний резистор pull-down: эти пины ATmega328
 *   не имеют цифрового входного буфера, поэтому читаются через ADC.
 */

// --- Настройки логирования ---
const bool kDebugMode = true;  // Установите false, чтобы отключить логирование в Serial

// --- Определение пинов ---

// Входы
const int PIN_BATTERY_ANALOG = A0;      // Напряжение АКБ (0-30V)
const int PIN_BRUSH_LIMIT_SW = A1;      // Концевик "щётки опущены" (active HIGH)
const int PIN_CHARGING_SIGNAL = A2;     // Сигнал зарядки (active HIGH)
const int PIN_AIR_BUTTON = A4;          // Кнопка "Воздух" (toggle)
const int PIN_VALVE_BUTTON = A5;        // Кнопка "Клапан" (toggle)
const int PIN_BRUSH_MOTOR_BTN = A6;     // Кнопка "вкл двиг щёток" (toggle)
const int PIN_PMM_BUTTON = A7;         // Кнопка "вкл ПММ" (toggle)

// Выходы
const int PIN_PMM_LAMP = 2;             // Лампа "ПММ вкл"
const int PIN_BRUSH_MOTOR_LAMP = 3;     // Лампа "двигатель щёток вращается"
const int PIN_BAT_BAD_LAMP = 4;         // Лампа АКБ "Плохо"
const int PIN_BAT_NORMAL_LAMP = 5;      // Лампа АКБ "Нормально"
const int PIN_BAT_GOOD_LAMP = 7;        // Лампа АКБ "Хорошо"
const int PIN_LIMIT_SW_LAMP = 8;        // Лампа "концевик щётки опущены"
const int PIN_AIR_LAMP = 9;             // Лампа "воздух"
const int PIN_VALVE_RELAY = 10;         // Реле Клапана
const int PIN_BRUSH_RELAY = 11;         // Реле Щёток (мотор)
const int PIN_AIR_RELAY = 12;           // Реле Воздуха

// --- Константы и пороги ---

const float VOLT_HIGH_THRESHOLD = 25.2;
const float VOLT_NORMAL_THRESHOLD = 23.0;
const float VOLT_LOW_THRESHOLD = 22.4;
const float HYSTERESIS = 0.3;

const unsigned long DEBOUNCE_DELAY = 50; 
const unsigned long BLINK_INTERVAL = 1000;

// Порог срабатывания для кнопок на пинах A6/A7.
// Эти пины ATmega328 не имеют цифрового входного буфера и НЕ поддерживают
// digitalRead() - их обязательно нужно читать через analogRead().
const int ANALOG_BUTTON_THRESHOLD = 512;

// Все выходы (реле и лампы) работают в активной логике LOW:
// LOW - выход включен, HIGH - выход выключен.
const int OUTPUT_ON = LOW;
const int OUTPUT_OFF = HIGH;

// --- Глобальные переменные состояния ---

// Состояние АКБ
enum BatteryState { STATE_BAD, STATE_NORMAL_BLINK, STATE_NORMAL_SOLID, STATE_GOOD };
BatteryState eCurrentBatState = STATE_GOOD;
float fCurrentVoltage = 0.0;

// Флаги системы
bool bPmmOn = false;
bool bAirWanted = false;
bool bValveWanted = false;
bool bMotorWanted = false;
bool bIsBlocked = false;

// Структуры для цифровых входов (антидребезг)
struct DigitalInput {
  int iPin;
  bool bLastRawState;
  bool bStableState;
  unsigned long ulLastDebounceTime;
};

struct Button {
  int iPin;
  bool bIsAnalogPin;   // true для A6/A7 - читать через analogRead()
  bool bLastRawState;
  bool bLastStableState;
  unsigned long ulLastDebounceTime;
};

DigitalInput diInputLimit = {PIN_BRUSH_LIMIT_SW, LOW, LOW, 0};
DigitalInput diInputCharging = {PIN_CHARGING_SIGNAL, LOW, LOW, 0};

Button btnAir = {PIN_AIR_BUTTON, false, LOW, LOW, 0};
Button btnValve = {PIN_VALVE_BUTTON, false, LOW, LOW, 0};
Button btnMotor = {PIN_BRUSH_MOTOR_BTN, true, LOW, LOW, 0};
Button btnPMM = {PIN_PMM_BUTTON, true, LOW, LOW, 0};

unsigned long ulLastBlinkTime = 0;
bool bBlinkToggle = false;

// --- Вспомогательные функции ---

// Логирование
void logMsg(const char* pszMsg) {
if (kDebugMode) {
  Serial.println(pszMsg);
}
}

void logMsg(const char* pszMsg, float fVal) {
if (kDebugMode) {
  Serial.print(pszMsg);
  Serial.println(fVal);
}
}

// Чтение аналогового пина и преобразование в Вольты
float readVoltage() {
  return (analogRead(PIN_BATTERY_ANALOG) / 1023.0) * 30.0;
}

// Получение стабильного состояния для непрерывных сигналов (концевик, зарядка)
bool getStableState(DigitalInput &di) {
  bool bCurrentRaw = digitalRead(di.iPin);
  bool bOldStable = di.bStableState;

  if (bCurrentRaw != di.bLastRawState) {
    di.ulLastDebounceTime = millis();
  }

  if ((millis() - di.ulLastDebounceTime) > DEBOUNCE_DELAY) {
    di.bStableState = bCurrentRaw;
  }
  
  di.bLastRawState = bCurrentRaw;
  
  if (bOldStable != di.bStableState) {
    if (di.iPin == PIN_BRUSH_LIMIT_SW) logMsg("Концевик щётки изменил состояние");
    if (di.iPin == PIN_CHARGING_SIGNAL) logMsg("Сигнал зарядки изменился");
  }
  
  return di.bStableState;
}

// Антидребезг и детектирование фронта для кнопок.
// Пины A6/A7 не поддерживают digitalRead() (нет цифрового буфера),
// поэтому для них читаем аналоговое значение и сравниваем с порогом.
bool checkButtonEdge(Button &b, const char* pszName) {
  bool bCurrentRaw;
  if (b.bIsAnalogPin) {
    bCurrentRaw = (analogRead(b.iPin) > ANALOG_BUTTON_THRESHOLD) ? HIGH : LOW;
  } else {
    bCurrentRaw = digitalRead(b.iPin);
  }
  bool bEdgeDetected = false;

  if (bCurrentRaw != b.bLastRawState) {
    b.ulLastDebounceTime = millis();
  }

  if ((millis() - b.ulLastDebounceTime) > DEBOUNCE_DELAY) {
    if (bCurrentRaw != b.bLastStableState) {
      if (bCurrentRaw == HIGH) {
        bEdgeDetected = true;
        if (kDebugMode) {
          Serial.print("Кнопка нажата: ");
          Serial.println(pszName);
        }
      }
      b.bLastStableState = bCurrentRaw;
    }
  }
  
  b.bLastRawState = bCurrentRaw;
  return bEdgeDetected;
}

// Определение состояния АКБ с гистерезисом.
// Переход в состояние с БОЛЕЕ высоким напряжением происходит при
// пересечении границы + HYSTERESIS, переход в состояние с более
// НИЗКИМ напряжением - при пересечении границы - HYSTERESIS.
// Это исключает "мёртвые зоны" и дребезг индикации возле порогов.
void updateBatteryState() {
  BatteryState eOldState = eCurrentBatState;

  switch (eCurrentBatState) {
    case STATE_GOOD:
      if (fCurrentVoltage <= (VOLT_HIGH_THRESHOLD - HYSTERESIS)) {
        eCurrentBatState = STATE_NORMAL_SOLID;
      }
      break;

    case STATE_NORMAL_SOLID:
      if (fCurrentVoltage > (VOLT_HIGH_THRESHOLD + HYSTERESIS)) {
        eCurrentBatState = STATE_GOOD;
      } else if (fCurrentVoltage <= (VOLT_NORMAL_THRESHOLD - HYSTERESIS)) {
        eCurrentBatState = STATE_NORMAL_BLINK;
      }
      break;

    case STATE_NORMAL_BLINK:
      if (fCurrentVoltage > (VOLT_NORMAL_THRESHOLD + HYSTERESIS)) {
        eCurrentBatState = STATE_NORMAL_SOLID;
      } else if (fCurrentVoltage <= (VOLT_LOW_THRESHOLD - HYSTERESIS)) {
        eCurrentBatState = STATE_BAD;
      }
      break;

    case STATE_BAD:
      if (fCurrentVoltage > (VOLT_LOW_THRESHOLD + HYSTERESIS)) {
        eCurrentBatState = STATE_NORMAL_BLINK;
      }
      break;
  }

  if (eOldState != eCurrentBatState) {
    logMsg("Состояние АКБ изменилось, напряжение: ", fCurrentVoltage);
  }
}

// --- Основной цикл ---

void setup() {
  Serial.begin(9600);
  logMsg("Запуск системы...");

  pinMode(PIN_PMM_LAMP, OUTPUT);
  pinMode(PIN_BRUSH_MOTOR_LAMP, OUTPUT);
  pinMode(PIN_BAT_BAD_LAMP, OUTPUT);
  pinMode(PIN_BAT_NORMAL_LAMP, OUTPUT);
  pinMode(PIN_BAT_GOOD_LAMP, OUTPUT);
  pinMode(PIN_LIMIT_SW_LAMP, OUTPUT);
  pinMode(PIN_AIR_LAMP, OUTPUT);
  pinMode(PIN_VALVE_RELAY, OUTPUT);
  pinMode(PIN_BRUSH_RELAY, OUTPUT);
  pinMode(PIN_AIR_RELAY, OUTPUT);

  pinMode(PIN_BRUSH_LIMIT_SW, INPUT);
  pinMode(PIN_CHARGING_SIGNAL, INPUT);
  pinMode(PIN_AIR_BUTTON, INPUT);
  pinMode(PIN_VALVE_BUTTON, INPUT);
  pinMode(PIN_BRUSH_MOTOR_BTN, INPUT);
  pinMode(PIN_PMM_BUTTON, INPUT);

  // Безопасное начальное состояние: все реле и лампы выключены
  digitalWrite(PIN_VALVE_RELAY, OUTPUT_OFF);
  digitalWrite(PIN_BRUSH_RELAY, OUTPUT_OFF);
  digitalWrite(PIN_AIR_RELAY, OUTPUT_OFF);
  digitalWrite(PIN_PMM_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_BRUSH_MOTOR_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_BAT_BAD_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_BAT_NORMAL_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_BAT_GOOD_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_LIMIT_SW_LAMP, OUTPUT_OFF);
  digitalWrite(PIN_AIR_LAMP, OUTPUT_OFF);
  logMsg("Система готова");
}

void loop() {
  fCurrentVoltage = readVoltage();
  bool bIsCharging = getStableState(diInputCharging);
  bool bBrushesLowered = getStableState(diInputLimit);

  updateBatteryState();

  if (checkButtonEdge(btnPMM, "ПММ")) {
    bool bChargingOrBad = (bIsCharging || eCurrentBatState == STATE_BAD);
    if (!bPmmOn) {
       if (!bChargingOrBad) {
         bPmmOn = true;
         logMsg("ПММ ВКЛ");
       }
    } else {
       bPmmOn = false;
       logMsg("ПММ ВЫКЛ");
    }
    if (!bPmmOn) {
      bAirWanted = false; bValveWanted = false; bMotorWanted = false;
    }
  }

  if (checkButtonEdge(btnAir, "Воздух")) {
    if (!bIsBlocked) {
      bAirWanted = !bAirWanted;
      logMsg("Запрос воздуха");
    }
  }

  if (checkButtonEdge(btnValve, "Клапан")) {
    if (!bIsBlocked) {
      bValveWanted = !bValveWanted;
      logMsg("Запрос клапана");
    }
  }

  if (checkButtonEdge(btnMotor, "Мотор щёток")) {
    if (!bIsBlocked && bBrushesLowered) {
      bMotorWanted = !bMotorWanted;
      logMsg("Запрос мотора щёток");
    }
  }

  bool bOldIsBlocked = bIsBlocked;
  bIsBlocked = (eCurrentBatState == STATE_BAD || bIsCharging || !bPmmOn);
  
  if (bOldIsBlocked != bIsBlocked) {
    if (bIsBlocked) {
      logMsg("!!! БЛОКИРОВКА СИСТЕМЫ !!!");
      bAirWanted = false; bValveWanted = false; bMotorWanted = false;
    } else {
      logMsg("Блокировка снята");
    }
  }

  if (bMotorWanted && !bBrushesLowered) {
    bMotorWanted = false;
    logMsg("Мотор выключен (щётки подняты)");
  }

  // Итоговое состояние выходов пересчитывается с явной проверкой блокировки
  // и условий безопасности (defense-in-depth), а не только на основе флагов
  // *Wanted, которые уже должны быть сброшены выше по логике.
  // Важно: все выходы активны по LOW (OUTPUT_ON = LOW, OUTPUT_OFF = HIGH).
  digitalWrite(PIN_PMM_LAMP, bPmmOn ? OUTPUT_ON : OUTPUT_OFF);

  bool bMotorActive = (bMotorWanted && !bIsBlocked && bBrushesLowered);
  digitalWrite(PIN_BRUSH_RELAY, bMotorActive ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(PIN_BRUSH_MOTOR_LAMP, bMotorActive ? OUTPUT_ON : OUTPUT_OFF);

  bool bAirActive = (bAirWanted && !bIsBlocked);
  digitalWrite(PIN_AIR_RELAY, bAirActive ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(PIN_AIR_LAMP, bAirActive ? OUTPUT_ON : OUTPUT_OFF);

  bool bValveActive = (bValveWanted && !bIsBlocked && bBrushesLowered);
  digitalWrite(PIN_VALVE_RELAY, bValveActive ? OUTPUT_ON : OUTPUT_OFF);

  digitalWrite(PIN_LIMIT_SW_LAMP, bBrushesLowered ? OUTPUT_ON : OUTPUT_OFF);

  digitalWrite(PIN_BAT_BAD_LAMP, (eCurrentBatState == STATE_BAD) ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(PIN_BAT_GOOD_LAMP, (eCurrentBatState == STATE_GOOD) ? OUTPUT_ON : OUTPUT_OFF);
  
  if (eCurrentBatState == STATE_NORMAL_SOLID) {
    digitalWrite(PIN_BAT_NORMAL_LAMP, OUTPUT_ON);
  } else if (eCurrentBatState == STATE_NORMAL_BLINK) {
    if (millis() - ulLastBlinkTime >= BLINK_INTERVAL) {
      ulLastBlinkTime = millis();
      bBlinkToggle = !bBlinkToggle;
    }
    digitalWrite(PIN_BAT_NORMAL_LAMP, bBlinkToggle ? OUTPUT_ON : OUTPUT_OFF);
  } else {
    digitalWrite(PIN_BAT_NORMAL_LAMP, OUTPUT_OFF);
  }
}
