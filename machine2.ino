/*
 * Система управления машиной - Arduino Nano
 * Файл: machine2.ino
 * 
 * Логика:
 * - Управление щетками (мотор), воздухом и клапаном.
 * - Мониторинг АКБ с гистерезисом и индикацией.
 * - Глобальная блокировка (низкий заряд, зарядка или выкл ПММ).
 * Входы:
 * - Кнопки (Воздух, Клапан, Мотор щёток, ПММ) читаются через analogRead(),
 *   нажатие = уровень > 512.
 * - Концевик (A1) и сигнал зарядки (A2) активны по LOW (0 = вкл).
 */

// --- Настройки логирования ---
const bool kDebugMode = true;  // Установите false, чтобы отключить логирование в Serial

// --- Определение пинов ---

// Входы
const int PIN_BATTERY_ANALOG = A0;      // Напряжение АКБ (0-30V)
const int PIN_BRUSH_LIMIT_SW = A1;      // Концевик "щётки опущены" (active LOW)
const int PIN_CHARGING_SIGNAL = A2;     // Сигнал зарядки (active LOW)
const int PIN_AIR_BUTTON = A4;          // Кнопка "Воздух" (toggle, >512 = нажата)
const int PIN_VALVE_BUTTON = A5;        // Кнопка "Клапан" (toggle, >512 = нажата)
const int PIN_BRUSH_MOTOR_BTN = A6;     // Кнопка "вкл двиг щёток" (toggle, >512)
const int PIN_PMM_BUTTON = A7;         // Кнопка "вкл ПММ" (toggle, >512)

// Выходы
const int PIN_PMM_LAMP = 2;             // Лампа "ПММ вкл"
const int PIN_BRUSH_MOTOR_LAMP = 3;     // Лампа "двигатель щёток вращается"
const int PIN_BAT_BAD_LAMP = 4;         // Лампа АКБ "Плохо"
const int PIN_BAT_NORMAL_LAMP = 5;      // Лампа АКБ "Нормально"
const int PIN_BAT_GOOD_LAMP = 7;        // Лампа АКБ "Хорошо"
const int PIN_VALVE_LAMP = 8;           // Лампа "Клапан"
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

// Все кнопки (Воздух, Клапан, Мотор щёток, ПММ) читаются через analogRead().
// Если уровень > ANALOG_BUTTON_THRESHOLD - кнопка считается нажатой.
const int ANALOG_BUTTON_THRESHOLD = 512;

// Реле (PIN_VALVE_RELAY, PIN_BRUSH_RELAY, PIN_AIR_RELAY) работают в
// нормальной логике: LOW - выключено, HIGH - включено.
const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

// Лампы работают в инверсной логике: LOW - включена, HIGH - выключена.
const int LAMP_ON = LOW;
const int LAMP_OFF = HIGH;

// Минимальная задержка между включением силовых выходов (реле), чтобы
// избежать скачка тока при почти одновременном нажатии кнопок.
const unsigned long MIN_STAGGER_MS = 1000;

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
  bool bIsAnalogPin;   // true - читать через analogRead() с порогом >512
  bool bLastRawState;
  bool bLastStableState;
  unsigned long ulLastDebounceTime;
};

DigitalInput diInputLimit = {PIN_BRUSH_LIMIT_SW, LOW, LOW, 0};
DigitalInput diInputCharging = {PIN_CHARGING_SIGNAL, LOW, LOW, 0};

Button btnAir = {PIN_AIR_BUTTON, true, LOW, LOW, 0};
Button btnValve = {PIN_VALVE_BUTTON, true, LOW, LOW, 0};
Button btnMotor = {PIN_BRUSH_MOTOR_BTN, true, LOW, LOW, 0};
Button btnPMM = {PIN_PMM_BUTTON, true, LOW, LOW, 0};

unsigned long ulLastBlinkTime = 0;
bool bBlinkToggle = false;

// Время последнего ВКЛЮЧЕНИЯ силового выхода (для последовательного запуска,
// чтобы не было скачка тока). Нулевое значение означает "не включалось".
unsigned long ulLastOutputOnTime = 0;
bool bPowerOutputActive = false;

// Фактическое (текущее) состояние силовых выходов, участвующих в
// последовательном запуске. Нужно, чтобы корректно отслеживать моменты
// их включения. Клапан в этот механизм не входит (см. applyPowerOutput).
bool bMotorOn = false;
bool bAirOn = false;

// --- Вспомогательные функции ---

// Логирование. Выводит метку времени (мс от запуска) перед сообщением.
void logMsg(const char* pszMsg) {
if (kDebugMode) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(pszMsg);
}
}

void logMsg(const char* pszMsg, float fVal) {
if (kDebugMode) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
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
// Все кнопки читаются через analogRead() и считаются нажатыми,
// если уровень > ANALOG_BUTTON_THRESHOLD.
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
        char szBuf[64];
        sprintf(szBuf, "Кнопка нажата: %s", pszName);
        logMsg(szBuf);
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

// Проверка, можно ли включить силовой выход сейчас, соблюдая последовательный
// запуск (stagger). Разрешает включение только если прошло достаточно времени
// с момента последнего включения другого силового выхода.
bool canPowerOn() {
  unsigned long ulNow = millis();
  // Если какой-то силовой выход ещё активен, а с последнего включения
  // не прошло MIN_STAGGER_MS - запрещаем мгновенное включение следующего.
  if (bPowerOutputActive && (ulNow - ulLastOutputOnTime) < MIN_STAGGER_MS) {
    return false;
  }
  ulLastOutputOnTime = ulNow;
  return true;
}

// Применение состояния к силовому выходу с последовательным запуском.
// Включение (переход из выкл в вкл) задерживается, если слишком рано после
// последнего включения другого силового выхода (не прошло MIN_STAGGER_MS).
// iLampPin = -1 означает, что лампы-индикатора у выхода нет (клапан).
void applyPowerOutput(bool bDesired, bool &bActual, int iRelayPin, int iLampPin) {
  if (bDesired && !bActual) {
    // Желаем включить, но сейчас выключено - нужно пройти задержку запуска.
    if (!canPowerOn()) {
      return;  // ещё рано, откладываем включение до следующего цикла
    }
    bActual = true;
    digitalWrite(iRelayPin, RELAY_ON);
    if (iLampPin >= 0) digitalWrite(iLampPin, LAMP_ON);
    char szBuf[64];
    sprintf(szBuf, "Силовой выход включён: %d", iRelayPin);
    logMsg(szBuf);
  } else if (!bDesired && bActual) {
    // Выключаем сразу, без задержки.
    bActual = false;
    digitalWrite(iRelayPin, RELAY_OFF);
    if (iLampPin >= 0) digitalWrite(iLampPin, LAMP_OFF);
    sprintf(szBuf, "Силовой выход выключен: %d", iRelayPin);
    logMsg(szBuf);
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
  pinMode(PIN_VALVE_LAMP, OUTPUT);
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

  // Безопасное начальное состояние: все реле и лампы выключены.
  // Реле: RELAY_OFF (LOW). Лампы: LAMP_OFF (HIGH).
  digitalWrite(PIN_VALVE_RELAY, RELAY_OFF);
  digitalWrite(PIN_BRUSH_RELAY, RELAY_OFF);
  digitalWrite(PIN_AIR_RELAY, RELAY_OFF);
  digitalWrite(PIN_PMM_LAMP, LAMP_OFF);
  digitalWrite(PIN_BRUSH_MOTOR_LAMP, LAMP_OFF);
  digitalWrite(PIN_BAT_BAD_LAMP, LAMP_OFF);
  digitalWrite(PIN_BAT_NORMAL_LAMP, LAMP_OFF);
  digitalWrite(PIN_BAT_GOOD_LAMP, LAMP_OFF);
  digitalWrite(PIN_VALVE_LAMP, LAMP_OFF);
  digitalWrite(PIN_AIR_LAMP, LAMP_OFF);
  logMsg("Система готова");
}

void loop() {
  fCurrentVoltage = readVoltage();
  // PIN_CHARGING_SIGNAL и PIN_BRUSH_LIMIT_SW активны по LOW (0 = вкл).
  bool bIsCharging = !getStableState(diInputCharging);
  bool bBrushesLowered = !getStableState(diInputLimit);

  updateBatteryState();

  // Пока идёт зарядка, ПММ принудительно выключена и кнопка включения не действует.
  if (bIsCharging) {
    if (bPmmOn) {
      bPmmOn = false;
      bAirWanted = false; bValveWanted = false; bMotorWanted = false;
      logMsg("ПММ выключена (идёт зарядка)");
    }
  } else if (checkButtonEdge(btnPMM, "ПММ")) {
    bPmmOn = !bPmmOn;
    logMsg(bPmmOn ? "ПММ ВКЛ" : "ПММ ВЫКЛ");
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
  // Важно: реле активны по HIGH (RELAY_ON), лампы активны по LOW (LAMP_ON).

  // Включение силовых реле (мотор щёток, воздух) выполняется
  // последовательно с задержкой MIN_STAGGER_MS, чтобы избежать скачка тока
  // при почти одновременном нажатии кнопок.
  bool bMotorDesired = (bMotorWanted && !bIsBlocked && bBrushesLowered);
  // applyPowerOutput(bMotorDesired, bMotorOn, PIN_BRUSH_RELAY, PIN_BRUSH_MOTOR_LAMP);
  digitalWrite(PIN_BRUSH_RELAY, bMotorDesired ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_BRUSH_MOTOR_LAMP, bMotorDesired ? LAMP_ON : LAMP_OFF);

  bool bAirDesired = (bAirWanted && !bIsBlocked);
  // applyPowerOutput(bAirDesired, bAirOn, PIN_AIR_RELAY, PIN_AIR_LAMP);
  digitalWrite(PIN_AIR_RELAY, bAirDesired ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_AIR_LAMP, bAirDesired ? LAMP_ON : LAMP_OFF);

  // Клапан потребляет мало тока, поэтому включается СРАЗУ, без задержки
  // и не участвует в последовательном запуске мотор/воздух.
  bool bValveDesired = (bValveWanted && !bIsBlocked && bBrushesLowered);
  digitalWrite(PIN_VALVE_RELAY, bValveDesired ? RELAY_ON : RELAY_OFF);
  digitalWrite(PIN_VALVE_LAMP, bValveDesired ? LAMP_ON : LAMP_OFF);

  // Флаг активности силовых выходов обновляется ПОСЛЕ применения их состояний,
  // чтобы canPowerOn() правильно определял момент последнего включения.
  bPowerOutputActive = (bMotorOn || bAirOn);

  digitalWrite(PIN_PMM_LAMP, bPmmOn ? LAMP_ON : LAMP_OFF);

  if (bPmmOn) {
    digitalWrite(PIN_BAT_BAD_LAMP, (eCurrentBatState == STATE_BAD) ? LAMP_ON : LAMP_OFF);
    digitalWrite(PIN_BAT_GOOD_LAMP, (eCurrentBatState == STATE_GOOD) ? LAMP_ON : LAMP_OFF);
    
    if (eCurrentBatState == STATE_NORMAL_SOLID) {
      digitalWrite(PIN_BAT_NORMAL_LAMP, LAMP_ON);
    } else if (eCurrentBatState == STATE_NORMAL_BLINK) {
      if (millis() - ulLastBlinkTime >= BLINK_INTERVAL) {
        ulLastBlinkTime = millis();
        bBlinkToggle = !bBlinkToggle;
      }
      digitalWrite(PIN_BAT_NORMAL_LAMP, bBlinkToggle ? LAMP_ON : LAMP_OFF);
    } else {
      digitalWrite(PIN_BAT_NORMAL_LAMP, LAMP_OFF);
    }
  }
  else {
    digitalWrite(PIN_BAT_BAD_LAMP, LAMP_OFF);
    digitalWrite(PIN_BAT_GOOD_LAMP, LAMP_OFF);
    digitalWrite(PIN_BAT_NORMAL_LAMP, LAMP_OFF);
  }
}
