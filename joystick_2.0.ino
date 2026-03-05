// ---------- Настройки пинов -----------------
constexpr int PIN_K2 = A4;
constexpr int PIN_K3 = A3;
constexpr int PIN_PWM = 5;
constexpr int PIN_JOY = A0;
constexpr int PIN_ENGINE_CURRENT = A2;

constexpr int PIN_BRUSH = A7;
constexpr int PIN_BRUSH_PWM = 10;

constexpr int PIN_AIR = A6;
constexpr int PIN_AIR_PWM = 9;

constexpr int PIN_PUMP_AND_VALVE = A1;
constexpr int PIN_VALVE = 6;
constexpr int PIN_PUMP_PWM = 3;

constexpr int PIN_PUMP_MODE_0_LED = 7;
constexpr int PIN_PUMP_MODE_1_LED = 8;
constexpr int PIN_PUMP_MODE_2_LED = 11;
constexpr int PIN_PUMP_MODE_3_LED = 12;

// ---------- Настройки движения машины -------
constexpr int iFwdEdge = 550;
constexpr int iBcwEdge = 350;

constexpr unsigned long ulRelayDelay = 100;      // Задержка переключения реле (мс)
constexpr unsigned long ulRelayStopDelay = 2500;      // Задержка переключения реле (мс) при стопе
constexpr unsigned long ulSpeedStepDelay = 10;   // Интервал шагов ШИМ (мс) обратное ускорению
constexpr float fFrwAcceleration = 1.0;          // Ускорение разгона вперёд
constexpr float fBkwAcceleration = 1.0;          // Ускорение разгона назад
constexpr float fFrwStopAcceleration = 1.0;      // Ускорение торможения вперёд
constexpr float fBkwStopAcceleration = 1.0;      // Ускорение торможения назад
constexpr int uiRelayStopEngineCurrent = 471;    // значение A2 остановка двигателя (2.3V / 5.0V) * 1024 = 471
constexpr int iMaxFwdSpeed = 230;
constexpr int iMaxBcwSpeed = 120;

// ---------- Настройки щёток и воздуха -------
constexpr unsigned long ulBrushAirStepDelay = 10;    // Интервал шагов ШИМ (мс)
constexpr unsigned long ulBrushAirRampTime  = 300;   // время разгона (мс)
constexpr int iBrushAirMaxPWM = 255;

// ---------- Настройка клапана и насосов -----
constexpr int iPumpMaxPWMLevels[] = {128, 192, 255};
constexpr int iPumpMinPWM = 0;    // мин скорость PWM насоса (0-255)
constexpr int iStartPumpTimeout = 100;  // время между открытием клапана и пуском насосов
constexpr int iStopPumpTimeout = 100;  // время между закрытием клапана и стопом насосов

// ----------- Проверки параметров ------------
static_assert(iFwdEdge >= 0 && iFwdEdge <= 1023, "iFwdEdge должен быть в диапазоне 0..1023");
static_assert(iBcwEdge >= 0 && iBcwEdge <= 1023, "iBcwEdge должен быть в диапазоне 0..1023");
static_assert(iFwdEdge > iBcwEdge, "iFwdEdge должен быть больше iBcwEdge");

static_assert(iMaxFwdSpeed >= 0 && iMaxFwdSpeed <= 255, "iMaxFwdSpeed должен быть в диапазоне 0..255");
static_assert(iMaxBcwSpeed >= 0 && iMaxBcwSpeed <= 255, "iMaxBcwSpeed должен быть в диапазоне 0..255");

static_assert(iPumpMinPWM >= 0 && iPumpMinPWM <= 255, "iPumpMinPWM должен быть в диапазоне 0..255");
static_assert((sizeof(iPumpMaxPWMLevels) / sizeof(iPumpMaxPWMLevels[0])) > 0, "iPumpMaxPWMLevels не должен быть пустым");
static_assert(iPumpMaxPWMLevels[0] >= iPumpMinPWM && iPumpMaxPWMLevels[0] <= 255, "iPumpMaxPWMLevels[0] должен быть в диапазоне 0..255");
static_assert(iPumpMaxPWMLevels[1] >= iPumpMinPWM && iPumpMaxPWMLevels[1] <= 255, "iPumpMaxPWMLevels[1] должен быть в диапазоне 0..255");
static_assert(iPumpMaxPWMLevels[2] >= iPumpMinPWM && iPumpMaxPWMLevels[2] <= 255, "iPumpMaxPWMLevels[2] должен быть в диапазоне 0..255");
static_assert(iPumpMaxPWMLevels[0] <= iPumpMaxPWMLevels[1] && iPumpMaxPWMLevels[1] <= iPumpMaxPWMLevels[2], "iPumpMaxPWMLevels должен быть неубывающим");

static_assert(iStartPumpTimeout >= 0, "iStartPumpTimeout должен быть >= 0");
static_assert(iStopPumpTimeout >= 0, "iStopPumpTimeout должен быть >= 0");
static_assert(ulRelayDelay > 0, "ulRelayDelay должен быть > 0");
static_assert(ulSpeedStepDelay > 0, "ulSpeedStepDelay должен быть > 0");
static_assert(fFrwAcceleration > 0.0f, "fFrwAcceleration должен быть > 0");
static_assert(fBkwAcceleration > 0.0f, "fBkwAcceleration должен быть > 0");
static_assert(ulBrushAirStepDelay > 0, "ulBrushAirStepDelay должен быть > 0");
static_assert(ulBrushAirRampTime > 0, "ulBrushAirRampTime должен быть > 0");

static_assert(PIN_PUMP_PWM != PIN_PUMP_MODE_0_LED, "Конфликт пинов: PWM насоса и LED режима");
static_assert(PIN_PUMP_PWM != PIN_PUMP_MODE_1_LED, "Конфликт пинов: PWM насоса и LED режима");
static_assert(PIN_PUMP_PWM != PIN_PUMP_MODE_2_LED, "Конфликт пинов: PWM насоса и LED режима");
static_assert(PIN_PUMP_PWM != PIN_PUMP_MODE_3_LED, "Конфликт пинов: PWM насоса и LED режима");
static_assert(PIN_PUMP_MODE_0_LED != PIN_PUMP_MODE_1_LED, "Пины LED режимов должны быть разными");
static_assert(PIN_PUMP_MODE_0_LED != PIN_PUMP_MODE_2_LED, "Пины LED режимов должны быть разными");
static_assert(PIN_PUMP_MODE_0_LED != PIN_PUMP_MODE_3_LED, "Пины LED режимов должны быть разными");
static_assert(PIN_PUMP_MODE_1_LED != PIN_PUMP_MODE_2_LED, "Пины LED режимов должны быть разными");
static_assert(PIN_PUMP_MODE_1_LED != PIN_PUMP_MODE_3_LED, "Пины LED режимов должны быть разными");
static_assert(PIN_PUMP_MODE_2_LED != PIN_PUMP_MODE_3_LED, "Пины LED режимов должны быть разными");

// ---------- Перечисление состояний машины ----------
enum State {
  E_STOP,
  E_TO_ZERO_FWD,
  E_TO_ZERO_BCW,
  E_RELAY_OFF,
  E_RELAY_K2_ON,
  E_RELAY_K3_ON,
  E_RUN_FORWARD,
  E_RUN_BACKWARD,
  E_RELAY_LOW_WAIT_FWD,
  E_RELAY_LOW_WAIT_BCW
};

State eState = E_STOP;

// ----- Перечисление состояний насосов и клапанов -----
enum PumpValveState {
  EPV_STOP,
  EPV_STARTING,
  EPV_RUNNING,
  EPV_STOPPING
};

PumpValveState ePumpValveState = EPV_STOP;

// ---------- Рабочие переменные ----------
float fSpeed = 0.0f;
float fSpeedTarget = 0.0f;

int iK2 = LOW;
int iK3 = LOW;

unsigned long ulStateStart = 0;
unsigned long ulLastSpeedStep = 0;
unsigned long ulLastSpeedUpdate = 0;

bool bForward = false;
bool bBackward = false;

unsigned long ulNow = 0;
float fJoy = 0.0;

unsigned int uiEngineCurrent = 0;

int iBrushPWM = 0;
int iAirPWM = 0;
unsigned long ulLastBrushAirStep = 0;

int iPumpPWM = 0;
int iValve = LOW;

uint8_t uiPumpMode = 0;
int iPumpBtnLast = LOW;
unsigned long ulPumpStateStart = 0;

// ---------- Буфер выходов ----------
int iOutK2 = LOW;
int iOutK3 = LOW;
int iOutPWM = 0;

// ---------- Прототипы ----------
float fJoystickValue();
void setState(State eNewState);
void brakeSpeed(float fMaxSpeed, float fAcceleration);
void accelerateSpeed(float fMaxSpeed, float fAcceleration);
void machine();
int speedSyncroPWM(int iMinPWM, int iMaxPWM);
void brushAndAir();
void pumpsAndValve();

// ------------ Антидребезг ---------
class DebounceInput {
public:
    DebounceInput(int pin, unsigned long debounceMs = 20)
        : pin(pin), debounceMs(debounceMs)
    {}

    void begin() {
        pinMode(pin, INPUT);
        lastStableState = digitalRead(pin);
        lastReadState = lastStableState;
        lastChangeTime = millis();
    }

    void update() {
        //int current = digitalRead(pin);
        int current = analogRead(pin) > 512 ? HIGH : LOW;

        if (current != lastReadState) {
            lastReadState = current;
            lastChangeTime = millis();
        }

        if ((millis() - lastChangeTime) >= debounceMs) {
            lastStableState = lastReadState;
        }
    }

    int read() const {
        return lastStableState;
    }

private:
    int pin;
    unsigned long debounceMs;

    int lastStableState;
    int lastReadState;
    unsigned long lastChangeTime;
};

DebounceInput debBrush(PIN_BRUSH);
DebounceInput debAir(PIN_AIR);
DebounceInput debPump(PIN_PUMP_AND_VALVE);

// ---------- SETUP ----------
void setup() {
  pinMode(PIN_K2, OUTPUT);
  pinMode(PIN_K3, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_JOY, INPUT);
  pinMode(PIN_ENGINE_CURRENT, INPUT);

  digitalWrite(PIN_K2, LOW);
  digitalWrite(PIN_K3, LOW);
  analogWrite(PIN_PWM, 0);
  
  pinMode(PIN_BRUSH, INPUT);
  pinMode(PIN_BRUSH_PWM, OUTPUT);
  analogWrite(PIN_BRUSH_PWM, 0);

  pinMode(PIN_AIR, INPUT);
  pinMode(PIN_AIR_PWM, OUTPUT);
  analogWrite(PIN_AIR_PWM, 0);
  
  debBrush.begin();
  debAir.begin();
  
  pinMode(PIN_PUMP_AND_VALVE, INPUT);

  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_VALVE, LOW);

  pinMode(PIN_PUMP_PWM, OUTPUT);
  analogWrite(PIN_PUMP_PWM, 0);
  
  debPump.begin();

  pinMode(PIN_PUMP_MODE_0_LED, OUTPUT);
  pinMode(PIN_PUMP_MODE_1_LED, OUTPUT);
  pinMode(PIN_PUMP_MODE_2_LED, OUTPUT);
  pinMode(PIN_PUMP_MODE_3_LED, OUTPUT);
  digitalWrite(PIN_PUMP_MODE_0_LED, LOW);
  digitalWrite(PIN_PUMP_MODE_1_LED, LOW);
  digitalWrite(PIN_PUMP_MODE_2_LED, LOW);
  digitalWrite(PIN_PUMP_MODE_3_LED, LOW);

  Serial.begin(115200);

  setState(E_STOP);
}

// ---------- LOOP ----------
void loop() {
  ulNow = millis();
  fJoy = fJoystickValue();
  bForward = fJoy > 0.05;
  bBackward = fJoy < -0.05;
  uiEngineCurrent = analogRead(PIN_ENGINE_CURRENT);
  
  Serial.println("-------------------------");
  Serial.println( millis() );
  Serial.print("Joystick: ");
  Serial.println( fJoy * 100 );
  Serial.print("Engine current: ");
  Serial.println(uiEngineCurrent);
  
  machine();
  brushAndAir();
  pumpsAndValve();
}

// ---------- Чтение джойстика ----------
float fJoystickValue() {
  int iValue = analogRead(PIN_JOY);
  float fJoystick = 0.0;

  if (iValue > iFwdEdge)
    fJoystick = float(iValue - iFwdEdge) / float(1023 - iFwdEdge);
  else if (iValue < iBcwEdge)
    fJoystick = -float(iBcwEdge - iValue) / float(iBcwEdge);

  return fJoystick;
}

void machine() {

  // ---------- Логика состояния ----------
  switch (eState) {

    case E_STOP: {
      fSpeed = 0.0f; 
      iOutK2 = LOW;
      iOutK3 = LOW;
      iOutPWM = 0;

      if (bForward) {
        fSpeedTarget = fJoy * iMaxFwdSpeed;
        setState(E_RELAY_K3_ON);
      } 
      else if (bBackward) {
        fSpeedTarget = -fJoy * iMaxBcwSpeed;
        setState(E_RELAY_K2_ON);
      }
      break;
    }

    case E_TO_ZERO_FWD: {
      iOutK2 = LOW;
      iOutK3 = HIGH;

      brakeSpeed(iMaxFwdSpeed, fFrwStopAcceleration);

      if (fSpeed <= 0.5f) {
        fSpeed = 0.0f;
        setState(E_RELAY_LOW_WAIT_FWD);
      }

      iOutPWM = int(fSpeed + 0.5f);  // округление
      break;
    }

    case E_TO_ZERO_BCW: {
      iOutK2 = HIGH;
      iOutK3 = LOW;

      brakeSpeed(iMaxBcwSpeed, fBkwStopAcceleration);

      if (fSpeed <= 0.5f) {
        fSpeed = 0.0f;
        setState(E_RELAY_LOW_WAIT_BCW);
      }

      iOutPWM = int(fSpeed + 0.5f);
      break;
    }

    case E_RELAY_LOW_WAIT_FWD: {
      iOutK2 = LOW;
      iOutK3 = HIGH;
      iOutPWM = 0;
      // if (uiEngineCurrent >= uiRelayStopEngineCurrent) {
      //    setState(E_RELAY_OFF);
      // }
      if (ulNow - ulStateStart >= ulRelayStopDelay) {
        setState(E_RELAY_OFF);
      }
      else if (bForward) {
        setState(E_RUN_FORWARD);
      }
      break;
    }

    case E_RELAY_LOW_WAIT_BCW: {
      iOutK2 = HIGH;
      iOutK3 = LOW;
      iOutPWM = 0;
      // if (uiEngineCurrent >= uiRelayStopEngineCurrent) {
      //    setState(E_RELAY_OFF);
      // }
      if (ulNow - ulStateStart >= ulRelayStopDelay) {
        setState(E_RELAY_OFF);
      }
      else if (bBackward) {
        setState(E_RUN_BACKWARD);
      }      
      break;
    }

    case E_RELAY_OFF: {
      iOutK2 = LOW;
      iOutK3 = LOW;
      iOutPWM = 0;
      if (ulNow - ulStateStart >= ulRelayDelay) {
        if (bForward)
          setState(E_RELAY_K3_ON);
        else if (bBackward)
          setState(E_RELAY_K2_ON);
        else
          setState(E_STOP);
      }
      break;
    }

    case E_RELAY_K2_ON: {
      iOutK2 = HIGH;
      iOutK3 = LOW;
      if (ulNow - ulStateStart >= ulRelayDelay)
        setState(E_RUN_BACKWARD);
      break;
    }

    case E_RELAY_K3_ON: {
      iOutK3 = HIGH;
      iOutK2 = LOW;
      if (ulNow - ulStateStart >= ulRelayDelay)
        setState(E_RUN_FORWARD);
      break;
    }

    case E_RUN_FORWARD: {
      iOutK3 = HIGH;
      iOutK2 = LOW;

      if (!bForward) {
        setState(E_TO_ZERO_FWD);
      } else {
        fSpeedTarget = fJoy * iMaxFwdSpeed;
        accelerateSpeed(iMaxFwdSpeed, fFrwAcceleration);
      }
      iOutPWM = int(fSpeed + 0.5f);
      break;
    }

    case E_RUN_BACKWARD: {
      iOutK2 = HIGH;
      iOutK3 = LOW;

      if (!bBackward) {
        setState(E_TO_ZERO_BCW);
      } else {
        fSpeedTarget = -fJoy * iMaxBcwSpeed;
        accelerateSpeed(iMaxBcwSpeed, fBkwAcceleration);
      }
      iOutPWM = int(fSpeed + 0.5f);
      break;
    }

    default:
      setState(E_STOP);
      break;
  }

  // ---------- Применение выходов ----------
  digitalWrite(PIN_K2, iOutK2);
  digitalWrite(PIN_K3, iOutK3);
  analogWrite(PIN_PWM, iOutPWM);

  Serial.print( "state: " );
  Serial.println( eState );
  Serial.print( "speed: " );
  Serial.println( iOutPWM );
}

// ---------- Переключение состояния ----------
void setState(State eNewState) {
  eState = eNewState;
  ulStateStart = ulNow;
  ulLastSpeedStep = ulNow;
  ulLastSpeedUpdate = ulNow;
}

// ---------- Расчёт торможения ---------------
void brakeSpeed(float fMaxSpeed, float fAcceleration) {
    float dt = (ulNow - ulLastSpeedUpdate) * 0.001f;
    ulLastSpeedUpdate = ulNow;

    float dec = fMaxSpeed * fAcceleration * dt;
    fSpeed -= dec;

    if (fSpeed < 0.1f)
        fSpeed = 0.0f;
}

// ---------- Расчёт разгона/изменения скорости ---------------
void accelerateSpeed(float fMaxSpeed, float fAcceleration) {
    float dt = (ulNow - ulLastSpeedUpdate) * 0.001f;
    ulLastSpeedUpdate = ulNow;

    float step = fMaxSpeed * fAcceleration * dt;

    if (fSpeed < fSpeedTarget) {
        fSpeed += step;
        if (fSpeed > fSpeedTarget)
            fSpeed = fSpeedTarget;
    } else if (fSpeed > fSpeedTarget) {
        fSpeed -= step;
        if (fSpeed < fSpeedTarget)
            fSpeed = fSpeedTarget;
    }

    if (fSpeed < 0.1f)
        fSpeed = 0.0f;
    if (fSpeed > fMaxSpeed)
        fSpeed = fMaxSpeed;
}

// --- Синхронное со скоростью значение ----
int speedSyncroPWM(int iMinPWM, int iMaxPWM) {
    if (fSpeed <= 0.5f)
        return 0;  // остановка — насос выключен

    // коэффициент скорости 0..1
    float k = fSpeed / float(iMaxFwdSpeed);

    // расчёт PWM
    int iPWM = int(k * float(iMaxPWM - iMinPWM)) + iMinPWM;

    // ограничение снизу (кроме случая fSpeed=0)
    if (iPWM < iMinPWM)
        iPWM = iMinPWM;

    // ограничение сверху
    if (iPWM > iMaxPWM)
        iPWM = iMaxPWM;

    return iPWM;
}

// Управление щетками и воздухом
void brushAndAir() {
  debBrush.update();
  debAir.update();
  
  // bool bBrushEnable = (bForward && digitalRead(PIN_BRUSH) == HIGH);
  // bool bAirEnable   = (bForward && digitalRead(PIN_AIR)   == HIGH);
  bool bBrushEnable = ((bForward || bBackward) && debBrush.read() == HIGH);
  bool bAirEnable   = (debAir.read()   == HIGH);

  int iTargetBrushPWM = bBrushEnable ? int(iBrushAirMaxPWM) : 0;
  int iTargetAirPWM   = bAirEnable   ? int(iBrushAirMaxPWM) : 0;
  
  // Плавное изменение PWM раз в ulBrushAirStepDelay мс
  if (ulNow - ulLastBrushAirStep >= ulBrushAirStepDelay) {
    ulLastBrushAirStep = ulNow;

    int iStep = int(float(iBrushAirMaxPWM) * (float(ulBrushAirStepDelay) / float(ulBrushAirRampTime)));

    // Щётки
    if ( !bBrushEnable )
      iBrushPWM = 0;
    else if (iBrushPWM < iTargetBrushPWM) 
      iBrushPWM = min(iBrushPWM + iStep, iTargetBrushPWM);
    else if (iBrushPWM > iTargetBrushPWM) 
      iBrushPWM = max(iBrushPWM - iStep, iTargetBrushPWM);

    // Воздух
    if ( !bAirEnable )
      iAirPWM = 0;
    else if (iAirPWM < iTargetAirPWM) 
      iAirPWM = min(iAirPWM + iStep, iTargetAirPWM);
    else if (iAirPWM > iTargetAirPWM) 
      iAirPWM = max(iAirPWM - iStep, iTargetAirPWM);

    analogWrite(PIN_BRUSH_PWM, iBrushPWM);
    analogWrite(PIN_AIR_PWM, iAirPWM);
    
    //Serial.print( "brush_di: " ); Serial.println( debBrush.read() );   
    //Serial.print( "bBrushEnable: " ); Serial.println( bBrushEnable ); 
    //Serial.print( "brush: " ); Serial.println( iBrushPWM );    
    //Serial.print( "air: " ); Serial.println( iAirPWM );
  }
}

// Управление клапаном и насосами
void pumpsAndValve() {
  debPump.update();
  
  // bool bEnable = (bForward && digitalRead(PIN_PUMP_AND_VALVE) == HIGH);
  int iPumpBtn = debPump.read();
  if (iPumpBtn == HIGH && iPumpBtnLast == LOW) {
    uiPumpMode = (uiPumpMode + 1) % 4;
  }
  iPumpBtnLast = iPumpBtn;

  int iPumpMaxPWM = 0;
  const int iPumpMaxPWMLevelsCount = sizeof(iPumpMaxPWMLevels) / sizeof(iPumpMaxPWMLevels[0]);
  if (uiPumpMode >= 1 && uiPumpMode <= iPumpMaxPWMLevelsCount) {
      iPumpMaxPWM = iPumpMaxPWMLevels[uiPumpMode - 1];
  }

  bool bEnable = (uiPumpMode != 0) && bForward;
  
  int iTargetPumpPWM = 0;
  if (bEnable && fSpeed > 0.5f)
      // iTargetPumpPWM = int(fSpeed / float(iMaxFwdSpeed) * float(iPumpMaxPWM));
      iTargetPumpPWM = speedSyncroPWM(iPumpMinPWM, iPumpMaxPWM);

  switch (ePumpValveState)
  {
      // ------------------------------------------------------
      case EPV_STOP:   // всё выключено, ждём разрешения
          iValve = LOW;
          iPumpPWM = 0;

          if (bEnable) { 
              // начинаем включение → открыть клапан
              iValve = HIGH;
              ulPumpStateStart = ulNow;
              ePumpValveState = EPV_STARTING;
          }
          break;

      // ------------------------------------------------------
      case EPV_STARTING:   // клапан уже открыт, ждём 100 мс до запуска насосов
          iValve = HIGH;

          if (!bEnable) {
              // отменили — закрываем клапан
              ePumpValveState = EPV_STOP;
              break;
          }

          if (ulNow - ulPumpStateStart >= iStartPumpTimeout) {
              // можно включать насосы
              iPumpPWM = iTargetPumpPWM;
              ePumpValveState = EPV_RUNNING;
          }
          break;

      // ------------------------------------------------------
      case EPV_RUNNING:    // клапан открыт, насосы крутятся
          iValve = HIGH;

          if (!bEnable) {
              // начинаем выключение
              iValve = LOW;
              ulPumpStateStart = ulNow;
              ePumpValveState = EPV_STOPPING;
              break;
          }

          // регулируем PWM насосов по скорости машины
          iPumpPWM = iTargetPumpPWM;
          break;

      // ------------------------------------------------------
      case EPV_STOPPING:   // клапан закрыт, ждём 100 мс чтобы остановить насосы
          iValve = LOW;

          if (ulNow - ulPumpStateStart >= iStopPumpTimeout) {
              iPumpPWM = 0;
              ePumpValveState = EPV_STOP;
          }
          break;
          
      default:
        ePumpValveState = EPV_STOP;
  }
  
  digitalWrite(PIN_VALVE, iValve);
  analogWrite(PIN_PUMP_PWM, iPumpPWM);

  digitalWrite(PIN_PUMP_MODE_0_LED, (uiPumpMode == 0) ? HIGH : LOW);
  digitalWrite(PIN_PUMP_MODE_1_LED, (uiPumpMode == 1) ? HIGH : LOW);
  digitalWrite(PIN_PUMP_MODE_2_LED, (uiPumpMode == 2) ? HIGH : LOW);
  digitalWrite(PIN_PUMP_MODE_3_LED, (uiPumpMode == 3) ? HIGH : LOW);
  
  // Serial.print("pumpState: "); Serial.println(ePumpValveState);
  // Serial.print("pumpPWM: ");   Serial.println(iPumpPWM);
}
