// ---------- Настройки пинов -----------------
const int PIN_K2 = A4;
const int PIN_K3 = A3;
const int PIN_PWM = 5;
const int PIN_JOY = A0;

const int PIN_BRUSH = A7;
const int PIN_BRUSH_PWM = 10;

const int PIN_AIR = A6;
const int PIN_AIR_PWM = 9;

const int PIN_PUMP_AND_VALVE = A1;
const int PIN_VALVE = 6;
const int PIN_PUMP_1_PWM = 3;
const int PIN_PUMP_2_PWM = 11;

// ---------- Настройки движения машины -------
const int iFwdEdge = 550;
const int iBcwEdge = 350;

const unsigned long ulRelayDelay = 100;      // Задержка переключения реле (мс)
const unsigned long ulRelayStopDelay = 2500;      // Задержка переключения реле (мс) при стопе
const unsigned long ulSpeedStepDelay = 10;    // Интервал шагов ШИМ (мс) обратное ускорению
const float fFrwStopAcceleration = 1.0;      // Ускорение торможения вперёд
const float fBkwStopAcceleration = 1.0;      // Ускорение торможения назад

const int iMaxFwdSpeed = 230;
const int iMaxBcwSpeed = 120;

// ---------- Настройки щёток и воздуха -------
const unsigned long ulBrushAirStepDelay = 10;    // Интервал шагов ШИМ (мс)
const unsigned long ulBrushAirRampTime  = 300;   // время разгона (мс)
const int iBrushAirMaxPWM = 255;

// ---------- Настройка клапана и насосов -----

const int iPumpMaxPWM = 165;  // макс скорость PWM насоса (0-255)
const int iPumpMinPWM = 0;    // мин скорость PWM насоса (0-255)
const int iStartPumpTimeout = 100;  // время между открытием клапана и пуском насосов
const int iStopPumpTimeout = 100;  // время между закрытием клапана и стопом насосов

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

int iBrushPWM = 0;
int iAirPWM = 0;
unsigned long ulLastBrushAirStep = 0;

int iPumpPWM = 0;
int iValve = LOW;

// ---------- Буфер выходов ----------
int iOutK2 = LOW;
int iOutK3 = LOW;
int iOutPWM = 0;

// ---------- Прототипы ----------
float fJoystickValue();
void setState(State eNewState);
void brakeSpeed(float fMaxSpeed, float fAcceleration);
void machine();
int speedSyncroPWM();
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

  pinMode(PIN_PUMP_1_PWM, OUTPUT);
  pinMode(PIN_PUMP_2_PWM, OUTPUT);
  analogWrite(PIN_PUMP_1_PWM, 0);
  analogWrite(PIN_PUMP_2_PWM, 0);
  
  debPump.begin();

  Serial.begin(115200);

  setState(E_STOP);
}

// ---------- LOOP ----------
void loop() {
  ulNow = millis();
  fJoy = fJoystickValue();
  bForward = fJoy > 0.05;
  bBackward = fJoy < -0.05;
  
  Serial.println("-------------------------");
  Serial.println( millis() );
  Serial.print("Joystick: ");
  Serial.println( fJoy * 100 );
  
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
        if (ulNow - ulLastSpeedStep >= ulSpeedStepDelay) {
          ulLastSpeedStep = ulNow;
          if (fSpeed < fSpeedTarget) fSpeed++;
          else if (fSpeed > fSpeedTarget) fSpeed--;
        }
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
        if (ulNow - ulLastSpeedStep >= ulSpeedStepDelay) {
          ulLastSpeedStep = ulNow;
          if (fSpeed < fSpeedTarget) fSpeed++;
          else if (fSpeed > fSpeedTarget) fSpeed--;
        }
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
    
    //Serial.print( "PIN_BRUSH: " ); Serial.println( digitalRead(PIN_BRUSH) );
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
  bool bEnable = (bForward && debPump.read() == HIGH);
  
  int iTargetPumpPWM = 0;
    if (fSpeed > 0.5f)
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
                ulStateStart = ulNow;
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

            if (ulNow - ulStateStart >= iStartPumpTimeout) {
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
                ulStateStart = ulNow;
                ePumpValveState = EPV_STOPPING;
                break;
            }

            // регулируем PWM насосов по скорости машины
            iPumpPWM = iTargetPumpPWM;
            break;

        // ------------------------------------------------------
        case EPV_STOPPING:   // клапан закрыт, ждём 100 мс чтобы остановить насосы
            iValve = LOW;

            if (ulNow - ulStateStart >= iStopPumpTimeout) {
                iPumpPWM = 0;
                ePumpValveState = EPV_STOP;
            }
            break;
            
        default:
          ePumpValveState = EPV_STOP;
    }
  
  digitalWrite(PIN_VALVE, iValve);
  analogWrite(PIN_PUMP_1_PWM, iPumpPWM);
  analogWrite(PIN_PUMP_2_PWM, iPumpPWM);
  
  // Serial.print("pumpState: "); Serial.println(ePumpValveState);
  // Serial.print("pumpPWM: ");   Serial.println(iPumpPWM);
}
