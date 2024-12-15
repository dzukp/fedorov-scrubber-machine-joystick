#define ENABLE_PRINT  // Закоментарить, чтобы отключить печать в ком-порт
#define LOOP_PERIOD 10  // время цикла

//  полупериод должен быть кратен LOOP_PERIOD
#define HALF_PERIOD_0 0 //  полупериод миандра помпы для скорости 0
#define HALF_PERIOD_1 40 //  полупериод миандра помпы для скорости 1
#define HALF_PERIOD_2 70 //  полупериод миандра помпы для скорости 2
#define HALF_PERIOD_3 100 //  полупериод миандра помпы для скорости 3

#define ACCUM_1_VALUE 720  // значение аккумулятора выше которого аккум заряжен
#define ACCUM_2_VALUE 650  // значение аккумулятора ниже которого аккум сел


const int numReadings = 10;

int readings[numReadings];  // показания с аналогового входа
int readIndex = 0;          // индекс текущего значения
int total = 0;              // текущее общее количество
int average = 0;            // среднее значение

float Vcc = 5.01;           // напряжение питания ардуино
float Voltagelcd;           // вывод значения с запятой

byte aiAccum = A0;       // аналоговый входной порт
byte doRed = 8;          // красный светодиод
byte doYellow = 9;       // желтый светодиод
byte doGreen = 10;       // зеленый светодиод
byte doAccOk = 13;
byte doPump1 = 6;
byte doPump2 = 7;


class Button {
  public:
    Button (byte pin) {
      _pin = pin;
      pinMode(_pin, INPUT_PULLUP);
    }
    bool clicked() {
      bool btnState = digitalRead(_pin);
      if (!btnState && !_flag && millis() - _tmr >= 100) {
        _flag = true;
        _tmr = millis();
        return true;
      }
      if (!btnState && _flag && millis() - _tmr >= 500) {
        _tmr = millis ();
        return true;
      }
      if (btnState && _flag) {
        _flag = false;
        _tmr = millis();
      }
      return false;
    }
  private:
    byte _pin;
    uint32_t _tmr;
    bool _flag;
};


bool accumOk = false;

Button incrBtn(11);
Button decrBtn(12);
bool incrBtnPressed = false;
bool decrBtnPressed = false;
int pumpSpeed = 0;
byte pumpLeds[] = {2, 3, 4, 5};
bool pumpPinState = false;
unsigned int previousPumpFreqMillis = 0;

unsigned int startLoopTime = 0;


void setup() {
  pinMode(aiAccum, INPUT);
  
  pinMode(doRed, OUTPUT);
  pinMode(doYellow, OUTPUT);
  pinMode(doGreen, OUTPUT);
  pinMode(doAccOk, OUTPUT);
  pinMode(doPump1, OUTPUT);
  pinMode(doPump2, OUTPUT);
  
  Serial.begin(115200);

  // инициализируйте все показания равным 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  for (int i = 0; i < sizeof(pumpLeds) / sizeof(pumpLeds[0]); i++) {
    pinMode(pumpLeds[i], OUTPUT);
  }

  previousPumpFreqMillis = millis();
  startLoopTime = millis();
}


void loop() {
  
  startLoopTime = millis();  // сохраняем время начала цикла

  accumProcess();
  pumpProcess();
  correctCyclePeriod();
}


void accumProcess() {
  /* Анализ состояния аккумулятора */

  // вычтите последнее значение:
  total = total - readings[readIndex];
  // считывание с датчика:
  readings[readIndex] = analogRead(aiAccum);
  // добавьте полученные данные к общему количеству:
  total = total + readings[readIndex];
  // перейдите к следующей позиции в массиве:
  readIndex = readIndex + 1;

  // если мы находимся в конце массива...
  if (readIndex >= numReadings) {
    // ...вернуться к началу:
    readIndex = 0;
  }

  // рассчет среднего значения:
  average = total / numReadings;
  // отправьте его на компьютер в виде цифр ASCII
  print("accum voltage", average);
      
  // Проверка уровня входного напряжения и включение светодиодов при напряжении 30В 4,4v  0,7mA  36k & 6k2
 
  if (average >= ACCUM_1_VALUE)
  {
    digitalWrite(doRed, LOW);
    digitalWrite(doYellow, LOW);
    digitalWrite(doGreen, HIGH);
    accumOk = true;
  }
  else if (average < ACCUM_1_VALUE && average > ACCUM_2_VALUE)
  {
    digitalWrite(doRed, LOW);
    digitalWrite(doYellow, HIGH);
    digitalWrite(doGreen, LOW);
    accumOk = true;
  }
  else
  {
    digitalWrite(doRed, HIGH);
    digitalWrite(doYellow, LOW);
    digitalWrite(doGreen, LOW);
    accumOk = false;
  }

  digitalWrite(doAccOk, accumOk ? HIGH : LOW); 
}


void pumpProcess() {
  /* Управление насосом */

  if (!incrBtnPressed && incrBtn.clicked() && pumpSpeed < 3) {
    // Нажата кнопка увеличения скорости
    pumpSpeed += 1;
  }

  if (!decrBtnPressed && decrBtn.clicked() && pumpSpeed > 0) {
    // Нажата кнопка уменьшения скорости
    pumpSpeed -= 1;
  }

  if (!accumOk) {
     // Аккумулятор сел - скорость 0
    pumpSpeed = 0;
  }

  print("pump speed", pumpSpeed);

  for (int i = 0; i < sizeof(pumpLeds) / sizeof(pumpLeds[0]); i++) {
    digitalWrite(pumpLeds[i], (i == pumpSpeed) ? HIGH : LOW);
  }

  unsigned long halfPeriod = 0;
  unsigned long currentMillis = millis();  // Получаем текущее время

  switch (pumpSpeed) {
    case 0:
      halfPeriod = HALF_PERIOD_0; //  полупериод миандра помпы для скорости 0
      break;
    case 1:
      halfPeriod = HALF_PERIOD_1; //  полупериод миандра помпы для скорости 1
      break;
    case 2:
      halfPeriod = HALF_PERIOD_2; //  полупериод миандра помпы для скорости 2
      break;
    case 3:
      halfPeriod = HALF_PERIOD_3; //  полупериод миандра помпы для скорости 3
      break;
    default:
      print("anomal pump speed");
      break;
  }

  // Если halfPeriod == 0, то выключаем сигнал
  if (halfPeriod == 0) {
    pumpPinState = LOW;
  }
  // Проверяем, прошло ли достаточно времени для смены состояния пина
  else if (currentMillis - previousPumpFreqMillis >= halfPeriod) {
    previousPumpFreqMillis = currentMillis;  // Обновляем время
    // Переключаем состояние пина
    pumpPinState = (pumpPinState == HIGH) ? LOW : HIGH;
  }

  digitalWrite(doPump1, pumpPinState);
  digitalWrite(doPump2, pumpPinState);
}


 void correctCyclePeriod() {

  // вычисляем время цикла
  unsigned int loopTime = millis() - startLoopTime;
  print("loop_time", loopTime);
  if (LOOP_PERIOD > loopTime) {
    // если время цикла короткое, то делаем паузу
    print("delay", LOOP_PERIOD - loopTime);
    delay(LOOP_PERIOD - loopTime);
  }
}


void print(const char* title, double value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %.2f", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, int value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, unsigned int value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, bool value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value ? 1 : 0);
  Serial.println(output);
#endif
}

void print(const char* text) {
#ifdef ENABLE_PRINT
  char output[64];
  snprintf(output, sizeof(output), "%s", text);
  Serial.println(output);
#endif
}
