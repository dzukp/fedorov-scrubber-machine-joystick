#include <Arduino.h>
#include <Smooth.h>

#define LOOP_PERIOD 50  //  период выполнения основного цикла мс
#define SMOOTHED_SAMPLE_SIZE 3 // кол-во значений для усреднения значения джойстика
#define SMOOTHED_PWD_SAMPLE_PERIOD 200  // период за который измеряется среднее значение ШИМ драйверов
#define PULSE_LEN 100  // длина импульса
#define PULSE_PERIOD 2000  // период импульсов
// среднее значение обратной связи (0-100) драйвера DR1, DR2. 
// Если усреднённое значение меньше, то считаем включенным.
// Среднее значение 0-100, кратно (SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD)
// значение DR_ACTIVE_EDGE лучше устанавливать, чтобы не было кратно (SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD)
#define DR_ACTIVE_EDGE 50
#define START_STOP_PAUSE 2000  // Время перехода из движения вперёд в движение назад и наоборот

#define ENABLE_PRINT  // Закоментарить, чтобы отключить печать в ком-порт

// Пороги работы джойстика
const int frwEdge = 550;
const int bcwEdge = 250;

void print(const char* title, double value);
void print(const char* title, int value);
void print(const char* title, uint32_t value);
void print(const char* title, bool value);
void print(const char* title, const char* value);
void print(const char* text);


class MovingAverage {
  /*
  Класс скользящего среднего
  */
private:
  int* values;
  int windowSize; 
  int currentSize;
  int index;
  long sum;

public:
  MovingAverage(int size) {
    windowSize = size;
    values = new int[windowSize];
    currentSize = 0;
    index = 0;
    sum = 0;
  }

  ~MovingAverage() {
    delete[] values;
  }

  void add(int value) {
    if (currentSize == windowSize) {
      sum -= values[index];
    } else {
      currentSize++;
    }

    values[index] = value;
    sum += value;

    index = (index + 1) % windowSize;
  }

  float get_avg() const {
    if (currentSize == 0) return 0;
    return (float)sum / currentSize;
  }
};


class Pulse {
  /*
  Класс для генерации импульса
  */
  public:
    Pulse() : _state(0) {}

    void start(int time=PULSE_LEN, int period=PULSE_PERIOD) {
      /*
      ф-ция генерирует импульсы длиной time и периодом period
      */
      switch (_state) {
        case 0:
          _end_time = millis() + time;
          _state = 1;
          break;

        case 1:
          if (millis() > _end_time) {
            _end_time = millis() + period - time;
            _state = 2;
          }
          break;

        case 2:
          if (millis() > _end_time) {
            _state = 0;
          }
          break;

        case -1:
          break;

        default:
          _state = 0;
      }
    }

    void stop() {
      /* ф-ция останавливает генерацию импульсов */
      _state = 0;
    }

    void restart() {
      stop();
      start();
    }

    bool state() const { 
      /* Возвращает состояние, true - верхний уровень, false - нижний уровень  */
      return _state == 1; 
    }

  private:
    uint32_t _end_time;
    int _state;
};


class Driver {
  public:
    Driver(const char* _name) : 
      desiredState(false), 
      relay(false), 
      state(SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD),
      name(_name) 
    {}

    void process() {
      if (desiredState) {
        // желаемое состояние работа
        if (relay && !_is_active()) {
          pulse.start();
          print(name, 1);
        }
        else {
          relay = true;
          pulse.stop();
          print(name, 0);
        }
      }
      else {
        // желаемое состояние стоп
        if (relay && _is_active()) {
          pulse.start();
          print(name, 1);
        }
        else {
          relay = false;
          pulse.stop();
          print(name, 0);
        }
      }
    }

    void start() {
      desiredState = true;
    }

    void stop() {
      desiredState = false;
    }

    void updateStateValue(float value) {state.add(value);}
    bool getRelayState() const {return relay;}
    bool getPulseState() const {return pulse.state();}
    float getStateValue() const {return state.get_avg();}

  private:
    bool _is_active() {
      return state.get_avg() < DR_ACTIVE_EDGE;
    }

    const char* name;
    Pulse pulse;
    MovingAverage state;
    bool relay;
    bool desiredState;
};


class Machine {
  /*
  Класс управления машиной,
  принимает команду
  forward() - вперед
  stop() - стоять
  backward() - назад
  Оценивает состояния DR и управляет драйвером и реле
  */
  public:
    Machine(Driver* frw, Driver* bck) : 
      _command(0),
      lastForwardMove(0),
      lastBackwardMove(0),
      recentBackwardMove(false),
      recentForwardMove(false),
      drvForward(frw),
      drvBackward(bck)
    {}

    void forward() {
      if (_command != 1) {
        _command = 1;
      }
    }

    void backward() {
      if (_command != -1) {
        _command = -1;
      }
    }

    void stop() {
      if (_command != 0) {
        _command = 0;
      }
    }

    void process() {
      /*
      Выбираем действие исходя из команды
      1 - вперёд
      -1 - назад
      0 - стоп
      */
      recentBackwardMove = (millis() - lastBackwardMove) < START_STOP_PAUSE;
      recentForwardMove = (millis() - lastForwardMove) < START_STOP_PAUSE;

      switch (_command) {
        case 1: _forwardProcess(); break; // вперёд
        case -1: _backwardProcess(); break; // назад
        case 0: _stopProcess(); break; // стоп
        default: 
          print("machine bad command", _command); 
          _command = 0; 
          break;
      }
    }

  private:
    void _forwardProcess() {
      drvBackward->stop();
      if (recentBackwardMove) {
        drvForward->stop();
        print("pause for bcw");
      }
      else {
        drvForward->start();
        lastForwardMove = millis();
      }
    }

    void _backwardProcess() {
      drvForward->stop();
      if (recentBackwardMove) {
        drvBackward->stop();
        print("pause for frw");
      }
      else {
        drvBackward->start();
        lastBackwardMove = millis();
      }
    }

    void _stopProcess() {
      // команда стоять
      drvForward->stop();
      drvBackward->stop();
    }

    bool recentForwardMove;
    bool recentBackwardMove;
    uint32_t lastForwardMove;
    uint32_t lastBackwardMove;
    int _command;
    Driver* drvForward;
    Driver* drvBackward;
};


byte diJoystickSwitch = 2;
byte aiJoystick = A2;
byte aiDr1 = A0;
byte aiDr2 = A1;
byte diAccum = 13;
byte diBrush = 6;
byte diAir = 11;
byte diValve = 12;

byte doForward = 10;
byte doBackward = 9;
byte doForwardPulse = 7;
byte doBackwardPulse = 8;
byte doBrush = 3;
byte doAir = 4;
byte doValve = 5;


Smooth joystick(SMOOTHED_SAMPLE_SIZE);
Driver drv1("frw_drw");
Driver drv2("bcw_drw");
Machine machine(&drv1, &drv2);


void setup() {
  pinMode(aiJoystick, INPUT);
  pinMode(diJoystickSwitch, INPUT);
  pinMode(aiDr1, INPUT);
  pinMode(aiDr2, INPUT);
  pinMode(diAccum, INPUT);
  pinMode(diBrush, INPUT);
  pinMode(diAir, INPUT);
  pinMode(diValve, INPUT);

  pinMode(doForward, OUTPUT);
  pinMode(doBackward, OUTPUT);
  pinMode(doForwardPulse, OUTPUT);
  pinMode(doBackwardPulse, OUTPUT);
  pinMode(doBrush, OUTPUT);
  pinMode(doAir, OUTPUT);
  pinMode(doValve, OUTPUT);
  
  Serial.begin(115200);
}


void loop() {

  // запоминаем текущее время, чтобы измерить время цикла
  uint32_t start_loop_time = millis();
  
  joystick.add(analogRead(aiJoystick));
  drv1.updateStateValue(digitalRead(aiDr1) ? 100.0 : 0.0);
  drv2.updateStateValue(digitalRead(aiDr2) ? 100.0 : 0.0);

  bool accum = digitalRead(diAccum);
  bool joystickSwitch = digitalRead(diJoystickSwitch);
  bool brush = digitalRead(diBrush);
  bool air = digitalRead(diAir);
  bool valve = digitalRead(diValve);

  print("-----", start_loop_time);
  print("Joyst avg", joystick.get_avg());
  print("Joyst cur", analogRead(aiJoystick));
  print("Switch", joystickSwitch);
  print("DR1", drv1.getStateValue());
  print("DR2", drv2.getStateValue());

  int cmdMove = 0;
  if (joystick.get_avg() > frwEdge && joystickSwitch && accum) {
    // Значение джойстика выше порога, свитч сработал, аккумулятор в норме - команда вперёд
    cmdMove = 1;
  }
  else if (joystick.get_avg() < bcwEdge && joystickSwitch && accum) {
    // Значение джойстика ниже порога, свитч сработал, аккумулятор в норме - команда назад
    cmdMove = -1;
  }

  if (accum) {
    // Аккумулятор в норме
    digitalWrite(doBrush, brush ? HIGH : LOW);
    digitalWrite(doAir, air ? HIGH : LOW);
    digitalWrite(doValve, valve ? HIGH : LOW);

    print("cmd", cmdMove);

    // Логика работы machine описана в классе Machine
    if (cmdMove == 1) {
      // команада вперёд для объекта machine
      machine.forward();
    }
    else if (cmdMove == -1) {
      // команада назад для объекта machine
      machine.backward();
    }
    else {
      // команада стоп для объекта machine
      machine.stop();
    }

    drv1.process();
    drv2.process();

    // получаем команду для драйвера вперёд и передаём её на соотв выход
    digitalWrite(doForwardPulse, drv1.getPulseState() ? HIGH : LOW);
    // получаем команду для драйвера назад и передаём её на соотв выход
    digitalWrite(doBackwardPulse, drv2.getPulseState() ? HIGH : LOW);

    print("DR1 pulse",  drv1.getPulseState());
    print("DR2 pulse",  drv2.getPulseState());
    
    digitalWrite(doForward, drv1.getRelayState() ? HIGH : LOW);
    digitalWrite(doBackward, drv2.getRelayState() ? HIGH : LOW);
    print("FRW relay", drv1.getRelayState());
    print("BCW relay", drv2.getRelayState());
  }
  else {
    // Аккумулятор сел, всё останавливаем
    digitalWrite(doBrush, LOW);
    digitalWrite(doAir, LOW);
    digitalWrite(doValve, LOW);

    drv1.stop();
    drv2.stop();
    drv1.process();
    drv2.process();

    digitalWrite(doForwardPulse, LOW);
    digitalWrite(doBackwardPulse, LOW);
    digitalWrite(doForward, LOW);
    digitalWrite(doBackward, LOW);

    print("accum is low");
  }

  // вычисляем время цикла
  uint32_t loop_time = millis() - start_loop_time;
  print("loop_time", loop_time);
  if (LOOP_PERIOD > loop_time) {
    // если время цикла короткое, то делаем паузу
    print("delay", LOOP_PERIOD - loop_time);
    delay(LOOP_PERIOD - loop_time);
  }
}

void print(const char* title, double value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: ", title);
  Serial.print(output);
  Serial.println(value);
#endif
}

void print(const char* title, int value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %d", title, value);
  Serial.println(output);
#endif
}

void print(const char* title, uint32_t value) {
#ifdef ENABLE_PRINT
  char output[64];
  sprintf(output, "%s: %u", title, value);
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

void print(const char* title, const char* value) {
#ifdef ENABLE_PRINT
  char output[64];
  snprintf(output, sizeof(output), "%s: %s", title);
  Serial.println(output);
#endif
}

void print(const char* text) {
#ifdef ENABLE_PRINT
  Serial.println(text);
#endif
}
