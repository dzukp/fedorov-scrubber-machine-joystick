#include <Arduino.h>
#include <Smooth.h>

#define LOOP_PERIOD 10  //  период выполнения основного цикла мс
#define SMOOTHED_SAMPLE_SIZE 1
#define SMOOTHED_PWD_SAMPLE_PERIOD 100  // период за который измеряется среднее значение ШИМ 
#define PULSE_LEN 1000  // длина импульса
#define PULSE_PERIOD 5000  // период импульсов

#define ENABLE_PRINT  // Закоментарить, чтобы отключить печать в ком-порт

// Пороги работы джойстика
const int frwEdge = 600;
const int bcwEdge = 130;

// Пороги работы DR, 
// если меньше drLoEdge, то меняем сотояние на выкл
// если больше drHiEdge, то меняем состояние на вкл
// если между, то состояние не меняется
const int drLoEdge = 100;
const int drHiEdge = 1000;

void print(char* title, double value);
void print(char* title, int value);
void print(char* title, bool value);
void print(char* text);

byte diJoystickSwitch = 2;
byte aiJoystick = A3;
byte aiDr1 = A0;
byte aiDr2 = A1;
byte diAccum = 13;
byte diBrush = 6;
byte diAir = 11;
byte diValve = 12;

byte doForward = 9;
byte doBackward = 10;
byte doForwardPulse = 7;
byte doBackwardPulse = 8;
byte doBrush = 3;
byte doAir = 4;
byte doValve = 5;

Smooth joystick(SMOOTHED_SAMPLE_SIZE);
Smooth dr1(SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD);
Smooth dr2(SMOOTHED_PWD_SAMPLE_PERIOD / LOOP_PERIOD);


class Pulse {
  public:
    Pulse() : _state(0) {}

    void start(int time=PULSE_LEN, int period=PULSE_PERIOD) {
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
      _state = 0;
    }

    void restart() {
      stop();
      start();
    }

    bool state() { return _state == 1; }

  private:
    uint32_t _end_time;
    int _state;
};


class Gisteresys {
  /*
  Класс гистерезиса, в зависисмости от значения аналогово сигнала меняем состояние
  если значение меньше _loEdge, то выкл
  если значение больше _hiEdge, то вкл
  если значение между _loEdge и _hiEdge, то остается неизменным
  */
  public:
    Gisteresys() : _state(false), _loEdge(drLoEdge), _hiEdge(drHiEdge) {}

    bool getState(double value) {
      if (_state && value < _loEdge) {
        _state = false;
      }
      if (!_state && value > _hiEdge) {
          _state = true;
      }
      return _state;
    }

  private:
    bool _state;
    int _loEdge;
    int _hiEdge;
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
    Machine() : _command(0) {}

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

      switch (_command) {
        case 1: _forwardProcess(); break;
        case -1: _backwardProcess(); break;
        case 0: _stopProcess(); break;
        default: print("machine bad command", _command); break;
      }
      do_frw = _is_dr1_active();
      do_bcw = _is_dr2_active() && !do_frw;
    }

  private:
    void _forwardProcess() {
      if (!_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.start();
        k2_bcw.stop();
        print("frw 1 0");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.start();
        print("frw 0 1");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.stop();
        print("frw 0 0");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        k1_frw.start();
        k2_bcw.start();
        print("frw 1 1");
      }
    }

    void _backwardProcess() {
      if (!_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.start();
        print("bcw 0 1");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.stop();
        print("bcw 0 0");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        k1_frw.start();
        k2_bcw.stop();
        print("bcw 1 0");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        k1_frw.start();
        k2_bcw.start();
        print("bcw 1 1");
      }
    }

    void _stopProcess() {
      if (!_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.stop();
        print("stop 0 0");
      }
      else if (_is_dr2_active() && !_is_dr1_active()) {
        k1_frw.stop();
        k2_bcw.start();
        print("stop 0 1");
      }
      else if (!_is_dr2_active() && _is_dr1_active()) {
        k1_frw.start();
        k2_bcw.stop();
        print("stop 1 0");
      }
      else if(_is_dr2_active() && _is_dr1_active()) {
        k1_frw.start();
        k2_bcw.start();
        print("stop 1 1");
      }
    }

    bool _is_dr1_active() { return dr1.get_avg() < 1; }
    bool _is_dr2_active() { return dr2.get_avg() < 1; }

    int _command;
    bool _wrong_state;

    Gisteresys dr1_gyst;
    Gisteresys dr2_gyst;

  public:
    Pulse k1_frw;
    Pulse k2_bcw;

    bool do_frw;
    bool do_bcw;
};


Machine machine;


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
  
  Serial.begin(9600);
}


void loop() {

  uint32_t start_loop_time = millis();
  
  joystick.add(analogRead(aiJoystick));
  dr1.add(digitalRead(aiDr1) ? 100 : 0);
  dr2.add(digitalRead(aiDr2) ? 100 : 0);

  print("-----");
  print("Joyst", joystick.get_avg());
  print("DR1", dr1.get_avg());
  print("DR2", dr2.get_avg());

  bool accum = digitalRead(diAccum);
  bool joystickSwitch = digitalRead(diJoystickSwitch);
  bool brush = digitalRead(diBrush);
  bool air = digitalRead(diAir);
  bool valve = digitalRead(diValve);

  int cmdMove = 0;
  if (joystick.get_avg() > frwEdge && joystickSwitch && accum) {
    cmdMove = 1;
  }
  else if (joystick.get_avg() < bcwEdge && joystickSwitch && accum) {
    cmdMove = -1;
  }

  if (accum) {
    digitalWrite(doBrush, brush ? HIGH : LOW);
    digitalWrite(doAir, air ? HIGH : LOW);
    digitalWrite(doValve, valve ? HIGH : LOW);

    if (cmdMove == 1) {
      machine.forward();
    }
    else if (cmdMove == -1) {
      machine.backward();
    }
    else {
      machine.stop();
    }

    print("machive cmd", cmdMove);

    machine.process();

    digitalWrite(doForwardPulse, machine.k1_frw.state() ? HIGH : LOW);
    digitalWrite(doBackwardPulse, machine.k2_bcw.state() ? HIGH : LOW);
    digitalWrite(doForward, machine.do_frw ? HIGH : LOW);
    digitalWrite(doBackward, machine.do_bcw ? HIGH : LOW);

    print("K1", machine.k1_frw.state());
    print("K2", machine.k2_bcw.state());

  }
  else {
    digitalWrite(doBrush, LOW);
    digitalWrite(doAir, LOW);
    digitalWrite(doValve, LOW);

    machine.stop();
    machine.process();

    digitalWrite(doForwardPulse, LOW);
    digitalWrite(doBackwardPulse, LOW);
    digitalWrite(doForward, LOW);
    digitalWrite(doBackward, LOW);

    print("accum is low");
  }

  uint32_t delay_time = max(LOOP_PERIOD - millis() - start_loop_time, 0);
  delay(delay_time);
}

void print(char* title, double value) {
#ifdef ENABLE_PRINT
  Serial.print(title);
  Serial.print(": ");
  Serial.println(value);
#endif
}

void print(char* title, int value) {
#ifdef ENABLE_PRINT
  Serial.print(title);
  Serial.print(": ");
  Serial.println(value);
#endif
}

void print(char* title, bool value) {
#ifdef ENABLE_PRINT
  Serial.print(title);
  Serial.print(": ");
  Serial.println(value ? 1 : 0);
#endif
}

void print(char* text) {
#ifdef ENABLE_PRINT
  Serial.println(text);
#endif
}
