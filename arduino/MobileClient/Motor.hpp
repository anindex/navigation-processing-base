#ifndef MOTOR_HPP
#define MOTOR_HPP

//Pinout parameters
#define ENA    9
#define IN1    4
#define IN2    5
#define IN3    6
#define IN4    7
#define ENB    10

namespace motor
{
  void motorInit()
  {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
  

    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
  
  void left_wheel_forward(int pwm)
  {
    analogWrite(ENA, pwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  void left_wheel_backward(int pwm)
  {
    analogWrite(ENA, pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  void right_wheel_forward(int pwm)
  {
    analogWrite(ENB, pwm);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  void right_wheel_backward(int pwm)
  {
    analogWrite(ENB, pwm);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  void emergency_stop()
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }
}

#endif // end MOTOR_HPP
