/****************************************************************************************************************************
  PWM_DynamicDutyCycle.ino
  For AVR-based boards  (UNO, Nano, Mega, 32U4, 16U4, etc. )
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/AVR_PWM
  Licensed under MIT license

  This is pure hardware-based PWM
*****************************************************************************************************************************/
/******************************************************************************************************************************
  // For UNO / Nano
  Timer0 ( 8-bit) used by delay(), millis() and micros(), and PWM generation on pins 5 (6 not usable)
  Timer1 (16-bit) used by the Servo.h library and PWM generation on pins 9 and 10
  Timer2 ( 8-bit) used by Tone() and PWM generation on pins 3 and 11
  // For Mega
  Timer0 ( 8-bit) used by delay(), millis() and micros(), and PWM generation on pins 4 (13 not usable)
  Timer1 (16-bit) used by the Servo.h library and PWM generation on pins 11, 12
  Timer2 ( 8-bit) used by Tone() and PWM generation on pins 9 and 10
  Timer3 (16-bit) used by PWM generation on pins  2,  3 and  5
  Timer4 (16-bit) used by PWM generation on pins  6,  7 and  8
  Timer5 (16-bit) used by PWM generation on pins 44, 45 and 46

  ////////////////////////////////////////////
  // For Mega (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46)
  Pin  2 => TIMER3B   // PE 4 ** 2  ** PWM2
  Pin  3 => TIMER3C   // PE 5 ** 3  ** PWM3
  Pin  4 => TIMER0B   // PG 5 ** 4  ** PWM4
  Pin  5 => TIMER3A   // PE 3 ** 5  ** PWM5
  Pin  6 => TIMER4A   // PH 3 ** 6  ** PWM6
  Pin  7 => TIMER4B   // PH 4 ** 7  ** PWM7
  Pin  8 => TIMER4C   // PH 5 ** 8  ** PWM8
  Pin  9 => TIMER2B   // PH 6 ** 9  ** PWM9
  Pin 10 => TIMER2A   // PB 4 ** 10 ** PWM10
  Pin 11 => TIMER1A   // PB 5 ** 11 ** PWM11
  Pin 12 => TIMER1B   // PB 6 ** 12 ** PWM12
  Pin 13 => TIMER0A   // PB 7 ** 13 ** PWM13
  Pin 44 => TIMER5C   // PL 5 ** 44 ** D44
  Pin 45 => TIMER5B   // PL 4 ** 45 ** D45
  Pin 46 => TIMER5A   // PL 3 ** 46 ** D46
  ////////////////////////////////////////////
  // For 32u4 (3, 5, 6, 9, 10, 11, 13)
  Pin  3 => TIMER0B
  Pin  5 => TIMER3A
  Pin  6 => TIMER4D
  Pin  9 => TIMER1A
  Pin 10 => TIMER1B
  Pin 11 => TIMER0A
  Pin 13 => TIMER4A
  ////////////////////////////////////////////
  // For UNO, Nano (3, 5, 6, 9, 10, 11)
  Pin  3 => TIMER2B,
  Pin  5 => TIMER0B
  Pin  6 => TIMER0A
  Pin  9 => TIMER1A
  Pin 10 => TIMER1B
  Pin 11 => TIMER2(A)
******************************************************************************************************************************/

#define _PWM_LOGLEVEL_ 4

#include "AVR_PWM.h"

#define LED_ON LOW
#define LED_OFF HIGH

#if (PWM_USING_ATMEGA2560)
// Pins tested OK in Mega
//#define pinToUse      12            // Timer1B on Mega
//#define pinToUse      11            // Timer1A on Mega
//#define pinToUse       9            // Timer2B on Mega
//#define pinToUse       2            // Timer3B on Mega
//#define pinToUse       3            // Timer3C on Mega
//#define pinToUse       5            // Timer3A on Mega
//#define pinToUse       6            // Timer4A on Mega
//#define pinToUse       7            // Timer4B on Mega
#define pinToUse 8  // Timer4C on Mega
//#define pinToUse      46            // Timer5A on Mega
//#define pinToUse      45            // Timer5B on Mega
//#define pinToUse      44            // Timer5C on Mega

#elif (PWM_USING_ATMEGA_32U4)
// Pins tested OK on 32u4
////#define pinToUse      3            // Timer0B on 32u4
//#define pinToUse      5            // Timer3A on 32u4
#define pinToUse 9  // Timer1A on 32u4
//#define pinToUse      10            // Timer1B on 32u4

#else

// Pins tested OK on Nano / UNO
//#define pinToUse      9            // Timer1A on UNO, Nano, etc
#define pinToUse 10  // Timer1B on UNO, Nano, etc
//#define pinToUse      5               // Timer0B on UNO, Nano, e
//#define pinToUse       3            // Timer2B on UNO, Nano, etc
#endif

AVR_PWM* PWM_Instance;

float frequency;
float dutyCycle;

char dashLine[] = "=====================================================================================";
float PWM_FREQUENCY = 75000;
float PWM_DUTY = 0;

float STANDBY_CURRENT_A = 0.1;
float STANDBY_PWM_DUTY = 20;

void setup() {
  Serial.begin(115200);
  pinMode(pinToUse, OUTPUT);
  digitalWrite(pinToUse, HIGH);
  pinMode(A0, INPUT);
  PWM_Instance = new AVR_PWM(pinToUse, PWM_FREQUENCY, 99);

  // if (PWM_Instance) {
  //   if (!PWM_Instance->setPWM()) {
  //     Serial.println(F("Stop here"));
  //     // stop here
  //     while (true)
  //       delay(1000);
  //   }
  // }
}


void loop() {
  PWM_Instance->setPWM(pinToUse, PWM_FREQUENCY, 100 - PWM_DUTY);
  float desired_current = 2.5f;
  float current_reading = abs(read_battery_charging_current());
  Serial.println(current_reading);
  if (current_reading < desired_current) {
    PWM_DUTY = PWM_DUTY + 1;
    if (PWM_DUTY > 99) {
      PWM_DUTY = 99;
    }
    delay(1);
  } else if (current_reading > desired_current) {
    PWM_DUTY = PWM_DUTY - 1;
    if (PWM_DUTY < 0) {
      PWM_DUTY = 0;
    }
    delay(1);
  }

  // Serial.println("Current Reading:" + String(read_battery_charging_current()));
  // delay(500);

  // if (PWM_DUTY < 50) {
  //   PWM_DUTY = PWM_DUTY + 1.0f;
  // } else {
  //   PWM_DUTY = 0;
  // }
}



float read_battery_charging_current() {
  const float digital_analog_ratio = 0.0390;

  float offset = 512;
  uint8_t number_of_samples = 25;
  uint8_t delay_between_samples_us = 5;

  float digital_sum = 0;
  for(uint8_t i = 0; i <number_of_samples; i++){
    float digital_read_value = analogRead(A0);
    digital_sum = digital_sum + digital_read_value;
    delayMicroseconds(delay_between_samples_us);
  }
  float digital_sum_average = digital_sum/number_of_samples;
  float offset_free_digital_read_value = digital_sum_average - offset;
  float current_A = offset_free_digital_read_value * digital_analog_ratio;
  return current_A;
}

void printPWMInfo(AVR_PWM* PWM_Instance) {
  Serial.println(dashLine);
  Serial.print("Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DC = ");
  Serial.print(PWM_Instance->getActualDutyCycle());
  Serial.print(", PWMPeriod = ");
  Serial.print(PWM_Instance->getPWMPeriod());
  Serial.print(", PWM Freq (Hz) = ");
  Serial.println(PWM_Instance->getActualFreq(), 4);
  Serial.println(dashLine);
}
