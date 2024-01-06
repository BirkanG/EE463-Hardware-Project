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
float NOT_PWM_DUTY = 99;

float STANDBY_CURRENT_1_A = 0.150;
float STANDBY_CURRENT_2_A = STANDBY_CURRENT_1_A * 3;
float STANDBY_CURRENT_3_A = STANDBY_CURRENT_1_A * 5;
float NOT_MAX_STANDBY_PWM = 40;

float read_battery_charging_current();

void setup() {
  Serial.begin(115200);
  pinMode(pinToUse, OUTPUT);
  digitalWrite(pinToUse, HIGH);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);  // Battery - to ground
  pinMode(A2, INPUT);  // RECT + to ground
  PWM_Instance = new AVR_PWM(pinToUse, PWM_FREQUENCY, 99);
}

uint8_t program_state = 0;
const float desired_current = 5.0f;
const float desired_current_digital = 512 + (desired_current/0.0390);
const float low_current_digital = 512+ (0.5f/0.0390);
void loop() {

  float rectifier_voltage_reading = analogRead(A2) * (0.00488) * 9.08;
  float battery_neg_reading = analogRead(A1) * (0.00488) * 9.08;

  //Serial.println("Rect: " + String(rectifier_voltage_reading) + " Bat: " + String(battery_neg_reading));
  //Serial.println("State: " + String(program_state) + " PWM: " + String(100 - NOT_PWM_DUTY));
  if (program_state == 0) {        //Ensures the load is connected
    digitalWrite(pinToUse, HIGH);  //turn-off mosfet
    delay(250);
    if (rectifier_voltage_reading > 20) {  //power is connected
      //rectifier_voltage_reading = analogRead(A2) * (0.00488) * 9.08;
      float battery_neg_reading_old = analogRead(A1) * (0.00488) * 9.08;
      delay(3500);
      float battery_neg_reading_new = analogRead(A1) * (0.00488) * 9.08;
      rectifier_voltage_reading = analogRead(A2) * (0.00488) * 9.08;

      if (rectifier_voltage_reading > 20) {  //battery and rectifier is ON
        if ((rectifier_voltage_reading - battery_neg_reading_new < 14.5) && abs(battery_neg_reading_new - battery_neg_reading_old) < 0.25) {
          NOT_PWM_DUTY = 99;
          PWM_Instance->setPWM(pinToUse, PWM_FREQUENCY, NOT_PWM_DUTY);
          while (true) {  // ensure that current reaches to a suitable level
            delay(1);
            float current_reading_A = read_battery_charging_current();
            if (current_reading_A < 1.5f) {
              NOT_PWM_DUTY = NOT_PWM_DUTY - 1;
              PWM_Instance->setPWM(pinToUse, PWM_FREQUENCY, NOT_PWM_DUTY);
              if (NOT_PWM_DUTY < 20) {
                NOT_PWM_DUTY = 99;
                break;
              }
            } else {
              program_state = 1;
              break;
            }
          }
        }
      }
    }
  } else if (program_state == 1) {
    delayMicroseconds(10);
    //float current_reading_A = read_battery_charging_current();
    float current_reading_A = analogRead(A0);
      if (current_reading_A < low_current_digital) {
      program_state = 0;
    }
    else if (current_reading_A < desired_current_digital) {
      NOT_PWM_DUTY = NOT_PWM_DUTY - 5;
      if (NOT_PWM_DUTY < 1) {
        NOT_PWM_DUTY = 1;
      }
      PWM_Instance->setPWM(pinToUse, PWM_FREQUENCY, NOT_PWM_DUTY);
    }
    else if (current_reading_A > desired_current_digital) {
      NOT_PWM_DUTY = NOT_PWM_DUTY + 5;
      if (NOT_PWM_DUTY > 99) {
        NOT_PWM_DUTY = 99;
      }
      PWM_Instance->setPWM(pinToUse, PWM_FREQUENCY, NOT_PWM_DUTY);
    }
  }
}


float read_battery_charging_current() {
  const float scale_factor = 1.25f;
  const float digital_analog_ratio = 0.0390;

  const float offset = 512;
  const uint8_t number_of_samples = 1;
  const uint8_t delay_between_samples_us = 5;

  float digital_sum = 0;
  for (uint8_t i = 0; i < number_of_samples; i++) {
    float digital_read_value = analogRead(A0);
    digital_sum = digital_sum + digital_read_value;
    delayMicroseconds(delay_between_samples_us);
  }

  float digital_sum_average = digital_sum / number_of_samples;
  float offset_free_digital_read_value = digital_sum_average - offset;
  float current_A = offset_free_digital_read_value * digital_analog_ratio;
  current_A = current_A * scale_factor;
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
