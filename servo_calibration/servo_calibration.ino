/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/**
 * usage: connect up an arduino/raspi pico with the PCA9685 servo board
 * then upload the code and open up a serial monitor
 * finally, you can send it commands like 3 455, turning the 3rd servo to position 455
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BUF_LEN 32
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  Serial.println("triped servo calibration");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void parse_input(String str, int &port, int &val) {
  for (int i = 1; i < str.length()-1; ++i) {
    if (str[i] == ' ') {
      port = str.substring(0, i).toInt();
      val = str.substring(i+1, str.length()).toInt();
    }
  }
}

char buf[BUF_LEN];
void loop() {
  if (Serial.available() > 0) {
    String str = Serial.readString();
    int port = 0, val = SERVOMIN + (SERVOMAX-SERVOMIN)/2;
    parse_input(str, port, val);
    Serial.print(port);
    Serial.print(" ");
    Serial.println(val);
    pwm.setPWM(port, 0, val);
  }
}
