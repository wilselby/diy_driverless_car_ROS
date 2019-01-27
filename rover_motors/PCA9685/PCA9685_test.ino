
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);


void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(60);  // 1600 is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  //Wire.setClock(400000);
}


void loop() {

  pwm.setPWM(0, 0, 350 );

  pwm.setPWM(0, 0, 400);
  delay(1000);
  pwm.setPWM(0, 0, 300);
  delay(1000);
  pwm.setPWM(0, 0, 350);
  delay(1000);
  pwm.setPWM(0, 0, 300);
  delay(2000);
  
  
  /*
  // Drive throttle in a wave
  for (uint16_t i=350; i<400; i += 5) {
      //pwm.setPWM(0, 0, i );
      //delay(200);
  }

  // Drive steering in a wave
  for (uint16_t i=275; i<400; i += 5) {
      pwm.setPWM(1, 0, i );
      //pwm.setPWM(0, 0, i+30);
      delay(200);
  }
  */
  

}
