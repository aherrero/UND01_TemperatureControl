#include <Arduino.h>
#include <Wire.h> // Used to establied serial communication on the I2C bus

#include "../lib/SparkFun_TMP102_Arduino_Library-master/src/SparkFunTMP102.h"

// Connections TMP102
// VCC = 3.3V
// GND = GND
// SDA = A4
// SCL = A5
const int ALERT_PIN = A3;

TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48
// Fan PWM
const int FAN_PIN = 3;

const int FAN_PWM_MAX = 255;
const int FAN_PWM_MIN = 0;

int incomingByte = 0;   // for incoming serial data
int pwmFunValue = 50;

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;

void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;

   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;

   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}

void setup()
{
    // put your setup code here, to run once:

    kp = 250;
    ki = 0;
    kd = 0;

    Setpoint = 35;

    Serial.begin(9600); // Start serial communication at 9600 baud
    pinMode(ALERT_PIN,INPUT);  // Declare alertPin as an input
    sensor0.begin();  // Join I2C bus

    // Initialize sensor0 settings
    // These settings are saved in the sensor, even if it loses power

    // set the number of consecutive faults before triggering alarm.
    // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
    sensor0.setFault(0);  // Trigger alarm immediately

    // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
    sensor0.setAlertPolarity(1); // Active HIGH

    // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
    sensor0.setAlertMode(0); // Comparator Mode.

    // set the Conversion Rate (how quickly the sensor gets a new reading)
    //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
    sensor0.setConversionRate(2);

    //set Extended Mode.
    //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
    sensor0.setExtendedMode(0);

    //set T_HIGH, the upper limit to trigger the alert on
    sensor0.setHighTempC(29.4); // set T_HIGH in C

    //set T_LOW, the lower limit to shut turn off the alert
    sensor0.setLowTempC(26.67); // set T_LOW in C

    // fan
    pinMode(FAN_PIN,OUTPUT);
}

void loop()
{
    // put your main code here, to run repeatedly:

    // when receive data:
    if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();

            // say what you got:
            //Serial.print("Received: ");
            //Serial.println(incomingByte);

            if(incomingByte == 43)
            {
              // +
              if(pwmFunValue < FAN_PWM_MAX)
                pwmFunValue+=1;
            }
            else if(incomingByte == 45)
            {
              // -
              if(pwmFunValue > FAN_PWM_MIN)
                pwmFunValue-=1;
            }
            else if(incomingByte == 48)
            {
              pwmFunValue = 0;
            }
            else if(incomingByte == 57)
            {
              pwmFunValue = 50;
            }
    }

    float temperature;
    boolean alertPinState, alertRegisterState;

    // Turn sensor on to start temperature measurement.
    // Current consumtion typically ~10uA.
    sensor0.wakeup();

    // read temperature data
    temperature = sensor0.readTempC();

    // Check for Alert
    alertPinState = digitalRead(ALERT_PIN); // read the Alert from pin
    alertRegisterState = sensor0.alert();   // read the Alert from register

    // Place sensor in sleep mode to save power.
    // Current consumtion typically <0.5uA.
    sensor0.sleep();

    // Print temperature and alarm state
    //Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(";");

    // Serial.print("\tAlert Pin: ");
    // Serial.print(alertPinState);
    //
    // Serial.print("\tAlert Register: ");
    // Serial.println(alertRegisterState);


    delay(1000);  // Wait 1000ms

    // PID
    Input = temperature;
    Compute();
    pwmFunValue = Output;

    // FAN_PIN
    if(pwmFunValue > FAN_PWM_MAX)
      pwmFunValue = FAN_PWM_MAX;
    else if (pwmFunValue < FAN_PWM_MIN)
      pwmFunValue = FAN_PWM_MIN;
    analogWrite(FAN_PIN, pwmFunValue);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255

    Serial.print("\t");
    Serial.print(pwmFunValue);
    Serial.println(";");

    // OTHER...
    // int val = analogRead(A2);     // read the input pin
    // Serial.println(val);             // debug value


}
