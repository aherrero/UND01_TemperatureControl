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

const int FAN_PWM_MAX = 225;
const int FAN_PWM_MIN = 0;

int incomingByte = 0;   // for incoming serial data
int pwmFunValue = 50;

void setup()
{
    // put your setup code here, to run once:

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
                pwmFunValue+=2;
            }
            else if(incomingByte == 45)
            {
              // -
              if(pwmFunValue > FAN_PWM_MIN)
                pwmFunValue-=2;
            }
            else if(incomingByte == 48)
            {
              pwmFunValue = 0;
            }
            else if(incomingByte == 57)
            {
              pwmFunValue = 200;
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
    Serial.print("Temperature: ");
    Serial.print(temperature);

    // Serial.print("\tAlert Pin: ");
    // Serial.print(alertPinState);
    //
    // Serial.print("\tAlert Register: ");
    // Serial.println(alertRegisterState);


    delay(100);  // Wait 1000ms

    // FAN_PIN
    analogWrite(FAN_PIN, pwmFunValue);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255

    Serial.print("\tPWM: ");
    Serial.println(pwmFunValue);
}
