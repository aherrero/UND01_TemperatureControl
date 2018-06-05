#include <Arduino.h>
#include <Wire.h> // Used to establied serial communication on the I2C bus

#include "../lib/SparkFun_TMP102_Arduino_Library-master/src/SparkFunTMP102.h"

// Temperature Sensor
const int ALERT_PIN = A3;
TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48
const int SENSOR_TEMP_MAX = 80;
const int SENSOR_TEMP_MIN = 0;

// Peltier Controller
const int PELTIER_PIN = 3;
const int PELTIER_PWM_MAX = 255;
const int PELTIER_PWM_MIN = 0;

float PID_LastTime = 0.0;
float PID_ErrSum = 0.0;
float PID_LastErr = 0.0;

/*
    Temperature Sensor Functions
*/

// Connections Tempereture Sensor TMP102
// VCC = 3.3V
// GND = GND
// SDA = A4
// SCL = A5

int InitTemperatureSensor()
{
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
    sensor0.setHighTempC(SENSOR_TEMP_MAX); // set T_HIGH in C

    //set T_LOW, the lower limit to shut turn off the alert
    sensor0.setLowTempC(SENSOR_TEMP_MIN); // set T_LOW in C

    return 0;
}

void GetTemperature(float &temperature, bool &alertPinState, boolean &alertRegisterState)
{
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
}

/*
    Peltier Controller
*/
float PIDController(float inputMeasured, float setPoint, float kp, float ki, float kd)
{
    // Time calculate from last time called
    unsigned long now = millis();
    float timeChange = (float)(now - PID_LastTime);

    // Calculate error variables
    float error = setPoint - inputMeasured;
    PID_ErrSum += (error * timeChange);
    float dErr = (error - PID_LastErr) / timeChange;

    // Compute PID
    float output = kp * error + ki * PID_ErrSum + kd * dErr;

    // Variables for next time
    PID_LastErr = error;
    PID_LastTime = now;

    // Return
    return output;
}

void UpdatePeltierAction(float pwmValue)
{
    // Limits
    if(pwmValue > PELTIER_PWM_MAX)
        pwmValue = PELTIER_PWM_MAX;
    else if (pwmValue < PELTIER_PWM_MIN)
        pwmValue = PELTIER_PWM_MIN;

    // Control
    analogWrite(PELTIER_PIN, pwmValue);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
}

void setup()
{
    // Sensor Init
    InitTemperatureSensor();

    // Peltier Init
    pinMode(PELTIER_PIN,OUTPUT);

    // Serial Start
    Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop()
{
    // Get Temperature
    float temperature;
    boolean alertPinState, alertRegisterState;
    GetTemperature(temperature, alertPinState, alertRegisterState);

    // Get PID Output
    float kp = 250;
    float ki = 0;
    float kd = 0;
    float setPoint = 35;    // 35 Degrees setpoint
    float output = PIDController(temperature, setPoint, kp, ki, kd);

    // Send Peltier Action
    UpdatePeltierAction(output);

    // Print serial
    Serial.print(millis());
    Serial.print(";");
    Serial.print(temperature);
    Serial.print(";");
    Serial.println(output);
}
