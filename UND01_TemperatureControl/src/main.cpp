#include <Arduino.h>
#include <Wire.h> // Used to establied serial communication on the I2C bus
#include <inttypes.h>

#include "../lib/SparkFun_TMP102_Arduino_Library-master/src/SparkFunTMP102.h"
#include "../lib/LiquidCrystal-master/src/LiquidCrystal.h"

// Temperature Sensor
const int ALERT_PIN = A3;
TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48
const int SENSOR_TEMP_MAX = 80;
const int SENSOR_TEMP_MIN = 0;

// Peltier Controller
const int PELTIER_PIN = 9;
const int FUN_PIN = 10;
const int PELTIER_PWM_MAX = 255;
const int PELTIER_PWM_MIN = -255;

// Controller
int CONTROLLER_ID;
const int SET_POINT = 35;

float PID_LastTime = 0.0;
float PID_ErrSum = 0.0;
float PID_LastErr = 0.0;

const int CONTROLLER_HYSTERESIS = 0;
const int CONTROLLER_PI = 1;
const int CONTROLLER_PI_ANTIWINDUP = 2;
const int CONTROLLER_PI_FILTERING = 3;
const int CONTROLLER_EVENTS_= 4;

// Display
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int PIN_rs = 12, PIN_en = 11, PIN_d4 = 5, PIN_d5 = 4, PIN_d6 = 3, PIN_d7 = 2;
LiquidCrystal lcd(PIN_rs, PIN_en, PIN_d4, PIN_d5, PIN_d6, PIN_d7);

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
    Controller
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

float HystereisController(float inputMeasured, float setPoint)
{
    if(inputMeasured < setPoint)
        return PELTIER_PWM_MAX;
    else
        return PELTIER_PWM_MIN;
}

void UpdatePeltierAction(float pwmValue)
{
    // Limits
    if(pwmValue > PELTIER_PWM_MAX)
        pwmValue = PELTIER_PWM_MAX;
    else if (pwmValue < PELTIER_PWM_MIN)
        pwmValue = PELTIER_PWM_MIN;

    // Control
    // Positve or negative: Peltier or Fun
    if(pwmValue > 0)
        analogWrite(PELTIER_PIN, pwmValue);  // analogWrite values from 0 to 255
    else
        analogWrite(FUN_PIN, pwmValue);  // analogWrite values from 0 to 255
}

int PrintMessageDisplay(String msg, int line)
{
    // clear the screen
    lcd.clear();

    // Check Line
    if(line < 0 || line > 1)
        return -1;

    // Convert Data
    uint8_t data[16];
    msg.toCharArray(data,16);            // or myString.StringToCharArray(data,sizeof(data));

    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, line);

    // Write info
    lcd.print(msg);

    return 0;
}

float ControllerSelected(int CONTROL_ID, float currentSensor)
{
    float output = 0.0;

    // const int CONTROLLER_HYSTERESIS = 0;
    // const int CONTROLLER_PI = 1;
    // const int CONTROLLER_PI_ANTIWINDUP = 2;
    // const int CONTROLLER_PI_FILTERING = 3;
    // const int CONTROLLER_EVENTS_= 4;
    switch (CONTROL_ID)
    {
        case CONTROLLER_HYSTERESIS:
            output = HystereisController(currentSensor, SET_POINT);
            break;
        case CONTROLLER_PI:
            output = PIDController(currentSensor, SET_POINT, 250, 0, 0);
            break;
        default:
            break;
    }

    return output;
}

void setup()
{
    // Sensor Init
    InitTemperatureSensor();

    // Peltier Init
    pinMode(PELTIER_PIN,OUTPUT);

    // Controller Init
    CONTROLLER_ID = 1;

    // Display Init
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

    // Serial Start
    Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop()
{
    // Get Temperature
    float temperature;
    boolean alertPinState, alertRegisterState;
    GetTemperature(temperature, alertPinState, alertRegisterState);

    // Get Output Controller
    float output = ControllerSelected(CONTROLLER_ID, temperature);

    // Send Peltier Action
    UpdatePeltierAction(output);

    // Print serial
    Serial.print(millis());
    Serial.print(";");
    Serial.print(temperature);
    Serial.print(";");
    Serial.println(output);

    // Print Display
    PrintMessageDisplay("Control: " + String(CONTROLLER_ID), 0);
    PrintMessageDisplay("Temp: " + String(temperature) + "ºC", 1);

}
