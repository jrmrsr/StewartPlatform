/* =================================================================================================== 
 *  Several lines of code are accredited to the Adafruit PWM Servo Driver Library example code.
 *  -> Add Adafruit PWM shield library
 *  
 *  Adapted by Jose Rondon and group 6
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Motor Limits
// Pulse length counts are out of 4096
// Indexed from servo 1 - 6
const int SERVOMINS[6] = {140, 165, 105, 130, 135, 155};
const int SERVOMAXS[6] = {575, 635, 480, 500, 505, 570};
// Gate Reading Limit
const int GATE_LIMIT = 100;

// Photocell Variables
// Pins
const int GATE_PIN_1 = 0;
const int GATE_PIN_2 = 1;
const int GATE_PIN_3 = 2;
const int GATE_PIN_4 = 3;
// Reading
int gate_reading = 0;
// Initializing PWM Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
NOTE
TO CONVERT DEGREES TO PULSE LENGTH, USE THE FOLLOWING
pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
from https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library 
*/
int servo_settings[6] = {0, 0, 0, 0, 0, 0}; // PWM var
String input;

void setup()
{
    Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
    Serial.setTimeout(10); // change default (1000ms) to have faster response time
    pwm.begin();
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

    for (int i = 0; i < 6; i++)
    {
        // Changing the order of the servo settings array allows us to change the orientation of the maze if
        // we use delta angle changes for our servos
        int mid = (SERVOMINS[i] + SERVOMAXS[i]) / 2;
        servo_settings[i] = mid;
    }
}

void loop()
{
    // Check keyboard string
    if (Serial.available() > 0)
    {
        ServoValues();
        Serial.println("Would you like me to solve this maze for you? (y/n)");
        input = Serial.readString();
    }
    // Add in pause keystroke after every servo change once the marble passes
    // a photocell gate for debugging
    if (input = "y")
    {
        SolveMaze();
    }
    else
    {
        Serial.println("Okay I will just wait until you say yes.");
    }
}

void ServoValues()
{
    Serial.print(" Servo values = [");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(servo_settings[i]);
        Serial.print(" ");
    }
    Serial.println("]");
}

// Takes in motor angle in degrees and then returns an array of PWM values for motors
void SetServos(int motor_1, int motor_2, int motor_3, int motor_4, int motor_5, int motor_6)
{
    // temp array
    int temp_servo_settings[6] = {motor_1,
                                  motor_2,
                                  motor_3,
                                  motor_4,
                                  motor_5,
                                  motor_6};

    for (int i = 0; i < 6; i++)
    {
        // Check that temp_servo_settings is not NULL and it is within our angle bounds
        // *NOTE* three checks are added for testing purposes. If they take too long to compute, they will be removed.
        if (temp_servo_settings[i] || temp_servo_settings[i] >= 0 || temp_servo_settings[i] <= 180 )
        {
            servo_settings[i] = map(temp_servo_settings[i], 0, 180, SERVOMINS[i], SERVOMAXS[i]);
        }
    }
    for (int j = 0; j < 6; j++)
    {
        pwm.setPWM(j + 1, 0, servo_settings[j]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
}

void SolveMaze()
{
    // First gate
    ServoValues(); // Servo values are used for debugging
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to first gate");
    gate_reading = analogRead(GATE_PIN_1); // Initialize gate reading for while loop
    // Maintain servos at required position using the while loop as a blocker
    while (gate_reading > GATE_LIMIT)
    {
        gate_reading = analogRead(GATE_PIN_1);
    }

    // Second gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the second gate");
    gate_reading = analogRead(GATE_PIN_2);
    while (gate_reading > GATE_LIMIT)
    {
        gate_reading = analogRead(GATE_PIN_2);
    }

    // Third gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the third gate");
    gate_reading = analogRead(GATE_PIN_3);
    while (gate_reading > GATE_LIMIT)
    {
        gate_reading = analogRead(GATE_PIN_3);
    }

    // Fourth gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the fourth gate");
    gate_reading = analogRead(GATE_PIN_4);
    while (gate_reading > GATE_LIMIT)
    {
        gate_reading = analogRead(GATE_PIN_4);
    }
}
