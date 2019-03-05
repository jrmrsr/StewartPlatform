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
const int SERVOMAXS[6] = {520, 580, 420, 450, 490, 520};
const int SERVOCHG = 5;
// Gate Reading Limit
const int GATE_LIMIT = 100;

// Photocell Variables
// Pins
const int GATE_PIN_1 = A8;
const int GATE_PIN_2 = A9;
const int GATE_PIN_3 = A10;
const int GATE_PIN_4 = A11;
const int GATE_PIN_5 = A12;
const int GATE_PIN_6 = A13;
// Reading
int gate_reading_1 = 0;
int gate_reading_2 = 0;
int gate_reading_3 = 0;
int gate_reading_4 = 0;
int gate_reading_5 = 0;
int gate_reading_6 = 0;
// Initializing PWM Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
NOTE
TO CONVERT DEGREES TO PULSE LENGTH, USE THE FOLLOWING
pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
from https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library 
*/
int servo_settings[6] = {0, 0, 0, 0, 0, 0}; // PWM var
String input = "0";

void setup()
{
    Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
    Serial.setTimeout(10); // change default (1000ms) to have faster response time
    pwm.begin();
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
    int mid_1 = 0;
    int mid_2 = 0;
    for (int i = 0; i < 6; i++)
    {
        // Changing the order of the servo settings array allows us to change the orientation of the maze if
        // we use delta angle changes for our servos
        Serial.println(SERVOMINS[i]);
        Serial.println(SERVOMAXS[i]);
        mid_1 = (SERVOMINS[i] + SERVOMAXS[i]);
        mid_2 = mid_1 / 2;
        servo_settings[i] = mid_2;
    }
    ServoValues();
    SetServos(servo_settings[0], servo_settings[1], servo_settings[2], servo_settings[3], servo_settings[4], servo_settings[5]);
    // SetServos(350, 350, 350, 350, 350, 350);
    Serial.println("Would you like me to solve this maze for you? (y/n/c)");
}

void loop()
{
    // Check keyboard string
    if (Serial.available() > 0)
    {
        ServoValues();
        input = Serial.readString();
        Serial.println(input);

        // Add in pause keystroke after every servo change once the marble passes
        // a photocell gate for debugging
        if (input[0] == 'y')
        {
            SolveMaze();
        }
        else if (input[0] == 'c')
        {
            SetServos(90, 90, 90, 90, 90, 90);
            ReadAllPhotocells();
        }
        else if (input[0] == 'n')
        {
            Serial.println("Okay I will just wait until you say yes.");
        }
        else if (input[0] == 's')
        {
            SetServos(350, 350, 350, 350, 350, 350);
        }
        else if (input[0] == '-')
        {
            SetServos(SERVOMINS[0], SERVOMINS[1], SERVOMINS[2], SERVOMINS[3], SERVOMINS[4], SERVOMINS[5]);
        }
        else if (input[0] == '+')
        {
            SetServos(SERVOMAXS[0], SERVOMAXS[1], SERVOMAXS[2], SERVOMAXS[3], SERVOMAXS[4], SERVOMAXS[5]);
        }
        else if (input[0] == 'm')
        {
            //TODO
            do
            {
                Serial.println("Manual Control Mode");
                input = Serial.readString();
                switch (input[0])
                {
                    // Input of "1" to "6" -> increase respective (1..6) values
                    // Input of [q,w,e,r,t,y] -> decrease respective (1..6) values

                case '1':
                    servo_settings[0] = min(servo_settings[0] + SERVOCHG, SERVOMAXS[0]);
                    break;
                case 'q':
                    servo_settings[0] = max(servo_settings[0] - SERVOCHG, SERVOMINS[0]);
                    break;

                case '2':
                    servo_settings[1] = min(servo_settings[1] + SERVOCHG, SERVOMAXS[1]);
                    break;
                case 'w':
                    servo_settings[1] = max(servo_settings[1] - SERVOCHG, SERVOMINS[1]);
                    break;

                case '3':
                    servo_settings[2] = min(servo_settings[2] + SERVOCHG, SERVOMAXS[2]);
                    break;
                case 'e':
                    servo_settings[2] = max(servo_settings[2] - SERVOCHG, SERVOMINS[2]);
                    break;

                case '4':
                    servo_settings[3] = min(servo_settings[3] + SERVOCHG, SERVOMAXS[3]);
                    break;
                case 'r':
                    servo_settings[3] = max(servo_settings[3] - SERVOCHG, SERVOMINS[3]);
                    break;

                case '5':
                    servo_settings[4] = min(servo_settings[4] + SERVOCHG, SERVOMAXS[4]);
                    break;
                case 't':
                    servo_settings[4] = max(servo_settings[4] - SERVOCHG, SERVOMINS[4]);
                    break;

                case '6':
                    servo_settings[5] = min(servo_settings[5] + SERVOCHG, SERVOMAXS[5]);
                    break;
                case 'y':
                    servo_settings[5] = max(servo_settings[5] - SERVOCHG, SERVOMINS[5]);
                    break;

                case '<':
                    for (int i = 0; i < 6; i++)
                    {
                        servo_settings[i] = SERVOMINS[i];
                    }
                    break;
                case '>':
                    for (int i = 0; i < 6; i++)
                    {
                        servo_settings[i] = SERVOMAXS[i];
                    }
                    break;

                case '-':
                    for (int i = 0; i < 6; i++)
                    {
                        servo_settings[i] = max(servo_settings[i] - 5, SERVOMINS[i]);
                    }
                    break;

                case '+':
                    for (int i = 0; i < 6; i++)
                    {
                        servo_settings[i] = min(servo_settings[i] + 5, SERVOMAXS[i]);
                    }
                    break;

                default:
                    Serial.print(" No action taken");
                } // end switch statement

                // Update servo commands:
                for (int i = 0; i < 6; i++)
                {
                    pwm.setPWM(i + 1, 0, servo_settings[i]); // added +1 to match PWM port numbering (pints 1..6 used)
                }
            } while (input[0] == 'esc');
        }
    }
    input = "";
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
        if (temp_servo_settings[i] != SERVOMINS[i] && temp_servo_settings[i] >= 0 && temp_servo_settings[i] < 180)
        {
            servo_settings[i] = map(temp_servo_settings[i], 0, 179, SERVOMINS[i], SERVOMAXS[i]);
            pwm.setPWM(i + 1, 0, servo_settings[i]); // added +1 to match PWM port numbering (pins 1..6 used)
        }
        else
        {
            servo_settings[i] = temp_servo_settings[i];
            pwm.setPWM(i + 1, 0, servo_settings[i]); // added +1 to match PWM port numbering (pins 1..6 used)
        }
    }
}

void SolveMaze()
{
    // First gate
    ServoValues(); // Servo values are used for debugging
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to first gate");
    int gate_reading = analogRead(GATE_PIN_1); // Initialize gate reading for while loop
    // Maintain servos at required position using the while loop as a blocker
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_1);
    }

    // Second gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the second gate");
    gate_reading = analogRead(GATE_PIN_2);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_2);
    }

    // Third gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the third gate");
    gate_reading = analogRead(GATE_PIN_3);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_3);
    }

    // Fourth gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the fourth gate");
    gate_reading = analogRead(GATE_PIN_4);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_4);
    }
}

void ReadAllPhotocells()
{
    gate_reading_1 = analogRead(GATE_PIN_1);
    gate_reading_2 = analogRead(GATE_PIN_2);
    gate_reading_3 = analogRead(GATE_PIN_3);
    gate_reading_4 = analogRead(GATE_PIN_4);
    gate_reading_5 = analogRead(GATE_PIN_5);
    gate_reading_6 = analogRead(GATE_PIN_6);
    Serial.print("Photocell Values 1-6:");
    Serial.print(gate_reading_1);
    Serial.print(", ");
    Serial.print(gate_reading_2);
    Serial.print(", ");
    Serial.print(gate_reading_3);
    Serial.print(", ");
    Serial.print(gate_reading_4);
    Serial.print(", ");
    Serial.print(gate_reading_5);
    Serial.print(", ");
    Serial.println(gate_reading_6);
}