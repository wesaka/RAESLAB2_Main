#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyMPU6050.h>
#include <PID_v1.h>

/*
 * Encoder variables
 */

#define ENCODER_DT 51
#define ENCODER_CLK 50

/*
 * DC Motor Variables
**/

#define M_SPEED 7
#define M_AIN1 49
#define M_AIN2 48
#define M_STBY 47

/*
 * Global Variables
**/
int lastCounter = 0;

int currentStateCLK;
int lastStateCLK;

/*
 * Timekeeping
**/
unsigned long timeForRPM = millis();

/*
 * Menu values
 * Each power of 10 is equal to a level in the menu
 *
 */
#define ROOT 100

#define AUTORUN 10
#define REMOTE 20
#define INFO 30

#define SPEEDOMETER 1
#define ODOMETER 2
#define ORIENTATION 3
#define CURRENT_LOCATION 4
#define REMOTE_CONTROL_MESSAGE 5
#define WHEEL_DIAMETER 6
#define DISTANCE_BETWEEN_WHEELS 7
#define CURRENT_STATUS 8

#define SPACE "   "

int menuValue = ROOT;
int menuSelection = 0;
String menuString;
String menuOptions;

/*
 * Encoder variables
 */

#define ENCODER_DT 51
#define ENCODER_CLK 50

int encoderCounter = 0; // Just to keep in mind that one revolution is approx 30 "counts"

/*
 * DC Motor Variables
**/

#define M_SPEED 7
#define M_AIN1 49
#define M_AIN2 48
#define M_STBY 47

bool isCW = true;

double odometer = 0.0;
double pwmSpeed = 1;
double speed = 0.0;
bool isIncreasing = true;

unsigned long currentRPM;

/*
 * PID
 */
double pidSetpoint, pidInput, pidOutput;
double Kp = 2, Ki = 5, Kd = 1;

PID pid(&speed, &pwmSpeed, &pidSetpoint, Kp, Ki, Kd, DIRECT);

/*
 * Control mode variables
 */
#define CONTROL_STANDBY 0
#define CONTROL_AUTO 1
#define CONTROL_REMOTE 2

int controlMode = CONTROL_STANDBY;

/*
 * OLED Variables
 **/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*
 * Joystick Variables
 **/
int JOY_X = A8;
int JOY_Y = A9;

/*
 * MPU
 */

MPU6050 mpu(Wire);

/*
 * Buttons variables
 */
#define B_1 53
#define B_2 65
#define B_3 64
#define B_UP 61
#define B_DOWN 60
#define B_LEFT 59
#define B_RIGHT 58

/*
 * Task variables
 */
int currentTask = 0;
String currentCommand = "";

#define TARGET_SPEED 1
#define TARGET_ODOMETER 2


/*
 * Function declarations
 */

void menuForward();

void menuBack();

void getRPM();

double getSpeed();

double getOdometer();

void getEncoderCount();

void handleMenuInformation();

void handleMenuSelection();

void processSelection(int);

void handleOLED();

void handleSerialComm();

void handleAsync();

void autoMotor();

void stopAll();

void targetOdometer(double);

void targetSpeed(double);

/*
 * General functions
 */
int selectLine(int line) { return line * 8; }


/*
 * Main Arduino functions
 */

void setup() {
    Serial.begin(9600);

// Set encoder pins as inputs
    pinMode(ENCODER_CLK, INPUT);
    pinMode(ENCODER_DT, INPUT);

    // Set up motor pins
    pinMode(M_SPEED, OUTPUT);
    pinMode(M_AIN1, OUTPUT);
    pinMode(M_AIN2, OUTPUT);
    pinMode(M_STBY, OUTPUT);
    digitalWrite(M_STBY, HIGH);

    // Read the initial state of CLK
    lastStateCLK = digitalRead(ENCODER_CLK);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }

    // Joystick switch needs to have pullup
    pinMode(B_1, INPUT_PULLUP);
    pinMode(B_2, INPUT_PULLUP);
    pinMode(B_3, INPUT_PULLUP);
    pinMode(B_UP, INPUT_PULLUP);
    pinMode(B_DOWN, INPUT_PULLUP);
    pinMode(B_RIGHT, INPUT_PULLUP);
    pinMode(B_LEFT, INPUT_PULLUP);
    pinMode(JOY_Y, INPUT);
    pinMode(JOY_X, INPUT);

    // IMU init
    mpu.Initialize();

    // PID init
    pid.SetMode(AUTOMATIC);
}

void loop() {
    handleAsync();
}


void menuForward() {
    int selection = menuValue % ROOT;
    if (selection == 0) {
        // We are in the root
        processSelection(ROOT + ((menuSelection + 1) * 10));
    } else {
        // We are in the second menu
        // Figure out what are the options
        switch (selection) {
            case AUTORUN:
                switch (menuSelection) {
                    // Speedometer
                    case 0:
                        processSelection(ROOT + selection + SPEEDOMETER);
                        break;

                        // Odometer
                    case 1:
                        processSelection(ROOT + selection + ODOMETER);
                        break;

                        // Orientation
                    case 2:
                        processSelection(ROOT + selection + ORIENTATION);
                        break;

                        // Current Location
                    case 3:
                        processSelection(ROOT + selection + CURRENT_LOCATION);
                        break;
                }
                break;

            case REMOTE:
                switch (menuSelection) {
                    // Speedometer
                    case 0:
                        processSelection(ROOT + selection + SPEEDOMETER);
                        break;

                        // Odometer
                    case 1:
                        processSelection(ROOT + selection + ODOMETER);
                        break;

                        // Orientation
                    case 2:
                        processSelection(ROOT + selection + ORIENTATION);
                        break;

                        // Current Location
                    case 3:
                        processSelection(ROOT + selection + CURRENT_LOCATION);
                        break;

                        // Remote Control Message
                    case 4:
                        processSelection(ROOT + selection + REMOTE_CONTROL_MESSAGE);
                        break;
                }
                break;

            case INFO:
                switch (menuSelection) {
                    // Wheel Diameter
                    case 0:
                        processSelection(ROOT + selection + WHEEL_DIAMETER);
                        break;

                        // Distance between two wheels
                    case 1:
                        processSelection(ROOT + selection + DISTANCE_BETWEEN_WHEELS);
                        break;

                    case 2:
                        processSelection(ROOT + selection + CURRENT_STATUS);
                        break;
                }
                break;
        }
    }

    // Put the cursor on top of the menu again
    menuSelection = 0;
}

void menuBack() {
    // Check what level are we
    int level = menuValue % ROOT;
    if (level > 0) {
        if (level % 10 > 0) {
            processSelection(ROOT + ((level / 10) * 10));
        } else {
            processSelection(ROOT);
        }

        // Don't forget to reset the cursor
        menuSelection = 0;
    }
}

void getRPM() {
    /*
     * To calculate the RPM I'll use 1 revolution of the wheel, that is approx 30 in the "encoderCounter"
     * Since every 30 the wheel does one revolution, but that's not exact :/
     */

    //Serial.println(encoderCounter);

    int counterResult = 0;

    if (encoderCounter > lastCounter) {
        counterResult = encoderCounter - lastCounter;
    } else if (encoderCounter < lastCounter) {
        counterResult = lastCounter - encoderCounter;
    }

    if (counterResult == 0) {
        currentRPM = 0;

    } else if (counterResult >= 30) {
        // 1 revolution have passed
        lastCounter = encoderCounter;

        // Add it to the odometer
        odometer += 0.215;

        // Get the time it took
        unsigned long elapsedTime = millis() - timeForRPM;

        timeForRPM = millis();

        currentRPM = (1 * (60000 / elapsedTime));
    }
}

double getSpeed() {
    // Returns in meters/hour
    return (0.215 * (float) currentRPM) * 60;
}

double getOdometer() {
    return odometer; // Returns in total meters traveled
}

void getEncoderCount() {
// Read the current state of CLK
    currentStateCLK = digitalRead(ENCODER_CLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {

        // If the DT state is different than the CLK state then
        // the encoder is rotating CCW so decrement
        if (digitalRead(ENCODER_DT) != currentStateCLK) {
            encoderCounter--;

        } else {
            // Encoder is rotating CW so increment
            encoderCounter++;

        }
    }

    // Remember last CLK state
    lastStateCLK = currentStateCLK;
}

void handleMenuInformation() {
    display.setCursor(0, 0);     // Start at top-left corner

    // Get where in the menu tree we are
    int remainder = menuValue % ROOT;
    if (remainder == 0 || remainder % 10 == 0) menuString = "Mode";

    // Initiate the default menu options
    menuOptions = SPACE;
    menuOptions.concat("Autorun\n");
    menuOptions.concat(SPACE);
    menuOptions.concat("Remote Control\n");
    menuOptions.concat(SPACE);
    menuOptions.concat("System Info");

    if (menuValue > ROOT && remainder % 10 == 0) {
        // Second level of the menu
        switch (remainder) {
            case AUTORUN:
                menuString += "/Autorun";
                menuOptions = SPACE;
                menuOptions.concat("Speedometer\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Odometer\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Orientation\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Current Location");

                break;

            case REMOTE:
                menuString += "/Remote-Control";
                menuOptions = SPACE;
                menuOptions.concat("Speedometer\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Odometer\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Orientation\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Current Location\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Remote Control Message");
                break;

            case INFO:
                menuString += "/System-Info";
                menuOptions = SPACE;
                menuOptions.concat("Wheel Diameter\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Dist Btwn Wheels\n");
                menuOptions.concat(SPACE);
                menuOptions.concat("Current Status");
                break;
        }
    } else if (menuValue > ROOT) {
        // This is the third level
        // Shows the whatever information we want
        switch (menuValue % 10) {
            case SPEEDOMETER:
                menuString = "Speedometer";
                break;

                // Odometer
            case ODOMETER:
                menuString = "Odometer";
                break;

                // Orientation
            case ORIENTATION:
                menuString = "Orientation";
                break;

                // Current Location
            case CURRENT_LOCATION:
                menuString = "Current Location";
                break;

                // Remote Control Message
            case REMOTE_CONTROL_MESSAGE:
                menuString = "Remote Control Message";
                break;

            case WHEEL_DIAMETER:
                menuString = "Wheel Diameter";
                break;

                // Distance between two wheels
            case DISTANCE_BETWEEN_WHEELS:
                menuString = "Distance Between Two Wheels";
                break;

            case CURRENT_STATUS:
                menuString = "Current Status";
                break;
        }
    }

    // Display the built menu string
    display.println(menuString);

    // Check if we are showing info or in menu
    // If the following if block is true, is a menu
    // Else is an information screen
    if (menuValue % 10 == 0) {
        // Display the options
        display.setCursor(0, selectLine(3));
        display.print(menuOptions);

        // Draw the first option selector
        // The number on the select line is the same as above
        display.setCursor(0, selectLine(3 + menuSelection));
        display.print('>');
    } else {
        // Clear the canvas
        display.fillRect(0, selectLine(3), SCREEN_WIDTH, SCREEN_HEIGHT - selectLine(3), SSD1306_BLACK);
        display.setCursor(0, selectLine(3));

        // Show the information of the desired mode
//        Serial.print("Showing info - mode ");
//        Serial.println(menuValue % 10);
        switch (menuValue % 10) {
            case SPEEDOMETER:
                // TODO maybe I broke something here
                display.print(speed);
                display.println(" meters per hour.");
                break;

            case ODOMETER:
                display.print(odometer);
                display.println(" meters traveled.");
                break;

            case ORIENTATION:
                //IMU checking
                mpu.Execute();
                display.print("X: ");
                display.println(mpu.GetAngX());
                display.print("Y: ");
                display.println(mpu.GetAngY());
                display.print("Z: ");
                display.println(mpu.GetAngZ());
                break;

            case REMOTE_CONTROL_MESSAGE:
                break;

            case WHEEL_DIAMETER:
                break;

            case DISTANCE_BETWEEN_WHEELS:
                break;

            case CURRENT_STATUS:
                mpu.Execute();
                display.print("Speed: ");
                display.println(speed);
                display.print("Odometer: ");
                display.println(speed);
                display.print("X:");
                display.print((int)mpu.GetAngX());
                display.print("|Y:");
                display.print((int)mpu.GetAngY());
                display.print("|Z:");
                display.println((int)mpu.GetAngZ());
                display.print("Current PWM: ");
                display.println(pwmSpeed);
                display.print("P:");
                display.print(Kp);
                display.print("|I:");
                display.print(Ki);
                display.print("|D:");
                display.println(Kd);
                break;
        }
    }

    display.display();
}

void handleMenuSelection() {
    // Go up on menu
    if (digitalRead(B_UP) == LOW) {
        if (menuSelection > 0)
            menuSelection--;
    }

    // Go down on menu
    if (digitalRead(B_DOWN) == LOW) {
        int selection = (((menuValue % ROOT) / 10) * 10);

        // Default value for root menu
        int maxValue = 2;

        switch (selection) {
            case AUTORUN:
                maxValue = 3;
                break;

            case REMOTE:
                maxValue = 4;
                break;

            case INFO:
                maxValue = 2;
                break;
        }

        if (menuSelection < maxValue)
            menuSelection++;
    }

    // Select menu item
    if (digitalRead(B_RIGHT) == LOW) {
        menuForward();
    }

    // Go back
    if (digitalRead(B_LEFT) == LOW) {
        menuBack();
    }
}

/**
 * Gets the selection and determines what to do with it
 * This is the only function allowed to make changes to the selection global variable
 *
 * @param selection
 */
void processSelection(int selection) {
    menuValue = selection;

    /*
     * Process any behavior change that is not associated with the OLED display here
     * The OLED display automatically gets the menuValue and displays the right thing
     * Defined in the handleMenuInformation function
     */

    // Check if autorun or remote control is currently selected
    if (menuValue - ROOT - (menuValue % 10) == AUTORUN) {
        controlMode = CONTROL_AUTO;
    } else if (menuValue - ROOT - (menuValue % 10) == REMOTE) {
        controlMode = CONTROL_REMOTE;
    } else {
        controlMode = CONTROL_STANDBY;
    }
}

void handleOLED() {
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
}

String receiveSerialString() {
    const int inputSize = 50;
    char inputArray[inputSize];
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    // if (Serial.available() > 0) {
    while (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            inputArray[ndx] = rc;
            ndx++;
            if (ndx >= inputSize) {
                ndx = inputSize - 1;
            }
        }
        else {
            inputArray[ndx] = '\0'; // terminate the string
            ndx = 0;
        }
    }

    return {inputArray};
}

/**
 * This is useful for both outputting messages in the serial monitor
 * And for getting input from there
 * We can use it for controlling the menus
 * For now, just the speedometer, odometer and orientation information are available
 *
 * Input in the serial monitor:
 * a: for automatic - increases and decreases speed automatically
 * m: for manual - still have to implement it
 *
 * s: for setting the target speed
 * o: for setting the target odometer reading
 * r: to see orientation stats in OLED
 * i: to get that information in serial monitor
 */
void handleSerialComm() {
    /*
     * This used for inputting data from serial
     * Use with Serial.readString()
     * inputString.startsWith("whatever")
     */
    char inputChar;

    if (Serial.available() > 0) {
        //inputChar = Serial.read();

        String option = receiveSerialString();

        Serial.print("Received string: ");
        Serial.println(option);

        currentCommand = option;

        //inputString = Serial.readString();
        if(option.startsWith("odom")) {
            currentTask = TARGET_ODOMETER;
        } else if (option.startsWith("speed")) {
            currentTask = TARGET_SPEED;
        } else if (option.startsWith("P")) {
            Kp = option.substring(2).toDouble();
        } else if (option.startsWith("I")) {
            Ki = option.substring(2).toDouble();
        } else if (option.startsWith("D")) {
            Kd = option.substring(2).toDouble();
        } else if (option.startsWith("info")) {
            Serial.print("Current RPM: ");
            Serial.print(currentRPM);
            Serial.print(" | Current PWM: ");
            Serial.print(pwmSpeed);
            Serial.print(" | Current speed (meters/minute): ");
            Serial.print(getSpeed());
            Serial.print(" | Current odometer reading :");
            Serial.println(getOdometer());

            //IMU checking
            Serial.print("[");
            mpu.Execute();
            Serial.print(mpu.GetAngX());
            Serial.print("  ");
            Serial.print(mpu.GetAngY());
            Serial.print("  ");
            Serial.print(mpu.GetAngZ());
            Serial.println("]");
        } else {
            currentCommand = "";
            currentTask = 0;
        }
    }
}

unsigned long hundredMillis = millis();
unsigned long oneMillis = millis();
unsigned long tenMicros = micros();

void handleAsync() {
    // Doing 10 micros for encoder
    if (micros() > tenMicros + 10) {
        tenMicros = micros();
        getEncoderCount();
    }

    if (millis() > oneMillis + 1) {
        oneMillis = millis();

        getRPM();
    }


    if (millis() > hundredMillis + ROOT) {
        // Reset the global time variable to reflect now
        hundredMillis = millis();
        //Serial.println(hundredMillis);

        // Handlers
        handleMenuInformation();
        handleMenuSelection();
        handleOLED();
        handleSerialComm();

        switch (controlMode) {
            case CONTROL_AUTO:
                autoMotor();
                break;

            case CONTROL_REMOTE:
                switch (currentTask) {
                    case TARGET_ODOMETER:
                        targetOdometer(currentCommand.substring(5).toDouble());
                        break;

                    case TARGET_SPEED:
                        targetSpeed(currentCommand.substring(6).toDouble());
                        break;
                }
                break;

            default:
                stopAll();
                break;
        }
    }
}

/*
 * Actuators
 */

void autoMotor() {
    // Handle the increase in speed automatically
    int speedStep = isIncreasing ? 1 : -1;
    pwmSpeed += speedStep;

    if (pwmSpeed > 255) {
        pwmSpeed = 255;
        isIncreasing = false;
        isCW = false;
    }

    if (pwmSpeed < 50) {
        pwmSpeed = 50;
        isIncreasing = true;
        isCW = true;
    }

    digitalWrite(M_STBY, HIGH);

    if (isCW) {
        digitalWrite(M_AIN1, HIGH);
        digitalWrite(M_AIN2, LOW);
    } else {
        digitalWrite(M_AIN1, LOW);
        digitalWrite(M_AIN2, HIGH);
    }

    analogWrite(M_SPEED, pwmSpeed);

//    Serial.print("Current speed: ");
//    Serial.println(speed);
}

void stopAll() {
    digitalWrite(M_STBY, LOW);
}

/**
 * This functions controls the wheel to run at full speed until reaches the desired odometer
 *
 * @param odometerReading The target total distance traveled
 */
void targetOdometer(double odometerReading) {
    if (odometerReading <= odometer) {
        digitalWrite(M_STBY, LOW);
    } else {
        digitalWrite(M_STBY, HIGH);
        digitalWrite(M_AIN1, HIGH);
        digitalWrite(M_AIN2, LOW);
        analogWrite(M_SPEED, 255);
    }
}

/**
 * The function that when called sets the robot to run at a certain speed
 *
 * @param speedReading Accepts the speed value to which you want the robot to maintain
 */
void targetSpeed(double speedReading) {
    digitalWrite(M_STBY, HIGH);
    digitalWrite(M_AIN1, HIGH);
    digitalWrite(M_AIN2, LOW);

    // Ramp up the speed until we hit the desired value
    // Use PID for that
    pidSetpoint = speedReading;

    speed = getSpeed();
    pid.Compute();
    analogWrite(M_SPEED, pwmSpeed);
}