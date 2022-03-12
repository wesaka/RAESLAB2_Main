#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyMPU6050.h>

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
int infoMode = 0;

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

bool isAuto = false;
bool isCW = true;

double odometer = 0.0;
int speed = 1;
bool isIncreasing = true;

unsigned long currentRPM;

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
 * General functions
 */
int selectLine(int line) { return line * 8; }

void menuForward() {
    int selection = menuValue % 100;
    if (selection == 0) {
        // We are in the root
        menuValue += ((menuSelection + 1) * 10);
    } else {
        // We are in the second menu
        // Figure out what are the options
        infoMode = 1;
        switch (selection) {
            case AUTORUN:
                switch (menuSelection) {
                    // Speedometer
                    case 0:
                        menuString = "Speedometer";
                        menuValue = 100 + selection + SPEEDOMETER;
                        break;

                        // Odometer
                    case 1:
                        menuString = "Odometer";
                        menuValue = 100 + selection + ODOMETER;
                        break;

                        // Orientation
                    case 2:
                        menuString = "Orientation";
                        menuValue = 100 + selection + ORIENTATION;
                        break;

                        // Current Location
                    case 3:
                        menuString = "Current Location";
                        menuValue = 100 + selection + CURRENT_LOCATION;
                        break;
                }
                break;

            case REMOTE:
                switch (menuSelection) {
                    // Speedometer
                    case 0:
                        menuString = "Speedometer";
                        menuValue = 100 + selection + SPEEDOMETER;
                        break;

                        // Odometer
                    case 1:
                        menuString = "Odometer";
                        menuValue = 100 + selection + SPEEDOMETER;
                        break;

                        // Orientation
                    case 2:
                        menuString = "Orientation";
                        menuValue = 100 + selection + ORIENTATION;
                        break;

                        // Current Location
                    case 3:
                        menuString = "Current Location";
                        menuValue = 100 + selection + CURRENT_LOCATION;
                        break;

                        // Remote Control Message
                    case 4:
                        menuString = "Remote Control Message";
                        menuValue = 100 + selection + REMOTE_CONTROL_MESSAGE;
                        break;
                }
                break;

            case INFO:
                switch (menuSelection) {
                    // Wheel Diameter
                    case 0:
                        menuString = "Wheel Diameter";
                        menuValue = 100 + selection + WHEEL_DIAMETER;
                        break;

                        // Distance between two wheels
                    case 1:
                        menuString = "Distance Between Two Wheels";
                        menuValue = 100 + selection + DISTANCE_BETWEEN_WHEELS;
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
    int level = menuValue % 100;
    if (level > 0) {
        if (level % 10 > 0) {
            menuValue = 100 + ((level / 10) * 10);
        } else {
            menuValue = 100;
        }

        // Don't forget to reset the cursor
        menuSelection = 0;
    }

    // Reset the infoMode to show menus
    infoMode = 0;
}

/*
 *  Constructing MPU-6050
 */
MPU6050 mpu(Wire);

void getRPM() {
    /*
     * To calculate the RPM I'll use 1 revolution of the wheel, that is approx 30 in the "encoderCounter"
     * Since every 30 the wheel does one revolution, but that's not exact :/
     */

    Serial.println(encoderCounter);

    int counterResult;

    if (encoderCounter > lastCounter) {
        counterResult = encoderCounter - lastCounter;
    } else {
        counterResult = lastCounter - encoderCounter;
    }

    if (counterResult >= 30) {
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

void autoMotor() {
    // Handle the increase in speed automatically
    int speedStep = isIncreasing ? 1 : -1;
    speed += speedStep;

    if (speed > 255) {
        speed = 255;
        isIncreasing = false;
        isCW = false;
    }

    if (speed < 50) {
        speed = 50;
        isIncreasing = true;
        isCW = true;
    }

    if (isCW) {
        digitalWrite(M_AIN1, HIGH);
        digitalWrite(M_AIN2, LOW);
    } else {
        digitalWrite(M_AIN1, LOW);
        digitalWrite(M_AIN2, HIGH);
    }

    analogWrite(M_SPEED, speed);

//    Serial.print("Current speed: ");
//    Serial.println(speed);
}

void handleMenu() {
    display.setCursor(0, 0);     // Start at top-left corner

    // Get where in the menu tree we are
    int remainder = menuValue % 100;
    if (remainder == 0 || remainder % 10 == 0) menuString = "Mode";

    // Initiate the default menu options
    menuOptions = SPACE;
    menuOptions.concat("Autorun\n");
    menuOptions.concat(SPACE);
    menuOptions.concat("Remote Control\n");
    menuOptions.concat(SPACE);
    menuOptions.concat("System Info");

    if (remainder > 0 && remainder % 10 == 0) {
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
                menuOptions.concat("Dist Btwn Wheels");
                break;
        }
    }

    // Display the built menu string
    display.println(menuString);

    // Check if we are showing info or in menu
    if (infoMode == 0) {
        // Display the options
        display.setCursor(0, selectLine(3));
        display.print(menuOptions);

        // Draw the first option selector
        // The number on the select line is the same as above
        display.setCursor(0, selectLine(3 + menuSelection));
        display.print('>');
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
        int selection = (((menuValue % 100) / 10) * 10);

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
                maxValue = 1;
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

void handleOLED() {
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    if (infoMode == 1) {
        // Clear the canvas
        display.fillRect(0, selectLine(3), SCREEN_WIDTH, SCREEN_HEIGHT - selectLine(3), SSD1306_BLACK);
        display.setCursor(0, selectLine(3));

        // Show the information of the desired mode
//        Serial.print("Showing info - mode ");
//        Serial.println(menuValue % 10);
        switch (menuValue % 10) {
            case SPEEDOMETER:
                display.print(getSpeed());
                display.println(" meters per hour.");
                break;

            case ODOMETER:
                display.print(getOdometer());
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
        }
    }
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
 * s: for seeing the speed in the OLED
 * o: for seeing odometer in OLED
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
        inputChar = Serial.read();


        //inputString = Serial.readString();
        if (inputChar == 's') {
            infoMode = 1;
            menuValue = 111;
        } else if (inputChar == 'o') {
            infoMode = 1;
            menuValue = 112;
        } else if (inputChar == 'r') {
            infoMode = 1;
            menuValue = 113;
        } else if (inputChar == 'm') {
            Serial.println("Changing to manual control mode.");
            isAuto = false;
        } else if (inputChar == 'a') {
            Serial.println("Changing to auto control mode.");
            isAuto = true;
        } else if (inputChar == 'i') {
            Serial.print("Current RPM: ");
            Serial.print(currentRPM);
            Serial.print(" | Current PWM: ");
            Serial.print(speed);
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
        }
    }
}

unsigned long hundredMillis = millis();
unsigned long oneMillis = millis();

void handleAsync() {
    if (millis() > oneMillis + 1) {
        oneMillis = millis();

        getEncoderCount();
        getRPM();
    }


    if (millis() > hundredMillis + 100) {
        // Reset the global time variable to reflect now
        hundredMillis = millis();
        //Serial.println(hundredMillis);

        // Handlers
        if (isAuto) autoMotor();
        handleMenu();
        handleMenuSelection();
        handleOLED();
        handleSerialComm();
    }
}

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
}

void loop() {
    handleAsync();
}
