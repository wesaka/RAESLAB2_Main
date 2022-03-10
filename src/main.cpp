#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#include "BasicStepperDriver.h"
#include "robot_functions.h"
#include "info_functions.h"

/*
 * Global Variables
**/

unsigned long myTime;
int secondCount = 0;

bool clicked = false;
int clickCount = 0;

int infoMode = 0;

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

int menuValue = 100;
int menuSelection = 0;
String menuString;
String menuOptions;

/*
 * DC Motor Variables
**/

#define M_SPEED 2
#define M_AIN1 16
#define M_AIN2 17
//#define M_STBY 49

bool isCW = true;

int speed = 1;
bool si = true;

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

int read_x;
int read_y;

bool joyManual = false;
bool serialManual = false;

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
            menuValue = 100 + ((level/10) * 10);
        } else {
            menuValue = 100;
        }

        // Don't forget to reset the cursor
        menuSelection = 0;
    }

    // Reset the infoMode to show menus
    infoMode = 0;
}

void handleDCMotor() {
    if (joyManual) {
        // Handle the increase in speed manually
        speed = deadzoneMap(read_x, 0, 1023, -255, 255);

        // Serial.print("Increasing speed manually. Current speed: ");
        // Serial.println(speed);

        if (speed < 0) {
            isCW = false;
            speed *= -1;
        } else {
            isCW = true;
        }
    } else if (!serialManual) {
        // Handle the increase in speed automatically
        int speedStep = si ? 1 : -1;
        speed += speedStep;

        if (speed > 255) {
            speed = 255;
            si = false;
        }

        if (speed < 0) {
            speed = 0;
            si = true;
            isCW = !isCW;
        }
    }

    if (isCW) {
        digitalWrite(M_AIN1, HIGH);
        digitalWrite(M_AIN2, LOW);
    } else {
        digitalWrite(M_AIN1, LOW);
        digitalWrite(M_AIN2, HIGH);
    }

    analogWrite(M_SPEED, speed);
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

    if(remainder > 0 && remainder % 10 == 0) {
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
        int selection = (((menuValue % 100)/10)*10);

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

        // Show the information of the desired mode
        switch (menuValue % 10) {
            case SPEEDOMETER:
                // TODO call functions and show the return value
                break;

            case ODOMETER:
                break;

            case CURRENT_LOCATION:
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

void checkJoySwitch() {
//    if ((digitalRead(B_1) == LOW) && (clicked == false)) {
//        joyManual = !joyManual;
//        serialManual = false;
//        Serial.println("Setting joyManual");
//        clicked = true;
//    }
}

void handleJoystick() {
    // Downwards smaller, upwards bigger
    read_x = analogRead(JOY_X);

    // Y goes from approx 20 to 950
    // Left smaller, right bigger
    read_y = analogRead(JOY_Y);

    // Serial.print("Joy X: ");
    // Serial.print(read_x);
    // Serial.print(" Joy Y: ");
    // Serial.println(read_y);
}

void handleSerialComm() {
    /**
     * This is the main function that regulates the Serial Communication
     * What keywords does the program understand?
     *
     * auto|manual - The same behavior as the joystick button push, setting either auto or manual
     * leds on  - Turns all leds on at full brightness
     * leds off - Turns off all leds
     * leds dim - The deafult behavior that the leds goes to full brightness and dim in different intervals
     * dc (speed - from -255 to 255) - The same behavior as the joy x on manual
     * stepper (degrees) - How mnay degrees you want the stepper to go
     * servo (x) (y) - x and y are the angles from 0 to 180 that the servos must assume
     **/

    String comm;

    if (Serial.available()) {
        comm = Serial.readString();

        if (comm.startsWith("auto")) {
            joyManual = false;
            serialManual = false;

        } else if (comm.startsWith("manual")) {
            joyManual = false;
            serialManual = true;

        } else if (comm.startsWith("leds")) {
            Serial.println("Leds serial called");
            String option = comm.substring(5, 7);

        } else if (comm.startsWith("dc")) {
            String stringSpeed = comm.substring(3);
            int serialSpeed = stringSpeed.toInt();

            if (serialSpeed < 0) {
                isCW = false;
                speed = serialSpeed * -1;
            } else {
                isCW = true;
                speed = serialSpeed;
            }

        } else if (comm.startsWith("stepper")) {
            String stringAngle = comm.substring(8);
            int angle = stringAngle.toInt();

        } else if (comm.startsWith("servo")) {
            String xyValues = comm.substring(6);
            int serialX = xyValues.substring(0, xyValues.indexOf(" ")).toInt();;
            int serialY = xyValues.substring(xyValues.indexOf("")).toInt();
        } else if (comm.startsWith("oled")) {
            Serial.print("LED0 Brightness: ");

            Serial.print("Dir: ");
            Serial.print(isCW ? "CW" : "CCW");
            Serial.print(" | Spd: ");
            Serial.println(speed);
            Serial.println("");

            Serial.print("Step Pos (Deg): ");
            Serial.println("");
        } else {
            Serial.println("Command not recognized. Try again.");
        }
    }
}

void handleAsync() {
    unsigned long timeUpdated = millis();


    if (timeUpdated > myTime + 100) {
        // Reset the global time variable to reflect now
        myTime = timeUpdated;
        //Serial.println(myTime);

        // Handlers
        checkJoySwitch();
        handleJoystick();
        handleDCMotor();
        handleMenu();
        handleMenuSelection();
        handleOLED();
        handleSerialComm();

        if (clicked) {
            clickCount++;
            if (clickCount >= 3) {
                clicked = false;
                clickCount = 0;
            }
        }
    }
}

void setup() {
    Serial.begin(9600);

    pinMode(M_SPEED, OUTPUT);
    pinMode(M_AIN1, OUTPUT);
    pinMode(M_AIN2, OUTPUT);
//  pinMode(M_STBY, OUTPUT);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Joystick switch needs to have pullup
    pinMode(B_1, INPUT_PULLUP);
    pinMode(B_2, INPUT_PULLUP);
    pinMode(B_3, INPUT_PULLUP);
    pinMode(B_UP, INPUT_PULLUP);
    pinMode(B_DOWN, INPUT_PULLUP);
    pinMode(B_RIGHT, INPUT_PULLUP);
    pinMode(B_LEFT, INPUT_PULLUP);
    pinMode(JOY_X, INPUT);
    pinMode(JOY_X, INPUT);
}

void loop() {
    handleAsync();
}
