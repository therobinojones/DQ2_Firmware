#include <AccelStepper.h>
#include <EEPROM.h>
#include <Bounce2.h>

// Pin definitions for focus stepper motor
#define FOCUS_STEP_PIN 9
#define FOCUS_DIR_PIN 10
#define FOCUS_ENABLE_PIN 11

// Button pin definitions for focus control
#define FOCUS_BACKWARD_BUTTON_PIN 29
#define FOCUS_FORWARD_BUTTON_PIN 31

#define CAPPING_SHUTTER_PIN 1
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 5
#define OPTO_PIN 23
#define BEEPER_PIN 16
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_E6  1319
#define TRIGGER_INPUT23 0
#define TRIGGER_DEBOUNCE_TIME 50 // Debounce time in milliseconds
#define OPTO_INTERRUPT_DEBOUNCE_TIME 50 // Debounce time for film recorder tick in milliseconds

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Create an instance of the AccelStepper class for focus stepper
AccelStepper focusStepper(AccelStepper::DRIVER, FOCUS_STEP_PIN, FOCUS_DIR_PIN);

// Create instances of the Bounce class for debouncing the focus buttons
Bounce focusBackwardButton = Bounce(); 
Bounce focusForwardButton = Bounce();

// Focus Stepper motor settings
const long focusStepperAccel = 500; // Default acceleration for focus stepper
const int focusMaxSpeed = 5000; // Maximum speed for focus stepper
const int focusStepperSpeed = 1500; // Speed for focus stepper during movement

volatile unsigned long lastTriggerMillis = 0;
String command = "";
bool stepperEnabled = true;
long motorSpeed = 1000;
long stepsPerFullRotations = 800;
bool stepperRunning = false;
int stepperDirection = 1;
long lastStepperPos = 0;
int frameNumber = 0;
volatile unsigned long fpsTriggerMillis = 0;
volatile unsigned long prevFpsTriggerMillis = 0;
volatile int prevFrameNumber = 0;
bool soundEnabled = false;
float introThemeSpeed = 0.7;  // Adjust this value for the desired speed. 1.0 is normal speed.
bool accMoveEnabled = false; 
volatile unsigned long startMillis = 0; 
unsigned long lastPrintMillis = 0;
unsigned long shutterOpenTimeCompensation = 150;
long stepperAccel = 6000; // Default acceleration
double prevRotationMillis = 0.0;
double fps = 0.0;
bool wasRunning = false;
int prevCommandDirection = 1;
bool needsHoming = false;
bool isShooting = false;    // Keeps track of whether the camera is shooting or not.
String shootingStr = "idle";    // Keeps track of the state of shooting: 'inProgress' or 'idle'.
int eepromAddress = 1; // Choose the address to store the frame
bool bulbModeActive = false;
unsigned long lastMillis = 1; //address of eeprom, there are 1024 eventually will need to cycle through addresses for less stress on the eeprom.
Bounce debouncer = Bounce(); //debouncing instance for the Bounce2.h library
volatile bool triggerState; // Trigger state
bool filmRecorderEnabled = false;
volatile unsigned long lastOptoInterruptMillis = 0;
unsigned long cappingShutterTimer = 0;
String lastBulbExposureCommand = "";
int serialFocusDirection = 0; // 0 = Stop, -1 = Backward, 1 = Forward


void setup(){
    pinMode(CAPPING_SHUTTER_PIN, OUTPUT);
    digitalWrite(CAPPING_SHUTTER_PIN, HIGH);
    pinMode(OPTO_PIN, INPUT_PULLUP);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(BEEPER_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    pinMode(TRIGGER_INPUT23, INPUT_PULLUP);
    delay(100); // Delay to allow pin state to stabilize
    debouncer.attach(TRIGGER_INPUT23);
    debouncer.interval(TRIGGER_DEBOUNCE_TIME);

    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(stepperAccel); //speed for moveto command used in homing
    attachInterrupt(digitalPinToInterrupt(OPTO_PIN), interruptHandler, CHANGE);
    stepper.setMinPulseWidth(10); //only when using Teensy4.1

    // Focus stepper motor pin setup
    pinMode(FOCUS_ENABLE_PIN, OUTPUT);
    digitalWrite(FOCUS_ENABLE_PIN, HIGH); // Enable focus stepper motor

    // Focus stepper settings
    focusStepper.setMaxSpeed(focusMaxSpeed);
    focusStepper.setAcceleration(focusStepperAccel);
    focusStepper.setMinPulseWidth(10); // Set min pulse width for focus stepper

    // Button setup for focus control
    pinMode(FOCUS_BACKWARD_BUTTON_PIN, INPUT_PULLUP);
    focusBackwardButton.attach(FOCUS_BACKWARD_BUTTON_PIN);
    pinMode(FOCUS_FORWARD_BUTTON_PIN, INPUT_PULLUP);
    focusForwardButton.attach(FOCUS_FORWARD_BUTTON_PIN);

    // Starts serial 
    Serial.begin(9600);
    delay(2000);
        
    // Prints available commands
    Serial.println("Welcome to RobinoMitch");
    Serial.println("Available commands:");
    Serial.println("h - Home Stepper. Default homing is forward");
    Serial.println("hf - Home Stepper Forward");
    Serial.println("hb - Home Stepper Backward");
    Serial.println("rf - Continuous mode. Forward");
    Serial.println("rb - Continuous mode. Backward");
    Serial.println("g - Frame By Frame mode. E.g.: gf50 shoots 50 frames forward/ gb5 shoots 5 frames backward.");
    Serial.println("t - Timed exposure mode. E.g.: tf5 shoots 1 frame for 5 seconds / tb120 shoots 1 frame backward for 2 minutes");
    Serial.println("bfo - Bulb mode open shutter. Once it starts shooting 1 frame forward");
    Serial.println("bbo - Bulb mode open shutter. Once it starts shooting 1 frame Reverse");
    Serial.println("bc - Bulb mode close shutter.");
    Serial.println("x - Stop motor");
    Serial.println("r - Reset frame counter");
    Serial.println("m - Enable/disable stepper motor. Useful to load camera or any time you need to manually rotate the stepper");
    Serial.println("h - Homing");
    Serial.println("s - Set speed. For example: s5000");
    Serial.println("a - Set acceleration. For example: a5000, max value is 200000");
    Serial.println("f - Print current frame");
    Serial.println("p - Mute/Unmute sound");
    Serial.println("go - Jog to particular frame. For example 'go10' will jog to 10th frame. 'go-1' will jog to previous frame");
    Serial.println("j - Reset frame number. J command follows the new frame. For example, 'j10' will reset current frame  to 10");
    homeStepper("f"); //home stepper forward
    
    // Initialise frameNumber from EEPROM
    frameNumber = EEPROM.read(eepromAddress);

    // Initialise frameNumber from EEPROM
    long lastSavedFrame;                           // changed to long type.
    EEPROM.get(eepromAddress, lastSavedFrame);     // used EEPROM.get instead of EEPROM.read
    setFrame(lastSavedFrame);

      // Read initial state and set triggerState
  triggerState = digitalRead(TRIGGER_INPUT23) == HIGH;

  //     if (triggerState) {
  //   Serial.println("TRIGGER OFF");
  // } else {
  //   Serial.println("TRIGGER ON");
  // }

  attachInterrupt(digitalPinToInterrupt(TRIGGER_INPUT23), handleInterrupt, CHANGE);
}

void loop() {

  focus_logic();

    // Check and handle incoming commands
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        if (inChar != '\n') {
            command += inChar;
        } else {
            handleCommand(command);
            command = "";
        }
    }

    if (cappingShutterTimer > 0 && (millis() - cappingShutterTimer >= 500)) {
        digitalWrite(CAPPING_SHUTTER_PIN, HIGH); // Deactivate the shutter; adjust HIGH/LOW as needed
        cappingShutterTimer = 0; // Reset the timer
    }

    // Trigger input
      debouncer.update();
  if (debouncer.fell()) { // Change state only on button press (falling edge)
    triggerState = !triggerState;
    if (triggerState) {
      Serial.println("TRIGGER OFF");
    } else {
      Serial.println("TRIGGER ON");
    }
  }

    // Stepper frame calculation and running
    long stepperPos = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
    if (stepperPos != lastStepperPos) { 
        frameNumber = stepperPos; 
        lastStepperPos = stepperPos; 
        Serial.print("FRAME = "); 
        Serial.println(-stepperPos); 
        if(soundEnabled) {
            tone(BEEPER_PIN, 1000, 20);
        }
        calculateAndPrintFPS();
    }


    if (!stepper.isRunning() && frameNumber != prevFrameNumber) {
        EEPROM.put(eepromAddress, -frameNumber);
        prevFrameNumber = frameNumber;
        Serial.println("Saved frame number to EEPROM. ");
        isShooting = false; 
    }

    if (stepperRunning) {
        if (stepperDirection == 1)
            stepper.setSpeed(motorSpeed);
        else
            stepper.setSpeed(-motorSpeed);
        stepper.runSpeed();
    }

    // Handle acceleration movement
    if (!stepperRunning && accMoveEnabled) {
        if (!stepper.isRunning()) {
            digitalWrite(ENABLE_PIN, HIGH);
            accMoveEnabled = false;
        } else {
            stepper.run();
        }
    }
    
    if(!stepper.isRunning() && wasRunning && isShooting) {
        wasRunning = false;
        isShooting = false;  // update shooting status to false
        shootingStr = "idle";  // update shooting status string to idle
        Serial.println("Motor fully stopped");
        if(needsHoming) {
            if(prevCommandDirection == -1) { // If the last movement command was "rf"
                homeStepper("f"); // home forward
            } else if(prevCommandDirection == 1) { // If the last movement command was "rb"
                homeStepper("b"); // home backward
            }
            needsHoming = false; // reset the flag
        }
    } else if(stepper.isRunning()) {
        wasRunning = true;
    }
    
    // Print shooting state every second
    if(millis() - lastPrintMillis >= 1000) {
        lastPrintMillis = millis();
        Serial.print("SHOOTING = ");
        Serial.println(shootingStr);
    }

        // Print elapsed time if bulb mode is active
        if (bulbModeActive && (millis() - lastMillis >= 1000)) {
        lastMillis = millis();
        Serial.print("Elapsed Time: ");
        Serial.print((millis() - startMillis) / 1000);
        Serial.println(" seconds");
    }
}

int calculateSteps(int time_in_seconds) {
    return motorSpeed * time_in_seconds; 
}

void handleCommand(String command){


    // Add handling for focus control commands
    if (command == "ff") {
        serialFocusDirection = 1; // Set to move forward
    } else if (command == "fb") {
        serialFocusDirection = -1; // Set to move backward
    } else if (command == "fs") {
        serialFocusDirection = 0; // Set to stop
    }

    if (command.startsWith("a")) {
        int newAccel = command.substring(1).toInt();
        if (newAccel > 0 && newAccel <= 200000) { 
            // Assuming 200000 is the maximum acceleration.
            stepper.setAcceleration(newAccel);
            stepperAccel = newAccel;  // Also update the global variable.
            Serial.print("Acceleration set to: ");
            Serial.println(newAccel);
        } else {
            Serial.println("Invalid acceleration. Please enter a value between 1 and 200000.");
        }
    }

        if (command == "w") {
        Serial.print("Motor speed: ");
        Serial.println(motorSpeed);
        Serial.print("Acceleration: ");
        Serial.println(stepperAccel);
    }

    if (command.startsWith("go")) {
        jogToFrame(command.substring(2).toInt());
    }

    if (command.startsWith("h")){
            String homing_dir = command.substring(1);
            homeStepper(homing_dir);
    }

    if (command == "m") {
        stepperEnable();
    }

    if (command == "p") {
        soundEnabled = !soundEnabled;
        String soundStatusString = (soundEnabled) ? "ON" : "OFF";
        Serial.println("Sound is now " + soundStatusString);
    }

if(command.startsWith("rf") || command.startsWith("rb")){
    digitalWrite(CAPPING_SHUTTER_PIN, LOW); // Activate the shutter before starting shooting
    String numTimeStr = command.substring(2);
    int numTime;
    if(numTimeStr.toInt() == 0 && numTimeStr[0] != '0'){
        Serial.println("No custom shot duration entered, setting time to infinty");
        numTime = 13000000; // use the infinity large step number if no time is provided
    } else {
        numTime = calculateSteps(numTimeStr.toInt());
    }

    int direction = command.startsWith("rf") ? -1 : 1;
    accMoveEnabled = true;
    needsHoming = true;
    stepper.setMaxSpeed(motorSpeed); // Set the max speed to the global variable
    digitalWrite(ENABLE_PIN, HIGH); // Enable stepper
    stepper.move(numTime * direction); // Move with calculated steps
    prevCommandDirection = direction; // Set the previous command direction so we can home properly once motor is stopped.
    isShooting = true;
    shootingStr = "inProgress";
}

if (command.startsWith("ef")) {
    if (command.substring(2) == "on") {
        filmRecorderEnabled = true;
        Serial.println("Film Recorder is ON");
    } else if (command.substring(2) == "off") {
        filmRecorderEnabled = false;
        Serial.println("Film Recorder is OFF");
    } else {
        Serial.println("Invalid command. Usage: ef on or ef off.");
    }
}

    if (command == "r") {
        stepper.setCurrentPosition(0);
        //isShooting = false;
        //needsHoming = true;
        lastStepperPos = 0;
        Serial.print("FRAME = ");
        Serial.println(lastStepperPos);
        EEPROM.put(eepromAddress, lastStepperPos); // used EEPROM.put instead of EEPROM.write
    }

if (command == "f") {
    long stepperPos = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
    Serial.print("FRAME = ");
    Serial.println(-stepperPos);

    long EEPROMFrame;        // changed to long type.
    EEPROM.get(eepromAddress, EEPROMFrame); // used EEPROM.get instead of EEPROM.read
    EEPROMFrame = -EEPROMFrame; // invert the value
    Serial.print("EEPROM FRAME = ");
    Serial.println(EEPROMFrame);
}

    if (command.startsWith("j")) {
        int newFrame = command.substring(1).toInt(); // Get the frame number after the "j"
        setFrame(newFrame);
    }


    if (command.startsWith("bfo") || command.startsWith("bbo")) {
        BulbExposureMode(command);
    }
    else if (command.startsWith("bc") ) {
        if(isShooting)
            BulbExposureCloseMode();
        else
            Serial.println("No bulb operation in progress");
    }

    if (command == "x") {
              cappingShutterTimer = millis(); // Start the timer

        if (stepperRunning) {
            // Calculate stop position at a full rotation from homing.
            long homePos = stepper.currentPosition() / stepsPerFullRotations * stepsPerFullRotations;
            long stopPos = homePos + stepperDirection * stepsPerFullRotations;

            stepperRunning = false;

            // Run until we're close to the stop position without deceleration.
            while ((stepperDirection > 0 ? stepper.currentPosition() < stopPos : stepper.currentPosition() > stopPos)) {
                stepper.runSpeed();
            }
        }
        Serial.println("Stopp procedure initiated");
        stepper.stop();

    }

    if (command.startsWith("tf") || command.startsWith("tb")) {
        TimedExposureMode(command);
    }


if (command.startsWith("gf") || command.startsWith("gb")) {
    int numFrames;
    if(command.substring(2).toInt() == 0 && command[2] != '0'){
        Serial.println("No frame number entered,  let's move 1 frame only");
        numFrames = 1;
    } else {
        numFrames = command.substring(2).toInt();
    }

    int direction = command.startsWith("gf") ? -1 : 1;
    digitalWrite(CAPPING_SHUTTER_PIN, LOW); // Activate the shutter before starting shooting

    for(int i=0; i < numFrames; i++) {
        isShooting = true;  // Shooting starts
        shootingStr = "inProgress"; 
        String tempCommand = "";
        long startingPosition = stepper.currentPosition();
        long endPosition = startingPosition + (stepsPerFullRotations * direction);
        
        stepper.setMaxSpeed(motorSpeed);
        stepper.moveTo(endPosition);
        while (stepper.distanceToGo() != 0) {
            if(Serial.available()) {
                char inChar = Serial.read();

                if(inChar == 'x') { 
                    Serial.println("Stopping after current frame");
                    numFrames = i+1;
                } else if(inChar == 'p') {
                    soundEnabled = !soundEnabled;
                    String soundStatusString = (soundEnabled) ? "ON" : "OFF";
                    Serial.println("Sound is now " + soundStatusString);
                } else if (inChar >= '0' && inChar <= '9' && tempCommand.startsWith("s")) {
                    tempCommand += inChar;
                } else if (inChar == 's') {
                    tempCommand = "s";
                } else if (inChar == '\n' && tempCommand.startsWith("s")) {
                    long newSpeed = tempCommand.substring(1).toInt();
                    if (newSpeed > 0 && newSpeed <= 200000) {
                        motorSpeed = newSpeed;
                        Serial.println("Motor speed set to " + String(motorSpeed));
                    } else {
                        Serial.println("Invalid speed. Please enter a value between 1 and 200000.");
                    }
                    tempCommand = "";
                }
            }
            stepper.run();
        }
        delay(1); 

        long currentStepperPos = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
        if (currentStepperPos != lastStepperPos) {
            frameNumber = currentStepperPos;
            lastStepperPos = currentStepperPos;

            if (frameNumber != prevFrameNumber) { 
                printShootingStatus();
            }

            prevFrameNumber = frameNumber;
            Serial.print("FRAME = ");
            Serial.println(-currentStepperPos);
            if(soundEnabled) {
                tone(BEEPER_PIN, 1000, 20);
            }
        }
    }
    digitalWrite(CAPPING_SHUTTER_PIN, HIGH); 
    isShooting = false;  // Shooting ends
    shootingStr = "idle"; 
    stepper.stop();
    Serial.println("Frame operation complete");
    EEPROM.put(eepromAddress, -frameNumber);
}



if (command.startsWith("s")) {
    int newSpeed = command.substring(1).toInt();
    if (newSpeed > 0 && newSpeed <= 200000) {  // Assuming 200000 is the maximum speed.
        motorSpeed = newSpeed;
        Serial.print("Motor speed set to: ");
        Serial.println(motorSpeed);
    } else {
        Serial.println("Invalid speed. Please enter a value between 1 and 200000.");
    }
}
}

void printShootingStatus() {
  Serial.print("SHOOTING = "); 
  Serial.println(shootingStr);
}

void homeStepper(String homing_direction) {

    if (stepperEnabled) { // Ensure stepper is enabled before moving
        long originalSpeed = motorSpeed; // Store the original speed
        motorSpeed = 1000; // Set the new speed for homing

        // Decision to move forward or backward during homing based on homing_direction
        if (homing_direction == "f") {
            Serial.println("Moving stepper out of opto zone forward before homing...");
            stepper.move(-50); // Move stepper 300 steps backward
            while (stepper.run()) {}; // Continue moving until we've completed the move
            delay(1);  // Add delay to let the motor stop
            stepper.setSpeed(-motorSpeed);  // Setting negative speed to go backwards
        } else if (homing_direction == "b") {
            Serial.println("Moving stepper out of opto zone backward before homing...");
            stepper.move(50); // Move stepper 300 steps forward
            while (stepper.run()) {}; // Continue moving until we've completed the move
            delay(1);  // Add delay to let the motor stop
            stepper.setSpeed(motorSpeed);  // Setting positive speed to go forward
        } else {
            // default case if no specific direction is specified, homing backward for instance
            Serial.println("Moving stepper out of opto zone forward before homing...");
            stepper.move(-50); 
            while (stepper.run()) {}; // Continue moving until we've completed the move
            delay(1);  // Add delay to let the motor stop
            stepper.setSpeed(-motorSpeed);  
        }

        Serial.println("Homing stepper...");

        // Use pin state of OPTO_PIN to control the homing process
        while (digitalRead(OPTO_PIN) == HIGH) {
            stepper.runSpeed();
        }

    long currentFrame = stepper.currentPosition(); // store the current position
        delay(10);
        stepper.setCurrentPosition(0); // reset the position
        stepper.stop();
        Serial.println("Homing complete");
        //isShooting = false;
        stepper.setCurrentPosition(currentFrame); // restore the frame count
        tone(BEEPER_PIN, 300, 50); // Simple beep
        delay(100);
        tone(BEEPER_PIN, 600, 35); // Simple beep
        motorSpeed = originalSpeed; // Reset the speed to the original global speed
    }
}


void stepperEnable(){
    if (stepperEnabled){
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println("Stepper disabled");
        stepperEnabled = false;
    } else {
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println("Stepper Enabled");
        stepperEnabled = true;
    }
}

void interruptHandler() {
    unsigned long currentMillis = millis();

    if (filmRecorderEnabled) {
        filmRecorderTick(); // Call the new function for handling film recorder ticks
    }
    lastTriggerMillis = millis(); // Retain this existing behavior of your original interrupt handler
}

void filmRecorderTick() {
    unsigned long currentMillis = millis();
    static bool lastOptoState = HIGH; // Keep track of the last state of OPTO_PIN

    // Read the current state of the OPTO_PIN
    bool currentOptoState = digitalRead(OPTO_PIN);

    // Check if current state is LOW and last state was HIGH (falling edge)
    if (currentOptoState == LOW && lastOptoState == HIGH &&
        (currentMillis - lastOptoInterruptMillis >= OPTO_INTERRUPT_DEBOUNCE_TIME)) {
        
        lastOptoInterruptMillis = currentMillis;  // Update the debounce timer
        Serial.println("filmRecorder_tick");      // Print the tick message
        tone(BEEPER_PIN, 6000, 80);
    }

    lastOptoState = currentOptoState; // Update the last state
}

void playIntro() {
    tone(BEEPER_PIN, NOTE_C6, 50);
    delay(500 * introThemeSpeed);

    tone(BEEPER_PIN, NOTE_C6, 50);
    delay(375 * introThemeSpeed);  // Beat 1 to Beat 5

    tone(BEEPER_PIN, NOTE_C6, 50);
    delay(250 * introThemeSpeed);  // Beat 5 to Beat 8

    tone(BEEPER_PIN, NOTE_C6, 50);
    delay(125 * introThemeSpeed);  // Beat 8 to Beat 10

    tone(BEEPER_PIN, NOTE_C6, 50);
    delay(250 * introThemeSpeed);   // Beat 10 to Beat 11

    tone(BEEPER_PIN, NOTE_D6, 50);
    delay(250 * introThemeSpeed);  // Beat 11 to Beat 13

    tone(BEEPER_PIN, NOTE_E6, 50);
    delay(250 * introThemeSpeed);  // Beat 13 to Beat 16

    tone(BEEPER_PIN, NOTE_C6, 50);
}


void updateAndPrintFps(){
    // get the elapsed time since the previous FPS measurement
    unsigned long elapsedMillis = fpsTriggerMillis - prevFpsTriggerMillis;
    // calculate the difference in frame count
    int frameDiff = frameNumber - prevFrameNumber; 
    
    //calculate FPS based on frameDiff and elapsedMillis
    float fps = ((float)frameDiff / (float)elapsedMillis) * 1000; // multiply by 1000 to convert from ms to seconds
    Serial.print("FPS = ");
    Serial.println(fps);
    
    //store the current frame count and time as previous for the next calculation
    prevFrameNumber = frameNumber;
    prevFpsTriggerMillis = fpsTriggerMillis;
}

void TimedExposureMode(String command) {
    // Store the current speed and acceleration.
    long prevSpeed = motorSpeed;
    long prevAccel = stepperAccel;
    
    // Set fixed speed and acceleration for bulb mode.
    int timedExpMotorSpeed = 2000;
    long timedExpMotorAccel = 2000;

    // Apply the new speed and acceleration.
    motorSpeed = timedExpMotorSpeed;
    stepperAccel = timedExpMotorAccel;
    stepper.setMaxSpeed(motorSpeed);
    stepper.setAcceleration(stepperAccel);

    int bulbTime = command.substring(2).toInt(); // Extract bulb time in seconds 

    // Store the current motor speed before changing it
    int direction = command.startsWith("tf") ? -1 : 1;

    long halfRotSteps = stepsPerFullRotations / 2; // Calculate half rotation steps 
    long startPos = stepper.currentPosition();

    // Perform first half rotation
    while(abs(stepper.currentPosition() - startPos) < halfRotSteps) {
      isShooting = true;
      shootingStr = "inProgress";
        stepper.setSpeed(motorSpeed * direction);
        stepper.runSpeed();
    }

    // Update the frame number after first rotation
    frameNumber = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
    //Serial.print("FRAME = ");
    //Serial.println(frameNumber);

      if (frameNumber != prevFrameNumber) {
        printShootingStatus();
      }
      
      prevFrameNumber = frameNumber;

    unsigned long startMillis = millis(); // Start time for the bulb time
    unsigned long lastMillis = millis();

    // Wait for bulbTime seconds, subtract the time it takes to open the shutter
    long exposureTime = bulbTime * 1000 - shutterOpenTimeCompensation; 

    while((millis() - startMillis) < exposureTime) { // Delay for adjusted bulbTime
        if((millis() - lastMillis) >= 1000) {
            lastMillis = millis();
            Serial.print("Elapsed Time: ");
            Serial.print((millis() - startMillis + shutterOpenTimeCompensation) / 1000); // Print elapsed time in seconds
            Serial.println(" seconds");
        }       
    }

    // Resume the wrap up the full rotation after the wait
    while(abs(stepper.currentPosition() - startPos) < stepsPerFullRotations) {
        stepper.setSpeed(motorSpeed * direction);
        stepper.runSpeed();
    }

     // Update the frame number after second rotation
    frameNumber = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
    //Serial.print("FRAME = ");
    //Serial.println(frameNumber);

    startMillis = 0; // set startMillis to 0 once the shutter is closed

    Serial.println("Bulb operation complete");
    isShooting = false;
    shootingStr = "idle";
    EEPROM.put(eepromAddress, -frameNumber); // Forcefully saving frame number
    Serial.println("Saved frame number after bc command");

    // Restore motor speed and acceleration to their original settings after bulb mode is done
    motorSpeed = prevSpeed;
    stepperAccel = prevAccel;
    stepper.setMaxSpeed(motorSpeed);
    stepper.setAcceleration(stepperAccel);

    // Reset Elapsed Time counter on serial
    Serial.print("Elapsed Time: ");
    Serial.println("0");
}

void calculateAndPrintFPS() {
  double now = micros();
  if (prevRotationMillis == 0.0) {
    prevRotationMillis = now;
  } else {
    fps = (2000000.0 / (now - prevRotationMillis)) / 2.0; // We're now dividing by 2 to compensate for the doubled speed
    Serial.print("FPS = ");
    Serial.println(fps, 6); //include 6 decimal places
    prevRotationMillis = now;
  }
}


void jogToFrame(int frameCount) {
    bool wasStopped = false;  // new flag
    long previousSpeed = motorSpeed;  // save the speed and acceleration
    long previousAccel = stepperAccel;
    int direction = frameCount < 0 ? 1 : -1;  // set direction based on frameCount
    long targetPosition = abs(frameCount) * stepsPerFullRotations * direction;

    motorSpeed = 3000;  // set the new speed for jogging
    stepper.setMaxSpeed(motorSpeed); 
    stepper.setAcceleration(6000);  // set the new acceleration for jogging
    prevCommandDirection = direction;  // save current jogging direction

    stepper.move(targetPosition);  // set the target position 

        auto printFrameNumber = [](bool shouldPrint){
        long stepperPos = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
        //long stepperPos = stepper.currentPosition() / stepsPerFullRotations;
        if (stepperPos != lastStepperPos) {
            frameNumber = stepperPos;
            lastStepperPos = stepperPos;
                  isShooting = true;
      shootingStr = "inProgress";
            if (shouldPrint) {
                Serial.print("FRAME = ");
                Serial.println(-stepperPos);
            }
            if(soundEnabled) {
                tone(BEEPER_PIN, 1000, 20);
            }
            calculateAndPrintFPS();
        }
    };

    while (stepper.distanceToGo() != 0) {
        if (Serial.available()) {
            char inChar = Serial.read();
            if (inChar == 'x') {
                wasStopped = true;  // set the flag
                Serial.println("Stopping after current command.");
                stepper.stop();  // set the stopped flag that will start a deceleration phase
            }
        }

        long currentFrame = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
        if (currentFrame != frameNumber) {
            frameNumber = currentFrame;
                    printFrameNumber(true);
        }

      if (frameNumber != prevFrameNumber) {
        printShootingStatus();
      }

      prevFrameNumber = frameNumber;  


        stepper.run();
    }

    stepper.stop();  // After running the stepper, stop.

    // if the motor was stopped prematurely, do the homing in the same direction
    if (wasStopped) {
        if (prevCommandDirection == 1) {
            homeStepper("b");
        } else if (prevCommandDirection == -1) {
            homeStepper("f");
        }
    }

    motorSpeed = previousSpeed; // restore to the original speed
    stepper.setAcceleration(previousAccel);  // restore to the original acceleration

    Serial.println("Jog To Frame done.");
    isShooting = false;
    shootingStr = "idle";
    //EEPROM.write(eepromAddress, frameNumber);
    EEPROM.put(eepromAddress, -frameNumber); // used EEPROM.put instead of EEPROM.write
}

void setFrame(int newFrame) {
    // Convert frame number to stepper position and invert the value for stepper direction
    long stepperPos = newFrame * stepsPerFullRotations * stepperDirection;
    stepperPos = stepperPos; // invert stepperPosition
    stepper.setCurrentPosition(-stepperPos);
    
    lastStepperPos = newFrame; // Update lastStepperPos to the reversed value of the new frame number

    Serial.print("FRAME = ");
    Serial.println(newFrame); // Print the new frame number
}


void BulbExposureMode(String command) {
    lastBulbExposureCommand = command; // Update the last command
    digitalWrite(CAPPING_SHUTTER_PIN, LOW); 
    motorSpeed = 2000;
    stepperAccel = 2000;
    stepper.setMaxSpeed(motorSpeed);
    stepper.setAcceleration(stepperAccel);
    long startPos = stepper.currentPosition();
    int direction = command.startsWith("bfo") ? -1 : 1;

    long extraSteps = 25; // Define the extra steps to fully open camera rotating shutter 
    if (command.startsWith("bbo")) {
        extraSteps = -extraSteps; // Reverse the extra steps for backward motion if rotating shutter
    }
    long halfRotSteps = stepsPerFullRotations / 2 + extraSteps;

    // Initialize the timer.
    startMillis = millis();
    bulbModeActive = true;

    while(abs(stepper.currentPosition() - startPos) < halfRotSteps) {
        isShooting = true;
        shootingStr = "inProgress";
        stepper.setSpeed(motorSpeed * direction);
        stepper.runSpeed();
    }

    frameNumber = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;

    if(frameNumber != prevFrameNumber) {
        printShootingStatus();
    }
    
    prevFrameNumber = frameNumber;
}

void BulbExposureCloseMode() {
    long startPos = stepper.currentPosition();
    long extraSteps = 25; // Define the extra steps to fully open camera rotating shutter 

    // Assume the direction is stored or can be inferred here
    // For example, if the last mode was 'bbo', set extraSteps to negative
    if (lastBulbExposureCommand.startsWith("bbo")) {
        extraSteps = -extraSteps;
    }

    long totalRotSteps = stepsPerFullRotations - (stepsPerFullRotations / 2 + extraSteps);

    while(abs(stepper.currentPosition() - startPos) < totalRotSteps) {
        stepper.runSpeed();
    }
    
    // Close operations
    bulbModeActive = false;

    frameNumber = round((float)stepper.currentPosition() / (float)stepsPerFullRotations) * stepperDirection;
   
    Serial.println("Bulb operation complete");
    digitalWrite(CAPPING_SHUTTER_PIN, HIGH); 
    isShooting = false;
    shootingStr = "idle";

    // Store frame numbers and reset for next operation
    EEPROM.put(eepromAddress, -frameNumber);
    Serial.println("Saved frame number after bc command");
    Serial.print("Elapsed Time: ");
    Serial.println((millis() - startMillis) / 1000);  // Print final elapsed time in seconds.
    
    // Return motor speed to regular speed.
    motorSpeed = 1000;
    stepperAccel = 6000;

    startMillis = 0;
}

void handleInterrupt() {
  // Empty interrupt routine - actual handling is done in loop
}

void focus_logic() {
    // Update the state of focus control buttons
    focusBackwardButton.update();
    focusForwardButton.update();

    int focusMoveDirection = 0; // 0 = Stop, -1 = Backward, 1 = Forward

    // Determine direction from physical buttons
    if (focusBackwardButton.read() == LOW) {
        focusMoveDirection = -1;
    } else if (focusForwardButton.read() == LOW) {
        focusMoveDirection = 1;
    }

    // Override direction if there's a serial command for focus control
    if (serialFocusDirection != 0) {
        focusMoveDirection = serialFocusDirection;
    }

    // Control the stepper based on the resolved direction
    if (focusMoveDirection == 1) {
        focusStepper.setSpeed(focusStepperSpeed); // Move forward
    } else if (focusMoveDirection == -1) {
        focusStepper.setSpeed(-focusStepperSpeed); // Move backward
    } else {
        focusStepper.setSpeed(0); // Stop
    }

    // Continuously run the focus stepper at the set speed
    focusStepper.runSpeed();
}
