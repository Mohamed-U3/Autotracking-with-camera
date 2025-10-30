#include <Arduino.h>
#include <AccelStepper.h>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>

// Timer instance
HardwareTimer *stepXTimer = new HardwareTimer(TIM2);
HardwareTimer *stepYTimer = new HardwareTimer(TIM3);

#define DT 0.01
//*********** Mechanism Physical Parameters ****************//

/* 80 worm gear teath - 1:0.75 motor to worm ratio */
#define ELEVATTION_STEP_PER_DEGREE  40    
#define ELEVATION_DEGREE_PER_PIXEL  0.034    //   37/1080
#define IMAGE_HEIGHT                1080

#define IMAGE_WIDTH                 1920
#define AZIMUTH_STEP_PER_DEGREE     55.5
#define AZIMUTH_DEGREE_PER_PIXEL    0.031    


#define MOTOR_INTERFACE_TYPE 1 // 1 = driver (EN, STEP, DIR)

//**************** Motor 1 Pin Definitions *****************//
#define EN1_PIN         PA0   // Enable1 pin
#define DIR1_PIN        PA1   // Direction1 pin
#define STEP1_PIN       PA2   // Step1 pin

//**************** Motor 2 Pin Definitions *****************//
#define EN2_PIN         PA3   // Enable2 pin
#define DIR2_PIN        PA4   // Direction2 pin
#define STEP2_PIN       PA5   // Step2 pin

//*************  IR Sensor Pin Definitions ****************//
#define IR1_PIN         PA6   // IR Sensor 1 pin
#define IR2_PIN         PA7   // IR Sensor 2 pin
#define IR3_PIN         PB10  // IR Sensor 3 pin

//****************  Button Pin Definitions *****************//
#define BTN1_PIN         PB11  // Button 1 pin
#define BTN2_PIN         PB12  // Button 2 pin
#define BTN3_PIN         PB13  // Button 3 pin
#define BTN4_PIN         PB14  // Button 4 pin
#define BTN5_PIN         PB15  // Button 4 pin

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, STEP1_PIN, DIR1_PIN);  //80000 means 180 degree rotation and 160000 means full rotation
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, STEP2_PIN, DIR2_PIN);

//recieved positions values.
signed long xValue = 0;
signed long yValue = 0;
long currPos1 = 0;
long currPos2 = 0;

bool parsePositionString(String str, long &x, long &y);

void SerialRxTask(void *pvParameters)
{
  for (;;)
  {
    if (Serial1.available() > 0)
    {
      String receivedString = Serial1.readStringUntil('\n');
      receivedString.trim();
     // xValue = (long)receivedString[0] ;
      bool validData = parsePositionString(receivedString, xValue, yValue);
      
      // int receivedString[2] = {0};
      // for(int i=0; i<2; i++)
      // {
      //   receivedString[i] = Serial1.read();
      // // Parse and set xValue, yValue, etc.
      // }
      //xValue = (long)receivedString[0] ;
      //yValue = (long)receivedString[2] ;
      Serial1.read();
    }
    vTaskDelay(pdMS_TO_TICKS(33));
  }
}

void SerialTxTask(void *pvParameters)
{
  for (;;)
  {
    Serial1.print("X Position: ");
    Serial1.print(xValue);
    Serial1.print(" ; Y Position: ");
    Serial1.println(yValue);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}


void StepperX_Task(void *pvParameters)
{
    // PID parameters
    //const long double Kp =  0.2;
    const long double Kp =  0.75;
    const long double Ki = 0.1;
    const long double Kd = 0.1;

    long double pid_integral = 0;
    long double pid_last_error = 0;
    // const long double MAX_INTEGRAL = 100.0; // Tune this value          ////////////////////
    
    // PID control loop
    long double error = 0; // xValue is the error from center
    
    // pid_integral = constrain(pid_integral, -MAX_INTEGRAL, MAX_INTEGRAL); /////////////////////
    long double derivative = 0;
    long double output =0;
    uint32_t stepFreq =0;
  for (;;)
  {
    //************************************* Pressed State *****************************************//
    while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW || digitalRead(BTN5_PIN) == LOW)
    {
      //nothing
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
    }
    // const long double MAX_INTEGRAL = 100.0; // Tune this value               ///////////////////

    // PID control loop
    error = xValue*AZIMUTH_DEGREE_PER_PIXEL*AZIMUTH_STEP_PER_DEGREE; // yValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
    //output = constrain(output, -360, 360);
    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign
    digitalWrite(DIR1_PIN, output > 0 ? HIGH : LOW);
    
    // Convert PID output to step frequency
    stepFreq = abs((int32_t)output * AZIMUTH_STEP_PER_DEGREE);
    stepFreq = constrain(stepFreq, 50, 20000);
    Serial1.println(stepFreq);
    if (stepFreq > 100) {  // Minimum speed threshold
      stepXTimer->setOverflow(stepFreq, HERTZ_FORMAT);
      stepXTimer->resume();  // Start/continue stepping
    } else {
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);

    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Keep your 10ms PID rate
  }

}


// void StepperX_Task(void *pvParameters)
// {
//     // BETTER PID parameters for stepper motors
//     const long double Kp = 1.5;   // Reduced from 0.75
//     const long double Ki = 0.01;  // Small integral for steady-state
//     const long double Kd = 0.1;   // Reduced derivative gain
    
//     long double pid_integral = 0;
//     long double pid_last_error = 0;
//     const long double MAX_INTEGRAL = 50.0;  // Prevent windup
    
//     // ADD: Input filtering
//     long double filtered_error = 0;
//     const long double FILTER_ALPHA = 0.6;  // Low-pass filter
    
//     // ADD: Command rate limiter
//     long double max_output_change = 10000.0;  // Max steps/sec change per iteration
//     long double last_output = 0;
//     long double output=0;
//     long double raw_error = 0;
//     long double output_change = 0;
//     long double derivative =0;
//     uint32_t stepFreq =0;
    
//     for (;;)
//     {
//         // ... existing button handling code ...
//         //     //************************************* Pressed State *****************************************//
//     while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW || digitalRead(BTN5_PIN) == LOW)
//     {
//       //nothing
//       vTaskDelay(pdMS_TO_TICKS(100));
//       digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
//     }
//         // Raw error from Python (in steps)
//         raw_error = xValue * AZIMUTH_DEGREE_PER_PIXEL * AZIMUTH_STEP_PER_DEGREE;
        
//         // LOW-PASS FILTER: Smooth noisy input
//         filtered_error = FILTER_ALPHA * raw_error + (1 - FILTER_ALPHA) * filtered_error;
        
//         // PID calculation with filtered error
//         derivative = (filtered_error - pid_last_error) / DT;
//         pid_integral += filtered_error * DT;
        
//         // Anti-windup: Clamp integral
//         pid_integral = constrain(pid_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        
//         output = Kp * filtered_error + Ki * pid_integral + Kd * derivative;
        
//         // RATE LIMITER: Prevent sudden speed changes
//         output_change = output - last_output;
//         output_change = constrain(output_change, -max_output_change, max_output_change);
//         output = last_output + output_change;
        
//         pid_last_error = filtered_error;
//         last_output = output;
        
//         // Set direction
//         digitalWrite(DIR1_PIN, output > 0 ? HIGH : LOW);
        
//         // Convert to frequency with better scaling
//         stepFreq = abs((int32_t)output*AZIMUTH_STEP_PER_DEGREE);
//         stepFreq = constrain(stepFreq, 100, 20000);  // Wider range, safer max
//         Serial1.println(stepFreq);
//         // ADD: Minimum threshold to prevent micro-movements
//         if (stepFreq > 100) {  // Only move if significant error
//             stepXTimer->setOverflow(stepFreq, HERTZ_FORMAT);
//             stepXTimer->resume();
//         } else {
//             stepXTimer->pause();
//             digitalWrite(STEP1_PIN, LOW);
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }
/************************************************************************/

void StepperY_Task(void *pvParameters)
{
    // PID parameters
    // const long double Kp = 30.0;
    // const long double Ki = 50.7;
    // const long double Kd = 0.756;
    const long double Kp = 0.75;
    const long double Ki =0;
    const long double Kd =0.1;

    long double pid_integral = 0;
    long double pid_last_error = 0;
    // const long double MAX_INTEGRAL = 100.0; // Tune this value          ////////////////////
    
    // PID control loop
    long double error = 0; // xValue is the error from center
    
    // pid_integral = constrain(pid_integral, -MAX_INTEGRAL, MAX_INTEGRAL); /////////////////////
    long double derivative = 0;
    long double output =0;
    uint32_t stepFreq = 0;
  for (;;)
  {
    //************************************* Pressed State *****************************************//
    while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW ||digitalRead(BTN5_PIN) == LOW)
    {
      //nothing
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
    }
    error = yValue*ELEVATION_DEGREE_PER_PIXEL*ELEVATTION_STEP_PER_DEGREE; // yValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
   // output = constrain(output, -360, 360);

    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign
    digitalWrite(DIR2_PIN, output > 0 ? HIGH : LOW);
    
    // Convert PID output to step frequency
    stepFreq = abs((int32_t)output*ELEVATTION_STEP_PER_DEGREE);
    stepFreq = constrain(stepFreq, 50, 20000);
    Serial1.println(stepFreq);
    if (stepFreq > 50) {  // Minimum speed threshold
      stepYTimer->setOverflow(stepFreq, HERTZ_FORMAT);
      stepYTimer->resume();  // Start/continue stepping
    } else {
      stepYTimer->pause();  // Stop if speed too low
      digitalWrite(STEP2_PIN, LOW);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Keep your 10ms PID rate
  }
}



//*******************************************************************************//
//**************************** Buttons Task *************************************//
//*******************************************************************************//
void ButtonsTask(void *pvParameters)
{
  for (;;)
  {
    static bool wasReleased1 = true; // To track button release state
    static bool wasReleased2 = true; // To track button release state
    
    //************************************* Released State ***************************************//
    while (digitalRead(BTN1_PIN) == HIGH && digitalRead(BTN2_PIN) == HIGH && digitalRead(BTN3_PIN) == HIGH && digitalRead(BTN4_PIN) == HIGH && digitalRead(BTN5_PIN) == HIGH)
    {
      if(wasReleased1)
      {
        // stepper1.stop();
        // stepper1.setCurrentPosition(0);
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);
        wasReleased1 = false;
      }
      if(wasReleased2)
      {
        // stepper2.stop();
        // stepper2.setCurrentPosition(0);
      stepYTimer->pause();  // Stop if speed too low
      digitalWrite(STEP2_PIN, LOW);
        wasReleased2 = false;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    //************************************* Pressed State *****************************************//
    //************************************* Motor 1 Control ***************************************//
    //************************************* X Axis Control  ***************************************//
    static bool wasRight = true;
    if (digitalRead(BTN3_PIN) == LOW)
    {
      while (digitalRead(BTN3_PIN) == LOW)
      {
        if(wasRight == false)
        {
          stepper1.setCurrentPosition(0);
          wasRight = true;
        }
        wasReleased1 = true;
        // stepper1.moveTo(1000000); // Move continuously
        // stepper1.run(); 
         digitalWrite(DIR1_PIN,1);
      stepXTimer->setOverflow(15000, HERTZ_FORMAT);
      stepXTimer->resume();  // Start/continue stepping
      }
      // currentPos1 = stepper1.currentPosition();
    }
    else if (digitalRead(BTN5_PIN) == LOW)
    {
      while (digitalRead(BTN5_PIN) == LOW)
      {
        if(wasRight == true)
        {
          stepper1.setCurrentPosition(0);
          wasRight = false;
        }
        wasReleased1 = true;
      digitalWrite(DIR1_PIN,0);
      stepXTimer->setOverflow(15000, HERTZ_FORMAT);
      stepXTimer->resume();
      }
      // currentPos1 = stepper1.currentPosition();
    }
    else
    {
      // stepper1.stop();
      // stepper1.setCurrentPosition(0);
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);
      xValue = 0;
    }
  
    //************************************* Pressed State *****************************************//
    //************************************* Motor 2 Control ***************************************//
    //************************************* Y Axis Control  ***************************************//
    if (digitalRead(BTN1_PIN) == LOW)
    {
      stepper2.setCurrentPosition(0);
      while (digitalRead(IR2_PIN) == LOW && digitalRead(BTN1_PIN) == LOW)
      {
        wasReleased2 = true;
        // stepper2.moveTo(-1000000); // Move continuously
        // stepper2.run();
      digitalWrite(DIR2_PIN,0);
      stepYTimer->setOverflow(15000, HERTZ_FORMAT);
      stepYTimer->resume();
      }
    }
    else if (digitalRead(BTN2_PIN) == LOW)
    {
      stepper2.setCurrentPosition(0);
      while (digitalRead(IR1_PIN) == LOW && digitalRead(BTN2_PIN) == LOW)
      {
        wasReleased2 = true;
        // stepper2.moveTo(1000000); // Move continuously
        // stepper2.run();
      digitalWrite(DIR2_PIN,1);
      stepYTimer->setOverflow(15000, HERTZ_FORMAT);
      stepYTimer->resume();
      }
    }
    else
    {
      // stepper2.stop();
      // stepper2.setCurrentPosition(0);
      stepYTimer->pause();  // Stop if speed too low
      digitalWrite(STEP2_PIN, LOW);
      yValue = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Timer ISR - keep it SHORT and FAST
void stepXTimerISR() {
  // Toggle step pin to generate pulse
  digitalWrite(STEP1_PIN, !digitalRead(STEP1_PIN));
  
  // Alternative (faster): direct register access
  // GPIOA->ODR ^= (1 << 0);  // Toggle PA0
}

// Timer ISR - keep it SHORT and FAST
void stepYTimerISR() {
  // Toggle step pin to generate pulse
  digitalWrite(STEP2_PIN, !digitalRead(STEP2_PIN));
  
  // Alternative (faster): direct register access
  // GPIOA->ODR ^= (1 << 0);  // Toggle PA0
}

//*****************************************************************************//
//**************************** Setup Function **********************************//
//*****************************************************************************//
void setup()
{
  // Remap Serial1 to use PB6 (TX) and PB7 (RX)
  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(9600);

  /************************************* */
    // Timer configuration with specific numbers
  stepXTimer->setPrescaleFactor(8);           // 8MHz / 8 = 1MHz timer clock
  stepXTimer->setOverflow(1000, HERTZ_FORMAT); // 1kHz step frequency (1000 steps/sec)

  stepYTimer->setPrescaleFactor(8);           // 8MHz / 8 = 1MHz timer clock
  stepYTimer->setOverflow(1000, HERTZ_FORMAT); // 1kHz step frequency (1000 steps/sec)
  
  // Set interrupt priority HIGHER than FreeRTOS (important!)
  stepXTimer->setInterruptPriority(3, 0);     // Preempt=3, Sub=0 (higher than RTOS tick at 15)

  stepYTimer->setInterruptPriority(3, 0);     // Preempt=3, Sub=0 (higher than RTOS tick at 15)
  
  // Attach ISR callback
  stepXTimer->attachInterrupt(stepXTimerISR);
  stepYTimer->attachInterrupt(stepYTimerISR);
  
  // Start timer
  stepXTimer->resume();
  stepYTimer->resume();

  //**********Pins Initialization**********//
  pinMode(LED_BUILTIN , OUTPUT  );
  pinMode(IR1_PIN     , INPUT   );
  pinMode(IR2_PIN     , INPUT   );
  pinMode(IR3_PIN     , INPUT   );
  pinMode(BTN1_PIN    , INPUT_PULLUP );
  pinMode(BTN2_PIN    , INPUT_PULLUP );
  pinMode(BTN3_PIN    , INPUT_PULLUP );
  pinMode(BTN4_PIN    , INPUT_PULLUP );
  pinMode(BTN5_PIN    , INPUT_PULLUP );
  pinMode(EN1_PIN     , OUTPUT  );   
  pinMode(EN2_PIN     , OUTPUT  );
  digitalWrite(EN1_PIN, HIGH); // Enable motor driver
  digitalWrite(EN2_PIN, HIGH); // Enable motor driver

 // stepper1.setMaxSpeed(100 * 10000);
 // stepper1.setAcceleration(1 * 10000);

  //stepper2.setMaxSpeed(100 * 10000);
 // stepper2.setAcceleration(1 * 10000);

  Serial1.println("Stepper motor control started");
  xTaskCreate(SerialRxTask, "SerialRxTask", 512, NULL, 1, NULL);
  xTaskCreate(StepperX_Task, "StepperX_Task", 512, NULL, 1, NULL);
  xTaskCreate(StepperY_Task, "StepperY_Task", 512, NULL, 1, NULL);
  xTaskCreate(ButtonsTask, "Buttons", 128, NULL, 4, NULL);
  vTaskStartScheduler();
}


//*****************************************************************************//
//****************************** Main Loop ************************************//
//*****************************************************************************//
void loop()
{
}

//*****************************************************************************//
//********************* Function to parse position string ********************//
//*****************************************************************************//

bool parsePositionString(String str, long &x, long &y)
{
  // // Find the position of "X:" and "Y:"
  // int xIndex = str[0];
  // int yIndex = str[7];
  // int dashIndex = str[5];
  int xIndex = str.indexOf("X:");
  int yIndex = str.indexOf("Y:");
  int dashIndex = str.indexOf('-');

  // Check if all required markers are present
  if (xIndex == -1 || yIndex == -1 || dashIndex == -1) {
    return false;
  }

  // Extract X value substring (between "X:" and "-")
  String xString = str.substring(xIndex + 2);
  xString.trim();

  // Extract Y value substring (after "Y:")
  String yString = str.substring(yIndex + 2);
  //int yString = str[10];
  yString.trim();

  // Convert strings to integers
  x = xString.toInt();
  // y = ((int)str[10]);
  y = yString.toInt();


  return true;
}


/*
bool parsePositionString(String str, long &x, long &y) {

int spaceIndex = str.indexOf(' ');
String first = str.substring(0, spaceIndex);
//String second = str.substring(spaceIndex + 1);

first.trim();
//second.trim();

x = first.toInt();
y = (int)str[2];

// Serial.print("X = "); Serial.println(x);
// Serial.print("Y = "); Serial.println(y);
return true;
}
*/