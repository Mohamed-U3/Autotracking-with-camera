#include <Arduino.h>
#include <AccelStepper.h>
#include <STM32FreeRTOS.h>

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
long xValue = 0;
long yValue = 0;
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
      bool validData = parsePositionString(receivedString, xValue, yValue);
      Serial1.read();
      // Parse and set xValue, yValue, etc.
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
    Serial1.print(" - Y Position: ");
    Serial1.println(yValue);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}


void StepperX_Task(void *pvParameters)
{
  for (;;)
  {
    //************************************* Pressed State *****************************************//
    while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW ||digitalRead(BTN5_PIN) == LOW)
    {
      //nothing
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
    }
    // PID parameters
    // const long double Kp = 50.0;
    // const long double Ki = 33.7;
    // const long double Kd = 0.45;

    const long double Kp = 50.0;
    const long double Ki = 33.7;
    const long double Kd = 0.45;

    long double pid_integral = 0;
    long double pid_last_error = 0;
    // const long double MAX_INTEGRAL = 100.0; // Tune this value               ///////////////////

    // PID control loop
    long double error = xValue; // xValue is the error from center
    pid_integral += error;
    // pid_integral = constrain(pid_integral, -MAX_INTEGRAL, MAX_INTEGRAL);  ///////////////////////// 
    long double derivative = error - pid_last_error;
    long double output = Kp * error + Ki * pid_integral + Kd * derivative;

    // Limit output to reasonable speed range
    output = constrain(output, -10000, 10000);

    // Set stepper speed and direction
    stepper1.setSpeed(output);
    // stepper1.runSpeed();

    pid_last_error = error;

    vTaskDelay(pdMS_TO_TICKS(10)); // Run PID every 10ms
  }
}
void StepperY_Task(void *pvParameters)
{
  for (;;)
  {
    //************************************* Pressed State *****************************************//
    while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW ||digitalRead(BTN5_PIN) == LOW)
    {
      //nothing
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
    }
    // PID parameters
     const long double Kp = 30.0;
     const long double Ki = 50.7;
     const long double Kd = 0.756;



    long double pid_integral = 0;
    long double pid_last_error = 0;
    // const long double MAX_INTEGRAL = 100.0; // Tune this value          ////////////////////
    
    // PID control loop
    long double error = yValue; // xValue is the error from center
    pid_integral += error;
    // pid_integral = constrain(pid_integral, -MAX_INTEGRAL, MAX_INTEGRAL); /////////////////////
    long double derivative = error - pid_last_error;
    long double output = Kp * error + Ki * pid_integral + Kd * derivative;

    // Limit output to reasonable speed range
    output = constrain(output, -10000, 10000);

    // Set stepper speed and direction
    if (digitalRead(IR1_PIN) == HIGH && output < 0) output = 0;
    if (digitalRead(IR2_PIN) == HIGH && output > 0) output = 0;
    if(output > 9000)   output = 9000;
    if(output < -9000)  output = -9000;
    stepper2.setSpeed(-output);
    // stepper2.runSpeed();

    pid_last_error = error;

    vTaskDelay(pdMS_TO_TICKS(10)); // Run PID every 10ms
  }
}

void Stepper_Task(void *pvParameters)
{
  for (;;)
  {
    stepper1.runSpeed();
    // taskYIELD();
    stepper2.runSpeed();
    taskYIELD();

    // vTaskDelay();
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
        stepper1.stop();
        stepper1.setCurrentPosition(0);
        wasReleased1 = false;
      }
      if(wasReleased2)
      {
        stepper2.stop();
        stepper2.setCurrentPosition(0);
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
        stepper1.moveTo(1000000); // Move continuously
        stepper1.run(); 
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
        stepper1.moveTo(-1000000); // Move continuously
        stepper1.run();
      }
      // currentPos1 = stepper1.currentPosition();
    }
    else
    {
      stepper1.stop();
      stepper1.setCurrentPosition(0);
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
        stepper2.moveTo(-1000000); // Move continuously
        stepper2.run();
      }
    }
    else if (digitalRead(BTN2_PIN) == LOW)
    {
      stepper2.setCurrentPosition(0);
      while (digitalRead(IR1_PIN) == LOW && digitalRead(BTN2_PIN) == LOW)
      {
        wasReleased2 = true;
        stepper2.moveTo(1000000); // Move continuously
        stepper2.run();
      }
    }
    else
    {
      stepper2.stop();
      stepper2.setCurrentPosition(0);
      yValue = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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

  stepper1.setMaxSpeed(100 * 10000);
  stepper1.setAcceleration(1 * 10000);

  stepper2.setMaxSpeed(100 * 10000);
  stepper2.setAcceleration(1 * 10000);

  Serial1.println("Stepper motor control started");
  xTaskCreate(SerialRxTask, "SerialRxTask", 512, NULL, 1, NULL);
  // xTaskCreate(SerialTxTask, "SerialTxTask", 256, NULL, 3, NULL);
  xTaskCreate(StepperX_Task, "StepperX_Task", 256, NULL, 1, NULL);
  xTaskCreate(StepperY_Task, "StepperY_Task", 256, NULL, 1, NULL);
  xTaskCreate(Stepper_Task, "Stepper_Task", 1024, NULL, 1, NULL);
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
  // Find the position of "X:" and "Y:"
  long xIndex = str.indexOf("X:");
  long yIndex = str.indexOf("Y:");
  int dashIndex = str.indexOf("-");

  // Check if all required markers are present
  if (xIndex == -1 || yIndex == -1 || dashIndex == -1) {
    return false;
  }

  // Extract X value substring (between "X:" and "-")
  String xString = str.substring(xIndex + 2);
  xString.trim();

  // Extract Y value substring (after "Y:")
  String yString = str.substring(yIndex + 2);
  yString.trim();

  // Convert strings to integers
  x = xString.toInt();
  y = yString.toInt();

  return true;
}
