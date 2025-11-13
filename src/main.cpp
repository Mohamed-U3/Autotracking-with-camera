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
#define DAY_IMAGE_HEIGHT                1080

#define DAY_IMAGE_WIDTH                 1920
#define AZIMUTH_STEP_PER_DEGREE     55.5
#define AZIMUTH_DEGREE_PER_PIXEL    0.031    // FOV / img_width

/********************************** */
float day_azimuth_degree_per_pixel = 0.0;
float day_elevation_degree_per_pixel = 0.0;

float day_azimuth_FOV = 0.0;
float day_elevation_FOV = 0.0;
/********************************** */
float thermal_azimuth_degree_per_pixel = 0.0;
float thermal_elevation_degree_per_pixel = 0.0;

float thermal_azimuth_FOV = 0.0;
float thermal_elevation_FOV = 0.0;
/********************************** */
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
signed long xValue = 0;                                  // Error distance in x-direction in pixels recieved from the high level
signed long yValue = 0;                                  // Error distance in y-direction in pixels recieved from the high level
signed long zValue = 0;                                  // Zoom level recieved from the high level
int  cam =0;                                             // Camera used (0 = Day Cam in use , 1 = Thermal Cam in use)

int  Xmax_speed =0 ;                                     // maximum speed limit per axis based on level of zoom recieved (speed decreases on zooming in & vice versa)
int  Ymax_speed =0 ;

bool parsePositionString(String str, long &x, long &y, long &z,int &camera);

void SerialRxTask(void *pvParameters)
{
  for (;;)
  {
    if (Serial1.available() > 0)
    {
      /*
      Recieve format is:
      "X: x - Y: y - Z: z"
      new format after adding thermal cam.:
      "X: x - Y: y - Z: z - C: c"          (C = used camera 1 or 0)
      */

      /* start recieving until find the end of string */
      String receivedString = Serial1.readStringUntil('\n');
      receivedString.trim();
      
     /* read out the numeric content from the recieved string  */
      bool validData = parsePositionString(receivedString, xValue, yValue, zValue,cam);
      
      Serial1.read();

      /* D A Y - C A M E R A */
      if(cam == 0)
      {

/* FUNCTIONS TO CALCULATE RE-SCLAE AND MAP RECIEVED ZOOM LEVEL TO UPDATE THE FIELD OF VIEW*/

      /* Day camera has FOV range from 61.9 to 1.9 degrees in horizontal */  
        day_azimuth_FOV = 61.9 - ((zValue/7.0f) * (60.0f/7.0f) * 60);
        day_azimuth_degree_per_pixel = day_azimuth_FOV / DAY_IMAGE_WIDTH;

      /* Day camera has FOV range from 37.2 to 1.1 degrees in vertical */ 
        day_elevation_FOV = 37.2 - ((zValue/7.0f) * (36.1f/70.0f) * 36.1);
        day_elevation_degree_per_pixel = day_elevation_FOV / DAY_IMAGE_HEIGHT;
      }
      else
      {
      /* Thermal camera has FOV range from 14.6 to 2.4 degrees in horizontal */ 
        thermal_azimuth_FOV = 14.6 - ((zValue/7.0f) * (12.2f/7.0f) * 12.2);
        thermal_azimuth_degree_per_pixel = thermal_azimuth_FOV / DAY_IMAGE_WIDTH;

        /* Thermal camera has FOV range from 11.7 to 2 degrees in vertical */ 
        thermal_elevation_FOV = 11.7 - ((zValue/7.0f) * (9.7f/70.0f) * 9.7);
        thermal_elevation_degree_per_pixel = thermal_elevation_FOV / DAY_IMAGE_HEIGHT;
      }

      /* Remap the maximum speed in both axes depending on the current zoom level */
      Xmax_speed = map(zValue, 0, 7, 20000, 300);
      Ymax_speed = map(zValue, 0, 7, 20000, 200);

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
    // const long double Kp =  1.6;
    // const long double Ki = 0.08;
    // const long double Kd = 0.04;

    long double Kp =  0;     //0.75
    long double Ki = 0.0;      //0.02
    long double Kd = 0.0;       //0.1

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
    if(xValue < 10 && xValue > -10)
    {
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);
    }
    else
    {
      if(cam ==0)
      {
        Kp=1.6;
        Ki=0.08;
        Kd=0.04;

    // Day PID control loop
    error = xValue*day_azimuth_degree_per_pixel*AZIMUTH_STEP_PER_DEGREE; // xValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
    //output = constrain(output, -360, 360);
    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign
      }
      else
      {
        Kp=0;
        Ki=0;
        Kd=0;

    // Thermal PID control loop
    error = xValue*thermal_azimuth_degree_per_pixel*AZIMUTH_STEP_PER_DEGREE; // xValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
    //output = constrain(output, -360, 360);
    // NEW: Control hardware timer instead of stepper library
  
      }
        // Set direction based on sign
    digitalWrite(DIR1_PIN, output > 0 ? HIGH : LOW);
    
    // Convert PID output to step frequency
    stepFreq = fabs(output * AZIMUTH_STEP_PER_DEGREE);
    stepFreq = constrain(stepFreq, 0, Xmax_speed);
    stepFreq = (int32_t)stepFreq;
    //Serial1.print("X=");
    Serial1.println(stepFreq);
    if (stepFreq > 50) {  // Minimum speed threshold
      stepXTimer->setOverflow(stepFreq, HERTZ_FORMAT);
      stepXTimer->resume();  // Start/continue stepping
    } else if(stepFreq < 50  ) {
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);

    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Keep your 10ms PID rate
  }
  }

}

/************************************************************************/

void StepperY_Task(void *pvParameters)
{
    // PID parameters
    // const long double Kp = 1.6;     //0.75
    // const long double Ki =0;           //0
    // const long double Kd =0.1;         //0.1

    long double Kp = 0;                //1;     //0.75
    long double Ki =0;           //0.01
    long double Kd =0;         //0.1

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
        if(yValue == 0)
    {
      stepYTimer->pause();  // Stop if speed too low
      digitalWrite(STEP2_PIN, LOW);
    }
    else{

      if(cam==0)
      {
        Kp=0.75;
        Kd=0.1;
        Ki=0.1;

    error = yValue*day_elevation_degree_per_pixel*ELEVATTION_STEP_PER_DEGREE; // yValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
      }
      else
      {
        Kp=0;
        Kd=0;
        Ki=0;

    error = yValue*thermal_elevation_degree_per_pixel*ELEVATTION_STEP_PER_DEGREE; // yValue is the error from center
    derivative = (error - pid_last_error)/DT; 
    pid_integral += error*DT;
    output =  Kp * error + Ki * pid_integral + Kd * derivative;
    pid_last_error = error;
      }
   // output = constrain(output, -360, 360);

    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign
    digitalWrite(DIR2_PIN, output > 0 ? HIGH : LOW);
    
    // Convert PID output to step frequency
    stepFreq = fabs(output*ELEVATTION_STEP_PER_DEGREE);
    stepFreq = constrain(stepFreq, 0, Ymax_speed);
    stepFreq = (int32_t) stepFreq;
   // Serial1.println(stepFreq);
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

  stepper1.setMaxSpeed(100 * 10000);
  stepper1.setAcceleration(1 * 10000);

  stepper2.setMaxSpeed(100 * 10000);
  stepper2.setAcceleration(1 * 10000);

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

bool parsePositionString(String str, long &x, long &y, long &z, int &camera)
{
  // Find markers for X, Y, and Z
  int xIndex = str.indexOf("X:");
  int yIndex = str.indexOf("Y:");
  int zIndex = str.indexOf("Z:");
  int cam    = str.indexOf("C:");
  // Validate all markers exist
  if (xIndex == -1 || yIndex == -1 || zIndex == -1) {
    return false;
  }

  // Extract substrings between markers
  String xString = str.substring(xIndex + 2, yIndex);  // from after "X:" to before "Y:"
  String yString = str.substring(yIndex + 2, zIndex);  // from after "Y:" to before "Z:"
  String zString = str.substring(zIndex + 2, cam);     // from after "Z:" to before "C:"
  String camString     = str.substring(zIndex + 2);              // from after "C:" to end
  // Clean spaces
  xString.trim();
  yString.trim();
  zString.trim();

  // Convert to integers
  x = xString.toInt();
  y = yString.toInt();
  z = zString.toInt();
  camera = camString.toInt();
  return true;
}