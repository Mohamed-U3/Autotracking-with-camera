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


#define X_MAX_ACCELERATION  10000

float azimuth_degree_per_pixel = 0.0;
float elevation_degree_per_pixel = 0.0;

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


signed long xValue = 0;
signed long yValue = 0;
signed long zValue = 0;
long X_stepFreq =0;
long Y_stepFreq =0;

bool x_direction =0;
bool y_direction =0;

long currPos1 = 0;
long currPos2 = 0;
int Xmax_speed =0 ;
int Ymax_speed =0 ;

bool parsePositionString(String str);


void SerialRxTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    if (Serial1.available() > 0)
    {
      /* start recieving until find the end of string */
      String receivedString = Serial1.readStringUntil('\n');
      receivedString.trim();
     // xValue = (long)receivedString[0] ;
      bool validData = parsePositionString(receivedString);
     //  Serial1.print(x_direction);
    //  Serial1.print(X_stepFreq);

    //   Serial1.print(y_direction);
     //   Serial1.println(Y_stepFreq);
      Serial1.read();

      // azimuth_FOV = 61.9 - ((zValue/7.0f) * (6.0f/7.0f) * 60);
      // azimuth_degree_per_pixel = azimuth_FOV / IMAGE_WIDTH;

      // elevation_FOV = 37.2 - ((zValue/7.0f) * (36.1f/7.0f) * 36.1);
      // elevation_degree_per_pixel = elevation_FOV / IMAGE_HEIGHT;

      // Xmax_speed = map(zValue, 0, 7, 20000, 1000);
      // Ymax_speed = map(zValue, 0, 7, 20000, 500);

    }
    //vTaskDelay(pdMS_TO_TICKS(33));
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
  }
}

void SerialTxTask(void *pvParameters)
{
  for (;;)
  {
    Serial1.print("X Position: ");
   // Serial1.print(xValue);
    Serial1.print(" ; Y Position: ");
  //  Serial1.println(yValue);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}


void StepperX_Task(void *pvParameters)
{
   
    /* direction , step frequency , acceleration , maximum acceleration */
    long double acceleration =0; 
    uint32_t stepFreq =0;
    float max_delta_frequency = 0;
    max_delta_frequency = X_MAX_ACCELERATION * DT;
    int32_t delta_frequency = 0;
    int32_t ramped_frequency =0;

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




    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign
    digitalWrite(DIR1_PIN, x_direction > 0 ? HIGH : LOW);
     
    // Convert PID output to step frequency

    stepFreq = (int32_t)X_stepFreq;

    delta_frequency = stepFreq-ramped_frequency;

    if(delta_frequency > max_delta_frequency)
    {
      delta_frequency = max_delta_frequency;
      ramped_frequency += delta_frequency;
    }
     else if(delta_frequency < -max_delta_frequency)
    {
      delta_frequency = -max_delta_frequency;
      ramped_frequency += delta_frequency;
    }
    else if(delta_frequency < max_delta_frequency)
    {
      ramped_frequency += delta_frequency;
    }
    

    Serial1.println(ramped_frequency);
    if (ramped_frequency > 50) {  // Minimum speed threshold
      stepXTimer->setOverflow(ramped_frequency, HERTZ_FORMAT);
      stepXTimer->resume();  // Start/continue stepping
    } else if(ramped_frequency < 50  ) {
      stepXTimer->pause();  // Stop if speed too low
      digitalWrite(STEP1_PIN, LOW);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Keep your 10ms PID rate
  }
  }



/************************************************************************/

void StepperY_Task(void *pvParameters)
{
    
    /* direction , step frequency , acceleration , maximum acceleration */
    long double acceleration =0; 
    uint32_t stepFreq =0;
    float max_delta_frequency = 0;
    max_delta_frequency = X_MAX_ACCELERATION * DT;
    int32_t delta_frequency = 0;
    int32_t ramped_frequency =0;

  for (;;)
  {
    //************************************* Pressed State *****************************************//
    while(digitalRead(BTN1_PIN) == LOW || digitalRead(BTN2_PIN) == LOW || digitalRead(BTN3_PIN) == LOW || digitalRead(BTN4_PIN) == LOW ||digitalRead(BTN5_PIN) == LOW)
    {
      //nothing
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
    }
    // NEW: Control hardware timer instead of stepper library
    // Set direction based on sign

   
    digitalWrite(DIR2_PIN, y_direction > 0 ? HIGH : LOW);
    
    // Convert PID output to step frequency

    stepFreq = (int32_t)Y_stepFreq;

    delta_frequency = stepFreq-ramped_frequency;

    if(delta_frequency > max_delta_frequency)
    {
      delta_frequency = max_delta_frequency;
      ramped_frequency += delta_frequency;

    }
    else if(delta_frequency < -max_delta_frequency)
    {
      delta_frequency = -max_delta_frequency;
      ramped_frequency += delta_frequency;

    }
    else if(delta_frequency < max_delta_frequency)
    {
      ramped_frequency += delta_frequency;
    }
        //Serial1.print("X=");
    if (ramped_frequency > 50) {  // Minimum speed threshold
      stepYTimer->setOverflow(ramped_frequency, HERTZ_FORMAT);
      stepYTimer->resume();  // Start/continue stepping
    } else if(ramped_frequency < 50  ) {
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


bool parsePositionString(String str)
{
    // Remove spaces and newlines
    str.trim();

    // Split by commas
    int firstComma  = str.indexOf(',');
    int secondComma = str.indexOf(',', firstComma + 1);
    int thirdComma  = str.indexOf(',', secondComma + 1);

    // Must have exactly 3 commas
    if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
        return false;
    }

    // Extract each field
    String sx_dir = str.substring(0, firstComma);
    String sXfreq = str.substring(firstComma + 1, secondComma);
    String sy_dir = str.substring(secondComma + 1, thirdComma);
    String sYfreq = str.substring(thirdComma + 1);

    // Trim each one
    sx_dir.trim();
    sXfreq.trim();
    sy_dir.trim();
    sYfreq.trim();

    // Convert
    x_direction = (bool)(sx_dir.toInt());
    X_stepFreq  = sXfreq.toInt();
    y_direction = (bool)(sy_dir.toInt());
    Y_stepFreq  = sYfreq.toInt();

    return true;
}