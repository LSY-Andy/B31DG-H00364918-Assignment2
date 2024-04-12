#include <Arduino.h>


// -----------------------Task1 -----------------------------------------
const int digitalSignalPin = 21; // Function1 Change this to the correct pin

// '''
// Perhaps a hardware timer can better solve Task1 tasks without consuming CPU
// '''

// -----------------------Task2-----------------------------------------
const int squareWavePin = 33; // GPIO pin for square wave input signal

// volatule: Their values need to be re-read from memory each time they are accessed, rather than using cached values.
volatile unsigned long firstEdgeTime = 0;
volatile unsigned long firstEdgeFlag = 0;
volatile unsigned long secondEdgeTime = 0;
volatile unsigned long secondEdgeFlag = 0;
volatile unsigned long hasCalculated = 0;

// A global variable used to store frequency1
// volatile int frequency = 0; 

// Task2 Interrupts the service routine
void IRAM_ATTR onSquareWaveRisingEdge() {
    if (firstEdgeFlag == 0) { 
        firstEdgeTime = micros();
        firstEdgeFlag = 1;
        hasCalculated++;
    }
    else if(firstEdgeFlag == 1 && secondEdgeFlag == 0){ 
        secondEdgeTime = micros();
        secondEdgeFlag = 1;
        hasCalculated++;
    }
}


// -----------------------Task3-----------------------------------------
const int squareWavePin2 = 34; // GPIO pin for square wave input signal
volatile unsigned int firstEdgeFlag2 = 0;
volatile unsigned int secondEdgeFlag2 = 0;

// Store the first and second rising edge time
volatile unsigned long firstEdgeTime2 = 0;
volatile unsigned long secondEdgeTime2 = 0;
volatile unsigned long hasCalculated2 = 0;
// A global variable used to store frequency2
// volatile int frequency2 = 0; 
// Task3 Interrupts the service routine
void IRAM_ATTR onSquareWaveRisingEdge2() {
    if (firstEdgeFlag2 == 0) {
        firstEdgeTime2 = micros();
        firstEdgeFlag2 = 1;
        hasCalculated2++;
    } else if (firstEdgeFlag2 != 0 && secondEdgeFlag2 == 0) {
        secondEdgeTime2 = micros();
        secondEdgeFlag2 = 1;
        hasCalculated2++;
    }
}



// -----------------------Task4-----------------------------------------
const int analogInputPin = 26; 
const int ledPin = 15; 
float readings[10]; 
int readIndex = 0; 
float total = 0; 
float average = 0; 


// -----------------------Task7-1-----------------------------------------
const int buttonPin = 4; 


// -----------------------Task7-2-----------------------------------------
const int ledPin_Button = 2; 


// -----------------------Task8-----------------------------------------
void CPU_work(int time) { 
    volatile long endTime = millis() + time; // use volatile it needs to be re-read from memory each time they are accessed
    while(millis() < endTime) {
        // empty recurrent only use to delay the time
    }
}


// Global structure and semaphore for frequency storage
typedef struct {
  volatile int freqTask2;
  volatile int freqTask3;
} FrequencyData;
FrequencyData frequencyData;
SemaphoreHandle_t freqSemaphore;


// Setup Event queue  to handle for button press events
QueueHandle_t buttonPressEventQueue = NULL; 



// Task function prototypes
void taskDigitalSignalOutput(void *pvParameters);
void taskMeasureFrequency2(void *pvParameters);
void taskMeasureFrequency3(void *pvParameters);
void taskSampleAnalogInput(void *pvParameters);
void taskLogInformation(void *pvParameters);
void taskMonitorPushButton(void *pvParameters);
void taskControlLED(void *pvParameters);
void taskCPULoad(void *pvParameters);


// -----------------------Setup: The Most Important Part in FreeRTOs-----------------------------------------
void setup(void) {
  Serial.begin(9600);

  pinMode(digitalSignalPin, OUTPUT);
  pinMode(squareWavePin, INPUT_PULLDOWN);
  pinMode(squareWavePin2, INPUT_PULLDOWN);
  pinMode(analogInputPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Assume that the key is grounded when pressed
  pinMode(ledPin_Button, OUTPUT); 

  // Initial analog reading array
  for (int thisReading = 0; thisReading < 10; thisReading++) {
        readings[thisReading] = 0;
    }


  // Create the frequency data mutex
  freqSemaphore = xSemaphoreCreateMutex();

  // Create the button press event queue
  // buttonPressEventQueue = xQueueCreate(10, sizeof(bool));
  buttonPressEventQueue = xQueueCreate(10, sizeof(bool)); // create a queue that contain 10 variables
    if (buttonPressEventQueue == NULL) {
        Serial.println("Error creating the queue");
    }


  // Create tasks
  xTaskCreate(taskDigitalSignalOutput, "DigitalOutput", 4096, NULL, 1, NULL); // Task1
  xTaskCreate(taskMeasureFrequency2, "MeasureFreq2", 4096, NULL, 2, NULL);    // Task2
  xTaskCreate(taskMeasureFrequency3, "MeasureFreq3", 4096, NULL, 2, NULL);    // Task3
  xTaskCreate(taskSampleAnalogInput, "AnalogInput", 2048, NULL, 2, NULL);     // Task4
  xTaskCreate(taskLogInformation, "LogInfo", 2048, NULL, 2, NULL);            // Task5
  xTaskCreate(taskMonitorPushButton, "MonitorButton", 2048, NULL, 2, NULL);   // Task7-1
  xTaskCreate(taskControlLED, "ControlLED", 2048, NULL, 2, NULL);             // Task7-2
  xTaskCreate(taskCPULoad, "CPULoad", 1000, NULL, 1, NULL);                   // Task8



}


void loop() {
  // In FreeRTOS, loop() is usually empty.
}


// -------------------- FreeRTOS task implementations ------------------------------

// Task1
void taskDigitalSignalOutput(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Gets the start time of the current task, which is used as a reference point for subsequent periodic execution

  // This task should never return, so it's enclosed in an infinite loop
  for (;;) {
    // Turn the digital signal HIGH for 180μs
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(180);  // Directly delay for microseconds

    // Turn the signal LOW for 40μs
    digitalWrite(digitalSignalPin, LOW);
    delayMicroseconds(40);
    
    // Turn the signal HIGH again for 530μs
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(530);
    
    // Finally, turn the signal LOW for 3.25ms
    digitalWrite(digitalSignalPin, LOW);
    delayMicroseconds(3250);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));// Make the task wait until the next 4ms cycle arrives
  }
}


// Task2
// If the sampling period does not match the signal frequency, it may lead to inconsistencies in edge counts, which in turn will affect the measurement accuracy of the frequency.

void taskMeasureFrequency2(void *pvParameters) {
    attachInterrupt(digitalPinToInterrupt(squareWavePin), onSquareWaveRisingEdge, RISING);

    // create mutex semaphore
    freqSemaphore = xSemaphoreCreateMutex();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(20);// 20ms period

    unsigned long tempFrequency; 
    unsigned long duration; 
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);
        hasCalculated = 0;
        if (firstEdgeFlag > 0 && secondEdgeFlag > 0 && hasCalculated<=2) { 
            xSemaphoreTake(freqSemaphore, portMAX_DELAY);//Gets a mutually exclusive semaphore for secure access to the shared resource

            duration = secondEdgeTime - firstEdgeTime; 
            tempFrequency = 1000000 / duration; // calculate the frequency

            if (tempFrequency >= 333 && tempFrequency <= 1000) {
                frequencyData.freqTask2 = tempFrequency; 
            } 
            else if (tempFrequency < 333 ){
                frequencyData.freqTask2 = 0; 
            }
            else if (tempFrequency > 1000 ){
                frequencyData.freqTask2 = 999; 
            }
            // Reset the timestamp for the next measurement
            firstEdgeTime = 0;
            secondEdgeTime = 0;
            firstEdgeFlag = 0;
            secondEdgeFlag = 0;
            xSemaphoreGive(freqSemaphore);//Release a mutex semaphore to allow other tasks or interrupt access to a shared resource
        } else {
            frequencyData.freqTask2 = 0;
        }
    }
}


// Task3
void taskMeasureFrequency3(void *pvParameters) {

    // create the mutex semaphore
    freqSemaphore = xSemaphoreCreateMutex();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(8); // 8ms period
    unsigned long tempFrequency2; 

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);
        hasCalculated2 = 0;

        attachInterrupt(digitalPinToInterrupt(squareWavePin2), onSquareWaveRisingEdge2, RISING);
        if (firstEdgeFlag2 > 0 && secondEdgeFlag2 > 0 && hasCalculated2<=2) {
            xSemaphoreTake(freqSemaphore, portMAX_DELAY);

            unsigned long duration = secondEdgeTime2 - firstEdgeTime2;
            tempFrequency2 = 1000000 / duration; 

            // Check the frequency is between 500Hz and 1000Hz
            if (tempFrequency2 >= 333 && tempFrequency2 <= 1000) {
                frequencyData.freqTask3 = tempFrequency2; 
            } 
            else if (tempFrequency2 < 333 ){
                frequencyData.freqTask3 = 0; 
            }
            else if (tempFrequency2 > 1000 ){
                frequencyData.freqTask3 = 999; 
            }
            // Reset the timestamp for the next measurement
            firstEdgeTime2 = 0;
            secondEdgeTime2 = 0;
            firstEdgeFlag2 = 0;
            secondEdgeFlag2 = 0;
            xSemaphoreGive(freqSemaphore);

        } else {
            frequencyData.freqTask3 = 0;
        }
    }
}


// Task4
void taskSampleAnalogInput(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(20);

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);

        // Subtract the last reading from the array
        total = total - readings[readIndex];

        // read analog input
        readings[readIndex] = analogRead(analogInputPin);

        // plus the new reading into total counter
        total = total + readings[readIndex];

        // move to next position
        readIndex = readIndex + 1;

        // resetup the array when count to 10
        if (readIndex >= 10) {
            readIndex = 0;
        }

        // calculate the average
        average = total / 10;

        if (average > 4095 / 2) { 
            digitalWrite(ledPin, HIGH); 
        } else {
            digitalWrite(ledPin, LOW); 
        }
    }
}



// Task5
void taskLogInformation(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xLogPeriod = pdMS_TO_TICKS(200); // 200ms

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xLogPeriod);

        int localFrequency1, localFrequency2;

        // Use Mutex semaphore to protect the frequency calculated from task 2 & 3
        xSemaphoreTake(freqSemaphore, portMAX_DELAY);
        localFrequency1 = frequencyData.freqTask2; // The frequency test from Task2
        localFrequency2 = frequencyData.freqTask3; // The frequency test from Task3
        xSemaphoreGive(freqSemaphore);

        // Scale and limit frequency values
        localFrequency1 = (localFrequency1 - 333) * (100 - 0) / (1000 - 333);
        localFrequency1 = max(0, min(99, localFrequency1)); 

        localFrequency2 = (localFrequency2 - 500) * (100 - 0) / (1000 - 500);
        localFrequency2 = max(0, min(99, localFrequency2)); 


        if(localFrequency1==0){
          Serial.printf("Frequency1 equals to 0 or out of range\n");
        }

        if(localFrequency1==99){
          Serial.printf("Frequency1 equals to 99 or out of range\n");
        }
          
        if(localFrequency2==0){
          Serial.printf("Frequency2 equals to 0 or out of range\n");
        }

        if(localFrequency2==99){
          Serial.printf("Frequency2 equals to 99 or out of range\n");
        }


        
        Serial.printf("Frequency1 and Frequency2 are %d,%d\n", localFrequency1, localFrequency2);
    }
}

// Task7-1
volatile unsigned long buttonPressTime = 0; // Time stamp of button press

void taskMonitorPushButton(void *pvParameters) {
    int lastButtonState = HIGH; // Assume that the initial state is not pressed
    int currentButtonState = lastButtonState;
    unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50; // Buffeting delay

    while (1) {
        int reading = digitalRead(buttonPin);
        
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > debounceDelay) {
            if (reading != currentButtonState) {
                currentButtonState = reading;

                if (currentButtonState == LOW) {
                    buttonPressTime = micros(); // Record the timestamp when the button was pressed
                    bool pushButtonPressed = true;
                    xQueueSend(buttonPressEventQueue, &pushButtonPressed, portMAX_DELAY);
                }
            }
        }

        lastButtonState = reading;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// Task7-2
void taskControlLED(void *pvParameters) {
    bool receivedSignal;

    while (1) {
        if (xQueueReceive(buttonPressEventQueue, &receivedSignal, portMAX_DELAY) == pdPASS) {
            digitalWrite(ledPin_Button, !digitalRead(ledPin_Button));
        }
    }
}


// Task8
void taskCPULoad(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 20ms

    while (1) {
        CPU_work(2); 

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}












