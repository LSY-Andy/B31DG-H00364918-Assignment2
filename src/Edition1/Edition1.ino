#include <Arduino.h>

// Task1
const int digitalSignalPin = 21; // Function1 Change this to the correct pin

// Task2
const int squareWavePin = 33; // 方波输入信号的GPIO引脚
volatile int edgeCount = 0;   // 用于ISR计数上升沿或下降沿

//Task3
const int squareWavePin2 = 34; // 第二个方波输入信号的GPIO引脚
volatile int edgeCount2 = 0;   // 用于ISR计数上升沿或下降沿（第二个信号）

// 用于存储频率的全局变量
volatile int frequency = 0; 
volatile int frequency2 = 0; // 存储Task3第二个信号频率的全局变量

// 保护上述变量的互斥锁
SemaphoreHandle_t freqSemaphore;


// Task2中断服务例程
void IRAM_ATTR onSquareWaveRisingEdge() {
  edgeCount++;
}


// Task3中断服务例程
void IRAM_ATTR onSquareWaveRisingEdge2() {
    edgeCount2++;
}




// Global structure and semaphore for frequency storage
typedef struct {
  int freqTask2;
  int freqTask3;
} FrequencyData;
FrequencyData frequencyData;
SemaphoreHandle_t frequencyDataMutex;

// Event queue handle for button press events
QueueHandle_t buttonPressEventQueue;

// Task function prototypes
void taskDigitalSignalOutput(void *pvParameters);
void taskMeasureFrequency2(void *pvParameters);
void taskMeasureFrequency3(void *pvParameters);
void taskSampleAnalogInput(void *pvParameters);
void taskLogInformation(void *pvParameters);
void taskMonitorPushButton(void *pvParameters);
void taskControlLED(void *pvParameters);
void taskCPULoad(void *pvParameters);

void setup(void) {
  Serial.begin(115200);

  // Create the frequency data mutex
  frequencyDataMutex = xSemaphoreCreateMutex();

  // Create the button press event queue
  buttonPressEventQueue = xQueueCreate(10, sizeof(bool));

  // Create tasks
  // xTaskCreate(taskDigitalSignalOutput, "DigitalOutput", 4096, NULL, 2, NULL);
  // xTaskCreate(taskMeasureFrequency2, "MeasureFreq2", 1000, NULL, 2, NULL);
  xTaskCreate(taskMeasureFrequency3, "MeasureFreq3", 1000, NULL, 3, NULL);
  // xTaskCreate(taskSampleAnalogInput, "AnalogInput", 1000, NULL, 2, NULL);
  // xTaskCreate(taskLogInformation, "LogInfo", 1000, NULL, 1, NULL);
  // xTaskCreate(taskMonitorPushButton, "MonitorButton", 1000, NULL, 2, NULL);
  // xTaskCreate(taskControlLED, "ControlLED", 1000, NULL, 2, NULL);
  // xTaskCreate(taskCPULoad, "CPULoad", 1000, NULL, 2, NULL);

  // Other setup code here (GPIO setup, ADC configuration, etc.)
}

void loop() {
  // In FreeRTOS, loop() is usually empty.
}

// FreeRTOS task implementations
// FreeRTOS task implementations
void taskDigitalSignalOutput(void *pvParameters) {
  // Configure the digital pin as an output
  pinMode(digitalSignalPin, OUTPUT);

  TickType_t xLastWakeTime = xTaskGetTickCount();

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

    // Wait until the next cycle which is every 4ms
    // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
  }
}

// 
// 如果采样周期与信号频率不匹配，可能导致边沿计数的不一致性，这反过来会影响频率的测量精度。
// 这个问题归根结底与信号采样的Nyquist定理有关，该定理指出：为了准确无失真地捕捉到一个连续信号的所有信息，采样频率应至少为信号最高频率的两倍。
void taskMeasureFrequency2(void *pvParameters) {
  // 初始化引脚和中断
  pinMode(squareWavePin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(squareWavePin), onSquareWaveRisingEdge, RISING);

  // 创建互斥锁
  freqSemaphore = xSemaphoreCreateMutex();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(20);

  while(1) {
    // 等待下一个测量周期
    vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);

    // 取得互斥锁
    xSemaphoreTake(freqSemaphore, portMAX_DELAY);
    // 计算频率：由于每个周期有两次变化（上升和下降），所以用边沿计数乘以50
    frequency = (edgeCount * 1000) / 20; // edgeCount * (1/20ms) * 1000 to get Hz
    edgeCount = 0; // 重置边沿计数
    // 释放互斥锁
    xSemaphoreGive(freqSemaphore);
    
    // 现在可以使用频率变量做其他事情，例如更新显示或日志记录
    // 使用Serial.print打印频率值到串口监视器
    Serial.print("Measured Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
  }
}

void taskMeasureFrequency3(void *pvParameters) {
  // Frequency measurement logic for task 3
  pinMode(squareWavePin2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(squareWavePin2), onSquareWaveRisingEdge2, RISING);

  // 创建互斥锁
  freqSemaphore = xSemaphoreCreateMutex();
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(8); // 测量周期为8ms

  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);

    xSemaphoreTake(freqSemaphore, portMAX_DELAY); // 假设你复用了同一个互斥锁
    frequency2 = (edgeCount2 * 1000) / 8; // 计算频率
    edgeCount2 = 0; // 重置边沿计数
    xSemaphoreGive(freqSemaphore);
    
    Serial.print("Second Measured Frequency: ");
    Serial.print(frequency2);
    Serial.println(" Hz");
  }
}
// Other task functions here...

