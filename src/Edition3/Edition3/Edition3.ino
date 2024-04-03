#include <Arduino.h>
#include <esp32-hal-timer.h>

// Task1
const int digitalSignalPin = 21; // Function1 Change this to the correct pin
// 硬件定时器
// hw_timer_t * timer = NULL;

// // 时序阶段和定时器的下一步延时
// enum SequenceStep { HIGH_SHORT, LOW_SHORT, HIGH_LONG, LOW_LONG };
// SequenceStep nextStep = HIGH_SHORT;

// void IRAM_ATTR onTimer() {
//     static unsigned long nextDelay = 0;

//     switch (nextStep) {
//         case HIGH_SHORT:
//             GPIO.out_w1ts = (1 << digitalSignalPin); // 设置GPIO为高电平
//             nextDelay = 180; // 下一阶段延时180μs
//             nextStep = LOW_SHORT;
//             break;
//         case LOW_SHORT:
//             GPIO.out_w1tc = (1 << digitalSignalPin); // 设置GPIO为低电平
//             nextDelay = 40; // 下一阶段延时40μs
//             nextStep = HIGH_LONG;
//             break;
//         case HIGH_LONG:
//             GPIO.out_w1ts = (1 << digitalSignalPin); // 设置GPIO为高电平
//             nextDelay = 530; // 下一阶段延时530μs
//             nextStep = LOW_LONG;
//             break;
//         case LOW_LONG:
//             GPIO.out_w1tc = (1 << digitalSignalPin); // 设置GPIO为低电平
//             nextDelay = 3250; // 下一阶段延时3.25ms
//             nextStep = HIGH_SHORT;
//             break;
//     }

//     // 重新设置硬件定时器的闹钟以触发下一步
//     timerAlarmWrite(timer, nextDelay, false);
//     timerAlarmEnable(timer);
// }

// Task2
const int squareWavePin = 33; // 方波输入信号的GPIO引脚
// 全局变量，用于存储上升沿时间戳
volatile unsigned long firstEdgeTime = 0;
volatile unsigned long firstEdgeFlag = 0;
volatile unsigned long secondEdgeTime = 0;
volatile unsigned long secondEdgeFlag = 0;
volatile unsigned long hasCalculated = 0;
// 用于存储频率的全局变量
volatile int frequency = 0; 
// Task2中断服务例程
void IRAM_ATTR onSquareWaveRisingEdge() {
    if (firstEdgeFlag == 0) { // 如果是第一个上升沿
        firstEdgeTime = micros();
        firstEdgeFlag = 1;
        hasCalculated++;
    }
    else if(firstEdgeFlag == 1 && secondEdgeFlag == 0){ // 如果是第二个上升沿
        secondEdgeTime = micros();
        secondEdgeFlag = 1;
        hasCalculated++;
    }
}


//Task3
const int squareWavePin2 = 34; // 第二个方波输入信号的GPIO引脚
// 用于跟踪是否捕获到第一个和第二个上升沿的标志
volatile unsigned int firstEdgeFlag2 = 0;
volatile unsigned int secondEdgeFlag2 = 0;
// 用于存储第二个方波信号上升沿时间戳的变量
volatile unsigned long firstEdgeTime2 = 0;
volatile unsigned long secondEdgeTime2 = 0;
volatile unsigned long hasCalculated2 = 0;
// 用于存储频率的全局变量
volatile int frequency2 = 0; 
// Task3中断服务例程
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


// 保护上述变量的互斥锁
SemaphoreHandle_t freqSemaphore;


// Task4
const int analogInputPin = 26; // 假定模拟输入连接到GPIO 26（VP）
const int ledPin = 15; // 使用GPIO 2作为指示LED的引脚
float readings[10]; // 存储最后10次读数的数组
int readIndex = 0; // 当前读数的索引
float total = 0; // 最后10次读数的总和
float average = 0; // 存储平均值

// Task7-1
const int buttonPin = 4; // 按键连接的GPIO引脚，根据实际情况调整
// Task7-2
const int ledPin_Button = 2; // LED连接的GPIO引脚，根据实际情况调整

// Task8

void CPU_work(int time) { 
    volatile long endTime = millis() + time; // 使用volatile防止编译器优化
    while(millis() < endTime) {
        // 空循环，仅消耗时间
    }
}


// Global structure and semaphore for frequency storage
typedef struct {
  int freqTask2;
  int freqTask3;
} FrequencyData;
FrequencyData frequencyData;
SemaphoreHandle_t frequencyDataMutex;

// Event queue handle for button press events
// QueueHandle_t buttonPressEventQueue;
QueueHandle_t buttonPressEventQueue = NULL; // 事件队列句柄


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
  Serial.begin(9600);
  // pinMode setup for Task1
  pinMode(digitalSignalPin, OUTPUT);
  // pinMode setup for Task2
  pinMode(squareWavePin, INPUT_PULLDOWN);
  // pinMode setup for Task3
  pinMode(squareWavePin2, INPUT_PULLDOWN);

  // pinMode setup for Task4
  pinMode(analogInputPin, INPUT);
  pinMode(ledPin, OUTPUT);

// pinMode setup for Task7-1
  pinMode(buttonPin, INPUT_PULLUP); // 假设按键按下时接地

// pinMode setup for Task7-2
  pinMode(ledPin_Button, OUTPUT); // 配置LED引脚为输出

  // 初始化读数数组
  for (int thisReading = 0; thisReading < 10; thisReading++) {
        readings[thisReading] = 0;
    }




  // 初始化硬件定时器

  // timer = timerBegin(0, 80, true); // 使用80预分频得到1MHz的定时器频率（即每个定时器tick代表1μs）
  // timerAttachInterrupt(timer, &onTimer, true); // 绑定中断服务程序
  // timerAlarmWrite(timer, 180, false); // 初始化时开始第一步，设置180μs后触发中断
  // timerAlarmEnable(timer); // 启动定时器


  // Create the frequency data mutex
  frequencyDataMutex = xSemaphoreCreateMutex();

  // Create the button press event queue
  // buttonPressEventQueue = xQueueCreate(10, sizeof(bool));
  buttonPressEventQueue = xQueueCreate(10, sizeof(bool)); // 创建包含10个元素的队列
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

// FreeRTOS task implementations


// Task1
void taskDigitalSignalOutput(void *pvParameters) {

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
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
  }
}


// Task2
// 如果采样周期与信号频率不匹配，可能导致边沿计数的不一致性，这反过来会影响频率的测量精度。
// 这个问题归根结底与信号采样的Nyquist定理有关，该定理指出：为了准确无失真地捕捉到一个连续信号的所有信息，采样频率应至少为信号最高频率的两倍。
void taskMeasureFrequency2(void *pvParameters) {
    attachInterrupt(digitalPinToInterrupt(squareWavePin), onSquareWaveRisingEdge, RISING);
    // 在这里出去计算时间的 F1flag==1 F1time=那个time  F2flag==1 F2time=那个time

    // 创建互斥锁
    freqSemaphore = xSemaphoreCreateMutex();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(20);
    unsigned long tempFrequency; // 添加这一行来声明tempFrequency变量
    unsigned long duration; // 用于存储两个上升沿之间的时间差
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);
        hasCalculated = 0;
        if (firstEdgeFlag > 0 && secondEdgeFlag > 0 && hasCalculated<=2) { // 确保已经捕获到两个上升沿
            xSemaphoreTake(freqSemaphore, portMAX_DELAY);

            duration = secondEdgeTime - firstEdgeTime; // 计算时间差
            tempFrequency = 1000000 / duration; // 计算频率，单位是Hz

            // 检查频率是否在333Hz到1000Hz的范围内
            if (tempFrequency >= 333 && tempFrequency <= 1000) {
                frequency = tempFrequency; // 如果在范围内，则更新全局频率变量
            } else {
                frequency = 0; // 如果不在范围内，可以设置为0或者其他标识值
            }
            // 重置时间戳以便下一次测量
            firstEdgeTime = 0;
            secondEdgeTime = 0;
            firstEdgeFlag = 0;
            secondEdgeFlag = 0;
            xSemaphoreGive(freqSemaphore);

            // 打印测量结果
            // Serial.print("Measured Frequency: ");
            // Serial.print(frequency);
            // Serial.println(" Hz");
        } else {
            // 如果在周期内没有捕获到足够的上升沿，可能需要打印一条消息或采取其他行动
            // Serial.println("Not enough data for frequency1 calculation.");
            frequency = 0;
        }
    }
}



// Task3
void taskMeasureFrequency3(void *pvParameters) {

    // 创建互斥锁
    freqSemaphore = xSemaphoreCreateMutex();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(8); // 8ms周期
    unsigned long tempFrequency2; // 添加这一行来声明tempFrequency变量

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);
        hasCalculated2 = 0;

        attachInterrupt(digitalPinToInterrupt(squareWavePin2), onSquareWaveRisingEdge2, RISING);
        if (firstEdgeFlag2 > 0 && secondEdgeFlag2 > 0 && hasCalculated2<=2) {
            xSemaphoreTake(freqSemaphore, portMAX_DELAY);

            unsigned long duration = secondEdgeTime2 - firstEdgeTime2;
            tempFrequency2 = 1000000 / duration; // 计算频率，单位是Hz

            // 检查频率是否在333Hz到1000Hz的范围内
            if (tempFrequency2 >= 500 && tempFrequency2 <= 1000) {
                frequency2 = tempFrequency2; // 如果在范围内，则更新全局频率变量
            } else {
                frequency2 = 0; // 如果不在范围内，可以设置为0或者其他标识值
            }

            // 重置变量以便下一次测量
            firstEdgeTime2 = 0;
            secondEdgeTime2 = 0;
            firstEdgeFlag2 = 0;
            secondEdgeFlag2 = 0;
            xSemaphoreGive(freqSemaphore);

            // 打印测量结果
            // Serial.print("Second Signal Measured Frequency: ");
            // Serial.print(frequency2);
            // Serial.println(" Hz");
        } else {
            // Serial.println  ("Not enough data for second signal frequency2 calculation.");
            frequency2 = 0;
        }
    }
}


// Task4
// potentionmeter 左右引脚接GND或者3.3v，中间引脚接读取analogdata的引脚26 LEDpin15接出来 接LED接电阻 再接回GND
void taskSampleAnalogInput(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyMeasurePeriod = pdMS_TO_TICKS(20);

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequencyMeasurePeriod);

        // 从数组中减去最后的读数
        total = total - readings[readIndex];

        // 读取模拟输入
        readings[readIndex] = analogRead(analogInputPin);

        // 加上最新的读数到总和中
        total = total + readings[readIndex];

        // 前进到下一个位置
        readIndex = readIndex + 1;

        // 如果我们到达数组的末尾，环绕到数组的开始
        if (readIndex >= 10) {
            readIndex = 0;
        }

        // 计算平均值
        average = total / 10;

        // 检查是否需要指示错误（如果平均值大于最大范围的一半）
        if (average > 4095 / 2) { // 假定使用12位ADC，最大值为4095
            digitalWrite(ledPin, HIGH); // 打开LED
        } else {
            digitalWrite(ledPin, LOW); // 关闭LED
        }

        // 可选：使用Serial.print输出平均值进行调试
        // Serial.print("Average Analog Reading: ");
        // Serial.println(average);
    }
}



// Task5
void taskLogInformation(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xLogPeriod = pdMS_TO_TICKS(200); // 200ms周期


    // xLastWakeTime记录了任务开始的初始时间点。
    // xLogPeriod是通过pdMS_TO_TICKS(200)计算出的周期时间，将200ms转换为FreeRTOS的滴答计数，这表示任务的执行频率是每200ms一次。
    // vTaskDelayUntil(&xLastWakeTime, xLogPeriod)是一个阻塞调用，它使任务等待直到下一个200ms周期到来。使用vTaskDelayUntil而不是vTaskDelay可以确保任务以准确的固定周期执行，不受任务执行时间的影响。


    while (1) {
        // 等待下一个日志周期
        vTaskDelayUntil(&xLastWakeTime, xLogPeriod);

        // 获取频率值的副本
        int localFrequency1, localFrequency2;
        xSemaphoreTake(freqSemaphore, portMAX_DELAY);
        localFrequency1 = frequency; // 从Task2测量得到的频率
        localFrequency2 = frequency2; // 从Task3测量得到的频率
        xSemaphoreGive(freqSemaphore);

        // 缩放和限制频率值
        localFrequency1 = max(0, min(99, localFrequency1 / 10)); // 假设缩放因子为10
        localFrequency2 = max(0, min(99, localFrequency2 / 10)); // 假设缩放因子为10

        if(localFrequency1==0){
          Serial.printf("For Signal 1:No signal or frequency out of range\n");
        }
          
        if(localFrequency2==0){
          Serial.printf("For Signal 2:No signal or frequency out of range\n");
        }

        
        // 日志输出到串口
        Serial.printf("LocalFrequency1 and LocalFrequency2 are %d,%d\n", localFrequency1, localFrequency2);
    }
}

// Task7-1
void taskMonitorPushButton(void *pvParameters) {

    int lastButtonState = HIGH; // 假设初始状态为未按下
    int currentButtonState = lastButtonState;
    unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50; // 按键去抖延时，50ms

    while (1) {
        int reading = digitalRead(buttonPin);
        
        // 如果按键状态发生变化，重置去抖计时器
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > debounceDelay) {
            // 如果按键状态在去抖时间后保持不变，则确认状态变化
            if (reading != currentButtonState) {
                currentButtonState = reading;

                // 按键被按下
                if (currentButtonState == LOW) {
                    bool pushButtonPressed = true;
                    // 向队列发送按键按下事件
                    xQueueSend(buttonPressEventQueue, &pushButtonPressed, portMAX_DELAY);
                }
            }
        }

        lastButtonState = reading;
        vTaskDelay(10 / portTICK_PERIOD_MS); // 简单的任务延时以减少CPU占用
    }
}

// Task7-2
void taskControlLED(void *pvParameters) {

    bool receivedSignal;
    while (1) {
        if (xQueueReceive(buttonPressEventQueue, &receivedSignal, portMAX_DELAY) == pdPASS) {
            // 如果从队列接收到按键按下事件，则切换LED状态
            digitalWrite(ledPin_Button, !digitalRead(ledPin_Button));
        }
    }
}

void taskCPULoad(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 设置任务周期为20ms

    while (1) {
        CPU_work(2); // 模拟大约2ms的CPU工作负载

        // 等待直到下一个周期
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}



