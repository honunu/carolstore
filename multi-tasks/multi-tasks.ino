#include <ESP32Servo.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// 硬件配置
#define SERVO_PIN 7       // 舵机信号线连接的GPIO
#define LED_PIN 5         // WS2812数据线连接的GPIO
#define LED_COUNT 8       // LED数量

// 舵机参数
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 270
#define SERVO_INITIAL_ANGLE 120
#define SERVO_MOVE_SPEED 100  // 增加延迟时间

// LED参数
#define LED_BRIGHTNESS 200

// 创建舵机对象
Servo myServo;
CRGB leds[LED_COUNT];
// 创建LED条对象
const CRGB WARM_WHITE = CRGB(255, 180, 130);
const CRGB COOL_WHITE = CRGB(220, 255, 255);

// 任务句柄
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// 消息队列
QueueHandle_t servoAngleQueue;
QueueHandle_t ledModeQueue;

// LED模式枚举
enum LedMode {
  LED_OFF,
  WARM_LIGHT,
  COLD_LIGHT
};

// 舵机控制任务
void servoTask(void *pvParameters) {
  int targetAngle = SERVO_INITIAL_ANGLE;
  int currentAngle = SERVO_INITIAL_ANGLE;
  
  Serial.println("舵机任务启动");
  
  // 获取初始堆栈高水位线
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("舵机任务初始堆栈高水位线: %d\n", uxHighWaterMark);
  
  while (1) {
    // 检查是否有新的目标角度
    if (xQueueReceive(servoAngleQueue, &targetAngle, 0) == pdPASS) {
      // 确保目标角度在有效范围内
      targetAngle = constrain(targetAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      Serial.printf("新舵机角度: %d度\n", targetAngle);
    }
    
    // 只有需要移动时才更新舵机
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      myServo.write(currentAngle);
    }
    
    // 非阻塞延迟
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_SPEED));
    
    // 监控堆栈使用
    static int stackCheckCounter = 0;
    if (++stackCheckCounter >= 100) {
      stackCheckCounter = 0;
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.printf("舵机任务堆栈高水位线: %d\n", uxHighWaterMark);
    }
  }
}

// LED控制任务
void ledTask(void *pvParameters) {
  Serial.println("LED任务启动");
  
  LedMode currentMode = WARM_LIGHT; // 默认暖光模式
  
  // 获取初始堆栈高水位线
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("LED任务初始堆栈高水位线: %d\n", uxHighWaterMark);
  
  while (1) {
    // 检查是否有新的LED模式命令
    if (xQueueReceive(ledModeQueue, &currentMode, 0) == pdPASS) {
      Serial.print("LED模式切换到: ");
      switch(currentMode) {
        case LED_OFF: Serial.println("关闭"); break;
        case WARM_LIGHT: Serial.println("暖光"); break;
        case COLD_LIGHT: Serial.println("冷光"); break;
      }
    }
    
    // 根据当前模式设置所有LED
    switch(currentMode) {
      case LED_OFF:
        for (int i = 0; i < LED_COUNT; i++) {
          leds[i] = CRGB::Black;
        }
        break;
        
      case WARM_LIGHT:
        for (int i = 0; i < LED_COUNT; i++) {
          leds[i] = WARM_WHITE;
        }
        break;
        
      case COLD_LIGHT:
        for (int i = 0; i < LED_COUNT; i++) {
          leds[i] = COOL_WHITE;
        }
        break;
    }
    
    // 更新LED条
    FastLED.show();
    
    // 控制更新速率 (10FPS)
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 监控堆栈使用
    static int stackCheckCounter = 0;
    if (++stackCheckCounter >= 50) {
      stackCheckCounter = 0;
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.printf("LED任务堆栈高水位线: %d\n", uxHighWaterMark);
    }
  }
}

// 打印重启原因
void print_reset_reason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("重启原因: ");
  switch(reason) {
    case ESP_RST_POWERON: Serial.println("上电重启"); break;
    case ESP_RST_SW: Serial.println("软件重启"); break;
    case ESP_RST_PANIC: Serial.println("异常/崩溃"); break;
    case ESP_RST_INT_WDT: Serial.println("中断看门狗"); break;
    case ESP_RST_TASK_WDT: Serial.println("任务看门狗"); break;
    case ESP_RST_WDT: Serial.println("其他看门狗"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("深度睡眠唤醒"); break;
    case ESP_RST_BROWNOUT: Serial.println("欠压复位"); break;
    default: Serial.printf("未知原因 (%d)\n", reason); break;
  }
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  delay(1000); // 等待串口稳定
  Serial.println("\n\n系统启动中...");
  
  // 打印重启原因
  print_reset_reason();
  
  // 打印芯片信息
  Serial.printf("芯片型号: %s\n", ESP.getChipModel());
  Serial.printf("CPU频率: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("闪存大小: %d KB\n", ESP.getFlashChipSize() / 1024);
  Serial.printf("可用堆内存: %d bytes\n", ESP.getHeapSize());
  Serial.printf("最小可用堆内存: %d bytes\n", ESP.getMinFreeHeap());
  
  // 初始化舵机 - 只在这里附加一次
  Serial.println("附加舵机到引脚7");
  if (myServo.attach(SERVO_PIN, 500, 2500) == -1) {
    Serial.println("舵机附加失败! 系统挂起");
    while(1) delay(1000);
  }
  
  // 初始位置设为中间角度
  myServo.write(SERVO_INITIAL_ANGLE);
  Serial.printf("舵机初始位置: %d度\n", SERVO_INITIAL_ANGLE);
  
  // 创建消息队列 - 使用适当大小
  servoAngleQueue = xQueueCreate(3, sizeof(int));
  ledModeQueue = xQueueCreate(3, sizeof(LedMode));
  
  // 检查队列是否创建成功
  if (!servoAngleQueue || !ledModeQueue) {
    Serial.println("错误: 无法创建队列! 系统挂起");
    while(1) delay(1000);
  }
  
  // 初始化LED
  Serial.println("初始化FastLED");
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  
  // 根据芯片信息设置堆栈大小
  size_t freeHeap = ESP.getFreeHeap();
  
  // 计算舵机任务堆栈大小 (总内存的10%，但不超过4096)
  size_t servoStackSize = 2048;  // 固定大小2048字节
  
  // 计算LED任务堆栈大小 (总内存的15%，但不超过4096)
  size_t ledStackSize = 3072;    // 固定大小3072字节
  
  Serial.printf("设置舵机任务堆栈: %d bytes\n", servoStackSize);
  Serial.printf("设置LED任务堆栈: %d bytes\n", ledStackSize);
  
  // 创建舵机任务 (动态堆栈大小)
  if (xTaskCreate(
    servoTask,        // 任务函数
    "ServoTask",      // 任务名称
    servoStackSize,   // 动态堆栈大小
    NULL,             // 参数
    1,                // 优先级
    &servoTaskHandle  // 任务句柄
  ) != pdPASS) {
    Serial.println("舵机任务创建失败! 系统挂起");
    while(1) delay(1000);
  }
  
  // 创建LED任务 (动态堆栈大小)
  if (xTaskCreate(
    ledTask,          // 任务函数
    "LEDTask",        // 任务名称
    ledStackSize,     // 动态堆栈大小
    NULL,             // 参数
    1,                // 优先级
    &ledTaskHandle    // 任务句柄
  ) != pdPASS) {
    Serial.println("LED任务创建失败! 系统挂起");
    while(1) delay(1000);
  }
  
  // 初始设置LED为暖光模式
  LedMode initialMode = WARM_LIGHT;
  if (xQueueSend(ledModeQueue, &initialMode, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: 无法设置初始LED模式");
  }
  
  Serial.println("系统初始化完成");
  Serial.println("可用命令: S<角度>, L0, L1, L2");
  Serial.println("例如: S90, L1");
  
  // 禁用看门狗 (ESP32-S2特定)
  disableCore0WDT();
  Serial.println("已禁用核心0看门狗");
}

void loop() {
  // 主循环处理串口命令
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("S")) {
      // 舵机角度命令
      int angle = input.substring(1).toInt();
      if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
        if (xQueueSend(servoAngleQueue, &angle, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
        } else {
          Serial.print("设置舵机角度为: ");
          Serial.println(angle);
        }
      } else {
        Serial.println("错误: 角度必须在0-270之间");
      }
    } 
    else if (input.startsWith("L")) {
      // LED模式命令
      int mode = input.substring(1).toInt();
      if (mode >= 0 && mode <= 2) {
        LedMode ledMode = static_cast<LedMode>(mode);
        if (xQueueSend(ledModeQueue, &ledMode, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: LED命令队列已满!");
        } else {
          Serial.print("设置LED模式为: ");
          switch(ledMode) {
            case LED_OFF: Serial.println("关闭"); break;
            case WARM_LIGHT: Serial.println("暖光"); break;
            case COLD_LIGHT: Serial.println("冷光"); break;
          }
        }
      } else {
        Serial.println("错误: LED模式必须是0-2");
      }
    }
    else if (input.equalsIgnoreCase("mem")) {
      Serial.printf("当前可用内存: %d字节\n", ESP.getFreeHeap());
      Serial.printf("最小可用内存: %d字节\n", ESP.getMinFreeHeap());
      
      // 报告任务状态
      Serial.printf("舵机任务状态: %d\n", eTaskGetState(servoTaskHandle));
      Serial.printf("LED任务状态: %d\n", eTaskGetState(ledTaskHandle));
      
      // 报告堆栈高水位线
      if (servoTaskHandle != NULL) {
        Serial.printf("舵机任务堆栈高水位线: %d\n", uxTaskGetStackHighWaterMark(servoTaskHandle));
      }
      if (ledTaskHandle != NULL) {
        Serial.printf("LED任务堆栈高水位线: %d\n", uxTaskGetStackHighWaterMark(ledTaskHandle));
      }
    }
    else if (input.equalsIgnoreCase("reset")) {
      ESP.restart();
    }
    else if (input.equalsIgnoreCase("reason")) {
      print_reset_reason();
    }
    else if (input.equalsIgnoreCase("info")) {
      Serial.printf("芯片型号: %s\n", ESP.getChipModel());
      Serial.printf("CPU频率: %d MHz\n", ESP.getCpuFreqMHz());
      Serial.printf("闪存大小: %d KB\n", ESP.getFlashChipSize() / 1024);
      Serial.printf("可用堆内存: %d bytes\n", ESP.getHeapSize());
    }
    else {
      Serial.println("未知命令");
      Serial.println("可用命令: S<角度>, L0, L1, L2, mem, reset, reason, info");
      Serial.println("例如: S90, L1");
    }
  }
  
  // 监控系统状态
  static unsigned long lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 10000) {
    lastHeapCheck = millis();
    Serial.printf("系统状态: 内存=%d字节, 舵机状态=%d, LED状态=%d\n", 
                  ESP.getFreeHeap(),
                  eTaskGetState(servoTaskHandle),
                  eTaskGetState(ledTaskHandle));
  }
  
  // 主循环延迟
  delay(100);
}