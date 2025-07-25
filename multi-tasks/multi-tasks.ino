#include <ESP32Servo.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// 硬件配置
#define ROOF_SERVO_PIN 7      // 舵机1信号线
#define LEFT_SERVO_PIN 4      // 舵机2信号线
#define RIGHT_SERVO_PIN 6      // 舵机3信号线
#define LED_PIN 5         // WS2812数据线
#define LED_COUNT 8       // LED数量

// 舵机参数
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 270
#define SERVO_INITIAL_ANGLE 120
#define SERVO_MOVE_SPEED 50  // 毫秒/度

// 舵机参数
#define DOOR_SERVO_MIN_ANGLE 0
#define DOOR_SERVO_MAX_ANGLE 180
#define LEFT_DOOR_SERVO_INITIAL_ANGLE 20
#define RIGHT_DOOR_SERVO_INITIAL_ANGLE 140

// LED参数
#define LED_BRIGHTNESS 200

// 舵机对象数组
Servo servos[3];
CRGB leds[LED_COUNT];
// LED颜色定义
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

// 舵机数据结构
struct ServoCommand {
  uint8_t servo_id;  // 舵机ID (0,1,2)
  int angle;         // 目标角度
};

// 舵机控制任务 - 控制所有舵机
void servoTask(void *pvParameters) {
  Serial.println("舵机任务启动");
  
  // 舵机当前位置
  int currentAngles[3] = {SERVO_INITIAL_ANGLE, LEFT_DOOR_SERVO_INITIAL_ANGLE, RIGHT_DOOR_SERVO_INITIAL_ANGLE};
  
  // 舵机目标位置
  int targetAngles[3] = {SERVO_INITIAL_ANGLE, SERVO_INITIAL_ANGLE, SERVO_INITIAL_ANGLE};
  
  // 获取初始堆栈高水位线
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("舵机任务初始堆栈高水位线: %d\n", uxHighWaterMark);
  
  while (1) {
    // 检查是否有新的舵机命令
    ServoCommand cmdReceived;
    if (xQueueReceive(servoAngleQueue, &cmdReceived, 0) == pdPASS) {
      if (cmdReceived.servo_id < 3) {
        // 确保目标角度在有效范围内
        Serial.printf("接收到的角度是 %d", cmdReceived.angle);
        targetAngles[cmdReceived.servo_id] = constrain(cmdReceived.angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        Serial.printf("舵机%d新角度: %d度\n", cmdReceived.servo_id, targetAngles[cmdReceived.servo_id]);
      }
    }
    
    // 更新所有舵机位置
    for (int i = 0; i < 3; i++) {
      if (currentAngles[i] != targetAngles[i]) {
        if (currentAngles[i] < targetAngles[i]) {
          currentAngles[i]++;
        } else {
          currentAngles[i]--;
        }
        servos[i].write(currentAngles[i]);
      }
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
  Serial.println("\n\n多舵机控制系统启动中...");
  
  // 打印重启原因
  print_reset_reason();
  
  // 打印芯片信息
  Serial.printf("芯片型号: %s\n", ESP.getChipModel());
  Serial.printf("CPU频率: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("闪存大小: %d KB\n", ESP.getFlashChipSize() / 1024);
  Serial.printf("可用堆内存: %d bytes\n", ESP.getHeapSize());
  Serial.printf("最小可用堆内存: %d bytes\n", ESP.getMinFreeHeap());
  
  // 初始化所有舵机
  const uint8_t servoPins[3] = {ROOF_SERVO_PIN, LEFT_SERVO_PIN, RIGHT_SERVO_PIN};
  for (int i = 0; i < 3; i++) {
    Serial.printf("附加舵机%d到引脚%d\n", i+1, servoPins[i]);
    if (servos[i].attach(servoPins[i], 500, 2500) == -1) {
      Serial.printf("舵机%d附加失败! 系统挂起\n", i+1);
      while(1) delay(1000);
    }
    servos[i].write(SERVO_INITIAL_ANGLE);
    Serial.printf("舵机%d初始位置: %d度\n", i+1, SERVO_INITIAL_ANGLE);
  }
  
  // 创建消息队列 - 使用适当大小
  servoAngleQueue = xQueueCreate(3, sizeof(ServoCommand));
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
  Serial.println("可用命令: ");
  Serial.println("  S<舵机ID><角度> - 设置舵机角度 (如 S0120)");
  Serial.println("  L0 - 关闭LED");
  Serial.println("  L1 - 暖光模式");
  Serial.println("  L2 - 冷光模式");
  Serial.println("  mem - 查看内存使用");
  Serial.println("  reset - 重启系统");
  
  // 禁用看门狗 (ESP32-S2特定)
  disableCore0WDT();
  Serial.println("已禁用核心0看门狗");
}

void loop() {
  // 主循环处理串口命令
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // 舵机控制命令: S<舵机ID><角度>
    if (input.startsWith("S") && input.length() >= 2) {
      uint8_t servoId = input.charAt(1) - '0'; // 转换为数字
      int angle = input.substring(2).toInt();
      
      if (servoId >= 0 && servoId <= 2 && 
          angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
        
        ServoCommand cmd;
        cmd.servo_id = servoId;
        cmd.angle = angle;
        
        if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
        } else {
          Serial.printf("设置舵机%d角度为: %d\n", servoId, angle);
        }
      } else {
        Serial.println("错误: 无效的舵机命令格式");
        Serial.println("格式: S<舵机ID><角度>");
        Serial.println("舵机ID: 0,1,2");
        Serial.println("角度范围: 0-270");
      }
    } 
    // LED模式命令
    else if (input.startsWith("L")) {
      if (input.length() >= 2) {
        int mode = input.charAt(1) - '0';
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
      } else {
        Serial.println("错误: 无效的LED命令格式");
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
    else if (input.equalsIgnoreCase("status")) {
      Serial.println("系统状态报告:");
      Serial.printf("  Free Heap: %d bytes\n", ESP.getFreeHeap());
      Serial.printf("  Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
      Serial.printf("  舵机任务状态: %d\n", eTaskGetState(servoTaskHandle));
      Serial.printf("  LED任务状态: %d\n", eTaskGetState(ledTaskHandle));
    }
    else {
      Serial.println("未知命令");
      Serial.println("可用命令: S<舵机ID><角度>, L<模式>, mem, reset, status");
      Serial.println("例如: S0120 (设置舵机0到120度)");
      Serial.println("      L1 (设置LED为暖光模式)");
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