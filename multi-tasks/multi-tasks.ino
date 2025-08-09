#include <ESP32Servo.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <WiFi.h>
#include <WebServer.h>
#include "LEDState.h" // 包含分离的头文件


// 硬件配置
#define ROOF_SERVO_PIN 7      // 舵机1信号线
#define LEFT_SERVO_PIN 4      // 舵机2信号线
#define RIGHT_SERVO_PIN 6      // 舵机3信号线
#define LED_PIN 5         // WS2812数据线
#define LED_COUNT 8       // LED数量

// 舵机参数
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 270
#define SERVO_INITIAL_ANGLE 160
#define SERVO_MOVE_SPEED 50  // 毫秒/度

// 舵机参数
#define DOOR_SERVO_MIN_ANGLE 0
#define DOOR_SERVO_MAX_ANGLE 180
#define LEFT_DOOR_SERVO_INITIAL_ANGLE 20
#define RIGHT_DOOR_SERVO_INITIAL_ANGLE 140

// LED参数
#define LED_BRIGHTNESS 200
WebServer server(80);
// 舵机对象数组
Servo servos[3];
CRGB leds[LED_COUNT];

// 任务句柄
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// 消息队列
QueueHandle_t servoAngleQueue;
QueueHandle_t ledModeQueue;
QueueHandle_t touchEventQueue = NULL;

// 舵机数据结构
struct ServoCommand {
  uint8_t servo_id;  // 舵机ID (0,1,2)
  int angle;         // 目标角度
};
struct LedCommand {
  LedMode mode;
  uint16_t duration; // 渐变时间
};
// 全局状态管理器
LEDState ledState;

// 硬件配置
const uint8_t TOUCH_PIN = 1;  // GPIO4连接TTP223输出
#define TOUCH_ACTIVE HIGH     // 根据模块设置：触摸时高电平或低电平

// 事件定义
enum TouchEvent { NONE, SINGLE_TAP, DOUBLE_TAP };

// 状态机状态
enum TouchState { 
  IDLE, 
  FIRST_PRESS_DETECTED, 
  WAITING_RELEASE, 
  WAITING_SECOND_TAP 
};

enum RoofState {
  ROOF_CLOSED,
  ROOF_OPENED,
  ROOF_OPENING,
  ROOF_CLOSING,
  ROOF_IDLE
};

// 全局变量
volatile RoofState roofState = ROOF_IDLE;

// 时间参数 (单位: 毫秒)
const uint32_t DEBOUNCE_DELAY = 20;    // TTP223需要更短的消抖时间
const uint32_t DOUBLE_TAP_INTERVAL = 350; // 双击间隔
const uint32_t PRESS_TIMEOUT = 1000;     // 长按超时

// 全局变量
volatile TouchState touchState = IDLE;
volatile uint32_t lastTouchEventTime = 0;

volatile bool servoPowerUp[3] = {true, true, true};


// 触摸检测任务
void touchDetectionTask(void *pvParameters) {
  pinMode(TOUCH_PIN, INPUT);
  
  while (1) {
    const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool isTouched = (digitalRead(TOUCH_PIN) == TOUCH_ACTIVE);

    switch (touchState) {
      case IDLE:
        if (isTouched) {
          touchState = FIRST_PRESS_DETECTED;
          lastTouchEventTime = currentTime;
        }
        break;

      case FIRST_PRESS_DETECTED:
        // 消抖处理
        if (currentTime - lastTouchEventTime > DEBOUNCE_DELAY) {
          if (isTouched) {
            touchState = WAITING_RELEASE;
          } else {
            touchState = IDLE; // 误触检测
          }
        }
        break;

      case WAITING_RELEASE:
        if (!isTouched) {
          touchState = WAITING_SECOND_TAP;
          lastTouchEventTime = currentTime;
        } 
        // 长按超时处理
        else if (currentTime - lastTouchEventTime > PRESS_TIMEOUT) {
          TouchEvent event = SINGLE_TAP;
          xQueueSend(touchEventQueue, &event, 0);
          touchState = IDLE;
        }
        break;

      case WAITING_SECOND_TAP:
        if (isTouched && (currentTime - lastTouchEventTime > DEBOUNCE_DELAY)) {
          TouchEvent event = DOUBLE_TAP;
          xQueueSend(touchEventQueue, &event, 0);
          touchState = IDLE;
        } 
        // 双击超时处理
        else if (currentTime - lastTouchEventTime > DOUBLE_TAP_INTERVAL) {
          TouchEvent event = SINGLE_TAP;
          xQueueSend(touchEventQueue, &event, 0);
          touchState = IDLE;
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 更快的检测周期
  }
}

// 事件处理任务
void touchEventHandlerTask(void *pvParameters) {
  TouchEvent event;
  while (1) {
    if (xQueueReceive(touchEventQueue, &event, portMAX_DELAY)) {
      switch (event) {
        case SINGLE_TAP:

        // 处理单击事件 - 切换屋顶状态
          Serial.println("单击事件触发 - 切换屋顶舵机");
          
          ServoCommand cmd;
          cmd.servo_id = 0; // 屋顶舵机ID

          if (roofState == ROOF_CLOSED) {
            cmd.angle = 90; // 关闭的情况按键打开
          } else if (roofState == ROOF_IDLE) {
            cmd.angle = 90; // 静止的情况按键打开
          } else if (roofState == ROOF_OPENING) {
            cmd.angle = 160; // 正在打开按键关闭
            Serial.println("正在打开 现在关闭");
          }else if (roofState == ROOF_CLOSING) {
            cmd.angle = 90; // 正在打开按键关闭
          }else if (roofState == ROOF_OPENED) {
            cmd.angle = 160; // 已经打开了按键关闭
          } else {
            cmd.angle = 160;
            Serial.println("状态未知 现在关闭");
          }


          
          if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100))) {
            Serial.printf("发送屋顶切换命令: %d度\n", cmd.angle);
          } else {
            Serial.println("错误: 舵机命令队列已满!");
          }
          // 处理单击事件
          break;
          
        case DOUBLE_TAP:
          // 处理双击事件
          Serial.println("双击事件触发");
          break;
      }
    }
  }
}


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
       if (i == 0) { // 屋顶舵机
          if (currentAngles[i] < targetAngles[i]) {
            roofState = ROOF_CLOSING;
          } else {
            roofState = ROOF_OPENING;
          }
        } 
        
        
        // 其他舵机正常更新
        if (currentAngles[i] < targetAngles[i]) {
          currentAngles[i]++;
        } else {
          currentAngles[i]--;
        }

        if(servoPowerUp[0]==false){
          if (servos[0].attach(ROOF_SERVO_PIN, 500, 2500) == -1) {
            Serial.printf("舵机%d附加失败! 系统挂起\n", i+1);
            while(1) delay(1000);
          }else{
            servoPowerUp[0]=true;
          }
          Serial.println("重新挂载roof电机");
        }

        servos[i].write(currentAngles[i]);
        
      }else{
        if (i!=0){
          continue;
        }

        if(servoPowerUp[0]==true){
            Serial.println("roof到达位置 停止供电");
            servos[0].detach();
            servoPowerUp[0]=false;
        }
      }

      

      // 检查是否到达目标位置
      if (i == 0 && currentAngles[i] == targetAngles[i]) {
        if (targetAngles[i] == 90) {
          roofState = ROOF_OPENED;
        } else if (targetAngles[i] == 160) {
          roofState = ROOF_CLOSED;
        }
      }
    }

    
    
    // 非阻塞延迟
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_SPEED));
    
    // 监控堆栈使用
    static int stackCheckCounter = 0;
    if (++stackCheckCounter >= 2000) {
      stackCheckCounter = 0;
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.printf("舵机任务堆栈高水位线: %d\n", uxHighWaterMark);
    }
  }
}

// LED任务实现
void ledTask(void *pvParameters) {
  Serial.println("LED任务启动");
  
  // 初始堆栈高水位线
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("LED任务初始堆栈高水位线: %d\n", uxHighWaterMark);
  
  while (1) {
    // 检查是否有新的LED模式命令
    if (uxQueueMessagesWaiting(ledModeQueue)) {
      // 定义队列消息结构体
      struct LedCommand {
        LedMode mode;
        uint16_t duration; // 渐变时间
      };
      
      LedCommand command;
      if (xQueueReceive(ledModeQueue, &command, 0) == pdPASS) {
        Serial.printf("切换到模式: %d, 渐变时间: %dms\n", command.mode, command.duration);
        ledState.setTargetMode(command.mode, command.duration);
      }
    }
    
    // 更新状态（处理渐变）
    ledState.update();
    
    // 获取当前颜色并应用到所有LED
    CRGB color = ledState.getCurrentColor();
    fill_solid(leds, LED_COUNT, color);
    
    // 更新LED条
    FastLED.show();
    
    // 控制更新速率 (30FPS 获得平滑渐变)
    vTaskDelay(pdMS_TO_TICKS(33));
    
    // 监控堆栈使用
    static int stackCheckCounter = 0;
    if (++stackCheckCounter >= 200) { // 每6.6秒检查一次
      stackCheckCounter = 0;
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.printf("LED任务堆栈高水位线: %d\n", uxHighWaterMark);
      
      // 调试信息：显示当前渐变状态
      if (ledState.getTransitionProgress() < 1.0f) {
        Serial.printf("渐变进度: %.1f%%, 剩余时间: %dms\n", 
                      ledState.getTransitionProgress() * 100,
                      ledState.getRemainingTransitionTime());
      }
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
  delay(100); // 等待串口稳定
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
    // servos[i].write(SERVO_INITIAL_ANGLE);
    // Serial.printf("舵机%d初始位置: %d度\n", i+1, SERVO_INITIAL_ANGLE);
  }
  
  // 创建消息队列 - 使用适当大小
  servoAngleQueue = xQueueCreate(3, sizeof(ServoCommand));
  ledModeQueue = xQueueCreate(3, sizeof(LedCommand));
  touchEventQueue = xQueueCreate(10, sizeof(TouchEvent));
  
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

  // 创建触摸检测任务
  xTaskCreate(
    touchDetectionTask,
    "TouchDetect",
    2048,
    NULL,
    3,  // 较高优先级
    NULL
  );

  // 创建事件处理任务
  xTaskCreate(
    touchEventHandlerTask,
    "TouchEventHandler",
    2048,
    NULL,
    1,  // 较低优先级
    NULL
  );
  
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

  init_server();
}

void init_server(){

  const char* ssid = "";
  const char* password = "";
  // 连接WiFi
  WiFi.begin(ssid, password);
  Serial.print("正在连接到WiFi ");
  Serial.println(ssid);
  
  //TODO 修改成失败在task中重试
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi已连接!");
  Serial.print("IP地址: ");
  Serial.println(WiFi.localIP());
  
  
  // 设置Web服务器路由
  server.on("/", handleRoot);
  server.on("/light/off", handleLightOff);
  server.on("/light/warm", handleLightWarm);
  server.on("/light/cold", handleLightCold);
  server.on("/light/yellow", handleLightYellow);
  server.on("/light/alarm", handleLightAlarm);
  server.on("/roof/open", handleRoofOpen);
  server.on("/roof/close", handleRoofClose);
  server.on("/door/left/open", handleLeftDoorOpen);
  server.on("/door/left/close", handleLeftDoorClose);
  server.on("/door/right/open", handleRightDoorOpen);
  server.on("/door/right/close", handleRightDoorClose);
  
  
  // server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("服务器已启动");
}

void handleLightOff(){
  LedCommand command;
  command.mode = LED_OFF;
  command.duration = 500; // 0.5秒渐变
  
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }
}

void handleLightWarm(){
  LedCommand command;
  command.mode = WARM_LIGHT;
  command.duration = 3000; // 3秒渐变
  
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }
}
void handleLightCold(){
  LedCommand command;
  command.mode = COLD_LIGHT;
  command.duration = 2000; // 使用默认时间
  
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }
}
void handleLightYellow(){
  LedCommand command;
  command.mode = WARM_YELLOW_LIGHT;
  command.duration = 2000; // 3秒渐变
  
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }
}


void handleLightAlarm(){
  LedCommand command;
  command.mode = ALARM_RED_LIGHT;
  command.duration = 1000; // 3秒渐变
  
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }

  command.mode = LED_OFF;
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }

  command.mode = ALARM_RED_LIGHT;
  if (xQueueSend(ledModeQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("错误: LED命令队列已满!");
  }

}


void handleRoofOpen() {
  ServoCommand cmd;
  cmd.servo_id = 0;
  cmd.angle = 90;

  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("房顶打开");
      server.send(200, "text/plain; charset=UTF-8", "房顶打开");
    }
}

void handleRoofClose() {
  ServoCommand cmd;
  cmd.servo_id = 0;
  cmd.angle = 160;

  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("房顶关闭");
      server.send(200, "text/plain; charset=UTF-8", "房顶关闭");
    }
}



void handleRoot(){

}

void handleLeftDoorOpen(){
  ServoCommand cmd;
  cmd.servo_id = 1;
  cmd.angle = 80;

  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("左侧门打开");
      server.send(200, "text/plain; charset=UTF-8", "左侧门打开");
    }
}
void handleRightDoorOpen(){
  ServoCommand cmd;
  cmd.servo_id = 2;
  cmd.angle = 70;

  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("右侧门打开");
      server.send(200, "text/plain; charset=UTF-8", "右侧门打开");
    }
}
void handleLeftDoorClose() {
  ServoCommand cmd;
  cmd.servo_id = 1;
  cmd.angle = 30;

  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("左侧门关闭");
      server.send(200, "text/plain; charset=UTF-8", "左侧门关闭");
    }
}

void handleRightDoorClose() {
  ServoCommand cmd;
  cmd.servo_id = 2;
  cmd.angle = 120;
  
  if (xQueueSend(servoAngleQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS){
              Serial.println("错误: 舵机命令队列已满!");
    } else {
      Serial.printf("右侧门关闭");
      server.send(200, "text/plain; charset=UTF-8", "右侧门关闭");
    }
}
void loop() {
  server.handleClient();
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

