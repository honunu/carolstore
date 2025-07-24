#include <ESP32Servo.h>
#include <FastLED.h>
#define LED_PIN 5
#define NUM_LEDS 8
#define BRIGHTNESS 150

CRGB leds[NUM_LEDS];
Servo myServo;  // 创建舵机对象

// 舵机引脚定义
const int servoPin = 7;

const CRGB WARM_WHITE = CRGB(255, 180, 130);
const CRGB COOL_WHITE = CRGB(220, 255, 255);
// 舵机角度参数
const int MIN_ANGLE = 0;    // 最小角度
const int MAX_ANGLE = 270;  // 最大角度

// 运动参数
const int MOVE_DELAY = 1000;  // 移动时间1秒
const int STEP_DELAY = 20;    // 每步延迟(ms)

// 当前舵机角度
int currentAngle = 135;  // 初始中间位置

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  Serial.println("270度舵机控制程序已启动");
  Serial.println("请输入角度值(0-270)或命令:");
  Serial.println("  loop - 开始自动循环");
  Serial.println("  stop - 停止自动循环");
  
  // 初始化舵机
  myServo.attach(servoPin);
  
  // 初始位置设为中间角度
  myServo.write(currentAngle);

  // 初始化LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  delay(1000); // 等待舵机初始化

}

// 缓慢移动到指定角度
void slowMove(int targetAngle) {
  // 限制角度范围
  targetAngle = constrain(targetAngle, MIN_ANGLE, MAX_ANGLE);
  
  // 计算步数和步长
  int steps = abs(targetAngle - currentAngle);
  if (steps == 0) return; // 已在目标位置
  
  int stepSize = (targetAngle > currentAngle) ? 1 : -1;
  
  // 计算每步延迟时间
  int stepDelay = MOVE_DELAY / steps;
  if (stepDelay < 1) stepDelay = 1; // 确保最小延迟
  
  // 缓慢移动到目标角度
  for (int angle = currentAngle; angle != targetAngle; angle += stepSize) {
    myServo.write(angle);
    delay(stepDelay);
  }
  myServo.write(targetAngle); // 确保到达最终位置
  
  // 更新当前角度
  currentAngle = targetAngle;
  
  // 串口反馈
  Serial.print("已移动到: ");
  Serial.print(currentAngle);
  Serial.println("°");
}

// 自动循环模式
void autoLoop() {
  Serial.println("开始自动循环模式");
  
  while (true) {
    // 检查串口是否有新命令
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      
      if (command.equalsIgnoreCase("stop")) {
        Serial.println("停止自动循环");
        return;
      }
    }
    
    
  }
}

void loop() {
  // 检查串口输入
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // 处理命令
    if (input.equalsIgnoreCase("loop")) {
      autoLoop();
      Serial.println("请输入角度值(0-270)或命令:");
    } 
    else if (input.equalsIgnoreCase("stop")) {
      Serial.println("自动循环未运行");
    }
    // 尝试解析为角度值
    else {
      int angle = input.toInt();
      if (angle >= MIN_ANGLE && angle <= MAX_ANGLE) {
        slowMove(angle);
      } else {
        Serial.print("无效输入: ");
        Serial.println(input);
        Serial.println("请输入0-270之间的角度值");
      }
    }
  }
  
  // 添加短暂延迟减少CPU负载
  delay(100);


  for (int i = 0; i < 4; i++) {
      leds[i] = WARM_WHITE;
      leds[i].fadeLightBy(255 * (1 - 0.5));
  }
  FastLED.show();


}