#include <FastLED.h>
#include <ESP32Servo.h>  // 使用ESP32专用舵机库
#include <WiFi.h>
#include <WebServer.h>


// 创建Web服务器对象，监听端口80
WebServer server(80);
#define LED_PIN 5       // 使用GPIO 5
#define NUM_LEDS 8      // 总共8个LED
#define BRIGHTNESS 150  // 日光灯亮度 (0-255)

CRGB leds[NUM_LEDS];
// WiFi配置
const char* ssid = "";      // 替换为您的WiFi名称
const char* password = "";  // 替换为您的WiFi密码

// 日光灯色温定义
const CRGB WARM_WHITE = CRGB(255, 180, 130); // 2700K暖白色温
const CRGB COOL_WHITE = CRGB(220, 255, 255); // 6500K冷白色温
const int MIN_ANGLE = 0;     // 最小角度
const int MAX_ANGLE = 180;   // 最大角度

// 舵机引脚定义
const int servoPin = 6;       // 右侧舵机信号引脚
const int servoPinLeft = 7;   // 左侧舵机信号引脚

// 舵机对象
Servo rightServo;  // 右侧舵机
Servo leftServo;   // 左侧舵机

// 舵机角度
int rightAngle = 45;  // 右侧舵机当前角度
int leftAngle = 45;   // 左侧舵机当前角度

// 灯光状态
bool lightsOn = false;  // 灯光开关状态

void setup() {
  Serial.begin(115200);
  Serial.println("系统启动 - 准备接收命令");
  Serial.println("命令: R角度, L角度, on, off");

  // 连接WiFi
  WiFi.begin(ssid, password);
  Serial.print("正在连接到WiFi ");
  Serial.println(ssid);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi已连接!");
  Serial.print("IP地址: ");
  Serial.println(WiFi.localIP());
  
  // 设置Web服务器路由
  server.on("/", handleRoot);     // 根路径
  server.on("/on", handleOn);     // 开灯路径
  server.on("/off", handleOff);   // 关灯路径
  server.on("/doorclose", handleDoorClose);   // 关灯路径
  server.on("/dooropen", handleDoorOpen);   // 关灯路径

  server.onNotFound(handleNotFound); // 未找到路径
  // 启动Web服务器
  server.begin();
  Serial.println("HTTP服务器已启动");

  
  // 初始化舵机
  rightServo.attach(servoPin, 500, 2500);
  leftServo.attach(servoPinLeft, 500, 2500);
  
  // 初始位置
  rightServo.write(rightAngle);
  leftServo.write(leftAngle);
  
  // 初始化LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  // 初始状态：所有LED关闭
  FastLED.clear();
  FastLED.show();
}

void loop() {
  server.handleClient();  // 处理客户端请求
  processSerialCommands();
  updateServos();
  updateLights();
}
// Web服务器路由处理函数
void handleRoot() {
  String html = "<html><body style='font-family:Arial;'>"
                "<h1>智能灯光控制系统</h1>"
                "<p>当前灯光状态: <strong>" + String(lightsOn ? "开启" : "关闭") + "</strong></p>"
                "<p><a href='/on'><button style='padding:10px 20px;background:green;color:white;border:none;'>开灯</button></a></p>"
                "<p><a href='/off'><button style='padding:10px 20px;background:red;color:white;border:none;'>关灯</button></a></p>"
                "<p>舵机控制: R90/L45 (通过串口)</p>"
                "</body></html>";
  
  server.send(200, "text/plain; charset=UTF-8", html);
}
void handleOn() {
  lightsOn = true;
  updateLights();
  server.send(200, "text/plain; charset=UTF-8", "灯光已开启");
  Serial.println("通过Web: 灯光已开启");
}

void handleOff() {
  lightsOn = false;
  updateLights();
  server.send(200, "text/plain; charset=UTF-8", "灯光已关闭");
  Serial.println("通过Web: 灯光已关闭");
}


void handleNotFound() {
  server.send(404, "text/plain; charset=UTF-8","404: 页面未找到");
}
void processSerialCommands() {
  if (Serial.available() > 0) {
    // 使用更安全的方式读取串口数据
    String command = Serial.readStringUntil('\n');
    command.trim(); // 移除空格和换行符

    // 检查命令是否为空
    if (command.length() == 0) return;

    // 处理灯光命令
    if (command == "on") {
      lightsOn = true;
      Serial.println("灯光已开启");
      return;
    } else if (command == "off") {
      lightsOn = false;
      Serial.println("灯光已关闭");
      return;
    }

    // 处理舵机命令
    if (command.length() >= 2) {
      char servo = command.charAt(0); // 第一个字符：R或L
      String angleStr = command.substring(1); // 剩余部分是角度值
      
      int angle = angleStr.toInt(); // 转换为整数
      
      // 确保角度在有效范围内
      angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
      
      if (servo == 'R' || servo == 'r') {
        rightAngle = angle;
        Serial.print("右侧舵机设置为: ");
        Serial.println(rightAngle);
      } else if (servo == 'L' || servo == 'l') {
        leftAngle = angle;
        Serial.print("左侧舵机设置为: ");
        Serial.println(leftAngle);
      } else {
        Serial.println("无效命令! 使用 R 或 L 开头");
      }
    } else {
      Serial.println("命令格式错误! 正确格式: R90 或 L45");
    }
  }
}

void updateServos() {
  // 更新舵机位置
  rightServo.write(rightAngle);
  leftServo.write(leftAngle);
}

void updateLights() {
  // 根据灯光状态设置LED
  if (lightsOn) {
    // 设置前4个灯（暖白色温）
    for (int i = 0; i < 4; i++) {
      leds[i] = WARM_WHITE;
    }
    // 设置后4个灯（冷白色温）
    for (int i = 4; i < NUM_LEDS; i++) {
      leds[i] = COOL_WHITE;
    }
  } else {
    // 关闭所有LED
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
  }
  
  FastLED.show();
}

void handleDoorClose(){
  leftDoorClose();
  rightDoorClose();
  updateServos(); // 立即更新舵机位置
  server.send(200, "text/plain; charset=UTF-8", "门已关闭"); // 添加字符集声明
  Serial.println("通过Web: 门已关闭");
}

void handleDoorOpen(){
  leftDoorOpen();
  rightDoorOpen();
  updateServos(); // 立即更新舵机位置
  server.send(200, "text/plain; charset=UTF-8", "门已打开"); // 添加字符集声明
  Serial.println("通过Web: 门已打开");
}
void leftDoorOpen(){
  leftAngle = 0;
}
void leftDoorClose(){
  leftAngle = 50;
}
void rightDoorOpen(){
  rightAngle= 50;
}
void rightDoorClose(){
  rightAngle= 0;
}
void leftLightsOn(){}
void leftLightsOff(){}
