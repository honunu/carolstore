#ifndef LEDSTATE_H
#define LEDSTATE_H

#include <FastLED.h> // 包含 FastLED 库

// 定义灯光模式枚举
enum LedMode {
  LED_OFF,
  WARM_LIGHT,
  COLD_LIGHT,
  BREATHING
};

// 全局颜色常量声明
extern const CRGB WARM_WHITE;
extern const CRGB COOL_WHITE;
extern const CRGB BREATH_COLOR;

class LEDState {
public:
  LEDState();
  
  // 设置目标模式（带渐变时间参数）
  void setTargetMode(LedMode newMode, uint16_t duration = 2000);
  
  // 更新状态（处理渐变）
  void update();
  
  // 获取当前颜色
  CRGB getCurrentColor() const;
  
  // 获取当前实际模式
  LedMode getCurrentMode() const;
  
  // 获取渐变进度 (0.0-1.0)
  float getTransitionProgress() const;
  
  // 获取剩余渐变时间 (毫秒)
  uint16_t getRemainingTransitionTime() const;
  
private:
  // 缓动函数 (二次缓入缓出)
  float easeInOutQuad(float t) const;
  
  LedMode currentMode;       // 当前实际模式
  LedMode targetMode;        // 目标模式
  CRGB currentColor;         // 当前显示的颜色
  CRGB startColor;           // 渐变起始颜色
  CRGB targetColor;          // 目标颜色
  float transitionProgress;  // 渐变进度 (0.0-1.0)
  uint16_t transitionDuration; // 渐变持续时间 (毫秒)
  unsigned long transitionStartTime; // 渐变开始时间
};

#endif // LEDSTATE_H