#include "LEDState.h"

// 定义全局颜色常量
const CRGB WARM_WHITE = CRGB(255, 180, 107); // 暖白色 (偏黄)
const CRGB COOL_WHITE = CRGB(200, 200, 255); // 冷白色 (偏蓝)
const CRGB BREATH_COLOR = WARM_WHITE; // 呼吸模式使用暖白色

LEDState::LEDState() : 
  currentMode(LED_OFF), 
  targetMode(LED_OFF), 
  transitionProgress(1.0f), 
  transitionDuration(2000) 
{
  // 初始化为黑色
  currentColor = CRGB::Black;
  startColor = CRGB::Black;
  targetColor = CRGB::Black;
}

void LEDState::setTargetMode(LedMode newMode, uint16_t duration) {
  if (newMode == targetMode) return;
  
  // 设置最小渐变时间 (防止除零错误)
  if (duration < 50) duration = 50;
  
  // 保存当前实际颜色作为渐变起点
  startColor = currentColor;
  
  // 设置目标模式
  targetMode = newMode;
  
  // 设置目标颜色
  switch(targetMode) {
    case LED_OFF: targetColor = CRGB::Black; break;
    case WARM_LIGHT: targetColor = WARM_WHITE; break;
    case COLD_LIGHT: targetColor = COOL_WHITE; break;
    case BREATHING: targetColor = BREATH_COLOR; break;
  }
  
  // 重置渐变进度
  transitionProgress = 0.0f;
  
  // 设置渐变持续时间
  transitionDuration = duration;
  
  // 记录开始时间
  transitionStartTime = millis();
}

void LEDState::update() {
  // 如果不需要渐变，直接使用目标颜色
  if (transitionProgress >= 1.0f) {
    currentColor = targetColor;
    currentMode = targetMode;
    return;
  }
  
  // 计算当前进度 (基于时间)
  unsigned long elapsed = millis() - transitionStartTime;
  transitionProgress = min(1.0f, (float)elapsed / (float)transitionDuration);
  
  // 使用缓动函数计算插值进度
  float easedProgress = easeInOutQuad(transitionProgress);
  
  // 线性插值计算当前颜色
  currentColor.r = startColor.r + (targetColor.r - startColor.r) * easedProgress;
  currentColor.g = startColor.g + (targetColor.g - startColor.g) * easedProgress;
  currentColor.b = startColor.b + (targetColor.b - startColor.b) * easedProgress;
}

CRGB LEDState::getCurrentColor() const {
  return currentColor;
}

LedMode LEDState::getCurrentMode() const {
  return currentMode;
}

float LEDState::getTransitionProgress() const {
  return transitionProgress;
}

uint16_t LEDState::getRemainingTransitionTime() const {
  if (transitionProgress >= 1.0f) return 0;
  unsigned long elapsed = millis() - transitionStartTime;
  if (elapsed >= transitionDuration) return 0;
  return transitionDuration - elapsed;
}

float LEDState::easeInOutQuad(float t) const {
  return t < 0.5 ? 2 * t * t : 1 - pow(-2 * t + 2, 2) / 2;
}