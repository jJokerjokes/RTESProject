#include <arduinoFFT.h>
#include "Arduino.h"
#include <Wire.h>
#include <LSM6DSLSensor.h>
#include <BLEDevice.h>
#include <STM32duinoBLE.h>


// I2C 实例（使用板载 I2C）
#define DEV_I2C Wire

// LSM6DSL 可能的 I2C 地址
#define LSM6DSL_I2C_ADD_L 0x6A  // SDO/SA0 接地
#define LSM6DSL_I2C_ADD_H 0x6B  // SDO/SA0 接 VCC

// LSM6DSL 传感器实例
LSM6DSLSensor AccGyr(&DEV_I2C, LSM6DSL_I2C_ADD_L);

// ========== B-L475E-IOT01A 板载 LED 资源（底层引脚名）==========
// 使用 STM32 底层引脚名称，绕过 Arduino 映射
#define PIN_LD1_GREEN   PA5   // 用户绿灯 (主指示灯)
#define PIN_LD2_BLUE    PB14  // 用户蓝灯 (用于震颤)
#define PIN_LD4_WIFI    PC9   // WiFi指示灯 (征用作为辅助灯)

// FFT 参数
#define SAMPLES 256
#define SAMPLING_FREQUENCY 52  // 52 Hz

// 数据缓冲区
float v_real[SAMPLES];
float v_imag[SAMPLES];
int bufferIndex = 0;

// ArduinoFFT 实例
ArduinoFFT<float> FFT = ArduinoFFT<float>(v_real, v_imag, SAMPLES, SAMPLING_FREQUENCY);

// 非阻塞采样控制
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 19; // 19ms ≈ 52.6 Hz

// 帕金森症状检测状态
bool isTremor = false;
bool isDyskinesia = false;
bool isFOG = false;
bool imuOk = false;
bool bleOk = false;

// FOG 检测相关参数
float previousVariance = 0.0;
bool wasMoving = false;
float currentVariance = 0.0;

// FOG 检测阈值
#define MOVEMENT_VARIANCE_THRESHOLD 0.012
#define STILLNESS_VARIANCE_THRESHOLD 0.003
#define VARIANCE_DROP_RATIO 0.55 
#define FOG_FREQUENCY_THRESHOLD 3.0 

// LED 状态枚举
enum LEDMode {
  MODE_STANDBY,
  MODE_TREMOR,
  MODE_DYSKINESIA,
  MODE_FOG
};

// LED 控制变量
LEDMode currentLEDMode = MODE_STANDBY;
LEDMode lastDetectedMode = MODE_STANDBY;
unsigned long modeStartTime = 0;
unsigned long lastLEDUpdate = 0;
bool ld1State = LOW;
bool ld2State = LOW;
bool ld4State = LOW;

// 状态保持时间
#define DISPLAY_HOLD_TIME 2000  // 2秒

// LED 时序参数
#define STANDBY_SLOW_BLINK 1000   // 待机慢闪：1秒
#define TREMOR_FAST_BLINK 50      // Tremor 极快闪：50ms (蓝灯频闪)
#define DYSKINESIA_ALTERNATE 150  // Dyskinesia 交替：150ms

// 调试计数器
int debugSampleCount = 0;
#define DEBUG_SAMPLE_PRINT 5

// BLE 定义
#define PD_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define TREMOR_CHAR_UUID      "19B10001-E8F2-537E-4F6C-D104768A1214"
#define DYSKINESIA_CHAR_UUID  "19B10002-E8F2-537E-4F6C-D104768A1214"
#define FOG_CHAR_UUID         "19B10003-E8F2-537E-4F6C-D104768A1214"

BLEService pdService(PD_SERVICE_UUID);
BLEUnsignedCharCharacteristic tremorChar(TREMOR_CHAR_UUID, BLERead | BLENotify);
BLEUnsignedCharCharacteristic dyskinesiaChar(DYSKINESIA_CHAR_UUID, BLERead | BLENotify);
BLEUnsignedCharCharacteristic fogChar(FOG_CHAR_UUID, BLERead | BLENotify);

bool lastTremorState = false;
bool lastDyskinesiaState = false;
bool lastFOGState = false;

// 函数声明
void analyzeFrequency();
float calculateMean(float data[], int length);
float calculateVariance(float data[], int length);
void updateBLECharacteristics();
void updateLEDs();
bool readAccelDirect(int32_t *data);
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t value);
void sampleAndAnalyze();

// ========== 底层寄存器操作函数 ==========
uint8_t readRegister(uint8_t reg) {
  DEV_I2C.beginTransmission(LSM6DSL_I2C_ADD_L);
  DEV_I2C.write(reg);
  DEV_I2C.endTransmission(false);
  
  DEV_I2C.requestFrom((uint8_t)LSM6DSL_I2C_ADD_L, (uint8_t)1);
  if (DEV_I2C.available()) {
    return DEV_I2C.read();
  }
  return 0xFF;
}

void writeRegister(uint8_t reg, uint8_t value) {
  DEV_I2C.beginTransmission(LSM6DSL_I2C_ADD_L);
  DEV_I2C.write(reg);
  DEV_I2C.write(value);
  DEV_I2C.endTransmission();
}

bool readAccelDirect(int32_t *data) {
  if (!imuOk) {
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    return false;
  }

  DEV_I2C.beginTransmission(LSM6DSL_I2C_ADD_L);
  DEV_I2C.write(0x28);
  byte error = DEV_I2C.endTransmission(false);
  
  if (error != 0) {
    return false;
  }
  
  if (DEV_I2C.requestFrom((uint8_t)LSM6DSL_I2C_ADD_L, (uint8_t)6) != 6) {
    return false;
  }
  
  uint8_t buffer[6];
  for (int i = 0; i < 6; i++) {
    buffer[i] = DEV_I2C.read();
  }
  
  int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
  
  data[0] = (int32_t)(raw_x * 0.061f);
  data[1] = (int32_t)(raw_y * 0.061f);
  data[2] = (int32_t)(raw_z * 0.061f);
  
  return true;
}

float calculateMean(float data[], int length) {
  float sum = 0.0;
  for (int i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum / length;
}

float calculateVariance(float data[], int length) {
  float mean = calculateMean(data, length);
  float variance = 0.0;
  
  for (int i = 0; i < length; i++) {
    float diff = data[i] - mean;
    variance += diff * diff;
  }
  
  return variance / length;
}

void updateBLECharacteristics() {
  if (!bleOk) {
    return;
  }

  bool updated = false;
  
  if (isTremor != lastTremorState) {
    tremorChar.writeValue((uint8_t)isTremor);
    lastTremorState = isTremor;
    updated = true;
    Serial.print("[BLE] Tremor characteristic updated: ");
    Serial.println(isTremor ? "1" : "0");
  }
  
  if (isDyskinesia != lastDyskinesiaState) {
    dyskinesiaChar.writeValue((uint8_t)isDyskinesia);
    lastDyskinesiaState = isDyskinesia;
    updated = true;
    Serial.print("[BLE] Dyskinesia characteristic updated: ");
    Serial.println(isDyskinesia ? "1" : "0");
  }
  
  if (isFOG != lastFOGState) {
    fogChar.writeValue((uint8_t)isFOG);
    lastFOGState = isFOG;
    updated = true;
    Serial.print("[BLE] FOG characteristic updated: ");
    Serial.println(isFOG ? "1" : "0");
  }
  
  if (updated) {
    Serial.println("[BLE] Characteristics updated and notified to connected devices");
  }
}

// ========== 非阻塞 LED 控制 - 三灯组合视觉效果 ==========
void updateLEDs() {
  unsigned long currentTime = millis();
  
  // 确定目标模式
  LEDMode targetMode = MODE_STANDBY;
  
  if (isFOG) {
    targetMode = MODE_FOG;
  } else if (isTremor) {
    targetMode = MODE_TREMOR;
  } else if (isDyskinesia) {
    targetMode = MODE_DYSKINESIA;
  }
  
  // 状态保持逻辑
  if (targetMode != MODE_STANDBY && targetMode != lastDetectedMode) {
    lastDetectedMode = targetMode;
    modeStartTime = currentTime;
  }
  
  if (lastDetectedMode != MODE_STANDBY) {
    if (currentTime - modeStartTime < DISPLAY_HOLD_TIME) {
      currentLEDMode = lastDetectedMode;
    } else {
      currentLEDMode = targetMode;
      if (targetMode == MODE_STANDBY) {
        lastDetectedMode = MODE_STANDBY;
      }
    }
  } else {
    currentLEDMode = targetMode;
  }
  
  // 根据模式执行 LED 动画
  switch (currentLEDMode) {
    
    // ========== 模式 0: 待机 - 绿色呼吸 ==========
    case MODE_STANDBY: {
      // LD1 (绿) 慢速闪烁，其他熄灭
      if (currentTime - lastLEDUpdate >= STANDBY_SLOW_BLINK) {
        ld1State = !ld1State;
        digitalWrite(PIN_LD1_GREEN, ld1State);
        digitalWrite(PIN_LD2_BLUE, LOW);
        digitalWrite(PIN_LD4_WIFI, LOW);
        lastLEDUpdate = currentTime;
      }
      break;
    }
    
    // ========== 模式 1: Tremor - 蓝光频闪 ==========
    case MODE_TREMOR: {
      // LD1 与 LD2 两个用户绿灯交替爆闪，LD4 保持熄灭
      if (currentTime - lastLEDUpdate >= TREMOR_FAST_BLINK) {
        ld1State = !ld1State;
        ld2State = !ld1State;  // 与 LD1 互补，形成交替
        digitalWrite(PIN_LD1_GREEN, ld1State);
        digitalWrite(PIN_LD2_BLUE, ld2State);
        digitalWrite(PIN_LD4_WIFI, LOW);
        lastLEDUpdate = currentTime;
      }
      break;
    }
    
    // ========== 模式 2: Dyskinesia - 绿/WiFi 交替乱序 ==========
    case MODE_DYSKINESIA: {
      // LD1 (绿) 和 LD4 (WiFi) 交替互闪 (150ms)
      if (currentTime - lastLEDUpdate >= DYSKINESIA_ALTERNATE) {
        ld1State = !ld1State;
        ld4State = !ld1State;  // 互补
        
        digitalWrite(PIN_LD1_GREEN, ld1State);
        digitalWrite(PIN_LD2_BLUE, LOW);
        digitalWrite(PIN_LD4_WIFI, ld4State);
        lastLEDUpdate = currentTime;
      }
      break;
    }
    
    // ========== 模式 3: FOG - 紧急全亮 ==========
    case MODE_FOG: {
      // 三灯全亮，最高警示
      digitalWrite(PIN_LD1_GREEN, HIGH);
      digitalWrite(PIN_LD2_BLUE, HIGH);
      digitalWrite(PIN_LD4_WIFI, HIGH);
      break;
    }
  }
}

void sampleAndAnalyze() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSampleTime < SAMPLE_INTERVAL) {
    return;
  }
  
  lastSampleTime = currentTime;
  
  int32_t accelerometer[3];
  if (!readAccelDirect(accelerometer)) {
    accelerometer[0] = 0;
    accelerometer[1] = 0;
    accelerometer[2] = 0;
  }
  
  float x = accelerometer[0] / 1000.0f;
  float y = accelerometer[1] / 1000.0f;
  float z = accelerometer[2] / 1000.0f;
  
  if (debugSampleCount < DEBUG_SAMPLE_PRINT) {
    Serial.print("[DEBUG Sample #");
    Serial.print(debugSampleCount + 1);
    Serial.print("] X=");
    Serial.print(x, 4);
    Serial.print("g, Y=");
    Serial.print(y, 4);
    Serial.print("g, Z=");
    Serial.print(z, 4);
    Serial.print("g | Raw[mg]: X=");
    Serial.print(accelerometer[0]);
    Serial.print(", Y=");
    Serial.print(accelerometer[1]);
    Serial.print(", Z=");
    Serial.println(accelerometer[2]);
    debugSampleCount++;
    
    if (debugSampleCount == DEBUG_SAMPLE_PRINT) {
      Serial.println("[DEBUG] First 5 samples printed.\n");
    }
  }
  
  float magnitude = sqrt(x * x + y * y + z * z);
  v_real[bufferIndex] = magnitude;
  bufferIndex++;
  
  if (bufferIndex >= SAMPLES) {
    Serial.println("<<<<<<<<<<<<< Author: Bai, Gengyuan; Zhang, Charles >>>>>>>>>>>>>");
    Serial.println("\n********** Buffer Full **********");
    Serial.print("Time elapsed: ");
    Serial.print((currentTime - (lastSampleTime - SAMPLE_INTERVAL * (SAMPLES - 1))));
    Serial.println(" ms");
    Serial.print("Expected: ");
    Serial.print(SAMPLE_INTERVAL * SAMPLES);
    Serial.println(" ms");
    
    analyzeFrequency();
    
    bufferIndex = 0;
    Serial.println("Ready for next data collection...\n");
  }
}

void analyzeFrequency() {
  Serial.println("\n========== FFT Analysis ==========");
  
  currentVariance = calculateVariance(v_real, SAMPLES);
  float stdDev = sqrt(currentVariance);
  
  Serial.print("Variance: ");
  Serial.println(currentVariance, 4);
  Serial.print("Std Dev: ");
  Serial.println(stdDev, 4);
  
  // ========== DC Removal（去直流）==========
  float mean = calculateMean(v_real, SAMPLES);
  Serial.print("Mean (DC component): ");
  Serial.print(mean, 4);
  Serial.println(" g");
  
  Serial.println("Removing DC component...");
  for (int i = 0; i < SAMPLES; i++) {
    v_real[i] = v_real[i] - mean;
  }
  
  float mean_after = calculateMean(v_real, SAMPLES);
  float variance_after = calculateVariance(v_real, SAMPLES);
  Serial.print("Mean after DC removal: ");
  Serial.print(mean_after, 6);
  Serial.println(" g (should be ~0)");
  Serial.print("Variance after DC removal: ");
  Serial.println(variance_after, 4);
  
  // 清空虚部数组
  for (int i = 0; i < SAMPLES; i++) {
    v_imag[i] = 0.0;
  }
  
  // FFT 处理
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  float peakFrequency = 0;
  float peakValue = 0;
  FFT.majorPeak(&peakFrequency, &peakValue);
  
  Serial.print("Peak Frequency: ");
  Serial.print(peakFrequency, 2);
  Serial.println(" Hz");
  Serial.print("Peak Magnitude: ");
  Serial.println(peakValue, 2);
  
  // 重置状态
  isTremor = false;
  isDyskinesia = false;
  isFOG = false;
  
  // ========== 幅度门限检查 ==========
  const float MAGNITUDE_THRESHOLD = 1.5; // Up to change
  
  if (peakValue < MAGNITUDE_THRESHOLD) {
    Serial.print("Low signal power (");
    Serial.print(peakValue, 2);
    Serial.print(" < ");
    Serial.print(MAGNITUDE_THRESHOLD, 1);
    Serial.println("), ignoring...");
    Serial.println("(Likely just noise or very subtle motion)");
  } else {
    Serial.print("Signal strength sufficient (");
    Serial.print(peakValue, 2);
    Serial.println("), analyzing frequency...");
    
    if (peakFrequency >= 3.0 && peakFrequency < 5.0) {
      isTremor = true;
      Serial.println(">>> TREMOR DETECTED (3-5 Hz) <<<");
    } 
    else if (peakFrequency >= 5.0 && peakFrequency <= 7.0) {
      isDyskinesia = true;
      Serial.println(">>> DYSKINESIA DETECTED (5-7 Hz) <<<");
    }
    else {
      Serial.println("Frequency out of symptom range.");
    }
  }
  
  // FOG 检测
  Serial.println("\n--- FOG Detection Analysis ---");
  bool isCurrentlyMoving = currentVariance > MOVEMENT_VARIANCE_THRESHOLD;
  
  Serial.print("Previous Moving State: ");
  Serial.println(wasMoving ? "YES" : "NO");
  Serial.print("Current Moving State: ");
  Serial.println(isCurrentlyMoving ? "YES" : "NO");
  
  if (wasMoving && previousVariance > 0) {
    float varianceRatio = currentVariance / previousVariance;
    
    Serial.print("Previous Variance: ");
    Serial.println(previousVariance, 4);
    Serial.print("Variance Ratio (current/previous): ");
    Serial.println(varianceRatio, 3);
    
    if (varianceRatio < VARIANCE_DROP_RATIO && 
        currentVariance > STILLNESS_VARIANCE_THRESHOLD &&
        currentVariance < MOVEMENT_VARIANCE_THRESHOLD &&
        peakFrequency < FOG_FREQUENCY_THRESHOLD) {
      
      isFOG = true;
      Serial.println(">>> FOG (FREEZING OF GAIT) DETECTED <<<");
      Serial.println("Indicators: Sudden variance drop during movement + Low frequency");
    }
  }
  
  previousVariance = currentVariance;
  wasMoving = isCurrentlyMoving;
  
  Serial.println("\n========== Status Summary ==========");
  Serial.print("Tremor: ");
  Serial.println(isTremor ? "YES" : "NO");
  Serial.print("Dyskinesia: ");
  Serial.println(isDyskinesia ? "YES" : "NO");
  Serial.print("FOG (Freezing of Gait): ");
  Serial.println(isFOG ? "YES" : "NO");
  Serial.println("====================================\n");
  
  updateBLECharacteristics();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("========================================");
  Serial.println("This is the RTES project of ECE-GY 6483 2025 Fall Group 30, author: gengyuan bai.");
  Serial.println("Team members: 1. Bai, Gengyuan  2. Hua, Bin  3. Huang, Yanjie  4. Lee, Kylie  5. Zhang, Charles");
  Serial.println("System Start");
  Serial.println("Parkinson's Disease Monitoring System");
  Serial.println("ST B-L475E-IOT01A Discovery Kit");
  Serial.println("========================================");
  
  // 立即配置 LED，提供心跳反馈，证明 MCU 正在运行
  pinMode(PIN_LD1_GREEN, OUTPUT);
  pinMode(PIN_LD2_BLUE, OUTPUT);
  pinMode(PIN_LD4_WIFI, OUTPUT);
  
  digitalWrite(PIN_LD1_GREEN, LOW);
  digitalWrite(PIN_LD2_BLUE, LOW);
  digitalWrite(PIN_LD4_WIFI, LOW);
  
  digitalWrite(PIN_LD1_GREEN, HIGH);
  delay(150);
  digitalWrite(PIN_LD1_GREEN, LOW);
  delay(150);
  
  // ========== LED 硬件自检（阻塞式）==========
  Serial.println("\n========== LED Hardware Test ==========");
  Serial.println("Using STM32 native pin names:");
  Serial.println("  LD1 (Green):  PA5");
  Serial.println("  LD2 (Blue):   PB14");
  Serial.println("  LD4 (WiFi):   PC9");
  
  Serial.println("\nSequential test (watch the LEDs!):");
  
  // 测试 LD1 (绿)
  Serial.println("  LD1 (Green) ON...");
  digitalWrite(PIN_LD1_GREEN, HIGH);
  delay(300);
  digitalWrite(PIN_LD1_GREEN, LOW);
  delay(200);
  
  // 测试 LD2 (蓝)
  Serial.println("  LD2 (Blue) ON...");
  digitalWrite(PIN_LD2_BLUE, HIGH);
  delay(300);
  digitalWrite(PIN_LD2_BLUE, LOW);
  delay(200);
  
  // 测试 LD4 (WiFi)
  Serial.println("  LD4 (WiFi) ON...");
  digitalWrite(PIN_LD4_WIFI, HIGH);
  delay(300);
  digitalWrite(PIN_LD4_WIFI, LOW);
  delay(200);
  
  // 全亮测试
  Serial.println("  ALL LEDs ON (FOG mode preview)...");
  digitalWrite(PIN_LD1_GREEN, HIGH);
  digitalWrite(PIN_LD2_BLUE, HIGH);
  digitalWrite(PIN_LD4_WIFI, HIGH);
  delay(1000);
  
  // 全灭
  digitalWrite(PIN_LD1_GREEN, LOW);
  digitalWrite(PIN_LD2_BLUE, LOW);
  digitalWrite(PIN_LD4_WIFI, LOW);
  delay(300);
  
  Serial.println("\nHardware Test: LD1(G), LD2(B), LD4(WIFI) checked.");
  Serial.println("  If all 3 LEDs lit up, hardware is OK.");
  Serial.println("  If LD2 didn't light, it may use different pin.");
  Serial.println("======================================");
  
  Serial.println("\n========== LED Mode Preview ==========");
  Serial.println("Showing MODE_TREMOR, MODE_DYSKINESIA, MODE_FOG for 2s each...");
  
  // 模式 1 预览（Tremor）
  for (int i = 0; i < 2000; i += TREMOR_FAST_BLINK) {
    ld1State = !ld1State;
    ld2State = !ld1State;
    digitalWrite(PIN_LD1_GREEN, ld1State);
    digitalWrite(PIN_LD2_BLUE, ld2State);
    digitalWrite(PIN_LD4_WIFI, LOW);
    delay(TREMOR_FAST_BLINK);
  }
  
  // 灯全灭再进入下一个模式
  digitalWrite(PIN_LD1_GREEN, LOW);
  digitalWrite(PIN_LD2_BLUE, LOW);
  digitalWrite(PIN_LD4_WIFI, LOW);
  delay(200);
  
  // 模式 2 预览（Dyskinesia）
  for (int i = 0; i < 2000; i += DYSKINESIA_ALTERNATE) {
    ld1State = !ld1State;
    ld4State = !ld1State;
    digitalWrite(PIN_LD1_GREEN, ld1State);
    digitalWrite(PIN_LD4_WIFI, ld4State);
    digitalWrite(PIN_LD2_BLUE, LOW);
    delay(DYSKINESIA_ALTERNATE);
  }
  
  digitalWrite(PIN_LD1_GREEN, LOW);
  digitalWrite(PIN_LD2_BLUE, LOW);
  digitalWrite(PIN_LD4_WIFI, LOW);
  delay(200);
  
  // 模式 3 预览（FOG）
  digitalWrite(PIN_LD1_GREEN, HIGH);
  digitalWrite(PIN_LD2_BLUE, HIGH);
  digitalWrite(PIN_LD4_WIFI, HIGH);
  delay(2000);
  digitalWrite(PIN_LD1_GREEN, LOW);
  digitalWrite(PIN_LD2_BLUE, LOW);
  digitalWrite(PIN_LD4_WIFI, LOW);
  
  Serial.println("LED mode preview finished.");
  Serial.println("<<<<<<<<<<<<< Author: Bai, Gengyuan; Zhang, Charles >>>>>>>>>>>>>");

  // I2C 初始化
  Serial.println("\n========== I2C Initialization ==========");
  Serial.println("I2C pins: SDA=PB11, SCL=PB10");
  
  DEV_I2C.setSDA(PB11);
  DEV_I2C.setSCL(PB10);
  DEV_I2C.begin();
  DEV_I2C.setClock(100000);
  
  Serial.println("I2C initialized: 100kHz");
  Serial.println("======================================");

  // LSM6DSL 初始化
  Serial.println("\n========== LSM6DSL Initialization ==========");
  
  AccGyr.begin();
  
  uint8_t sensorId = readRegister(0x0F);
  Serial.print("WHO_AM_I (0x0F): 0x");
  Serial.println(sensorId, HEX);
  
  if (sensorId == 0x6A) {
    imuOk = true;
    Serial.println("✓ Sensor ID verified");
  } else {
    imuOk = false;
    Serial.println("ERROR: Wrong sensor ID! IMU data will be replaced with zeros.");
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_LD2_BLUE, HIGH);
      delay(120);
      digitalWrite(PIN_LD2_BLUE, LOW);
      delay(120);
    }
  }
  
  if (imuOk) {
    // ========== 寄存器级配置 ==========
    Serial.println("\n========== Register Configuration ==========");
    
    uint8_t ctrl1_before = readRegister(0x10);
    Serial.print("CTRL1_XL before: 0x");
    Serial.println(ctrl1_before, HEX);
    
    // 写入配置：416Hz, ±2g
    Serial.println("Writing 0x60 to CTRL1_XL (416Hz, ±2g)...");
    writeRegister(0x10, 0x60);
    delay(10);
    
    // 配置 CTRL3_C：BDU + IF_INC
    Serial.println("Writing 0x44 to CTRL3_C (BDU + IF_INC)...");
    writeRegister(0x12, 0x44);
    delay(10);
    
    uint8_t ctrl1_after = readRegister(0x10);
    Serial.print("CTRL1_XL after: 0x");
    Serial.println(ctrl1_after, HEX);
    
    if (ctrl1_after == 0x60) {
      Serial.println("✓ Configuration verified");
    }
    
    delay(100);
    
    // 测试数据读取
    int32_t test_data[3];
    if (readAccelDirect(test_data)) {
      Serial.print("Test read: X=");
      Serial.print(test_data[0]);
      Serial.print(" mg, Y=");
      Serial.print(test_data[1]);
      Serial.print(" mg, Z=");
      Serial.print(test_data[2]);
      Serial.println(" mg");
      
      if (test_data[0] != 0 || test_data[1] != 0 || test_data[2] != 0) {
        Serial.println("✓ Sensor is outputting data!");
      } else {
        Serial.println("WARNING: Data is still zero");
      }
    }
    
    Serial.println("==========================================");
  }

  // BLE 初始化
  Serial.println("\nInitializing BLE...");
  bleOk = BLE.begin();
  if (!bleOk) {
    Serial.println("Failed to initialize BLE! Program will run WITHOUT BLE.");
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_LD4_WIFI, HIGH);
      delay(150);
      digitalWrite(PIN_LD4_WIFI, LOW);
      delay(150);
    }
  } else {
    BLE.setLocalName("PD_Monitor");
    BLE.setDeviceName("PD_Monitor");
    // BLE.setAdvertisedService(pdService);  // 暂时禁用，避免广播包过大影响 iOS 扫描
    
    pdService.addCharacteristic(tremorChar);
    pdService.addCharacteristic(dyskinesiaChar);
    pdService.addCharacteristic(fogChar);
    BLE.addService(pdService);
    
    tremorChar.writeValue((uint8_t)0);
    dyskinesiaChar.writeValue((uint8_t)0);
    fogChar.writeValue((uint8_t)0);
    
    BLE.advertise();
    
    Serial.println("BLE initialized and advertising as PD_Monitor");
    // NOTE: iOS 系统设置里看不到该类自定义 BLE 外设，请使用 nRF Connect / LightBlue 扫描。
    Serial.println("NOTE: Use nRF Connect / LightBlue on iOS to scan custom BLE peripherals (系统设置里不会显示)。");
  }
  
  Serial.println("\n========== LED Display Modes ==========");
  Serial.println("Mode 0 (Standby):    LD1 slow blink");
  Serial.println("Mode 1 (Tremor):     LD2 RAPID strobe (50ms)");
  Serial.println("Mode 2 (Dyskinesia): LD1 ⇄ LD4 alternate");
  Serial.println("Mode 3 (FOG):        ALL ON (LD1+LD2+LD4)");
  Serial.println("Display hold: 2 seconds");
  Serial.println("========================================\n");
  
  Serial.println("Starting data acquisition...");
  Serial.println("Entering main loop...\n");

  unsigned long now = millis();
  lastSampleTime = now;
  lastLEDUpdate = now;
  modeStartTime = now;
}

void loop() {
  updateLEDs();
  
  if (bleOk) {
    BLE.poll();
    static bool wasConnected = false;
    BLEDevice central = BLE.central();
    
    if (central && !wasConnected) {
      Serial.print("[BLE] Connected: ");
      Serial.println(central.address());
      wasConnected = true;
    } else if (!central && wasConnected) {
      Serial.println("[BLE] Disconnected");
      wasConnected = false;
    }
  }
  
  sampleAndAnalyze();
}