#include <ESP32Servo.h>

#define SERVO1_PIN 5
#define SERVO2_PIN 18
#define SERVO3_PIN 19

#define SERIAL1_RX 16  // 语言识别
#define SERIAL1_TX 4
#define SERIAL2_RX 21  // 视觉识别
#define SERIAL2_TX 17

Servo myServo1;
Servo myServo2;
Servo myServo3;

void setup() {
  delay(2000);
  // 串口初始化
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
  myServo1.attach(SERVO1_PIN, 500, 2500); // 指定脉冲宽度范围
  myServo2.attach(SERVO2_PIN, 500, 2500);
  myServo3.attach(SERVO3_PIN, 500, 2500);
  setServoAngle270(myServo1, 135); // 初始化舵机角度为135度
  setServoAngle270(myServo2, 135);
  setServoAngle270(myServo3, 63);
  delay(2000);
}

// void loop() {
//   while (Serial2.available()) {
//     uint8_t b = Serial2.read();
//     // 打印为两位十六进制，不足补0
//     if (b < 0x10) Serial.print("0");
//     Serial.print(b, HEX);
//     Serial.print(" ");
//   }
// }
//语音识别测试

void testServo3FromSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int angle = input.toInt();
    if (angle < 0) angle = 0;
    if (angle > 270) angle = 270;
    setServoAngle270(myServo3, angle);
    Serial.print("Set servo3 to angle: ");
    Serial.println(angle);
  }
}

// void loop() {
//   testServo3FromSerial();
// }
//舵机测试

uint8_t buffer[5];
int bufIndex = 0;


// 检测模式全局变量
bool detectionMode = false;
bool waitingAfterAction = false;
unsigned long actionStartTime = 0;

void loop() {
  receiveAndHandleCommand(); // 始终处理语音指令

  // 检测模式下持续读取视觉数据
  if (detectionMode) {
    if (waitingAfterAction) {
      if (millis() - actionStartTime >= 6000) {
        waitingAfterAction = false;
        Serial.println("Ready for new vision detection.");
      }
    } else {
        // 清空视觉串口缓冲区，避免读到旧数据
      while (Serial2.available() > 0) {
        Serial2.read();
      }
      int visionCmd = tryGetVisionCommand();
      if (visionCmd != -1) {
        handleCommand(visionCmd);
        waitingAfterAction = true;
        actionStartTime = millis();
      }
    }
  }
}

// 接收并处理串口指令，自动解码并执行动作
void receiveAndHandleCommand() {
  static uint8_t buffer[5];
  static int bufIndex = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    buffer[bufIndex++] = b;
    if (bufIndex == 5) {
      // 检查帧头帧尾，支持检测控制指令
      if (buffer[0] == 0xAA && buffer[1] == 0x55 && (buffer[2] == 0x00 || buffer[2] == 0x04) && buffer[4] == 0xFB) {
        uint8_t cmd = buffer[3];
        if (buffer[2] == 0x04) {
          if (cmd == 0x1E) { // 开启检测
            detectionMode = true;
            waitingAfterAction = false;
            Serial.println("Detection mode started");
          } else if (cmd == 0x1F) { // 停止检测
            detectionMode = false;
            waitingAfterAction = false;
            Serial.println("Detection mode stopped");
          }
        } else {
          Serial.println("Received command: " + String(cmd, HEX));
          handleCommand(cmd);
        }
      }
      bufIndex = 0;
    }
  }
}
// 非阻塞方式尝试从Serial2读取视觉数据，读取到完整数据则返回命令，否则返回-1
int tryGetVisionCommand() {
  static uint8_t vBuffer[5];
  static int vBufIndex = 0;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    vBuffer[vBufIndex++] = b;
    if (vBufIndex == 5) {
      if (vBuffer[0] == 0xAA && vBuffer[1] == 0x55 && vBuffer[2] == 0x00 && vBuffer[4] == 0xFB) {
        uint8_t cmd = vBuffer[3];
        Serial.println("Vision command: " + String(cmd, HEX));
        // 如果视觉检测到目标为以下任一物品，则转发完整帧到串口1
        if (cmd == 0x06 || cmd == 0x07 || 
            cmd == 0x09 || cmd == 0x10 ||
            cmd == 0x13 || cmd == 0x14 ||
            cmd == 0x16 || cmd == 0x17) {
          Serial1.write(vBuffer, 5);
          Serial.println("Forwarded vision frame to Serial1.");
        }
        vBufIndex = 0;
        return cmd;
      }
      vBufIndex = 0;
    }
  }
  return -1;
}


void handleCommand(uint8_t cmd) {
  // 开关门指令
  if (cmd == 0x01 || cmd == 0x02) { // 开门
    openDoor();
    return;
  }
  if (cmd == 0x03 || cmd == 0x04) { // 关门
    closeDoor();
    return;
  }
  // 可回收垃圾类
  if (cmd == 0x05 || cmd == 0x06 || cmd == 0x07) { // 0x05:可回收, 0x06:塑料瓶, 0x07：纸杯
    dropGarbage(1);
    return;
  }
  // 不可回收垃圾类
  if (cmd == 0x08 || cmd == 0x09 || cmd == 0x10) { // 0x08:不可回收, 0x09:打火机, 0x10:抹布
    dropGarbage(3);
    return;
  }
  // 厨余垃圾类
  if (cmd == 0x11 || cmd == 0x12 || cmd == 0x13 || cmd == 0x14) { // 0x11:厨余, 0x12：厨房, 0x13：苹果，0x14:橙子
    dropGarbage(2);
    return;
  }
  // 有害垃圾类
  if (cmd == 0x15 || cmd == 0x16 || cmd == 0x17 || cmd == 0x18) { // 0x15:有害, 0x16:电池, 0x17:过期药品
    dropGarbage(4);
    return;
  }
}

// 舵机3开门
void openDoor() {
  setServoAngle270(myServo3, 155);
  Serial.println("Door opened");
}

// 舵机3关门
void closeDoor() {
  setServoAngle270(myServo3, 63);
  Serial.println("Door closed");
}

// 0~270度线性映射到500~2500us脉宽的舵机控制函数
void setServoAngle270(Servo &servo, int angle) {
  if (angle < 0) angle = 45;
  if (angle > 270) angle = 135;
  int pulse = map(angle, 0, 270, 500, 2500);
  servo.writeMicroseconds(pulse);
  
  // 通过比较舵机对象的地址来判断是哪个舵机
  Serial.print("Servo");
  if (&servo == &myServo1) {
    Serial.print("servo1: ");
  } else if (&servo == &myServo2) {
    Serial.print("servo2: "); 
  } else if (&servo == &myServo3) {
    Serial.print("servo3: ");
  }
  Serial.print(" Angle: ");
  Serial.println(angle);
}

// 垃圾投放动作函数
// pos: 1~4，对应四个投放位置
void dropGarbage(int pos) {
  int s1 = 135, s2 = 135;
  switch(pos) {
    case 1: s1 = 135; s2 = 90; break;
    case 2: s1 = 45; s2 = 90; break;
    case 3: s1 = 135;  s2 = 180; break;
    case 4: s1 = 45;  s2 = 180; break;
    default: return;
  }
  setServoAngle270(myServo1, s1); // 下方舵机先动
  delay(1000);
  setServoAngle270(myServo2, s2); // 上方舵机后动
  delay(2000); // 停顿2秒
  setServoAngle270(myServo2, 135); // 上方舵机恢复
}

// 阻塞式读取串口2，直到收到一帧有效数据
int waitForVisionCommand() {
  // 清空上次残留的数据
  while (Serial2.available() > 0) {
    Serial2.read();
  }
  
  uint8_t buffer[5];
  int bufIndex = 0;
  while (true) {
    while (Serial2.available()) {
      uint8_t b = Serial2.read();
      buffer[bufIndex++] = b;
      if (bufIndex == 5) {
        if (buffer[0] == 0xAA && buffer[1] == 0x55 && buffer[2] == 0x00 && buffer[4] == 0xFB) {
          uint8_t cmd = buffer[3];
          Serial.println("Vision command: " + String(cmd, HEX));
          // 如果视觉检测到目标为以下任一物品，则转发原始帧到串口1
          if (cmd == 0x06 || cmd == 0x07 || 
              cmd == 0x09 || cmd == 0x10 ||
              cmd == 0x13 || cmd == 0x14 ||
              cmd == 0x16 || cmd == 0x17) {
            Serial1.write(buffer, 5);
            Serial.println("Forwarded vision frame to Serial1.");
            Serial.print(buffer[0], HEX);
            Serial.print(buffer[1], HEX);
            Serial.print(buffer[2], HEX);
            Serial.print(buffer[3], HEX);
            Serial.println(buffer[4], HEX);
          }
          return cmd;
        }
        bufIndex = 0; // 错帧则重置缓冲区
      }
    }
  }
}