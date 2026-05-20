#include <Arduino.h>
#include <math.h>

// PlatformIO 的 ESP32 Arduino core 使用 Serial 作为默认调试串口。
#ifndef Serial0
#define Serial0 Serial
#endif

// 硬件状态：四个电机共同拉着一个小球，电机分布在左下、左上、右上、右下；
// 对应 (x,y) 坐标为 (0,0), (0,15cm), (15cm,15cm), (15cm,0)。

// PlatformIO/C++ 不会像 Arduino IDE 一样自动生成函数声明。
void readAllMotorAngle();
void calcRealCoefficient();

// ------------------- 硬件配置 -------------------
#define UART2_TX 17
#define UART2_RX 16
HardwareSerial uart2(2);

// 模式标志 false=单电机模式 true=全体电机同步模式
bool allMotorMode = false;
const uint8_t allMotorList[] = {1, 2, 3, 4};
const int motorTotal = 4;

// ------------------- 通信底层 -------------------
void send_single(uint8_t addr, const uint8_t *cmd, size_t cmd_len) {
    uart2.write(addr);
    uart2.write(cmd, cmd_len);
    uart2.write(0x6B);
    delayMicroseconds(2000);
}

void send_multi_motor(const uint8_t *commands, size_t data_len) {
    uint16_t total_len = data_len + 3;
    uart2.write(0x00);
    uart2.write(0xAA);
    uart2.write((total_len >> 8) & 0xFF);
    uart2.write(total_len & 0xFF);
    uart2.write(commands, data_len);
    uart2.write(0x6B);
    delayMicroseconds(4000);
}

// ------------------- 功能函数 -------------------
// 锁定面板按键
void lock_key(uint8_t addr) {
    const uint8_t cmd[] = {0xD0, 0xB3, 0x01, 0x01};
    send_single(addr, cmd, sizeof(cmd));
    Serial0.printf("电机%d → 面板按键锁定\n", addr);
}

// 解锁面板按键
void unlock_key(uint8_t addr) {
    const uint8_t cmd[] = {0xD0, 0xB3, 0x00, 0x00};
    send_single(addr, cmd, sizeof(cmd));
    Serial0.printf("电机%d → 面板按键解锁\n", addr);
}

// 清零位置
void clear_pos(uint8_t addr) {
    const uint8_t cmd[] = {0x0A, 0x6D};
    send_single(addr, cmd, sizeof(cmd));
    Serial0.printf("电机%d → 位置清零\n", addr);
}

// 电机使能/失能
void enable_motor(uint8_t addr, bool en) {
    uint8_t state = en ? 1 : 0;
    uint8_t cmd[] = {0xF3, 0xAB, state, 0x00};
    send_single(addr, cmd, sizeof(cmd));
    Serial0.printf("电机%d → 使能:%d\n", addr, en);
}

// 读位置
float get_pos(uint8_t addr) {
    while (uart2.available()) uart2.read();
    const uint8_t cmd[] = {0x36};
    send_single(addr, cmd, sizeof(cmd));
    uint8_t buf[32];
    int len = 0;
    uint32_t start = millis();
    while (millis() - start < 50 && len < 32) {
        if (uart2.available()) buf[len++] = uart2.read();
    }
    if (len >= 8) {
        uint8_t sign = buf[2];
        int32_t val = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8) | buf[6];
        if (sign) val = -val;
        return (float)val / 10.0f;
    }
    return 0.0f;
}

// 角度控制（修复负角度疯狂转圈问题）
void move_motor(uint8_t addr, float angle) {
    uint8_t buf[32];
    size_t idx = 0;
    uint8_t dir = 0x00;
    uint32_t pos_val = 0;

    if (angle < 0) {
        dir = 0x01;
        int32_t pos_neg = (int32_t)(angle * 10);
        pos_val = (uint32_t)(-pos_neg);
    } else {
        dir = 0x00;
        pos_val = (uint32_t)(angle * 10);
    }

    uint16_t max_current = 2000;
    int speed = 2000;
    uint16_t speed_val = speed * 10;

    buf[idx++] = addr;
    buf[idx++] = 0xCB;
    buf[idx++] = dir;
    buf[idx++] = (speed_val >> 8) & 0xFF;
    buf[idx++] = speed_val & 0xFF;
    buf[idx++] = (pos_val >> 24) & 0xFF;
    buf[idx++] = (pos_val >> 16) & 0xFF;
    buf[idx++] = (pos_val >> 8) & 0xFF;
    buf[idx++] = pos_val & 0xFF;
    buf[idx++] = 0x01;
    buf[idx++] = 0x00;
    buf[idx++] = (max_current >> 8) & 0xFF;
    buf[idx++] = max_current & 0xFF;
    buf[idx++] = 0x6B;

    send_multi_motor(buf, idx);
    Serial0.printf("电机%d → 角度:%.1f°\n", addr, angle);
}

// 力矩模式（支持正负电流，自动判断方向）
void torque_motor(uint8_t addr, int16_t current) {
    uint8_t buf[32];
    size_t idx = 0;
    int slope = 2000;

    // 自动根据电流正负确定方向：正=0，负=1
    uint8_t dir = (current >= 0) ? 0 : 1;
    uint16_t abs_current = abs(current);

    buf[idx++] = addr;
    buf[idx++] = 0xF5;
    buf[idx++] = dir;
    buf[idx++] = (slope >> 8) & 0xFF;
    buf[idx++] = slope & 0xFF;
    buf[idx++] = (abs_current >> 8) & 0xFF;
    buf[idx++] = abs_current & 0xFF;
    buf[idx++] = 0x00;
    buf[idx++] = 0x6B;

    send_multi_motor(buf, idx);
    Serial0.printf("电机%d → 力矩:%dmA 方向:%d\n", addr, current, dir);
}

// 编码器校准
void calibrate_encoder(uint8_t addr) {
    const uint8_t cmd[] = {0x06, 0x45};
    Serial0.printf("电机%d → 开始编码器校准...\n", addr);
    send_single(addr, cmd, sizeof(cmd));
    uint8_t resp[8];
    int len = 0;
    uint32_t start = millis();
    while (millis() - start < 1000 && len < 8) {
        if (uart2.available()) resp[len++] = uart2.read();
    }
    if (len >= 4 && resp[1] == 0x06) {
        if (resp[2] == 0x02) Serial0.println("校准确认，执行中...");
        else if (resp[2] == 0xE2) Serial0.println("错误：请先使能电机进入闭环");
        else Serial0.printf("返回码:0x%02X\n", resp[2]);
    } else {
        Serial0.println("未收到校准确认");
    }
}

// 全体电机统一执行函数
void allLockKey() { for (int i = 0; i < motorTotal; i++) lock_key(allMotorList[i]); }
void allUnLockKey() { for (int i = 0; i < motorTotal; i++) unlock_key(allMotorList[i]); }
void allClearPos() { for (int i = 0; i < motorTotal; i++) clear_pos(allMotorList[i]); }
void allEnable(bool en) { for (int i = 0; i < motorTotal; i++) enable_motor(allMotorList[i], en); }
void allCalibrate() { for (int i = 0; i < motorTotal; i++) calibrate_encoder(allMotorList[i]); }

void allMove(float ang) {
    for (int i = 0; i < motorTotal; i++) {
        move_motor(allMotorList[i], ang);
    }
}

void allTorque(int cur) {
    for (int i = 0; i < motorTotal; i++) {
        torque_motor(allMotorList[i], cur);
    }
}

void getAllPos() {
    Serial0.print("全部位置：");
    for (int i = 0; i < motorTotal; i++) {
        Serial0.print(get_pos(allMotorList[i]));
        if (i != motorTotal - 1) Serial0.print(",");
    }
    Serial0.println();
}

// ------------------- 调试指令 -------------------
void show_help() {
    Serial0.println("\n========== 调试指令 ==========");
    Serial0.println("M  切换模式【单电机/全体电机】");
    Serial0.println("---通用指令---");
    Serial0.println("p 读位置  l锁按键  u解锁  e使能  d失能  c清零  ca校准");
    Serial0.println("数字=转到对应角度(支持正负)  t数值=设置力矩(支持正负)");
    Serial0.println("单电机格式：1p 1e 1-90 1t-500");
    Serial0.println("全体模式：直接输入p/e/d/c/ca/-90/t-500 全局生效");
    Serial0.println("---坐标指令---");
    Serial0.println("home  四点自动校准");
    Serial0.println("goto X Y  移动到坐标，单位mm，例如 goto 75 75");
    Serial0.println("==============================\n");
}

void parse_command(String s) {
    s.trim();
    if (s == "M" || s == "m") {
        allMotorMode = !allMotorMode;
        if (allMotorMode) Serial0.println("✅ 当前模式：全体电机同步测试");
        else Serial0.println("✅ 当前模式：单电机独立测试");
        return;
    }

    if (allMotorMode) {
        if (s == "p" || s == "P") getAllPos();
        else if (s == "l" || s == "L") allLockKey();
        else if (s == "u" || s == "U") allUnLockKey();
        else if (s == "e" || s == "E") allEnable(true);
        else if (s == "d" || s == "D") allEnable(false);
        else if (s == "c" || s == "C") allClearPos();
        else if (s == "ca" || s == "CA") allCalibrate();
        else if (s[0] == 't' || s[0] == 'T') {
            int val = s.substring(1).toInt();
            allTorque(val);
        } else if (isDigit(s[0]) || s[0] == '-') {
            float ang = s.toFloat();
            allMove(ang);
        }
        return;
    }

    if (s.length() < 2) return;
    uint8_t id = s[0] - '0';
    if (id < 1 || id > 4) return;

    if (s[1] == 'p' || s[1] == 'P') Serial0.printf("电机%d 位置:%.1f\n", id, get_pos(id));
    else if (s[1] == 'l' || s[1] == 'L') lock_key(id);
    else if (s[1] == 'u' || s[1] == 'U') unlock_key(id);
    else if (s[1] == 'e' || s[1] == 'E') enable_motor(id, true);
    else if (s[1] == 'd' || s[1] == 'D') enable_motor(id, false);
    else if (s[1] == 'c' && s.length() == 2) clear_pos(id);
    else if (s[1] == 'c' && (s[2] == 'a' || s[2] == 'A')) calibrate_encoder(id);
    else if (s[1] == 't' || s[1] == 'T') {
        int c = s.substring(2).toInt();
        torque_motor(id, c);
    } else if (isDigit(s[1]) || s[1] == '-') {
        float ang = s.substring(1).toFloat();
        move_motor(id, ang);
    }
}

// ====================== 正方向修正 + 坐标校准系统 ======================
// 1. 电机正方向定义：1、3正常，2、4反转
int16_t fixTorqueDir(uint8_t addr, int16_t current) {
    if (addr == 2 || addr == 4) return -current;
    return current;
}

// 2. 坐标与物理参数定义（单位：mm）
const float L0 = 150.0f;

float motorAngle[5] = {0};
float ropeLength[5] = {0};

// caliAngle[点位][电机]
// 点位：1=左下(0,0) 2=左上(0,150) 3=右上(150,150) 4=右下(150,0)
float caliAngle[5][5] = {0};

// 4个电机的真实换算系数 (°/mm)，校准后自动计算
float realDeg2mm[5] = {0, -4.3, 4.3, -4.3, 4.3};
const int8_t motorAngleSign[5] = {0, -1, 1, -1, 1};

// 理论系数（校准后会被实测值覆盖）
const float deg2mm_theory = 3.14159f * 13.5f / 360.0f;

// 4个校准点坐标
const float caliX[5] = {0, 0, 0, 150, 150};
const float caliY[5] = {0, 0, 150, 150, 0};

// 新版四点自动校准（左下→左上→右上→右下）
void autoCalibrateHome() {
    Serial0.println("\n========== 开始 四点自动校准 ==========");
    Serial0.println("顺序：左下(0,0) → 左上(0,150) → 右上(150,150) → 右下(150,0)");

    allEnable(true);

    Serial0.println("\n步骤1：所有电机 100mA 预拉紧");
    for (int i = 0; i < motorTotal; i++) {
        int addr = allMotorList[i];
        int cur = fixTorqueDir(addr, 100);
        torque_motor(addr, cur);
    }
    delay(1000);

    for (int point = 1; point <= 4; point++) {
        Serial0.printf("\n===== 拉到第 %d 个点：(%.0f, %.0f) ====\n",
                       point, caliX[point], caliY[point]);

        int pullMotor = point;

        for (int i = 0; i < motorTotal; i++) {
            int addr = allMotorList[i];
            int cur = fixTorqueDir(addr, 150);
            torque_motor(addr, cur);
        }

        int cur = fixTorqueDir(pullMotor, 800);
        torque_motor(pullMotor, cur);
        delay(1000);

        readAllMotorAngle();
        for (int m = 1; m <= 4; m++) {
            caliAngle[point][m] = motorAngle[m];
        }
        clear_pos(pullMotor);
        Serial0.printf("→ 电机%d 已记录角度并清零\n", pullMotor);

        Serial0.printf("已记录 点%d 角度：M1=%.1f  M2=%.1f  M3=%.1f  M4=%.1f\n",
                       point,
                       caliAngle[point][1],
                       caliAngle[point][2],
                       caliAngle[point][3],
                       caliAngle[point][4]);

        cur = fixTorqueDir(pullMotor, 100);
        torque_motor(pullMotor, cur);
        delay(500);
    }

    calcRealCoefficient();
    allEnable(false);

    Serial0.println("\n========== 校准完成！==========");
}

// 读取全部4个电机当前角度
void readAllMotorAngle() {
    for (int m = 1; m <= 4; m++) {
        motorAngle[m] = get_pos(m);
    }
}

void calcRealCoefficient() {
    Serial0.println("\n==== 计算每个电机真实角度/毫米系数 ====");

    for (int m = 1; m <= 4; m++) {
        float total_deg = 0;
        float total_mm = 0;
        int count = 0;

        for (int p = 1; p <= 4; p++) {
            if (p == m) continue;

            float dx = caliX[p] - caliX[m];
            float dy = caliY[p] - caliY[m];
            float mm = sqrt(dx * dx + dy * dy);
            float deg = caliAngle[p][m];

            total_deg += fabs(deg);
            total_mm += mm;
            count++;
        }

        float coeffAbs = deg2mm_theory;
        if (count > 0 && total_mm > 0) {
            coeffAbs = total_deg / total_mm;
        }

        realDeg2mm[m] = coeffAbs * motorAngleSign[m];
    }

    for (int m = 1; m <= 4; m++) {
        Serial0.printf("电机%d 真实系数：%.3f °/mm\n", m, realDeg2mm[m]);
    }
    Serial0.println("=====================================\n");
}

// 坐标(X,Y) → 电机角度（使用校准后真实系数）
void moveToXY(float X, float Y) {
    ropeLength[1] = sqrt(X * X + Y * Y);
    ropeLength[2] = sqrt(X * X + (L0 - Y) * (L0 - Y));
    ropeLength[3] = sqrt((L0 - X) * (L0 - X) + (L0 - Y) * (L0 - Y));
    ropeLength[4] = sqrt((L0 - X) * (L0 - X) + Y * Y);

    float ang1 = ropeLength[1] * realDeg2mm[1];
    float ang2 = ropeLength[2] * realDeg2mm[2];
    float ang3 = ropeLength[3] * realDeg2mm[3];
    float ang4 = ropeLength[4] * realDeg2mm[4];

    Serial0.printf("目标(%.0f,%.0f) | M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n",
                   X, Y, ang1, ang2, ang3, ang4);

    allEnable(true);
    move_motor(1, ang1);
    move_motor(2, ang2);
    move_motor(3, ang3);
    move_motor(4, ang4);
}

// 调试指令扩展
bool parseNewCommand(String s) {
    s.trim();
    if (s == "home") {
        autoCalibrateHome();
        return true;
    }

    if (s.startsWith("goto")) {
        int sp = s.indexOf(' ');
        int sp2 = s.indexOf(' ', sp + 1);
        if (sp != -1 && sp2 != -1) {
            float X = s.substring(sp + 1, sp2).toFloat();
            float Y = s.substring(sp2 + 1).toFloat();
            moveToXY(X, Y);
            return true;
        }
    }

    return false;
}

// ------------------- 初始化 -------------------
void setup() {
    Serial0.begin(115200);
    uart2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    delay(100);
    Serial0.println("===== 全功能电机调试器 =====");
    Serial0.println("默认模式：单电机独立测试，输入M切换全体同步");
    show_help();
}

// ------------------- 主循环 -------------------
void loop() {
    static String serialLine;

    while (Serial0.available()) {
        char ch = (char)Serial0.read();

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            String str = serialLine;
            serialLine = "";
            str.trim();

            if (str.length() == 0) {
                continue;
            }

            if (!parseNewCommand(str)) {
                parse_command(str);
            }
            continue;
        }

        serialLine += ch;
        if (serialLine.length() > 96) {
            serialLine = "";
            Serial0.println("指令过长，已丢弃");
        }
    }
}
