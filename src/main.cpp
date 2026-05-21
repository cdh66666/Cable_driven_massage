#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "rice_preset.h"

// PlatformIO 的 ESP32 Arduino core 使用 Serial 作为默认调试串口。
#ifndef Serial0
#define Serial0 Serial
#endif

// 硬件状态：四个电机共同拉着一个小球，电机分布在左下、左上、右上、右下；
// 对应 (x,y) 坐标为 (0,0), (0,15cm), (15cm,15cm), (15cm,0)。

// PlatformIO/C++ 不会像 Arduino IDE 一样自动生成函数声明。
void readAllMotorAngle();
void calcRealCoefficient();
void loadCalibration();
void saveCalibration();
void printCoefficients();
void drawCenterCircles();
void drawSquareTest();
void drawFigureEightTest();
void drawSpiralTest();
void drawShapeTests();
void setupWebPortal();
void handleWebPortal();
void handleWebPlayback();

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
const uint16_t moveMaxCurrent = 2000;
const int moveSpeedDps = 2000;

size_t appendMoveCommand(uint8_t *buf, size_t idx, uint8_t addr, float angle) {
    uint8_t dir = (angle < 0) ? 0x01 : 0x00;
    uint32_t pos_val = (uint32_t)lroundf(fabs(angle) * 10.0f);
    uint16_t speed_val = moveSpeedDps * 10;

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
    buf[idx++] = (moveMaxCurrent >> 8) & 0xFF;
    buf[idx++] = moveMaxCurrent & 0xFF;
    buf[idx++] = 0x6B;

    return idx;
}

void move_motor(uint8_t addr, float angle) {
    uint8_t buf[32];
    size_t idx = appendMoveCommand(buf, 0, addr, angle);

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
    Serial0.println("coef  show saved/current coefficients");
    Serial0.println("circles  draw fine expanding circles around (75,75)");
    Serial0.println("square / eight / spiral / shapes  draw extra smooth test shapes");
    Serial0.println("ricepreset  load built-in 10000-point rice preset without moving");
    Serial0.println("wifi  drawing portal: SSID MassageDraw, no password");
    Serial0.println("reboot  restart board and reload saved calibration");
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
const float defaultDeg2mm[5] = {0, -4.3, 4.3, -4.3, 4.3};
const int8_t motorAngleSign[5] = {0, -1, 1, -1, 1};
const char *calibrationNamespace = "motorcal";
const uint32_t calibrationMagic = 0x43414C31; // "CAL1"
const float calibrationCoeffMinAbs = 4.00f;
const float calibrationCoeffMaxAbs = 4.70f;
const float calibrationCoeffMaxSpread = 0.15f;

// 理论系数（校准后会被实测值覆盖）
const float deg2mm_theory = 3.14159f * 13.5f / 360.0f;

// 4个校准点坐标
const float caliX[5] = {0, 0, 0, 150, 150};
const float caliY[5] = {0, 0, 150, 150, 0};
const int defaultCalibrationBaseCurrent = 80;
const int defaultCalibrationPullCurrent = 900;
int calibrationBaseCurrent = defaultCalibrationBaseCurrent;
int calibrationPullCurrent = defaultCalibrationPullCurrent;
const uint16_t calibrationPullMs = 3500;

bool coefficientsAreValid(const float coeffs[5]) {
    float minAbs = 1000.0f;
    float maxAbs = 0.0f;

    for (int m = 1; m <= 4; m++) {
        float absCoeff = fabs(coeffs[m]);
        if (absCoeff < calibrationCoeffMinAbs || absCoeff > calibrationCoeffMaxAbs) {
            return false;
        }
        if ((coeffs[m] > 0 && motorAngleSign[m] < 0) ||
            (coeffs[m] < 0 && motorAngleSign[m] > 0)) {
            return false;
        }
        minAbs = min(minAbs, absCoeff);
        maxAbs = max(maxAbs, absCoeff);
    }

    return (maxAbs - minAbs) <= calibrationCoeffMaxSpread;
}

void resetCalibrationToDefaults() {
    for (int m = 1; m <= 4; m++) {
        realDeg2mm[m] = defaultDeg2mm[m];
    }
}

void printCoefficients() {
    Serial0.println("Current coefficients:");
    for (int m = 1; m <= 4; m++) {
        Serial0.printf("M%d: %.3f deg/mm\n", m, realDeg2mm[m]);
    }
}

void saveCalibration() {
    Preferences prefs;
    if (!prefs.begin(calibrationNamespace, false)) {
        Serial0.println("Calibration save failed: NVS open failed");
        return;
    }

    prefs.putUInt("magic", calibrationMagic);
    for (int m = 1; m <= 4; m++) {
        char key[4];
        snprintf(key, sizeof(key), "m%d", m);
        prefs.putFloat(key, realDeg2mm[m]);
    }
    prefs.end();

    Serial0.println("Calibration saved to flash.");
}

void loadCalibration() {
    Preferences prefs;
    if (!prefs.begin(calibrationNamespace, true)) {
        Serial0.println("Calibration load skipped: NVS open failed");
        return;
    }

    uint32_t magic = prefs.getUInt("magic", 0);
    if (magic != calibrationMagic) {
        prefs.end();
        resetCalibrationToDefaults();
        Serial0.println("No saved calibration, using defaults.");
        printCoefficients();
        return;
    }

    for (int m = 1; m <= 4; m++) {
        char key[4];
        snprintf(key, sizeof(key), "m%d", m);
        realDeg2mm[m] = prefs.getFloat(key, realDeg2mm[m]);
    }
    prefs.end();

    if (!coefficientsAreValid(realDeg2mm)) {
        resetCalibrationToDefaults();
        Serial0.println("Saved calibration is invalid, using defaults.");
        printCoefficients();
        return;
    }

    Serial0.println("Loaded calibration from flash.");
    printCoefficients();
}

// 新版四点自动校准（左下→左上→右上→右下）
void autoCalibrateHome() {
    Serial0.println("\n========== 开始 四点自动校准 ==========");
    Serial0.println("顺序：左下(0,0) → 左上(0,150) → 右上(150,150) → 右下(150,0)");

    allEnable(true);
    delay(300);

    Serial0.println("\n步骤1：所有电机 100mA 预拉紧");
    for (int i = 0; i < motorTotal; i++) {
        int addr = allMotorList[i];
        int cur = fixTorqueDir(addr, calibrationBaseCurrent);
        torque_motor(addr, cur);
    }
    delay(1200);

    for (int point = 1; point <= 4; point++) {
        Serial0.printf("\n===== 拉到第 %d 个点：(%.0f, %.0f) ====\n",
                       point, caliX[point], caliY[point]);

        int pullMotor = point;

        for (int i = 0; i < motorTotal; i++) {
            int addr = allMotorList[i];
            int cur = fixTorqueDir(addr, calibrationBaseCurrent);
            torque_motor(addr, cur);
        }

        delay(200);

        int cur = fixTorqueDir(pullMotor, calibrationPullCurrent);
        torque_motor(pullMotor, cur);
        Serial0.printf("Pull M%d to its corner: current=%dmA wait=%ums\n",
                       pullMotor, calibrationPullCurrent, calibrationPullMs);
        delay(calibrationPullMs);

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

        cur = fixTorqueDir(pullMotor, calibrationBaseCurrent);
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
    float newDeg2mm[5] = {0};

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
            if (p < m) {
                deg -= caliAngle[m][m];
            }

            Serial0.printf("Coeff sample M%d point%d: raw=%.1f zero=%.1f used=%.1f mm=%.1f\n",
                           m, p, caliAngle[p][m], caliAngle[m][m], deg, mm);

            total_deg += fabs(deg);
            total_mm += mm;
            count++;
        }

        float coeffAbs = deg2mm_theory;
        if (count > 0 && total_mm > 0) {
            coeffAbs = total_deg / total_mm;
        }

        newDeg2mm[m] = coeffAbs * motorAngleSign[m];
    }

    if (!coefficientsAreValid(newDeg2mm)) {
        Serial0.println("Calibration result invalid, keeping previous coefficients and not saving.");
        Serial0.printf("Expected abs range %.2f..%.2f deg/mm, max spread %.2f deg/mm\n",
                       calibrationCoeffMinAbs, calibrationCoeffMaxAbs, calibrationCoeffMaxSpread);
        Serial0.printf("Candidate: M1=%.3f M2=%.3f M3=%.3f M4=%.3f\n",
                       newDeg2mm[1], newDeg2mm[2], newDeg2mm[3], newDeg2mm[4]);
        printCoefficients();
        return;
    }

    for (int m = 1; m <= 4; m++) {
        realDeg2mm[m] = newDeg2mm[m];
    }

    for (int m = 1; m <= 4; m++) {
        Serial0.printf("电机%d 真实系数：%.3f °/mm\n", m, realDeg2mm[m]);
    }
    Serial0.println("=====================================\n");
    saveCalibration();
}

struct MotorTargets {
    float m1;
    float m2;
    float m3;
    float m4;
};

const float pathSegmentMm = 1.5f;        // About 10x finer than the old 24-point 60 mm circle.
const uint16_t pathPointDelayMs = 20;    // Keep updates frequent instead of pausing at each vertex.
const uint16_t settleDelayMs = 250;

float currentX = L0 * 0.5f;
float currentY = L0 * 0.5f;
bool currentXYValid = false;

MotorTargets calcMotorTargets(float X, float Y) {
    ropeLength[1] = sqrt(X * X + Y * Y);
    ropeLength[2] = sqrt(X * X + (L0 - Y) * (L0 - Y));
    ropeLength[3] = sqrt((L0 - X) * (L0 - X) + (L0 - Y) * (L0 - Y));
    ropeLength[4] = sqrt((L0 - X) * (L0 - X) + Y * Y);

    MotorTargets target;
    target.m1 = ropeLength[1] * realDeg2mm[1];
    target.m2 = ropeLength[2] * realDeg2mm[2];
    target.m3 = ropeLength[3] * realDeg2mm[3];
    target.m4 = ropeLength[4] * realDeg2mm[4];
    return target;
}

void moveAllMotors(const MotorTargets &target, bool verbose) {
    uint8_t buf[80];
    size_t idx = 0;
    idx = appendMoveCommand(buf, idx, 1, target.m1);
    idx = appendMoveCommand(buf, idx, 2, target.m2);
    idx = appendMoveCommand(buf, idx, 3, target.m3);
    idx = appendMoveCommand(buf, idx, 4, target.m4);
    send_multi_motor(buf, idx);

    if (verbose) {
        Serial0.printf("M1:%.1f M2:%.1f M3:%.1f M4:%.1f\n",
                       target.m1, target.m2, target.m3, target.m4);
    }
}

// 坐标(X,Y) → 电机角度（使用校准后真实系数）
void moveToXY(float X, float Y, bool ensureEnabled = true, bool verbose = true) {
    X = constrain(X, 0.0f, L0);
    Y = constrain(Y, 0.0f, L0);

    MotorTargets target = calcMotorTargets(X, Y);

    if (verbose) {
        Serial0.printf("目标(%.1f,%.1f) | ", X, Y);
    }

    if (ensureEnabled) {
        allEnable(true);
    }

    moveAllMotors(target, verbose);
    currentX = X;
    currentY = Y;
    currentXYValid = true;
}

void lineToXY(float toX, float toY, float segmentMm = pathSegmentMm, uint16_t delayMs = pathPointDelayMs) {
    if (!currentXYValid) {
        moveToXY(toX, toY, false, false);
        delay(delayMs);
        return;
    }

    float fromX = currentX;
    float fromY = currentY;
    float dx = toX - fromX;
    float dy = toY - fromY;
    float distance = sqrt(dx * dx + dy * dy);
    int steps = max(1, (int)ceil(distance / segmentMm));

    for (int step = 1; step <= steps; step++) {
        float t = (float)step / steps;
        moveToXY(fromX + dx * t, fromY + dy * t, false, false);
        delay(delayMs);
    }
}

void beginShapeTest(const char *name) {
    Serial0.printf("\n%s start. segment=%.1f mm delay=%u ms\n", name, pathSegmentMm, pathPointDelayMs);
    allEnable(true);
    moveToXY(L0 * 0.5f, L0 * 0.5f, false, true);
    delay(settleDelayMs);
}

void endShapeTest(const char *name) {
    lineToXY(L0 * 0.5f, L0 * 0.5f);
    delay(settleDelayMs);
    allEnable(false);
    Serial0.printf("%s done.\n", name);
}

void drawCenterCircles() {
    const float centerX = L0 * 0.5f;
    const float centerY = L0 * 0.5f;
    const float radii[] = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f};
    const int radiusCount = sizeof(radii) / sizeof(radii[0]);

    beginShapeTest("Center circles");

    for (int r = 0; r < radiusCount; r++) {
        float radius = radii[r];
        int stepsPerCircle = max(240, (int)ceil((2.0f * PI * radius) / pathSegmentMm));
        Serial0.printf("Circle radius: %.0f mm, points: %d\n", radius, stepsPerCircle);

        lineToXY(centerX + radius, centerY);

        for (int step = 0; step <= stepsPerCircle; step++) {
            float theta = 2.0f * PI * step / stepsPerCircle;
            float x = centerX + radius * cos(theta);
            float y = centerY + radius * sin(theta);
            moveToXY(x, y, false, false);
            delay(pathPointDelayMs);
        }
    }

    endShapeTest("Center circles");
}

void drawSquareTest() {
    const float left = 30.0f;
    const float right = 120.0f;
    const float bottom = 30.0f;
    const float top = 120.0f;

    beginShapeTest("Square");
    Serial0.println("Square points: (30,30) -> (120,30) -> (120,120) -> (30,120)");
    lineToXY(left, bottom);
    lineToXY(right, bottom);
    lineToXY(right, top);
    lineToXY(left, top);
    lineToXY(left, bottom);
    endShapeTest("Square");
}

void drawFigureEightTest() {
    const float centerX = L0 * 0.5f;
    const float centerY = L0 * 0.5f;
    const float ampX = 45.0f;
    const float ampY = 32.0f;
    const int steps = 360;

    beginShapeTest("Figure eight");
    lineToXY(centerX, centerY);

    for (int step = 0; step <= steps; step++) {
        float theta = 2.0f * PI * step / steps;
        float x = centerX + ampX * sin(theta);
        float y = centerY + ampY * sin(theta) * cos(theta);
        moveToXY(x, y, false, false);
        delay(pathPointDelayMs);
    }

    endShapeTest("Figure eight");
}

void drawSpiralTest() {
    const float centerX = L0 * 0.5f;
    const float centerY = L0 * 0.5f;
    const float maxRadius = 58.0f;
    const int turns = 3;
    const int steps = 540;

    beginShapeTest("Spiral");

    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float theta = 2.0f * PI * turns * t;
        float radius = maxRadius * t;
        float x = centerX + radius * cos(theta);
        float y = centerY + radius * sin(theta);
        moveToXY(x, y, false, false);
        delay(pathPointDelayMs);
    }

    endShapeTest("Spiral");
}

void drawShapeTests() {
    Serial0.println("\nRunning shape test suite: circles, square, figure eight, spiral.");
    drawCenterCircles();
    delay(500);
    drawSquareTest();
    delay(500);
    drawFigureEightTest();
    delay(500);
    drawSpiralTest();
    Serial0.println("Shape test suite done.");
}

// ====================== WiFi drawing portal ======================
const char *apSsid = "MassageDraw";
const byte dnsPort = 53;
const IPAddress apIP(192, 168, 4, 1);
const IPAddress apGateway(192, 168, 4, 1);
const IPAddress apSubnet(255, 255, 255, 0);

DNSServer dnsServer;
WebServer webServer(80);

struct WebPathPoint {
    uint16_t x10;
    uint16_t y10;
    uint8_t draw;
};

const int maxWebPathPoints = 12000;
const float webResampleMm = 0.15f;
const float webTravelResampleMm = 1.5f;
const uint16_t minWebStepDelayMs = 8;
const uint16_t maxWebStepDelayMs = 80;

WebPathPoint webPath[maxWebPathPoints];
int webPathPointCount = 0;
int webStrokeCount = 0;
int webPlaybackIndex = 0;
bool webPlaybackActive = false;
bool webLoopPlayback = true;
uint8_t webSpeedPercent = 50;
uint8_t webTravelSpeedPercent = 15;
uint32_t nextWebStepAt = 0;
bool webTravelActive = false;
float webTravelFromX = 0.0f;
float webTravelFromY = 0.0f;
float webTravelToX = 0.0f;
float webTravelToY = 0.0f;
int webTravelStep = 0;
int webTravelSteps = 0;
bool webCalibrationRequested = false;
bool webCalibrationRunning = false;
bool webCalibrationDone = false;

const char indexHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Massage Draw</title>
<style>
*{box-sizing:border-box}
body{margin:0;min-height:100vh;font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#f2f3f5;color:#111;display:flex;align-items:flex-start;justify-content:center}
main{width:min(92vw,560px);padding:calc(42px + env(safe-area-inset-top)) 0 28px}
h1{text-align:center;font-size:32px;line-height:1.25;margin:0 0 22px;font-weight:750}
#pad{display:block;width:100%;aspect-ratio:1/1;background:#fff;border:2px solid #111;touch-action:none}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:14px}
.controls.extra{grid-template-columns:1fr 1fr;margin-top:12px}
button,.upload{height:48px;border:0;border-radius:8px;font-size:18px;font-weight:650;color:#fff;background:#111;display:flex;align-items:center;justify-content:center;text-align:center}
button.stop{background:#9b1c1c}
button.clear{background:#4b5563}
button.mode{background:#115e59}
button.reset{background:#1d4ed8}
button.calib{background:#7c2d12}
button.image{background:#374151}
.speed{margin-top:16px;display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center;font-size:15px}
input[type=range]{width:100%;accent-color:#111}
input[type=file]{position:absolute;left:-9999px;width:1px;height:1px;opacity:0}
#status{min-height:22px;margin-top:10px;font-size:14px;color:#333}
#coef{min-height:42px;margin-top:6px;font-size:13px;line-height:1.45;color:#111;white-space:pre-wrap}
</style>
</head>
<body>
<main>
<h1>轨迹绘制</h1>
<canvas id="pad"></canvas>
<div class="controls">
  <button id="start">开始</button>
  <button id="stop" class="stop">停止</button>
</div>
<div class="controls extra">
  <button id="clear" class="clear">清除画布</button>
  <button id="mode" class="mode">循环: 开</button>
  <button id="reset" class="reset">重置参数</button>
  <button id="calibrate" class="calib">校准参数</button>
  <button id="riceBtn" class="image">询问笔仙</button>
</div>
<input id="imageInput" type="file" accept="image/*">
<div class="speed">
  <span>绘制</span>
  <input id="speed" type="range" min="1" max="100" value="50">
  <span id="speedText">50</span>
</div>
<div class="speed">
  <span>移动</span>
  <input id="travelSpeed" type="range" min="1" max="100" value="15">
  <span id="travelSpeedText">15</span>
</div>
<div class="speed">
  <span>保持</span>
  <input id="baseTorque" type="range" min="20" max="300" step="5" value="80">
  <span id="baseTorqueText">80</span>
</div>
<div class="speed">
  <span>校准</span>
  <input id="pullTorque" type="range" min="300" max="1500" step="10" value="900">
  <span id="pullTorqueText">900</span>
</div>
<div id="status"></div>
<div id="coef"></div>
</main>
<script>
const L0=150;
const pad=document.getElementById('pad');
const ctx=pad.getContext('2d');
const statusEl=document.getElementById('status');
const speed=document.getElementById('speed');
const speedText=document.getElementById('speedText');
const travelSpeed=document.getElementById('travelSpeed');
const travelSpeedText=document.getElementById('travelSpeedText');
const baseTorque=document.getElementById('baseTorque');
const baseTorqueText=document.getElementById('baseTorqueText');
const pullTorque=document.getElementById('pullTorque');
const pullTorqueText=document.getElementById('pullTorqueText');
const modeBtn=document.getElementById('mode');
const coefEl=document.getElementById('coef');
let drawing=false;
let strokes=[];
let activeStroke=null;
let loopMode=true;
let ricePresetSelected=false;
const ricePresetPointCount=10000;
let cssSize=0;

function fitCanvas(){
  const r=pad.getBoundingClientRect();
  const dpr=Math.max(1,window.devicePixelRatio||1);
  cssSize=r.width;
  pad.width=Math.round(r.width*dpr);
  pad.height=Math.round(r.width*dpr);
  ctx.setTransform(dpr,0,0,dpr,0,0);
  redraw();
}

function redraw(){
  ctx.fillStyle='#fff';
  ctx.fillRect(0,0,cssSize,cssSize);
  ctx.lineWidth=3;
  ctx.lineCap='round';
  ctx.lineJoin='round';
  ctx.strokeStyle='#000';
  for(const stroke of strokes){
    if(stroke.length<2)continue;
    ctx.beginPath();
    for(let i=0;i<stroke.length;i++){
      const p=stroke[i];
      const x=p.x/L0*cssSize;
      const y=(1-p.y/L0)*cssSize;
      if(i===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
}

function toPoint(e){
  const r=pad.getBoundingClientRect();
  const x=Math.min(Math.max(e.clientX-r.left,0),r.width);
  const y=Math.min(Math.max(e.clientY-r.top,0),r.height);
  return {x:x/r.width*L0,y:(1-y/r.height)*L0,px:x,py:y};
}

function addPoint(p){
  if(!activeStroke)return;
  const last=activeStroke[activeStroke.length-1];
  if(last){
    const lx=last.x/L0*cssSize, ly=(1-last.y/L0)*cssSize;
    if(Math.hypot(p.px-lx,p.py-ly)<2)return;
  }
  activeStroke.push({x:p.x,y:p.y});
  redraw();
}

pad.addEventListener('pointerdown',e=>{
  e.preventDefault();
  pad.setPointerCapture(e.pointerId);
  ricePresetSelected=false;
  drawing=true;
  activeStroke=[];
  strokes.push(activeStroke);
  addPoint(toPoint(e));
  statusEl.textContent='正在绘制第 '+strokes.length+' 笔...';
});
pad.addEventListener('pointermove',e=>{if(drawing){e.preventDefault();addPoint(toPoint(e));}});
function endDraw(){
  if(!drawing)return;
  drawing=false;
  if(activeStroke && activeStroke.length<2){strokes.pop();}
  activeStroke=null;
  redraw();
  statusEl.textContent='已记录 '+strokes.length+' 笔，'+pointCount()+' 个点';
}
pad.addEventListener('pointerup',endDraw);
pad.addEventListener('pointercancel',endDraw);

function pointCount(){
  return strokes.reduce((sum,s)=>sum+s.length,0);
}

function pathBody(){
  return strokes
    .filter(s=>s.length>=2)
    .map(s=>s.map(p=>`${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(';'))
    .join('\n');
}

async function post(url,body){
  const opt={method:'POST'};
  if(body!==undefined){opt.headers={'Content-Type':'text/plain'};opt.body=body;}
  const res=await fetch(url,opt);
  return await res.text();
}

function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms));}

async function fetchStatus(timeout=2500){
  const controller=new AbortController();
  const timer=setTimeout(()=>controller.abort(),timeout);
  try{
    const res=await fetch('/status',{cache:'no-store',signal:controller.signal});
    return await res.json();
  }finally{
    clearTimeout(timer);
  }
}

function renderCoef(s){
  if(!s || !Number.isFinite(s.m1))return;
  coefEl.textContent=
    `M1: ${s.m1.toFixed(3)} deg/mm  M2: ${s.m2.toFixed(3)} deg/mm\n`+
    `M3: ${s.m3.toFixed(3)} deg/mm  M4: ${s.m4.toFixed(3)} deg/mm`;
}

async function sendMode(){
  modeBtn.textContent=loopMode?'循环: 开':'循环: 关';
  return await post('/mode?loop='+(loopMode?1:0));
}

function setModeButton(){
  modeBtn.textContent=loopMode?'循环: 开':'循环: 关';
}

async function loadStatus(){
  try{
    const s=await fetchStatus();
    if(Number.isFinite(s.speed)){
      speed.value=s.speed;
      speedText.textContent=s.speed;
    }
    if(Number.isFinite(s.travelSpeed)){
      travelSpeed.value=s.travelSpeed;
      travelSpeedText.textContent=s.travelSpeed;
    }
    if(Number.isFinite(s.baseTorque)){
      baseTorque.value=s.baseTorque;
      baseTorqueText.textContent=s.baseTorque;
    }
    if(Number.isFinite(s.pullTorque)){
      pullTorque.value=s.pullTorque;
      pullTorqueText.textContent=s.pullTorque;
    }
    loopMode=!!s.loop;
    setModeButton();
    renderCoef(s);
    statusEl.textContent='已连接';
  }catch(e){
    statusEl.textContent='已打开画板';
  }
}

document.getElementById('start').onclick=async()=>{
  if(!ricePresetSelected && strokes.filter(s=>s.length>=2).length<1){statusEl.textContent='请先在白色框里画一条轨迹';return;}
  statusEl.textContent='上传轨迹...';
  await sendMode();
  const saved=ricePresetSelected ? await post('/ricePreset') : await post('/path',pathBody());
  const started=await post('/start');
  statusEl.textContent=`${saved} ${started}`;
};

document.getElementById('stop').onclick=async()=>{
  statusEl.textContent=await post('/stop');
};

document.getElementById('clear').onclick=async()=>{
  strokes=[];
  activeStroke=null;
  drawing=false;
  ricePresetSelected=false;
  redraw();
  statusEl.textContent=await post('/clear');
};

modeBtn.onclick=async()=>{
  loopMode=!loopMode;
  statusEl.textContent=await sendMode();
};

document.getElementById('reset').onclick=async()=>{
  statusEl.textContent=await post('/resetSettings');
  await loadStatus();
};

async function pollCalibration(){
  const end=Date.now()+140000;
  while(Date.now()<end){
    await sleep(1800);
    try{
      const s=await fetchStatus(1800);
      renderCoef(s);
      if(!s.calibrating){
        statusEl.textContent=s.calibrated?'校准完成，参数已保存':'校准未完成';
        return;
      }
    }catch(e){}
  }
  statusEl.textContent='校准仍在进行，请稍后刷新状态';
}

document.getElementById('calibrate').onclick=async()=>{
  statusEl.textContent='校准中...';
  coefEl.textContent='';
  const msg=await post('/calibrate');
  statusEl.textContent=msg;
  pollCalibration();
};

function addLinePoints(out,a,b,steps){
  for(let i=1;i<=steps;i++){
    const t=i/steps;
    out.push({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t});
  }
}

function addBezierPoints(out,p0,p1,p2,p3,steps){
  for(let i=1;i<=steps;i++){
    const t=i/steps;
    const u=1-t;
    out.push({
      x:u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,
      y:u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y
    });
  }
}

function addLeafLoop(out,base,tip,width,steps){
  const last=out[out.length-1]||base;
  addLinePoints(out,last,base,4);
  const dx=tip.x-base.x;
  const dy=tip.y-base.y;
  const d=Math.hypot(dx,dy)||1;
  const nx=-dy/d;
  const ny=dx/d;
  const mid={x:(base.x+tip.x)*0.5,y:(base.y+tip.y)*0.5};
  const sideA={x:mid.x+nx*width,y:mid.y+ny*width};
  const sideB={x:mid.x-nx*width,y:mid.y-ny*width};
  addBezierPoints(out,base,sideA,sideA,tip,steps);
  addBezierPoints(out,tip,sideB,sideB,base,steps);
}

function addDropLoop(out,top,bottom,width){
  const last=out[out.length-1]||top;
  addLinePoints(out,last,top,5);
  const left={x:(top.x+bottom.x)*0.5-width,y:(top.y+bottom.y)*0.5};
  const right={x:(top.x+bottom.x)*0.5+width,y:(top.y+bottom.y)*0.5};
  addBezierPoints(out,top,left,{x:bottom.x-width*0.7,y:bottom.y+2},bottom,10);
  addBezierPoints(out,bottom,{x:bottom.x+width*0.7,y:bottom.y+2},right,top,10);
}

function addStem(out,points){
  let last=out[out.length-1]||points[0];
  addLinePoints(out,last,points[0],4);
  for(let i=1;i<points.length;i++){
    addLinePoints(out,points[i-1],points[i],5);
  }
}

function addCurve(out,p0,p1,p2,p3,steps){
  const last=out[out.length-1]||p0;
  addLinePoints(out,last,p0,4);
  addBezierPoints(out,p0,p1,p2,p3,steps);
}

function curveStroke(p0,p1,p2,p3,steps){
  const out=[p0];
  addBezierPoints(out,p0,p1,p2,p3,steps);
  return out;
}

function leafStroke(base,tip,width,steps){
  const out=[base];
  const dx=tip.x-base.x;
  const dy=tip.y-base.y;
  const d=Math.hypot(dx,dy)||1;
  const nx=-dy/d;
  const ny=dx/d;
  const mid={x:(base.x+tip.x)*0.5,y:(base.y+tip.y)*0.5};
  const sideA={x:mid.x+nx*width,y:mid.y+ny*width};
  const sideB={x:mid.x-nx*width,y:mid.y-ny*width};
  addBezierPoints(out,base,sideA,sideA,tip,steps);
  addBezierPoints(out,tip,sideB,sideB,base,steps);
  return out;
}

function dropStroke(top,bottom,width,steps){
  const out=[top];
  const left={x:(top.x+bottom.x)*0.5-width,y:(top.y+bottom.y)*0.5};
  const right={x:(top.x+bottom.x)*0.5+width,y:(top.y+bottom.y)*0.5};
  addBezierPoints(out,top,left,{x:bottom.x-width*0.7,y:bottom.y+2},bottom,steps);
  addBezierPoints(out,bottom,{x:bottom.x+width*0.7,y:bottom.y+2},right,top,steps);
  return out;
}

function jitterStroke(stroke,amp){
  return stroke.map((p,i)=>{
    if(i===0||i===stroke.length-1)return p;
    const n=Math.sin(i*12.9898+p.x*0.37+p.y*0.71)*43758.5453;
    const f=n-Math.floor(n);
    const a=(f-0.5)*amp;
    return {x:Math.min(Math.max(p.x+a,0),L0),y:Math.min(Math.max(p.y-a*0.45,0),L0)};
  });
}

function densifyStroke(stroke,target){
  if(stroke.length<2)return stroke.slice();
  target=Math.max(2,target);
  const lengths=[];
  let total=0;
  for(let i=1;i<stroke.length;i++){
    const a=stroke[i-1], b=stroke[i];
    total+=Math.hypot(b.x-a.x,b.y-a.y);
    lengths.push(total);
  }
  const out=[stroke[0]];
  for(let i=1;i<target;i++){
    const dist=total*i/(target-1);
    let seg=0;
    while(seg<lengths.length-1&&lengths[seg]<dist)seg++;
    const prevLen=seg===0?0:lengths[seg-1];
    const span=Math.max(0.0001,lengths[seg]-prevLen);
    const t=(dist-prevLen)/span;
    const a=stroke[seg], b=stroke[seg+1];
    out.push({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t});
  }
  return out;
}

function balanceStrokePoints(strokes,target){
  const lens=strokes.map(s=>{
    let len=0;
    for(let i=1;i<s.length;i++)len+=Math.hypot(s[i].x-s[i-1].x,s[i].y-s[i-1].y);
    return Math.max(0.1,len);
  });
  const totalLen=lens.reduce((a,b)=>a+b,0);
  let used=0;
  const counts=lens.map((len,i)=>{
    const count=Math.max(2,Math.round(target*len/totalLen));
    used+=count;
    return count;
  });
  while(used!==target){
    let best=0;
    for(let i=1;i<counts.length;i++)if(lens[i]>lens[best])best=i;
    if(used<target){counts[best]++;used++;}
    else if(counts[best]>2){counts[best]--;used--;}
    else break;
  }
  return strokes.map((s,i)=>densifyStroke(s,counts[i]));
}

function riceEarStrokes(){
  return [[{"x":133.68,"y":122.82},{"x":132.76,"y":122.05},{"x":133.5,"y":120.81},{"x":133.49,"y":122.8},{"x":131.39,"y":123.62},{"x":128.75,"y":123.87},{"x":126.09,"y":124.06},{"x":123.42,"y":124.19},{"x":120.75,"y":124.32},{"x":118.08,"y":124.25},{"x":115.44,"y":123.88},{"x":112.96,"y":122.96},{"x":110.38,"y":122.36},{"x":108.88,"y":121.03},{"x":111.43,"y":121.34},{"x":114.06,"y":121.79},{"x":116.72,"y":121.98},{"x":119.39,"y":121.85},{"x":122.05,"y":121.66},{"x":124.7,"y":121.33},{"x":127.34,"y":120.96},{"x":129.97,"y":120.52},{"x":131.91,"y":121.77},{"x":130.69,"y":123.98},{"x":128.91,"y":125.98},{"x":127.12,"y":127.99},{"x":125.34,"y":129.99},{"x":123.55,"y":131.99},{"x":123.65,"y":133.93},{"x":123.34,"y":135.0},{"x":120.85,"y":134.7},{"x":118.8,"y":133.0},{"x":116.65,"y":131.43},{"x":114.4,"y":129.98},{"x":112.1,"y":128.62},{"x":112.98,"y":127.68},{"x":115.09,"y":127.32},{"x":117.32,"y":128.79},{"x":119.45,"y":130.41},{"x":121.54,"y":132.07},{"x":121.59,"y":132.26},{"x":119.38,"y":130.74},{"x":117.17,"y":129.23},{"x":114.96,"y":127.71},{"x":112.8,"y":126.62},{"x":110.34,"y":127.04},{"x":107.86,"y":126.25},{"x":109.19,"y":126.04},{"x":111.75,"y":126.48},{"x":111.86,"y":124.28},{"x":109.3,"y":123.86},{"x":109.29,"y":123.18},{"x":111.87,"y":123.74},{"x":113.23,"y":125.88},{"x":111.06,"y":125.69},{"x":110.02,"y":124.57},{"x":110.65,"y":125.2},{"x":108.13,"y":124.28},{"x":106.34,"y":125.69},{"x":103.84,"y":125.23},{"x":101.26,"y":124.66},{"x":98.8,"y":123.78},{"x":99.81,"y":121.77},{"x":98.39,"y":119.53},{"x":98.37,"y":117.93},{"x":101.0,"y":118.31},{"x":103.6,"y":118.92},{"x":106.2,"y":119.51},{"x":108.05,"y":121.18},{"x":107.57,"y":123.78},{"x":105.09,"y":123.52},{"x":102.47,"y":122.96},{"x":99.85,"y":122.39},{"x":97.5,"y":122.07},{"x":95.33,"y":120.53},{"x":93.56,"y":118.83},{"x":91.82,"y":116.89},{"x":90.11,"y":114.84},{"x":88.85,"y":112.52},{"x":89.22,"y":110.47},{"x":91.63,"y":111.26},{"x":93.31,"y":113.34},{"x":94.88,"y":115.5},{"x":96.25,"y":117.79},{"x":97.47,"y":120.17},{"x":98.62,"y":122.16},{"x":95.95,"y":121.9},{"x":93.33,"y":121.47},{"x":90.75,"y":120.8},{"x":88.84,"y":119.59},{"x":91.34,"y":120.0},{"x":93.83,"y":120.86},{"x":93.76,"y":120.75},{"x":91.7,"y":119.16},{"x":89.16,"y":118.35},{"x":90.44,"y":117.63},{"x":91.95,"y":119.21},{"x":89.3,"y":118.78},{"x":87.76,"y":119.98},{"x":86.08,"y":121.66},{"x":83.48,"y":121.36},{"x":83.83,"y":118.82},{"x":83.95,"y":116.15},{"x":83.89,"y":113.48},{"x":83.77,"y":110.81},{"x":83.69,"y":108.13},{"x":83.76,"y":105.46},{"x":83.95,"y":102.8},{"x":84.92,"y":103.59},{"x":85.77,"y":106.12},{"x":86.68,"y":108.63},{"x":87.48,"y":111.17},{"x":88.01,"y":113.78},{"x":88.12,"y":116.44},{"x":87.46,"y":118.53},{"x":84.79,"y":118.36},{"x":82.43,"y":118.75},{"x":79.78,"y":118.7},{"x":78.74,"y":117.98},{"x":81.36,"y":117.64},{"x":82.84,"y":117.23},{"x":80.36,"y":116.74},{"x":77.76,"y":117.34},{"x":75.14,"y":117.68},{"x":72.54,"y":117.15},{"x":69.96,"y":116.56},{"x":69.46,"y":115.18},{"x":72.01,"y":114.4},{"x":74.65,"y":114.43},{"x":77.25,"y":115.03},{"x":79.81,"y":115.82},{"x":82.11,"y":116.08},{"x":80.0,"y":114.81},{"x":78.26,"y":113.56},{"x":75.95,"y":112.26},{"x":74.05,"y":110.55},{"x":72.63,"y":108.3},{"x":71.25,"y":106.04},{"x":71.53,"y":105.16},{"x":73.95,"y":106.3},{"x":76.21,"y":107.73},{"x":78.35,"y":109.31},{"x":79.91,"y":111.48},{"x":80.98,"y":113.92},{"x":82.78,"y":115.7},{"x":82.76,"y":115.88},{"x":82.16,"y":113.55},{"x":82.92,"y":113.39},{"x":80.39,"y":113.65},{"x":77.71,"y":113.6},{"x":75.1,"y":113.39},{"x":73.26,"y":111.98},{"x":70.86,"y":110.8},{"x":68.38,"y":109.79},{"x":66.07,"y":108.44},{"x":64.24,"y":106.56},{"x":62.9,"y":104.27},{"x":62.1,"y":101.9},{"x":64.39,"y":103.26},{"x":66.71,"y":104.6},{"x":68.85,"y":106.18},{"x":70.59,"y":108.21},{"x":71.99,"y":110.49},{"x":74.0,"y":112.23},{"x":76.16,"y":113.54},{"x":73.56,"y":112.88},{"x":70.92,"y":112.56},{"x":68.29,"y":113.04},{"x":66.01,"y":113.95},{"x":63.73,"y":113.12},{"x":61.5,"y":112.02},{"x":59.39,"y":110.97},{"x":57.11,"y":110.0},{"x":59.59,"y":109.72},{"x":62.27,"y":109.77},{"x":64.92,"y":110.06},{"x":67.52,"y":110.66},{"x":70.05,"y":111.53},{"x":72.22,"y":112.67},{"x":70.06,"y":113.88},{"x":69.93,"y":113.45},{"x":70.95,"y":112.84},{"x":68.72,"y":111.36},{"x":66.49,"y":109.87},{"x":64.07,"y":108.93},{"x":61.4,"y":108.91},{"x":58.74,"y":108.75},{"x":56.24,"y":107.92},{"x":53.92,"y":106.63},{"x":51.58,"y":105.37},{"x":50.79,"y":104.53},{"x":53.46,"y":104.34},{"x":56.12,"y":104.35},{"x":58.7,"y":105.05},{"x":60.95,"y":106.49},{"x":63.13,"y":108.04},{"x":65.28,"y":109.05},{"x":63.48,"y":107.06},{"x":61.35,"y":105.52},{"x":59.6,"y":103.57},{"x":58.0,"y":101.43},{"x":56.81,"y":99.04},{"x":55.52,"y":96.76},{"x":57.84,"y":98.04},{"x":59.94,"y":99.68},{"x":61.12,"y":101.55},{"x":61.85,"y":104.09},{"x":63.02,"y":106.49},{"x":60.94,"y":105.2},{"x":58.7,"y":103.72},{"x":56.42,"y":102.38},{"x":54.2,"y":100.89},{"x":52.06,"y":99.29},{"x":50.38,"y":97.33},{"x":49.12,"y":95.15},{"x":47.72,"y":93.06},{"x":46.92,"y":90.79},{"x":49.12,"y":92.3},{"x":51.34,"y":93.79},{"x":53.34,"y":95.55},{"x":54.86,"y":97.75},{"x":55.95,"y":100.18},{"x":57.04,"y":102.62},{"x":54.61,"y":103.26},{"x":51.94,"y":103.32},{"x":49.32,"y":103.19},{"x":47.28,"y":101.75},{"x":45.07,"y":100.26},{"x":44.07,"y":98.83},{"x":46.73,"y":98.92},{"x":49.34,"y":99.51},{"x":51.8,"y":100.49},{"x":54.02,"y":101.99},{"x":55.38,"y":102.94},{"x":53.18,"y":101.41},{"x":50.99,"y":99.87},{"x":48.79,"y":98.34},{"x":46.64,"y":96.82},{"x":44.6,"y":95.09},{"x":42.59,"y":93.33},{"x":40.77,"y":91.37},{"x":39.09,"y":89.28},{"x":37.39,"y":87.22},{"x":35.66,"y":85.18},{"x":33.71,"y":83.4},{"x":31.5,"y":82.02},{"x":28.94,"y":81.32},{"x":26.89,"y":79.7},{"x":25.27,"y":77.6},{"x":24.06,"y":75.24},{"x":25.6,"y":75.27},{"x":28.01,"y":76.44},{"x":30.15,"y":78.04},{"x":31.85,"y":80.09},{"x":33.32,"y":82.32},{"x":35.18,"y":83.28},{"x":36.67,"y":85.48},{"x":38.29,"y":87.61},{"x":39.93,"y":89.72},{"x":41.64,"y":91.77},{"x":43.51,"y":93.68},{"x":45.49,"y":95.47},{"x":47.52,"y":97.22},{"x":47.3,"y":98.02},{"x":44.66,"y":97.95},{"x":42.22,"y":98.15},{"x":40.25,"y":96.55},{"x":38.25,"y":94.89},{"x":36.16,"y":93.22},{"x":33.98,"y":91.69},{"x":31.82,"y":90.13},{"x":29.61,"y":88.63},{"x":27.38,"y":87.16},{"x":25.04,"y":85.94},{"x":22.91,"y":84.52},{"x":20.41,"y":83.6},{"x":21.01,"y":83.33},{"x":23.61,"y":83.9},{"x":26.13,"y":84.77},{"x":28.55,"y":85.88},{"x":30.94,"y":87.07},{"x":33.23,"y":88.43},{"x":35.44,"y":89.95},{"x":37.62,"y":91.49},{"x":39.77,"y":93.09},{"x":41.88,"y":94.71},{"x":43.91,"y":96.41},{"x":46.17,"y":97.68},{"x":46.97,"y":95.51},{"x":47.61,"y":92.9},{"x":48.11,"y":90.32},{"x":46.35,"y":88.35},{"x":44.81,"y":86.18},{"x":43.37,"y":83.93},{"x":41.96,"y":81.67},{"x":40.64,"y":79.39},{"x":39.36,"y":77.04},{"x":38.15,"y":74.66},{"x":36.97,"y":72.26},{"x":35.97,"y":69.79},{"x":35.06,"y":67.28},{"x":34.18,"y":64.77},{"x":33.31,"y":62.24},{"x":34.0,"y":62.79},{"x":35.19,"y":64.89},{"x":36.02,"y":67.43},{"x":36.87,"y":69.96},{"x":37.84,"y":72.46},{"x":38.9,"y":74.91},{"x":40.1,"y":77.31},{"x":41.38,"y":79.66},{"x":42.76,"y":81.94},{"x":44.17,"y":84.22},{"x":45.64,"y":86.46},{"x":47.14,"y":88.66},{"x":48.7,"y":90.06},{"x":50.81,"y":88.4},{"x":52.92,"y":86.75},{"x":53.77,"y":84.88},{"x":53.4,"y":82.71},{"x":52.4,"y":80.22},{"x":51.48,"y":77.71},{"x":50.42,"y":75.26},{"x":49.66,"y":72.71},{"x":49.25,"y":70.36},{"x":48.21,"y":67.93},{"x":47.17,"y":65.48},{"x":46.83,"y":62.84},{"x":46.59,"y":60.38},{"x":47.09,"y":57.78},{"x":48.26,"y":57.77},{"x":49.33,"y":60.22},{"x":50.15,"y":62.76},{"x":50.69,"y":65.37},{"x":50.62,"y":68.03},{"x":49.99,"y":70.61},{"x":50.34,"y":73.24},{"x":51.13,"y":75.78},{"x":52.07,"y":78.29},{"x":53.03,"y":80.78},{"x":54.13,"y":83.22},{"x":55.29,"y":85.63},{"x":56.59,"y":87.97},{"x":58.04,"y":90.21},{"x":59.61,"y":92.37},{"x":59.6,"y":92.7},{"x":57.79,"y":90.74},{"x":56.18,"y":88.6},{"x":54.77,"y":86.34},{"x":53.26,"y":84.14},{"x":51.41,"y":82.27},{"x":49.41,"y":80.51},{"x":47.73,"y":78.47},{"x":46.59,"y":76.09},{"x":45.6,"y":73.64},{"x":45.52,"y":71.66},{"x":47.5,"y":73.45},{"x":49.2,"y":75.51},{"x":50.45,"y":77.87},{"x":51.42,"y":80.37},{"x":52.44,"y":82.84},{"x":50.52,"y":81.51},{"x":48.39,"y":79.88},{"x":46.26,"y":78.27},{"x":45.27,"y":78.07},{"x":46.15,"y":78.28},{"x":44.09,"y":80.0},{"x":42.04,"y":81.72},{"x":41.88,"y":83.73},{"x":41.0,"y":82.6},{"x":38.84,"y":82.04},{"x":36.19,"y":81.67},{"x":33.75,"y":81.0},{"x":32.29,"y":78.76},{"x":30.62,"y":76.68},{"x":28.84,"y":74.68},{"x":27.3,"y":72.49},{"x":25.95,"y":70.24},{"x":23.86,"y":68.66},{"x":22.11,"y":66.67},{"x":20.8,"y":64.35},{"x":19.69,"y":61.95},{"x":18.78,"y":59.46},{"x":20.87,"y":60.92},{"x":22.7,"y":62.88},{"x":24.19,"y":65.09},{"x":25.65,"y":67.26},{"x":26.71,"y":69.72},{"x":28.13,"y":71.57},{"x":29.21,"y":74.01},{"x":30.83,"y":76.01},{"x":30.24,"y":73.83},{"x":29.01,"y":71.48},{"x":28.56,"y":68.88},{"x":28.62,"y":66.23},{"x":29.07,"y":63.6},{"x":29.55,"y":60.99},{"x":30.24,"y":61.25},{"x":31.22,"y":63.73},{"x":32.24,"y":66.19},{"x":32.72,"y":68.81},{"x":32.47,"y":71.47},{"x":31.69,"y":74.02},{"x":31.98,"y":76.61},{"x":32.98,"y":79.09},{"x":34.2,"y":81.47},{"x":34.64,"y":84.11},{"x":35.18,"y":86.72},{"x":37.0,"y":88.53},{"x":38.66,"y":90.63},{"x":36.71,"y":89.44},{"x":34.8,"y":88.01},{"x":35.0,"y":86.4},{"x":33.98,"y":83.93},{"x":32.91,"y":81.47},{"x":31.85,"y":79.01},{"x":30.78,"y":76.55},{"x":29.71,"y":74.09},{"x":28.65,"y":71.63},{"x":27.58,"y":69.18},{"x":26.92,"y":66.6},{"x":26.5,"y":64.0},{"x":25.38,"y":61.57},{"x":24.64,"y":59.01},{"x":24.59,"y":56.35},{"x":25.02,"y":53.71},{"x":25.53,"y":51.09},{"x":26.82,"y":52.79},{"x":27.87,"y":55.23},{"x":28.5,"y":57.81},{"x":28.62,"y":60.46},{"x":27.99,"y":63.04},{"x":27.11,"y":65.55},{"x":27.34,"y":68.21},{"x":26.21,"y":66.07},{"x":25.11,"y":63.62},{"x":23.9,"y":61.24},{"x":23.03,"y":58.8},{"x":23.31,"y":56.27},{"x":23.66,"y":58.24},{"x":24.13,"y":60.87},{"x":24.54,"y":62.35},{"x":23.42,"y":59.91},{"x":22.1,"y":57.68},{"x":20.26,"y":55.79},{"x":18.49,"y":53.8},{"x":17.29,"y":51.54},{"x":16.49,"y":49.05},{"x":15.87,"y":46.59},{"x":18.15,"y":47.82},{"x":19.96,"y":49.78},{"x":21.33,"y":52.07},{"x":22.49,"y":54.44},{"x":22.51,"y":57.12},{"x":23.16,"y":56.25},{"x":23.37,"y":53.76},{"x":22.13,"y":51.4},{"x":20.92,"y":49.01},{"x":20.41,"y":46.42},{"x":20.53,"y":43.75},{"x":21.46,"y":42.57},{"x":23.1,"y":44.67},{"x":24.39,"y":47.01},{"x":25.0,"y":49.6},{"x":24.33,"y":52.15},{"x":24.06,"y":54.38},{"x":26.49,"y":53.25},{"x":27.35,"y":51.52},{"x":25.91,"y":49.38},{"x":25.34,"y":46.77},{"x":24.95,"y":44.88},{"x":26.95,"y":46.64},{"x":28.64,"y":48.71},{"x":29.91,"y":51.06},{"x":30.83,"y":53.57},{"x":31.3,"y":56.19},{"x":32.09,"y":58.72},{"x":32.34,"y":60.26},{"x":31.43,"y":57.81},{"x":29.66,"y":55.81},{"x":28.31,"y":53.53},{"x":29.4,"y":51.9},{"x":30.62,"y":50.27},{"x":29.89,"y":47.74},{"x":29.98,"y":45.19},{"x":30.39,"y":47.81},{"x":31.28,"y":47.71},{"x":31.96,"y":45.13},{"x":33.14,"y":42.74},{"x":34.49,"y":40.45},{"x":35.14,"y":42.75},{"x":35.33,"y":45.41},{"x":34.81,"y":47.5},{"x":32.66,"y":47.8},{"x":31.94,"y":50.36},{"x":31.04,"y":51.32},{"x":32.76,"y":50.73},{"x":33.49,"y":48.16},{"x":34.84,"y":49.92},{"x":35.69,"y":52.46},{"x":36.03,"y":55.1},{"x":35.93,"y":57.75},{"x":35.23,"y":60.32},{"x":34.04,"y":61.04},{"x":33.04,"y":58.56},{"x":32.24,"y":56.01},{"x":32.15,"y":53.36},{"x":31.99,"y":50.85},{"x":30.6,"y":48.55},{"x":29.21,"y":46.26},{"x":27.62,"y":44.15},{"x":26.39,"y":41.79},{"x":25.75,"y":39.22},{"x":25.46,"y":36.61},{"x":25.72,"y":33.96},{"x":26.82,"y":34.18},{"x":28.16,"y":36.5},{"x":29.26,"y":38.92},{"x":29.57,"y":41.56},{"x":29.2,"y":44.2},{"x":29.42,"y":44.95},{"x":30.46,"y":42.48},{"x":30.46,"y":39.8},{"x":31.1,"y":37.22},{"x":32.55,"y":34.98},{"x":34.34,"y":32.99},{"x":36.28,"y":31.78},{"x":35.85,"y":34.41},{"x":35.2,"y":36.99},{"x":33.89,"y":39.32},{"x":32.01,"y":41.2},{"x":30.37,"y":41.79},{"x":30.02,"y":39.14},{"x":29.35,"y":36.57},{"x":29.19,"y":33.94},{"x":29.48,"y":31.29},{"x":30.03,"y":28.68},{"x":31.32,"y":26.35},{"x":33.03,"y":24.31},{"x":34.83,"y":22.33},{"x":35.11,"y":24.48},{"x":34.54,"y":27.08},{"x":33.68,"y":29.61},{"x":32.45,"y":31.82},{"x":30.7,"y":33.67},{"x":29.32,"y":35.91},{"x":30.79,"y":35.52},{"x":30.05,"y":37.35},{"x":31.5,"y":35.1},{"x":32.94,"y":32.84},{"x":34.39,"y":30.58},{"x":35.81,"y":28.31},{"x":37.2,"y":26.03},{"x":38.94,"y":24.01},{"x":41.38,"y":22.95},{"x":43.16,"y":23.1},{"x":41.88,"y":25.45},{"x":40.0,"y":27.33},{"x":37.63,"y":28.51},{"x":35.66,"y":28.06},{"x":36.03,"y":25.41},{"x":37.07,"y":22.96},{"x":38.26,"y":20.58},{"x":39.78,"y":18.38},{"x":41.8,"y":16.64},{"x":44.25,"y":15.59},{"x":46.28,"y":15.5},{"x":44.89,"y":17.77},{"x":43.46,"y":20.02},{"x":41.13,"y":21.3},{"x":38.58,"y":22.07},{"x":36.91,"y":23.91},{"x":36.23,"y":26.1},{"x":38.64,"y":27.28},{"x":41.04,"y":28.46},{"x":43.45,"y":29.63},{"x":45.86,"y":30.81},{"x":48.27,"y":31.99},{"x":50.68,"y":33.17},{"x":53.09,"y":34.35},{"x":54.53,"y":32.35},{"x":54.8,"y":34.63},{"x":54.25,"y":37.25},{"x":53.49,"y":39.8},{"x":52.38,"y":42.24},{"x":51.06,"y":44.08},{"x":51.2,"y":41.42},{"x":51.63,"y":38.79},{"x":52.54,"y":36.28},{"x":53.61,"y":35.22},{"x":54.62,"y":37.71},{"x":55.62,"y":40.19},{"x":56.47,"y":42.51},{"x":56.25,"y":45.17},{"x":55.7,"y":47.79},{"x":54.55,"y":50.19},{"x":52.9,"y":52.29},{"x":51.26,"y":54.37},{"x":50.42,"y":56.91},{"x":50.43,"y":55.69},{"x":50.75,"y":53.05},{"x":51.16,"y":50.41},{"x":51.95,"y":47.87},{"x":53.35,"y":45.61},{"x":54.85,"y":43.4},{"x":55.74,"y":41.35},{"x":53.53,"y":42.86},{"x":51.32,"y":44.38},{"x":49.12,"y":45.9},{"x":49.08,"y":48.07},{"x":49.8,"y":50.63},{"x":49.88,"y":53.3},{"x":49.5,"y":55.94},{"x":48.52,"y":56.22},{"x":46.95,"y":54.45},{"x":46.83,"y":51.8},{"x":47.26,"y":49.17},{"x":47.62,"y":46.52},{"x":49.5,"y":47.37},{"x":51.57,"y":49.07},{"x":53.64,"y":50.77},{"x":55.71,"y":52.48},{"x":57.79,"y":54.18},{"x":59.86,"y":55.88},{"x":61.93,"y":57.58},{"x":64.0,"y":59.28},{"x":65.48,"y":61.4},{"x":66.41,"y":63.9},{"x":66.88,"y":66.53},{"x":66.61,"y":69.18},{"x":65.88,"y":71.75},{"x":65.42,"y":74.37},{"x":65.55,"y":77.04},{"x":65.41,"y":77.55},{"x":65.04,"y":74.91},{"x":64.94,"y":72.28},{"x":63.99,"y":69.83},{"x":63.16,"y":67.37},{"x":63.46,"y":64.73},{"x":63.94,"y":62.12},{"x":64.1,"y":59.46},{"x":65.54,"y":61.54},{"x":67.01,"y":63.77},{"x":68.49,"y":66.01},{"x":69.79,"y":68.15},{"x":70.26,"y":70.78},{"x":70.29,"y":73.45},{"x":69.9,"y":76.08},{"x":68.94,"y":78.57},{"x":67.41,"y":80.45},{"x":66.54,"y":77.94},{"x":66.18,"y":75.32},{"x":66.54,"y":72.68},{"x":67.62,"y":70.24},{"x":68.87,"y":67.87},{"x":69.36,"y":69.15},{"x":69.52,"y":71.82},{"x":69.69,"y":74.5},{"x":69.86,"y":77.18},{"x":70.73,"y":79.45},{"x":71.81,"y":81.88},{"x":72.62,"y":84.43},{"x":73.6,"y":86.92},{"x":74.3,"y":89.49},{"x":74.27,"y":92.14},{"x":73.32,"y":93.79},{"x":71.79,"y":91.61},{"x":70.68,"y":89.2},{"x":70.0,"y":86.63},{"x":70.32,"y":84.0},{"x":70.45,"y":81.35},{"x":69.78,"y":79.18},{"x":67.85,"y":81.05},{"x":67.44,"y":83.48},{"x":67.32,"y":83.77},{"x":66.48,"y":81.24},{"x":66.15,"y":79.71},{"x":66.9,"y":81.81},{"x":66.2,"y":83.65},{"x":67.06,"y":86.18},{"x":65.17,"y":85.94},{"x":63.13,"y":84.28},{"x":61.7,"y":82.06},{"x":60.53,"y":79.65},{"x":59.6,"y":77.16},{"x":61.19,"y":77.78},{"x":63.23,"y":79.51},{"x":64.93,"y":81.56},{"x":66.39,"y":83.8},{"x":67.79,"y":86.09},{"x":68.99,"y":88.48},{"x":70.19,"y":90.87},{"x":69.77,"y":90.78},{"x":68.4,"y":88.52},{"x":68.03,"y":88.58},{"x":66.99,"y":91.05},{"x":68.95,"y":92.37},{"x":71.32,"y":93.59},{"x":73.33,"y":95.36},{"x":74.84,"y":97.52},{"x":76.01,"y":99.93},{"x":74.72,"y":100.26},{"x":72.46,"y":98.83},{"x":70.48,"y":97.04},{"x":69.15,"y":94.73},{"x":67.7,"y":92.48},{"x":67.85,"y":91.01},{"x":70.37,"y":90.09},{"x":72.89,"y":89.16},{"x":75.4,"y":88.24},{"x":77.24,"y":86.79},{"x":76.42,"y":84.63},{"x":75.8,"y":82.25},{"x":75.48,"y":79.97},{"x":76.34,"y":78.32},{"x":77.56,"y":76.12},{"x":79.3,"y":74.39},{"x":81.14,"y":73.08},{"x":83.64,"y":72.57},{"x":86.14,"y":73.0},{"x":88.44,"y":73.35},{"x":90.28,"y":74.61},{"x":91.88,"y":75.99},{"x":92.47,"y":78.23},{"x":93.07,"y":79.86},{"x":93.07,"y":81.94},{"x":92.67,"y":84.22},{"x":91.09,"y":86.15},{"x":90.13,"y":88.46},{"x":88.16,"y":90.07},{"x":87.69,"y":89.34},{"x":89.21,"y":87.15},{"x":89.03,"y":85.35},{"x":87.65,"y":87.63},{"x":85.83,"y":89.48},{"x":83.29,"y":89.57},{"x":81.52,"y":88.54},{"x":80.27,"y":86.17},{"x":79.33,"y":87.39},{"x":80.71,"y":89.68},{"x":80.08,"y":90.51},{"x":78.55,"y":88.78},{"x":77.55,"y":87.85},{"x":79.9,"y":89.15},{"x":82.25,"y":90.44},{"x":84.25,"y":91.08},{"x":83.27,"y":91.66},{"x":83.66,"y":93.12},{"x":84.23,"y":95.72},{"x":85.11,"y":97.93},{"x":83.24,"y":98.73},{"x":81.52,"y":98.4},{"x":83.7,"y":97.38},{"x":83.45,"y":96.29},{"x":81.33,"y":97.94},{"x":79.22,"y":99.58},{"x":77.1,"y":101.22},{"x":78.29,"y":103.45},{"x":79.89,"y":105.59},{"x":80.05,"y":106.16},{"x":78.34,"y":104.1},{"x":76.64,"y":102.06},{"x":75.8,"y":100.29},{"x":73.84,"y":98.45},{"x":71.89,"y":96.62},{"x":69.93,"y":94.78},{"x":67.98,"y":92.95},{"x":66.03,"y":91.11},{"x":64.07,"y":89.28},{"x":62.12,"y":87.44},{"x":60.16,"y":85.61},{"x":58.21,"y":83.77},{"x":56.25,"y":81.94},{"x":54.3,"y":80.1},{"x":52.35,"y":78.27},{"x":50.39,"y":76.43},{"x":48.44,"y":74.6},{"x":46.48,"y":72.76},{"x":44.53,"y":70.93},{"x":42.57,"y":69.09},{"x":40.62,"y":67.26},{"x":38.66,"y":65.42},{"x":36.71,"y":63.59},{"x":34.76,"y":61.75},{"x":32.8,"y":59.92},{"x":30.85,"y":58.08},{"x":28.89,"y":56.25},{"x":26.94,"y":54.41},{"x":24.98,"y":52.58},{"x":23.03,"y":50.74},{"x":21.08,"y":48.91},{"x":19.04,"y":47.21},{"x":17.09,"y":45.47},{"x":16.55,"y":42.88},{"x":16.36,"y":40.22},{"x":16.74,"y":37.66},{"x":18.14,"y":39.91},{"x":19.37,"y":42.29},{"x":19.61,"y":44.89},{"x":19.64,"y":47.56}]];
}

function countStrokePoints(list){
  return list.reduce((sum,s)=>sum+s.length,0);
}

document.getElementById('riceBtn').onclick=async()=>{
  strokes=riceEarStrokes();
  activeStroke=null;
  drawing=false;
  ricePresetSelected=true;
  loopMode=false;
  setModeButton();
  redraw();
  statusEl.textContent='询问笔仙...';
  try{
    await sendMode();
    const saved=await post('/ricePreset');
    const started=await post('/start');
    statusEl.textContent=`${saved} ${started}`;
  }catch(e){
    statusEl.textContent='笔仙没连上，请重新连接热点';
  }
};

document.getElementById('imageInput').onchange=async e=>{
  const file=e.target.files && e.target.files[0];
  if(!file)return;
  statusEl.textContent='正在提取轮廓...';
  try{
    const stroke=await contourFromImage(file);
    strokes=[stroke];
    activeStroke=null;
    drawing=false;
    ricePresetSelected=false;
    redraw();
    statusEl.textContent='已生成一笔轮廓轨迹，'+stroke.length+' 个点';
  }catch(err){
    statusEl.textContent='图片轮廓提取失败';
  }finally{
    e.target.value='';
  }
};

function loadImageFile(file){
  return new Promise((resolve,reject)=>{
    const img=new Image();
    img.onload=()=>{URL.revokeObjectURL(img.src);resolve(img);};
    img.onerror=reject;
    img.src=URL.createObjectURL(file);
  });
}

function otsuThreshold(hist,total){
  let sum=0;
  for(let i=0;i<256;i++)sum+=i*hist[i];
  let sumB=0,wB=0,best=0,maxVar=-1;
  for(let t=0;t<256;t++){
    wB+=hist[t];
    if(wB===0)continue;
    const wF=total-wB;
    if(wF===0)break;
    sumB+=t*hist[t];
    const mB=sumB/wB;
    const mF=(sum-sumB)/wF;
    const between=wB*wF*(mB-mF)*(mB-mF);
    if(between>maxVar){maxVar=between;best=t;}
  }
  return best;
}

async function contourFromImage(file){
  const img=await loadImageFile(file);
  const N=128;
  const off=document.createElement('canvas');
  off.width=N; off.height=N;
  const c=off.getContext('2d',{willReadFrequently:true});
  c.fillStyle='#fff';
  c.fillRect(0,0,N,N);
  const scale=Math.min(N/img.width,N/img.height);
  const w=img.width*scale, h=img.height*scale;
  c.drawImage(img,(N-w)/2,(N-h)/2,w,h);
  const data=c.getImageData(0,0,N,N).data;
  const gray=new Uint8Array(N*N);
  const hist=new Array(256).fill(0);
  for(let i=0,p=0;i<data.length;i+=4,p++){
    const g=Math.round(0.299*data[i]+0.587*data[i+1]+0.114*data[i+2]);
    gray[p]=g; hist[g]++;
  }
  const threshold=otsuThreshold(hist,N*N);
  let borderSum=0,borderCount=0;
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      if(x===0||y===0||x===N-1||y===N-1){borderSum+=gray[y*N+x];borderCount++;}
    }
  }
  const borderMean=borderSum/borderCount;
  const foregroundDark=borderMean>128;
  const mask=new Uint8Array(N*N);
  for(let p=0;p<N*N;p++){
    mask[p]=foregroundDark ? (gray[p]<threshold ? 1:0) : (gray[p]>threshold ? 1:0);
  }
  const comp=largestComponent(mask,N);
  const boundary=[];
  for(let y=1;y<N-1;y++){
    for(let x=1;x<N-1;x++){
      const p=y*N+x;
      if(!comp[p])continue;
      if(!comp[p-1]||!comp[p+1]||!comp[p-N]||!comp[p+N])boundary.push({x,y});
    }
  }
  if(boundary.length<8)throw new Error('no contour');
  const ordered=orderBoundary(boundary);
  const mm=ordered.map(p=>({x:p.x/(N-1)*L0,y:(1-p.y/(N-1))*L0}));
  const simplified=simplifyStroke(mm,1.2,520);
  if(simplified.length<3)throw new Error('short contour');
  const first=simplified[0], last=simplified[simplified.length-1];
  if(Math.hypot(first.x-last.x,first.y-last.y)>1.5)simplified.push({x:first.x,y:first.y});
  return simplified;
}

function largestComponent(mask,N){
  const visited=new Uint8Array(N*N);
  let best=[];
  const qx=new Int16Array(N*N), qy=new Int16Array(N*N);
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const start=y*N+x;
      if(!mask[start]||visited[start])continue;
      let head=0,tail=0;
      const pts=[];
      qx[tail]=x; qy[tail]=y; tail++; visited[start]=1;
      while(head<tail){
        const cx=qx[head], cy=qy[head]; head++;
        pts.push(cy*N+cx);
        const nb=[[cx+1,cy],[cx-1,cy],[cx,cy+1],[cx,cy-1]];
        for(const [nx,ny] of nb){
          if(nx<0||ny<0||nx>=N||ny>=N)continue;
          const np=ny*N+nx;
          if(mask[np]&&!visited[np]){
            visited[np]=1; qx[tail]=nx; qy[tail]=ny; tail++;
          }
        }
      }
      if(pts.length>best.length)best=pts;
    }
  }
  const out=new Uint8Array(N*N);
  for(const p of best)out[p]=1;
  return out;
}

function orderBoundary(points){
  let cx=0,cy=0;
  for(const p of points){cx+=p.x;cy+=p.y;}
  cx/=points.length; cy/=points.length;
  points.sort((a,b)=>Math.atan2(a.y-cy,a.x-cx)-Math.atan2(b.y-cy,b.x-cx));
  return points;
}

function simplifyStroke(points,minDist,maxPoints){
  const out=[];
  for(const p of points){
    const last=out[out.length-1];
    if(!last||Math.hypot(p.x-last.x,p.y-last.y)>=minDist)out.push(p);
  }
  while(out.length>maxPoints){
    const reduced=[];
    for(let i=0;i<out.length;i+=2)reduced.push(out[i]);
    out.length=0; out.push(...reduced);
  }
  return out;
}

let speedTimer=0;
speed.oninput=()=>{
  speedText.textContent=speed.value;
  clearTimeout(speedTimer);
  speedTimer=setTimeout(()=>post('/speed?value='+encodeURIComponent(speed.value)),250);
};

let travelSpeedTimer=0;
travelSpeed.oninput=()=>{
  travelSpeedText.textContent=travelSpeed.value;
  clearTimeout(travelSpeedTimer);
  travelSpeedTimer=setTimeout(()=>post('/travelSpeed?value='+encodeURIComponent(travelSpeed.value)),250);
};

let baseTorqueTimer=0;
baseTorque.oninput=()=>{
  baseTorqueText.textContent=baseTorque.value;
  clearTimeout(baseTorqueTimer);
  baseTorqueTimer=setTimeout(()=>post('/calibrationTorque?base='+encodeURIComponent(baseTorque.value)),250);
};

let pullTorqueTimer=0;
pullTorque.oninput=()=>{
  pullTorqueText.textContent=pullTorque.value;
  clearTimeout(pullTorqueTimer);
  pullTorqueTimer=setTimeout(()=>post('/calibrationTorque?pull='+encodeURIComponent(pullTorque.value)),250);
};

window.addEventListener('resize',fitCanvas);
fitCanvas();
loadStatus();
setInterval(async()=>{
  try{
    const s=await fetchStatus(1200);
    renderCoef(s);
  }catch(e){}
},3000);
</script>
</body>
</html>
)rawliteral";

uint16_t webStepDelayMs() {
    int speed = constrain((int)webSpeedPercent, 1, 100);
    return maxWebStepDelayMs - ((maxWebStepDelayMs - minWebStepDelayMs) * (speed - 1)) / 99;
}

uint16_t webTravelStepDelayMs() {
    int speed = constrain((int)webTravelSpeedPercent, 1, 100);
    return maxWebStepDelayMs - ((maxWebStepDelayMs - minWebStepDelayMs) * (speed - 1)) / 99;
}

void saveWebSettings() {
    Preferences prefs;
    if (!prefs.begin(calibrationNamespace, false)) {
        Serial0.println("Web settings save failed: NVS open failed");
        return;
    }

    prefs.putUInt("drawSpd", webSpeedPercent);
    prefs.putUInt("travelSpd", webTravelSpeedPercent);
    prefs.putUInt("calBase", calibrationBaseCurrent);
    prefs.putUInt("calPull", calibrationPullCurrent);
    prefs.end();
    Serial0.printf("Web settings saved: draw=%u travel=%u calBase=%d calPull=%d\n",
                   webSpeedPercent, webTravelSpeedPercent,
                   calibrationBaseCurrent, calibrationPullCurrent);
}

void loadWebSettings() {
    Preferences prefs;
    if (!prefs.begin(calibrationNamespace, true)) {
        Serial0.println("Web settings load skipped: NVS open failed");
        return;
    }

    webSpeedPercent = (uint8_t)constrain((int)prefs.getUInt("drawSpd", webSpeedPercent), 1, 100);
    webTravelSpeedPercent = (uint8_t)constrain((int)prefs.getUInt("travelSpd", webTravelSpeedPercent), 1, 100);
    calibrationBaseCurrent = constrain((int)prefs.getUInt("calBase", calibrationBaseCurrent), 20, 300);
    calibrationPullCurrent = constrain((int)prefs.getUInt("calPull", calibrationPullCurrent), 300, 1500);
    prefs.end();

    Serial0.printf("Loaded web settings: draw=%u travel=%u calBase=%d calPull=%d\n",
                   webSpeedPercent, webTravelSpeedPercent,
                   calibrationBaseCurrent, calibrationPullCurrent);
}

void stopWebPlayback(bool disableMotors) {
    if (webPlaybackActive) {
        Serial0.println("Web playback stopped.");
    }
    webPlaybackActive = false;
    webTravelActive = false;
    webPlaybackIndex = 0;
    if (disableMotors) {
        allEnable(false);
    }
}

void appendWebPoint(float x, float y, bool draw) {
    if (webPathPointCount >= maxWebPathPoints) {
        return;
    }

    x = constrain(x, 0.0f, L0);
    y = constrain(y, 0.0f, L0);
    uint16_t x10 = (uint16_t)lroundf(x * 10.0f);
    uint16_t y10 = (uint16_t)lroundf(y * 10.0f);

    if (webPathPointCount > 0 &&
        webPath[webPathPointCount - 1].x10 == x10 &&
        webPath[webPathPointCount - 1].y10 == y10 &&
        webPath[webPathPointCount - 1].draw == (draw ? 1 : 0)) {
        return;
    }

    webPath[webPathPointCount].x10 = x10;
    webPath[webPathPointCount].y10 = y10;
    webPath[webPathPointCount].draw = draw ? 1 : 0;
    webPathPointCount++;
}

void appendWebSegment(float fromX, float fromY, float toX, float toY, bool draw) {
    float dx = toX - fromX;
    float dy = toY - fromY;
    float distance = sqrt(dx * dx + dy * dy);
    int steps = max(1, (int)ceil(distance / webResampleMm));

    for (int step = 1; step <= steps && webPathPointCount < maxWebPathPoints; step++) {
        float t = (float)step / steps;
        appendWebPoint(fromX + dx * t, fromY + dy * t, draw);
    }
}

bool readNextPair(const String &body, int &idx, float &x, float &y) {
    int len = body.length();
    while (idx < len) {
        char ch = body[idx];
        if (ch == ';' || ch == '\n' || ch == '\r' || ch == ' ' || ch == '\t') idx++;
        else break;
    }
    if (idx >= len) return false;

    int comma = body.indexOf(',', idx);
    if (comma < 0) return false;
    int end = comma + 1;
    while (end < len) {
        char ch = body[end];
        if (ch == ';' || ch == '\n' || ch == '\r' || ch == ' ' || ch == '\t') break;
        end++;
    }

    x = body.substring(idx, comma).toFloat();
    y = body.substring(comma + 1, end).toFloat();
    idx = end;
    return true;
}

bool loadWebStroke(const String &stroke, int &rawCount) {
    int idx = 0;
    int startPointCount = webPathPointCount;
    float x = 0;
    float y = 0;
    float prevX = 0;
    float prevY = 0;
    bool hasPrev = false;
    bool hasDrawSegment = false;

    while (readNextPair(stroke, idx, x, y)) {
        x = constrain(x, 0.0f, L0);
        y = constrain(y, 0.0f, L0);

        if (!hasPrev) {
            appendWebPoint(x, y, false);
            hasPrev = true;
        } else {
            appendWebSegment(prevX, prevY, x, y, true);
            hasDrawSegment = true;
        }

        prevX = x;
        prevY = y;
        rawCount++;

        if (webPathPointCount >= maxWebPathPoints) {
            break;
        }
    }

    if (hasDrawSegment) {
        webStrokeCount++;
    } else {
        webPathPointCount = startPointCount;
    }

    return hasDrawSegment;
}

bool loadWebPath(const String &body) {
    stopWebPlayback(true);
    webPathPointCount = 0;
    webStrokeCount = 0;
    int rawCount = 0;

    int start = 0;
    while (start <= body.length() && webPathPointCount < maxWebPathPoints) {
        int end = body.indexOf('\n', start);
        if (end < 0) end = body.length();

        String stroke = body.substring(start, end);
        stroke.trim();
        if (stroke.length() > 0) {
            loadWebStroke(stroke, rawCount);
        }

        if (end >= body.length()) break;
        start = end + 1;
    }

    Serial0.printf("Web path loaded: strokes=%d raw=%d resampled=%d\n",
                   webStrokeCount, rawCount, webPathPointCount);
    return webStrokeCount > 0 && webPathPointCount >= 2;
}

bool loadRicePresetPath() {
    stopWebPlayback(true);
    webPathPointCount = 0;
    webStrokeCount = 0;

    const int count = min((int)ricePresetPointCount, maxWebPathPoints);
    for (int i = 0; i < count; i++) {
        RicePresetPoint preset;
        memcpy_P(&preset, &ricePresetPath[i], sizeof(preset));
        webPath[webPathPointCount].x10 = preset.x10;
        webPath[webPathPointCount].y10 = preset.y10;
        webPath[webPathPointCount].draw = preset.draw ? 1 : 0;
        if (!preset.draw) {
            webStrokeCount++;
        }
        webPathPointCount++;
    }

    Serial0.printf("Rice preset loaded: strokes=%d points=%d\n",
                   webStrokeCount, webPathPointCount);
    return webPathPointCount >= 2;
}

void handleRootPage() {
    webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
    webServer.send_P(200, "text/html; charset=utf-8", indexHtml);
}

void handlePathUpload() {
    String body = webServer.arg("plain");
    bool ok = loadWebPath(body);
    if (ok) {
        webServer.send(200, "text/plain; charset=utf-8",
                       "轨迹已保存: " + String(webStrokeCount) + " 笔, " + String(webPathPointCount) + " 点");
    } else {
        webServer.send(400, "text/plain; charset=utf-8", "轨迹太短");
    }
}

void handleRicePresetUpload() {
    bool ok = loadRicePresetPath();
    if (ok) {
        webServer.send(200, "text/plain; charset=utf-8",
                       "稻穗轨迹已载入: " + String(webStrokeCount) + " 笔, " + String(webPathPointCount) + " 点");
    } else {
        webServer.send(500, "text/plain; charset=utf-8", "稻穗轨迹载入失败");
    }
}

void handleStartPlayback() {
    if (webPathPointCount < 2) {
        webServer.send(400, "text/plain; charset=utf-8", "没有可播放轨迹");
        return;
    }

    allEnable(true);
    webPlaybackActive = true;
    webTravelActive = false;
    webPlaybackIndex = 0;
    nextWebStepAt = 0;
    Serial0.printf("Web playback started: strokes=%d points=%d drawSpeed=%u drawDelay=%u ms travelSpeed=%u travelDelay=%u ms loop=%d\n",
                   webStrokeCount, webPathPointCount, webSpeedPercent, webStepDelayMs(),
                   webTravelSpeedPercent, webTravelStepDelayMs(),
                   webLoopPlayback ? 1 : 0);
    webServer.send(200, "text/plain; charset=utf-8",
                   webLoopPlayback ? "开始循环绘制" : "开始单次绘制");
}

void handleStopPlayback() {
    stopWebPlayback(true);
    webServer.send(200, "text/plain; charset=utf-8", "已停止");
}

void handleSpeedChange() {
    if (webServer.hasArg("value")) {
        uint8_t newSpeed = (uint8_t)constrain(webServer.arg("value").toInt(), 1, 100);
        if (newSpeed != webSpeedPercent) {
            webSpeedPercent = newSpeed;
            saveWebSettings();
        }
    }
    Serial0.printf("Web speed: %u delay=%u ms\n", webSpeedPercent, webStepDelayMs());
    webServer.send(200, "text/plain; charset=utf-8", "绘制速度 " + String(webSpeedPercent));
}

void handleTravelSpeedChange() {
    if (webServer.hasArg("value")) {
        uint8_t newSpeed = (uint8_t)constrain(webServer.arg("value").toInt(), 1, 100);
        if (newSpeed != webTravelSpeedPercent) {
            webTravelSpeedPercent = newSpeed;
            saveWebSettings();
        }
    }
    Serial0.printf("Web travel speed: %u delay=%u ms\n", webTravelSpeedPercent, webTravelStepDelayMs());
    webServer.send(200, "text/plain; charset=utf-8", "移动速度 " + String(webTravelSpeedPercent));
}

void handleCalibrationTorqueChange() {
    bool changed = false;

    if (webServer.hasArg("base")) {
        int newBase = constrain(webServer.arg("base").toInt(), 20, 300);
        if (newBase != calibrationBaseCurrent) {
            calibrationBaseCurrent = newBase;
            changed = true;
        }
    }

    if (webServer.hasArg("pull")) {
        int newPull = constrain(webServer.arg("pull").toInt(), 300, 1500);
        if (newPull != calibrationPullCurrent) {
            calibrationPullCurrent = newPull;
            changed = true;
        }
    }

    if (changed) {
        saveWebSettings();
    }

    Serial0.printf("Calibration torque settings: base=%d pull=%d\n",
                   calibrationBaseCurrent, calibrationPullCurrent);
    webServer.send(200, "text/plain; charset=utf-8",
                   "校准扭矩: 保持 " + String(calibrationBaseCurrent) +
                   ", 当前 " + String(calibrationPullCurrent));
}

void handleSaveSettings() {
    saveWebSettings();
    webServer.send(200, "text/plain; charset=utf-8",
                   "参数已保存: 绘制 " + String(webSpeedPercent) +
                   ", 移动 " + String(webTravelSpeedPercent));
}

void handleResetSettings() {
    stopWebPlayback(true);
    resetCalibrationToDefaults();
    webSpeedPercent = 50;
    webTravelSpeedPercent = 15;
    calibrationBaseCurrent = defaultCalibrationBaseCurrent;
    calibrationPullCurrent = defaultCalibrationPullCurrent;
    saveCalibration();
    saveWebSettings();
    Serial0.println("Web settings and calibration reset to defaults.");
    webServer.send(200, "text/plain; charset=utf-8", "参数已重置");
}

void handleModeChange() {
    if (webServer.hasArg("loop")) {
        webLoopPlayback = webServer.arg("loop").toInt() != 0;
    }
    Serial0.printf("Web playback mode: %s\n", webLoopPlayback ? "loop" : "single");
    webServer.send(200, "text/plain; charset=utf-8", webLoopPlayback ? "循环绘制" : "单次绘制");
}

void handleClearPath() {
    stopWebPlayback(true);
    webPathPointCount = 0;
    webStrokeCount = 0;
    Serial0.println("Web canvas cleared.");
    webServer.send(200, "text/plain; charset=utf-8", "画布已清除");
}

void handleCalibrateRequest() {
    if (webCalibrationRunning || webCalibrationRequested) {
        webServer.send(200, "text/plain; charset=utf-8", "校准已经在进行");
        return;
    }

    stopWebPlayback(true);
    webCalibrationRequested = true;
    webCalibrationDone = false;
    Serial0.println("Web calibration requested.");
    webServer.send(200, "text/plain; charset=utf-8", "校准已开始，请等待完成");
}

void handleStatus() {
    String json = "{\"points\":" + String(webPathPointCount) +
                  ",\"strokes\":" + String(webStrokeCount) +
                  ",\"playing\":" + String(webPlaybackActive ? "true" : "false") +
                  ",\"loop\":" + String(webLoopPlayback ? "true" : "false") +
                  ",\"speed\":" + String(webSpeedPercent) +
                  ",\"travelSpeed\":" + String(webTravelSpeedPercent) +
                  ",\"baseTorque\":" + String(calibrationBaseCurrent) +
                  ",\"pullTorque\":" + String(calibrationPullCurrent) +
                  ",\"calibrating\":" + String(webCalibrationRunning || webCalibrationRequested ? "true" : "false") +
                  ",\"calibrated\":" + String(webCalibrationDone ? "true" : "false") +
                  ",\"m1\":" + String(realDeg2mm[1], 3) +
                  ",\"m2\":" + String(realDeg2mm[2], 3) +
                  ",\"m3\":" + String(realDeg2mm[3], 3) +
                  ",\"m4\":" + String(realDeg2mm[4], 3) + "}";
    webServer.send(200, "application/json", json);
}

void setupWebPortal() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apGateway, apSubnet);
    WiFi.softAP(apSsid);
    delay(100);

    dnsServer.start(dnsPort, "*", apIP);

    webServer.on("/", HTTP_GET, handleRootPage);
    webServer.on("/path", HTTP_POST, handlePathUpload);
    webServer.on("/ricePreset", HTTP_POST, handleRicePresetUpload);
    webServer.on("/start", HTTP_POST, handleStartPlayback);
    webServer.on("/stop", HTTP_POST, handleStopPlayback);
    webServer.on("/speed", HTTP_POST, handleSpeedChange);
    webServer.on("/travelSpeed", HTTP_POST, handleTravelSpeedChange);
    webServer.on("/calibrationTorque", HTTP_POST, handleCalibrationTorqueChange);
    webServer.on("/saveSettings", HTTP_POST, handleSaveSettings);
    webServer.on("/resetSettings", HTTP_POST, handleResetSettings);
    webServer.on("/mode", HTTP_POST, handleModeChange);
    webServer.on("/clear", HTTP_POST, handleClearPath);
    webServer.on("/calibrate", HTTP_POST, handleCalibrateRequest);
    webServer.on("/status", HTTP_GET, handleStatus);

    webServer.on("/generate_204", HTTP_GET, handleRootPage);
    webServer.on("/gen_204", HTTP_GET, handleRootPage);
    webServer.on("/hotspot-detect.html", HTTP_GET, handleRootPage);
    webServer.on("/ncsi.txt", HTTP_GET, handleRootPage);
    webServer.on("/connecttest.txt", HTTP_GET, handleRootPage);
    webServer.on("/canonical.html", HTTP_GET, handleRootPage);
    webServer.on("/success.txt", HTTP_GET, handleRootPage);
    webServer.on("/redirect", HTTP_GET, handleRootPage);
    webServer.onNotFound(handleRootPage);
    webServer.begin();

    Serial0.println("Open WiFi drawing portal ready.");
    Serial0.printf("SSID: %s  Password: none  URL: http://%s/\n",
                   apSsid, apIP.toString().c_str());
}

void handleWebPortal() {
    dnsServer.processNextRequest();
    webServer.handleClient();
}

void handleWebCalibration() {
    if (!webCalibrationRequested || webCalibrationRunning) {
        return;
    }

    webCalibrationRequested = false;
    webCalibrationRunning = true;
    webCalibrationDone = false;
    Serial0.println("Web calibration started.");
    allEnable(true);
    delay(500);
    autoCalibrateHome();
    webCalibrationRunning = false;
    webCalibrationDone = true;
    Serial0.println("Web calibration finished and saved.");
}

void finishWebPlaybackCycle() {
    if (webLoopPlayback) {
        webPlaybackIndex = 0;
        webTravelActive = false;
        return;
    }

    Serial0.println("Web single playback complete.");
    stopWebPlayback(true);
}

void handleWebPlayback() {
    if (!webPlaybackActive || webPathPointCount < 2) {
        return;
    }

    uint32_t now = millis();
    if ((int32_t)(now - nextWebStepAt) < 0) {
        return;
    }

    if (webTravelActive) {
        webTravelStep++;
        float t = (float)webTravelStep / webTravelSteps;
        float x = webTravelFromX + (webTravelToX - webTravelFromX) * t;
        float y = webTravelFromY + (webTravelToY - webTravelFromY) * t;
        moveToXY(x, y, false, false);

        if (webTravelStep >= webTravelSteps) {
            webTravelActive = false;
            webPlaybackIndex++;
        }

        nextWebStepAt = millis() + webTravelStepDelayMs();
        return;
    }

    if (webPlaybackIndex >= webPathPointCount) {
        finishWebPlaybackCycle();
        return;
    }

    WebPathPoint &point = webPath[webPlaybackIndex];
    float x = point.x10 * 0.1f;
    float y = point.y10 * 0.1f;

    if (!point.draw) {
        float fromX = currentXYValid ? currentX : (L0 * 0.5f);
        float fromY = currentXYValid ? currentY : (L0 * 0.5f);
        float dx = x - fromX;
        float dy = y - fromY;
        float distance = sqrt(dx * dx + dy * dy);

        webTravelFromX = fromX;
        webTravelFromY = fromY;
        webTravelToX = x;
        webTravelToY = y;
        webTravelSteps = max(1, (int)ceil(distance / webTravelResampleMm));
        webTravelStep = 0;
        webTravelActive = true;
        nextWebStepAt = 0;
        Serial0.printf("Web travel to (%.1f,%.1f): steps=%d speed=%u delay=%u ms\n",
                       x, y, webTravelSteps, webTravelSpeedPercent, webTravelStepDelayMs());
        return;
    }

    moveToXY(x, y, false, false);
    webPlaybackIndex++;
    nextWebStepAt = millis() + webStepDelayMs();
}

// 调试指令扩展
bool parseNewCommand(String s) {
    s.trim();
    if (s == "home") {
        autoCalibrateHome();
        return true;
    }

    if (s == "coef") {
        printCoefficients();
        return true;
    }

    if (s == "circles" || s == "circle") {
        drawCenterCircles();
        return true;
    }

    if (s == "square") {
        drawSquareTest();
        return true;
    }

    if (s == "eight" || s == "8") {
        drawFigureEightTest();
        return true;
    }

    if (s == "spiral") {
        drawSpiralTest();
        return true;
    }

    if (s == "shapes" || s == "testshapes") {
        drawShapeTests();
        return true;
    }

    if (s == "ricepreset" || s == "rice") {
        loadRicePresetPath();
        return true;
    }

    if (s == "reboot") {
        Serial0.println("Restarting...");
        delay(100);
        ESP.restart();
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
    loadCalibration();
    loadWebSettings();
    setupWebPortal();
    show_help();
}

// ------------------- 主循环 -------------------
void loop() {
    static String serialLine;

    handleWebPortal();
    handleWebCalibration();
    handleWebPlayback();

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
