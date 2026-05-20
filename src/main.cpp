#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

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

const int maxWebPathPoints = 1600;
const float webResampleMm = 1.5f;
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
button{height:48px;border:0;border-radius:8px;font-size:18px;font-weight:650;color:#fff;background:#111}
button.stop{background:#9b1c1c}
button.clear{background:#4b5563}
button.mode{background:#115e59}
button.reset{background:#1d4ed8}
button.calib{background:#7c2d12}
.speed{margin-top:16px;display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center;font-size:15px}
input[type=range]{width:100%;accent-color:#111}
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
</div>
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
  if(strokes.filter(s=>s.length>=2).length<1){statusEl.textContent='请先在白色框里画一条轨迹';return;}
  statusEl.textContent='上传轨迹...';
  await sendMode();
  const saved=await post('/path',pathBody());
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
