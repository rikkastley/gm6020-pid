/**
 * @file    ControlTask.cpp
 * @brief   GM6020 跟随正弦速度 / 位置曲线
 *
 *  核心流程（每 1ms 一次）：
 *    喂狗 -> 算正弦目标 -> (摩擦前馈 +) PID -> 限幅 -> 发电流帧 0x1FE
 *  另有：反馈掉线保护（20ms 收不到反馈就输出 0）
 */
#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

#include <cmath>
#include <cstring>

/* ============ 模式开关 ============ */
#define mode 1              /* 0=速度跟随  1=位置跟随 */

/* ============ 可调参数 ============ */
float pi       = 3.14159265358979323846;
float rpmtorad = 2 * pi / 60.0;      /* rpm -> rad/s 换算系数 */
float freq     = 0.1;                /* 正弦频率 (Hz) */
float va       = 100.0;              /* 速度正弦幅值 (rpm) */
float xa       = 250.0;               /* 位置正弦幅值 (deg) */
float vlimit   = 8000.0;             /* 速度环输出限幅（电流） */
float xlimit   = 80.0 * rpmtorad;    /* 位置环输出限幅 (±80rpm) */
float friction = 450.0;              /* 摩擦前馈大小（低速起动助推），可调 */
uint32_t timeout = 20;               /* 掉线判定：超过 20ms 没反馈就算掉线 */

/* ============ 对象 ============ */
Gm6020 motor(1);                          /* 电机（拨码 ID=1） */
Pid vpid(200.0, 0.1, 0.1, vlimit, -vlimit);   /* 速度环：Kp Ki Kd */
Pid xpid(80.0, 0.0, 0.0, xlimit, -xlimit);   /* 位置环：Kp Ki Kd */

/* ============ 状态变量 ============ */
uint32_t tick      = 0;      /* 1ms 计数 */
uint8_t  ready     = 0;      /* 是否已收到第一帧反馈 */
float    zeroangle = 0.0;    /* 上电锁定的角度（位置模式零点） */
uint32_t lasttick  = 0;      /* 最近一次收到反馈时的 tick（掉线判断用） */
uint8_t  tx_buf[8];          /* 发送缓冲（8 字节） */
float realv = 0.0;   /* 实际速度 rad/s（给 Ozone 看，每拍在 MainTask 里更新） */
float v0    = 0.0;   /* 目标速度 rad/s（给 Ozone 看，每拍在 MainTask 里更新） */
float realx = 0.0;   /* 实际位置 rad（给 Ozone 看，每拍在 MainTask 里更新） */
float x0    = 0.0;   /* 目标位置 rad（给 Ozone 看，每拍在 MainTask 里更新） */

/* ============ 内部函数 ============ */

/* 角度误差折返到 [-pi, pi]（位置模式走近路，不绕圈） */
float wraptopi(float e) {
    while (e >  pi)
    e -= 2 * pi;
    while (e < -pi)
    e += 2 * pi;
    return e;
}

/* CAN 滤波器：只让本电机反馈帧 0x205 进来 */
void filter(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef f = {0};
    f.FilterIdHigh     = (uint16_t)(motor.rxId() << 5);   /* 要匹配的 ID = 0x205 */
    f.FilterMaskIdHigh = (uint16_t)(0x7FFu << 5);         /* 掩码全 1：完全相同才通过 */
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterBank       = 0;                               /* CAN1 用组 0 */
    f.FilterMode       = CAN_FILTERMODE_IDMASK;
    f.FilterScale      = CAN_FILTERSCALE_32BIT;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &f);
}

/* 发送一帧（3 个邮箱，满了就丢掉这一拍） */
void send(const uint8_t *data, uint32_t id, uint8_t len) {
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mailbox;
    h.StdId = id;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = len;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return;
    HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)data, &mailbox);
}

/* ============ 对外 3 个接口 ============ */

/* 上电初始化：装滤波器 -> 启动 CAN -> 开接收中断 -> 启动 1kHz 定时器 */
extern "C" void ControlTaskInit(void) {
    filter(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_TIM_Base_Start_IT(&htim6);
}

/* 收到反馈：拆包；第一帧时记下零点；记录收帧时刻 */
extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == motor.rxId()) {
        motor.decode(data);
        lasttick = tick;
        if (!ready) {
            zeroangle = motor.angle();
            ready = 1;
        }
    }
}

/* 1kHz 主循环 */
extern "C" void MainTask(void) {
    HAL_IWDG_Refresh(&hiwdg);       /* 喂狗 */
    tick++;
    realv = motor.vel();            /* 更新“实际速度”（每 1ms 更新一次！） */
    realx = motor.angle();          /* 更新“实际位置” */

    if (!ready) return;             /* 还没收到反馈，先不控制 */

    float u = 0.0;
    bool online = (tick - lasttick) <= timeout;   /* 20ms 内收过帧 = 在线 */

    if (online) {
        /* 1. 生成正弦目标 */
        float t = tick * 0.001;                     /* 秒 */
        float s = sinf(2 * pi * freq * t);

        /* 2. 得到速度环的目标速度 v0（全局） */
        if (mode == 0) 
        {
            v0 = va * rpmtorad * s;
        } else 
        {
            x0 = zeroangle + (xa * pi / 180.0) * s;                 /* 目标位置 */
            float x1 = motor.angle() + wraptopi(x0 - motor.angle());//折返//
            v0 = xpid.calc(x1, motor.angle());
        }

        //摩擦前馈//
        float ff = friction * tanhf(v0 / 0.5);
        u = vpid.calc(v0, realv) + ff;

        //限幅//
        if (u >  vlimit) u =  vlimit;
        if (u < -vlimit) u = -vlimit;
    } else
    {
        //掉线//
        vpid.reset();
        xpid.reset();
        u = 0.0;
    }

    //发出电流帧//
    motor.setVoltage((int16_t)u);
    memset(tx_buf, 0, 8);
    motor.encode(tx_buf);
    send(tx_buf, 0x1FE, 8);
}
