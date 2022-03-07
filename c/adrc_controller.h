#pragma once

struct adrc_td
{
    float x1; // 跟踪微分期状态量
    float x2; // 跟踪微分期状态量微分项
    float r;  // 时间尺度
    float h;  // ADRC系统积分时间, 取控制周期 dt 的 N 倍
};

struct adrc_eso
{
    float z1; // 状态估计
    float z2; // 状态微分估计
    float z3; // 状态扰动估计
    float fe;
    float fe1;
    float zeta;
    float b;
    float beta_01;
    float beta_02;
    float beta_03;
};

struct adrc_pid
{
    float e0;        /* 状态误差积分 */
    float integ_min; /* 积分下限 */
    float integ_max; /* 积分上限 */

    float e1;       /* 状态误差 */
    float e2;       /* 状态误差微分 */
    float diff_min; /* 微分下限 */
    float diff_max; /* 状态误差微分上限 */

    float u; /* 控制输出 */

    float u_min; /* 输出限幅 */
    float u_max;

    /* 非线性组合 fal 滤波参数 */
    float beta0;
    float beta1;
    float beta2;
    float alpha0;
    float alpha1;
    float alpha2;
    float zeta;
};

struct adrc_controller
{
    /* TD */
    struct adrc_td td;

    /* ESO */
    struct adrc_eso eso;

    /* pid */
    struct adrc_pid pid;
};

void adrc_controller_update(struct adrc_controller *adrc,
                            float input,
                            float feedback,
                            float dt);
