#include "adrc_controller.h"
#include "helpler_funtions.h"
#include <math.h>

static inline float _fsg(float alpha, float y)
{
    return 0.5f * (sign(alpha + y) - sign(alpha - y));
}

/**
 * @brief fal 非线性滤波
 *
 * @param e 输入
 * @param alpha 滤波强度参数，x 小于 1
 * @param zeta 滤波门限参数，当 e 的模大于 zeta，e 会被衰减，否则 e 被增加
 * @return float
 */
static inline float _fal(float e, float alpha, float zeta)
{

    float ret;
    float s = _fsg(e, zeta);
    ret = e * s / (powf(zeta, 1 - alpha)) + powf(fabsf(e), alpha) * sign(e) * (1 - s);
    return ret;
}

/**
 * @brief TD 最速微分跟踪器
 *
 * @param td td 对象
 * @param input
 * @return float
 */
static float _td_fst(struct adrc_td *td, float input)
{
    float r, h, delta, delta0, a, a0, delt_x1;

    float fst = 0.0f;

    r = td->r;
    h = td->h;

    delta = r * h;
    delta0 = r * h * h;

    /* 计算 x1 增量 */
    delt_x1 = td->x1 - input + h * td->x2;
    a0 = sqrtf(delta * delta + 8 * r * fabsf(delt_x1));

    if (fabsf(delt_x1) <= delta0)
    {
        a = td->x2 + delt_x1 / h;
    }
    else
    {
        a = td->x2 + 0.5f * (a0 - delta) * sign(delt_x1);
    }

    /* 对二阶微分进行限幅 */
    if (fabsf(a) <= delta)
    {
        fst = -r * a / delta;
    }
    else
    {
        fst = -r * sign(a);
    }

    return fst;
}

void adrc_td_update(struct adrc_controller *adrc, float input, float dt)
{
    struct adrc_td *td = &adrc->td;
    /* 更新对 input 的二阶微分 */
    float fst = _td_fst(td, input);

    /* 更新对 input 的跟踪 */
    td->x1 += dt * td->x2;

    /* 更新对 input 的一阶微分 */
    td->x2 += dt * fst;
}

void adrc_eso_update(struct adrc_controller *adrc, float feedback, float dt)
{
    struct adrc_eso *eso = &adrc->eso;
    float u = adrc->pid.u;

    float error = eso->z1 - feedback;
    eso->fe = _fal(error, 0.5f, eso->zeta);
    eso->fe1 = _fal(error, 0.25f, eso->zeta);

    eso->z1 += dt * (eso->z2 - eso->beta_01 * error);
    eso->z2 += dt * (eso->z3 - eso->beta_02 * eso->fe + eso->b * u);
    eso->z3 += dt * (-eso->beta_03 * eso->fe1);
}

static float adrc_nolinear_pid(struct adrc_controller *adrc, float dt)
{
    float ret = 0;

    adrc->pid.e0 += adrc->pid.e1 * dt;         //状态积分项
    adrc->pid.e1 = adrc->td.x1 - adrc->eso.z1; //状态偏差项
    adrc->pid.e2 = adrc->td.x2 - adrc->eso.z2; //状态微分项

    /* 对状态误差微分取限幅 */
    float e0 = adrc->pid.e0;
    float e1 = adrc->pid.e1;
    float e2 = adrc->pid.e2;

    /* 将状态误差做非线性组合 */
    ret = adrc->pid.beta0 * _fal(e0, adrc->pid.alpha0, adrc->pid.zeta) +
          adrc->pid.beta1 * _fal(e1, adrc->pid.alpha1, adrc->pid.zeta) +
          adrc->pid.beta2 * _fal(e2, adrc->pid.alpha2, adrc->pid.zeta);
    return ret;
}

static float adrc_linear_pid(struct adrc_controller *adrc, float dt)
{
    float ret = 0;

    adrc->pid.e0 += adrc->pid.e1 * dt;         //状态积分项
    adrc->pid.e1 = adrc->td.x1 - adrc->eso.z1; //状态偏差项
    adrc->pid.e2 = adrc->td.x2 - adrc->eso.z2; //状态微分项

    /* 对状态误差微分取限幅 */
    float e0 = adrc->pid.e0;
    float e1 = adrc->pid.e1;
    float e2 = adrc->pid.e2;

    /* 将状态误差做线性组合 */
    ret = adrc->pid.beta0 * e0 + adrc->pid.beta1 * e1 +
          adrc->pid.beta2 * e2;
    return ret;
}

void adrc_controller_update(struct adrc_controller *adrc,
                            float input,
                            float feedback,
                            float dt)
{
    adrc_td_update(adrc, input, dt);

    adrc_eso_update(adrc, feedback, dt);

    float u0 = adrc_nolinear_pid(adrc, dt);
    // float u0 = adrc_linear_pid(adrc, dt);

    adrc->pid.u = constrain_float(u0, adrc->pid.u_min, adrc->pid.u_max);
}
