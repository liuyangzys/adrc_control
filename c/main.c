#include "adrc_controller.h"
#include <stdio.h>

/* 积分过程对象模型 */
struct integ_p
{
  float k;
  float output;
};
void integ_p_update(struct integ_p *p, float dt, float u)
{
  p->output += p->k * dt * u;
}

/* 简单的 pid 控制器 */
struct pid_controller
{
  float kp;
  float ki;
  float kd;

  float e0;
  float integ;
  float output;
};

void pid_update(struct pid_controller *pid, float sp, float feedback,
                float dt)
{
  float e = sp - feedback;

  pid->integ += dt * e;

  pid->output =
      e * pid->kp + pid->ki * pid->integ + pid->kd * (e - pid->e0) / dt;
}

int main(void)
{
  struct integ_p p, p1;
  struct adrc_controller adrc;
  struct pid_controller pid;

  /* 目标值 sp */
  float sp = 0;
  /* 仿真时间，步长 */
  float t = 0, dt = 1 / 400.f;
  /* 仿真总时长 */
  const float s_time = 100;

  /* 1阶积分过程对象初始化 */
  p.k = 1;
  p.output = 0;
  p1 = p;

  /* pid 控制器初始化 */
  pid.kp = p.k * 2;
  pid.ki = 0.1;
  pid.kd = 0.05;
  pid.integ = 0;
  pid.e0 = 0;

  /* adrc 控制器初始化 */
  adrc.td.x1 = adrc.td.x2 = 0;
  adrc.td.h = 3 * dt;
  adrc.td.r = 40000;

  adrc.eso.z1 = adrc.eso.z2 = adrc.eso.z3 = 0;
  adrc.eso.fe = adrc.eso.fe1 = 0;
  float omega0 = 100;
  adrc.eso.beta_01 = 3 * omega0;
  adrc.eso.beta_02 = 3 * omega0 * omega0;
  adrc.eso.beta_03 = omega0 * omega0 * omega0;
  adrc.eso.zeta = 0.05;

  adrc.pid.e0 = adrc.pid.e1 = adrc.pid.e2 = 0;
  adrc.pid.u = 0;
  adrc.pid.zeta = 0.05;
  adrc.pid.alpha0 = 0.8;
  adrc.pid.alpha1 = 0.9;
  adrc.pid.alpha2 = 0.9;
  adrc.pid.beta0 = 0.02;
  adrc.pid.beta1 = 10.0;
  adrc.pid.beta2 = 0.02;
  adrc.pid.diff_max = 100;
  adrc.pid.diff_min = -100;
  adrc.pid.integ_max = 150;
  adrc.pid.integ_min = -150;
  adrc.pid.u_max = 100;
  adrc.pid.u_min = -100;

  FILE *fp = NULL;
  fopen_s(&fp, "./pid.txt", "w");
  if (fp)
  {
    fprintf_s(fp, "pid ad_out ad_esoz1 ad_esoz2 ad_tdx1 ad_tdx2\r");
    for (; t < 100; t += dt)
    {
      if (t > 1)
      {
        sp = 10; /* 阶跃信号测试 */
      }

      /* 简单的 PID 仿真 */
      pid_update(&pid, sp, p.output, dt);
      integ_p_update(&p, dt, pid.output);

      /* adrc 仿真 */
      adrc_controller_update(&adrc, sp, p1.output, dt);
      integ_p_update(&p1, dt, adrc.pid.u);

      fprintf_s(fp, "%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\r",
                p.output, p1.output, adrc.eso.z1, adrc.eso.z2, adrc.td.x1, adrc.td.x2);
    }
    fclose(fp);
  }
}
