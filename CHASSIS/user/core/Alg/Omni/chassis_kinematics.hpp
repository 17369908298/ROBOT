#pragma once

#include "user/core/Alg/Omni/kinematics_base.hpp"
#include <cmath>

namespace ALG
{
namespace Omni
{

/** @brief 根据车轮转速计算底盘线速度和角速度。 */
class ForwardKinematicsBase
{
  public:
    ForwardKinematicsBase()
    {
        for (int i = 0; i < 4; i++)
            w[i] = 0.0f;
    }
    virtual ~ForwardKinematicsBase() = default;

    void Set_w0w1w2w3(float w0, float w1, float w2, float w3)
    {
        w[0] = w0;
        w[1] = w1;
        w[2] = w2;
        w[3] = w3;
    }

    float Get_w(int index) const
    {
        if (index >= 0 && index < 4)
            return w[index];
        return 0.0f;
    }

  protected:
    float w[4];
};

/** @brief 根据底盘运动目标求各轮的动力学输出。 */
class InverseDynamicsBase
{
  public:
    InverseDynamicsBase() : Fx(0.0f), Fy(0.0f), Torque(0.0f) {}
    virtual ~InverseDynamicsBase() = default;

    float GetFx() const { return Fx; }
    float GetFy() const { return Fy; }
    float GetTorque() const { return Torque; }

    void Set_FxFyTor(float fx, float fy, float torque)
    {
        Fx = fx;
        Fy = fy;
        Torque = torque;
    }

  protected:
    float Fx, Fy, Torque;
};

/** @brief 将平面运动速度换算为各轮速度和方向。 */
class InverseKinematicsBase
{
  public:
    InverseKinematicsBase()
        : Signal_x(0.0f), Signal_y(0.0f), Signal_w(0.0f), Phase(0.0f), SpeedGain(1.0f),
          RotationalGain(1.0f)
    {
    }
    virtual ~InverseKinematicsBase() = default;

    void SetSignal_xyw(float x, float y, float w_)
    {
        Signal_x = x;
        Signal_y = y;
        Signal_w = w_;
    }
    void SetPhase(float phase) { Phase = phase; }
    void SetSpeedGain(float gain) { SpeedGain = gain; }
    void SetRotationalGain(float gain) { RotationalGain = gain; }

    float GetSignal_x() const { return Signal_x; }
    float GetSignal_y() const { return Signal_y; }
    float GetSignal_w() const { return Signal_w; }
    float GetPhase() const { return Phase; }
    float GetSpeedGain() const { return SpeedGain; }
    float GetRotationalGain() const { return RotationalGain; }

  protected:
    float Signal_x, Signal_y, Signal_w;
    float Phase, SpeedGain, RotationalGain;
};

/** @brief 麦克纳姆轮底盘正运动学。 */
class Omni_FK : public ForwardKinematicsBase
{
  public:
    Omni_FK(float r, float s, float n, float wheel_azimuth[4], float wheel_direction[4])
        : R(r), S(s), N(n), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            WheelAzimuth[i] = wheel_azimuth[i];
            WheelDirection[i] = wheel_direction[i];
        }
    }

    void ForKinematics()
    {
        float wheel_vx = 0.0f, wheel_vy = 0.0f;
        ChassisVx = 0.0f;
        ChassisVy = 0.0f;
        ChassisVw = 0.0f;

        for (int i = 0; i < 4; i++)
        {

            wheel_vx = S * Get_w(i) * cosf(WheelDirection[i]);
            wheel_vy = S * Get_w(i) * sinf(WheelDirection[i]);
            ChassisVx += wheel_vx;
            ChassisVy += wheel_vy;
            float delta_angle = WheelAzimuth[i] - WheelDirection[i];
            ChassisVw += (Get_w(i) * S * sinf(delta_angle)) / R;
        }
        ChassisVx /= (N / 2);
        ChassisVy /= (N / 2);
        ChassisVw /= N;
    }

    void OmniForKinematics(float w0, float w1, float w2, float w3)
    {
        Set_w0w1w2w3(w0, w1, w2, w3);
        ForKinematics();
    }

    float GetChassisVx() const { return ChassisVx; }
    float GetChassisVy() const { return ChassisVy; }
    float GetChassisVw() const { return ChassisVw; }
    float GetWheelAzimuth(int index) { return WheelAzimuth[index]; }
    float GetWheelDirection(int index) { return WheelDirection[index]; }

  private:
    float R, S, N;
    float ChassisVx, ChassisVy, ChassisVw;
    float WheelAzimuth[4], WheelDirection[4];
};

/** @brief 全向轮底盘逆动力学计算。 */
class Omni_ID : public InverseDynamicsBase
{
  public:
    Omni_ID(float r, float s, float n, const float wheel_azimuth[4], const float wheel_direction[4])
        : R(r), S(s), N(n), AllocationValid(false)
    {
        for (int i = 0; i < 4; i++)
        {
            MotorTorque[i] = 0.0f;
            WheelAzimuth[i] = wheel_azimuth[i];
            WheelDirection[i] = wheel_direction[i];
        }
        build_allocation();
    }

    void InverseDynamics()
    {
        if (!AllocationValid)
        {
            for (int i = 0; i < 4; ++i)
            {
                MotorTorque[i] = 0.0f;
            }
            return;
        }

        for (int i = 0; i < 4; i++)
        {
            MotorTorque[i] = Allocation[i][0] * GetFx() + Allocation[i][1] * GetFy() +
                             Allocation[i][2] * GetTorque();
        }
    }

    void OmniInvDynamics(float fx, float fy, float torque)
    {
        Set_FxFyTor(fx, fy, torque);
        InverseDynamics();
    }

    float GetMotorTorque(int index) const
    {
        if (index >= 0 && index < 4)
            return MotorTorque[index];
        return 0.0f;
    }

    bool IsAllocationValid() const { return AllocationValid; }

  private:
    static constexpr int WHEEL_COUNT = 4;

    static bool invert_3x3(const float input[3][3], float inverse[3][3])
    {
        const float a00 = input[0][0];
        const float a01 = input[0][1];
        const float a02 = input[0][2];
        const float a10 = input[1][0];
        const float a11 = input[1][1];
        const float a12 = input[1][2];
        const float a20 = input[2][0];
        const float a21 = input[2][1];
        const float a22 = input[2][2];

        const float det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
                          a02 * (a10 * a21 - a11 * a20);
        if (!std::isfinite(det) || std::fabs(det) < 1.0e-8f)
        {
            return false;
        }

        const float inv_det = 1.0f / det;
        inverse[0][0] = (a11 * a22 - a12 * a21) * inv_det;
        inverse[0][1] = -(a01 * a22 - a02 * a21) * inv_det;
        inverse[0][2] = (a01 * a12 - a02 * a11) * inv_det;
        inverse[1][0] = -(a10 * a22 - a12 * a20) * inv_det;
        inverse[1][1] = (a00 * a22 - a02 * a20) * inv_det;
        inverse[1][2] = -(a00 * a12 - a02 * a10) * inv_det;
        inverse[2][0] = (a10 * a21 - a11 * a20) * inv_det;
        inverse[2][1] = -(a00 * a21 - a01 * a20) * inv_det;
        inverse[2][2] = (a00 * a11 - a01 * a10) * inv_det;
        return true;
    }

    void build_allocation()
    {
        float wheel_jacobian[WHEEL_COUNT][3]{};
        float gram[3][3]{};

        if (!std::isfinite(R) || !std::isfinite(S) || !std::isfinite(N) || R <= 0.0f || S <= 0.0f ||
            N <= 0.0f)
        {
            return;
        }

        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_jacobian[i][0] = cosf(WheelDirection[i]);
            wheel_jacobian[i][1] = sinf(WheelDirection[i]);
            wheel_jacobian[i][2] = R * sinf(WheelAzimuth[i] - WheelDirection[i]);

            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    gram[row][column] += wheel_jacobian[i][row] * wheel_jacobian[i][column];
                }
            }
        }

        float gram_inverse[3][3]{};
        if (!invert_3x3(gram, gram_inverse))
        {
            return;
        }

        for (int i = 0; i < WHEEL_COUNT; ++i)
        {
            for (int column = 0; column < 3; ++column)
            {
                Allocation[i][column] = S * (wheel_jacobian[i][0] * gram_inverse[0][column] +
                                             wheel_jacobian[i][1] * gram_inverse[1][column] +
                                             wheel_jacobian[i][2] * gram_inverse[2][column]);
            }
        }
        AllocationValid = true;
    }

    float R, S, N;
    float MotorTorque[4];
    float WheelAzimuth[4], WheelDirection[4];
    float Allocation[4][3]{};
    bool AllocationValid;
};

/** @brief 全向轮底盘逆运动学及转向角平滑处理。 */
class Omni_IK : public InverseKinematicsBase, public IKinematics
{
  public:
    Omni_IK(float r, float s, const float wheel_azimuth[4], const float wheel_direction[4])
        : R(r), S(s), Vx(0.0f), Vy(0.0f), Vw(0.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            Motor[i] = 0.0f;
            WheelAzimuth[i] = wheel_azimuth[i];
            WheelDirection[i] = wheel_direction[i];
        }
    }

    void CalculateVelocities()
    {
        Vx = GetSpeedGain() * GetSignal_x();
        Vy = GetSpeedGain() * GetSignal_y();
        Vw = GetRotationalGain() * GetSignal_w();
    }

    void InvKinematics()
    {
        for (int i = 0; i < 4; i++)
        {
            Motor[i] = (cosf(WheelDirection[i]) * Vx + sinf(WheelDirection[i]) * Vy +
                        Vw * R * sinf(WheelAzimuth[i] - WheelDirection[i])) /
                       S;
        }
    }

    void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain,
                           float rotate_gain)
    {
        SetPhase(phase);
        SetSpeedGain(speed_gain);
        SetRotationalGain(rotate_gain);
        SetSignal_xyw(vx, vy, vw);
        CalculateVelocities();
        InvKinematics();
    }

    void inverse_kinematics(float vx, float vy, float vw, WheelOutput out[4])
    {
        OmniInvKinematics(vx, vy, vw, 0.0f, 1.0f, 1.0f);
        for (int i = 0; i < 4; i++)
        {
            out[i].speed_rpm = Motor[i];
            out[i].angle_enc = 0.0f;
        }
    }

    float GetMotor(int index) const
    {
        if (index >= 0 && index < 4)
            return Motor[index];
        return 0.0f;
    }

    float GetVx() const { return Vx; }
    float GetVy() const { return Vy; }
    float GetVw() const { return Vw; }

  private:
    float R, S, Vx, Vy, Vw;
    float Motor[4];
    float WheelAzimuth[4], WheelDirection[4];
};

} // namespace Omni
} // namespace ALG
