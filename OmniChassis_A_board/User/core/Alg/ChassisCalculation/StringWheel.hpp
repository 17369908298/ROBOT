#ifndef STRINGWHEEL_HPP
#define STRINGWHEEL_HPP

#include "../user/core/Alg/ChassisCalculation/CalculationBase.hpp"
#include <math.h>

#define M_PI 3.14159265358979323846

namespace Alg::CalculationBase
{
    class String_FK : public ForwardKinematicsBase
    {
        public:
            String_FK(float r, float s, float wheel_azimuth[4], float phase[4])
                : R(r), S(s), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f)
            {
                for(int i = 0; i < 4; i++)
                {
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    current_steer_angles[i] = 0.0f;
                    Phase[i] = phase[i];
                }
            }

            void ForKinematics()
            {
                float wheel_vx = 0.0f, wheel_vy = 0.0f, sumVw = 0.0f;
                int validWheels = 0;

                ChassisVx = 0.0f;
                ChassisVy = 0.0f;
                ChassisVw = 0.0f;

                for (int i = 0; i < 4; i++)
                {
                    wheel_vx = Get_w(i) * S * cosf(current_steer_angles[i] - Phase[i]);
                    wheel_vy = Get_w(i) * S * sinf(current_steer_angles[i] - Phase[i]);

                    ChassisVx += wheel_vx;
                    ChassisVy += wheel_vy;
                    validWheels++;

                    ChassisVw += (Get_w(i) * S * sinf((current_steer_angles[i] - Phase[i]) - Wheel_Azimuth[i])) / R;
                }

                if (validWheels > 0)
                {
                    ChassisVx /= validWheels;
                    ChassisVy /= validWheels;
                    ChassisVw /= validWheels;
                }
                else
                {
                    ChassisVx = 0;
                    ChassisVy = 0;
                    ChassisVw = 0;
                }
            }

            void StringForKinematics(float w0, float w1, float w2, float w3)
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            float GetRadius() const { return R; }
            float GetScaling() const { return S; }
            float GetChassisVx() const { return ChassisVx; }
            float GetChassisVy() const { return ChassisVy; }
            float GetChassisVw() const { return ChassisVw; }
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

        private:
            float R;
            float S;
            float ChassisVx;
            float ChassisVy;
            float ChassisVw;
            float current_steer_angles[4] = {0};
            float Wheel_Azimuth[4];
            float Phase[4];
    };

    class String_ID : public InverseDynamicsBase
    {
        public:
            String_ID(float r, float s, float wheel_azimuth[4], float phase[4])
                : R(r), S(s)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    current_steer_angles[i] = 0.0f;
                    Phase[i] = phase[i];
                }
            }

            void InverseDynamics()
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = (GetFx() / 4.0f) * cosf(current_steer_angles[i] - Phase[i]) + (GetFy() / 4.0f) * sinf(current_steer_angles[i] - Phase[i]) - (GetTorque() / 4.0f) / R * S * sinf(Wheel_Azimuth[i] - (current_steer_angles[i] - Phase[i]));
                }
            }

            void StringInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            float GetMotorTorque(int index) const
            {
                if(index >= 0 && index < 4)
                {
                    return MotorTorque[index];
                }
                return 0.0f;
            }

            float GetCurrent_steer_angles(int index) { return current_steer_angles[index]; }
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

        private:
            float R;
            float S;
            float MotorTorque[4];
            float Wheel_Azimuth[4];
            float current_steer_angles[4] = {0};
            float Phase[4];
    };

    class String_IK : public InverseKinematicsBase
    {
        public:
            String_IK(float r, float s, float wheel_azimuth[4], float phase[4])
                : R(r), S(s)
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor_wheel[i] = 0.0f;
                    Motor_direction[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    Phase[i] = phase[i];
                }
            }

            float NormalizeAngle(float angle, float tar_angle)
            {
                (void)tar_angle;
                while (angle > M_PI)
                    angle -= 2.0f * M_PI;
                while (angle < -M_PI)
                    angle += 2.0f * M_PI;
                return angle;
            }

            void _Steer_Motor_Kinematics_Nearest_Transposition()
            {
                for (int i = 0; i < 4; i++)
                {
                    float tmp_delta_angle = NormalizeAngle(Motor_direction[i] - current_steer_angles[i], 2.0f * M_PI);

                    if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
                    {
                        Motor_direction[i] = tmp_delta_angle + current_steer_angles[i];
                    }
                    else
                    {
                        Motor_direction[i] = NormalizeAngle(tmp_delta_angle + M_PI, 2.0f * M_PI) + current_steer_angles[i];
                        Motor_wheel[i] *= -1.0f;
                    }
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
                    float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

                    tmp_velocity_x = Vx  - Vw * R * sinf(Wheel_Azimuth[i]);
                    tmp_velocity_y = Vy  + Vw * R * cosf(Wheel_Azimuth[i]);

                    tmp_velocity_modulus = sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y) / S;

                    Motor_wheel[i] = tmp_velocity_modulus * 60.0f / (2.0f * M_PI);

                    if (tmp_velocity_modulus < 0.05f)
                    {
                        Motor_direction[i] = current_steer_angles[i];
                        Motor_wheel[i] = 0.0f;
                    }
                    else
                    {
                        Motor_direction[i] = atan2f(tmp_velocity_y, tmp_velocity_x) + Phase[i];
                    }
                }

                _Steer_Motor_Kinematics_Nearest_Transposition();
            }

            void StringInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities();
                InvKinematics();
            }

            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            float GetMotor_wheel(int index) const
            {
                if(index >= 0 && index < 4)
                {
                    return Motor_wheel[index];
                }
                return 0.0f;
            }

            float GetMotor_direction(int index) const
            {
                if(index >= 0 && index < 4)
                {
                    return Motor_direction[index];
                }
                return 0.0f;
            }

            float GetVx() const { return Vx; }
            float GetVy() const { return Vy; }
            float GetVw() const { return Vw; }
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }
            float GetCurrent_steer_angles(int index) { return current_steer_angles[index]; }

        private:
            float Vx{0.0f};
            float Vy{0.0f};
            float Vw{0.0f};
            float R;
            float S;
            float Motor_wheel[4];
            float Motor_direction[4];
            float Wheel_Azimuth[4];
            float current_steer_angles[4] = {0};
            float Phase[4];
    };
}

#endif
