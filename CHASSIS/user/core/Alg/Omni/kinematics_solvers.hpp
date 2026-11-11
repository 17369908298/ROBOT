#pragma once

#include "user/core/Alg/Omni/kinematics_solver_base.hpp"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cmath>

namespace ALG
{
namespace Omni
{

using namespace Sentry::Utils::Constants;

/** @brief 麦克纳姆轮正、逆运动学求解器。 */
class MecanumSolver : public IKinematicsSolver
{
  public:
    struct Config
    {
        float wheel_radius = 0.05f;
        float wheelbase_x = 0.4f;
        float wheelbase_y = 0.3f;
        float gear_ratio = 1.0f;
        float chassis_direction[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    };

    explicit MecanumSolver(const Config &config) : config_(config) {}

    uint8_t get_wheel_count() const { return 4; }

    void inverse_kinematics(float vx, float vy, float vw, WheelOutput *out)
    {
        float r = config_.wheel_radius;
        float lx = config_.wheelbase_x;
        float ly = config_.wheelbase_y;
        float k = config_.gear_ratio;

        float v[4] = {-(vx + vy + vw * (lx + ly)), (vx - vy - vw * (lx + ly)),
                      -(vx - vy + vw * (lx + ly)), (vx + vy - vw * (lx + ly))};

        for (int i = 0; i < 4; i++)
        {
            out[i].speed_rpm = v[i] / r * k * config_.chassis_direction[i] * RAD_PER_SEC_TO_RPM;
        }
    }

    void forward_kinematics(const float *wheel_speeds, ChassisOdometry &odom)
    {
        float r = config_.wheel_radius;
        float lx = config_.wheelbase_x;
        float ly = config_.wheelbase_y;
        float k = config_.gear_ratio;

        float w[4];
        for (int i = 0; i < 4; i++)
        {
            w[i] = wheel_speeds[i] * RPM_TO_RAD_PER_SEC / k * config_.chassis_direction[i];
        }

        odom.vx = (-w[0] + w[1] - w[2] + w[3]) * r / 4.0f;
        odom.vy = (-w[0] - w[1] + w[2] - w[3]) * r / 4.0f;
        odom.vw = (-w[0] + w[1] - w[2] + w[3]) * r / (4.0f * (lx + ly));
    }

    const char *get_solver_name() const { return "MecanumSolver"; }

  private:
    Config config_;
};

/** @brief 全向轮正、逆运动学求解器。 */
class OmniSolver : public IKinematicsSolver
{
  public:
    struct Config
    {
        float radius = 0.2f;
        float scaling = 0.04f;
        float wheel_azimuth[4] = {0.785f, 2.356f, 3.927f, 5.498f};
        float wheel_direction[4] = {0.785f, 2.356f, 3.927f, 5.498f};
        float chassis_direction[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    };

    explicit OmniSolver(const Config &config)
    {
        const float linear_scale = RPM_TO_RAD_PER_SEC * config.scaling;
        for (int i = 0; i < 4; ++i)
        {
            const float c = std::cos(config.wheel_direction[i]);
            const float s = std::sin(config.wheel_direction[i]);
            const float rotational = std::sin(config.wheel_azimuth[i] - config.wheel_direction[i]);
            const float rpm_scale =
                config.chassis_direction[i] * RAD_PER_SEC_TO_RPM / config.scaling;
            inverse_[i][0] = c * rpm_scale;
            inverse_[i][1] = s * rpm_scale;
            inverse_[i][2] = config.radius * rotational * rpm_scale;
            forward_[i][0] = linear_scale * c * 0.5f;
            forward_[i][1] = linear_scale * s * 0.5f;
            forward_[i][2] = linear_scale * rotational / config.radius * 0.25f;
        }
    }

    uint8_t get_wheel_count() const { return 4; }

    void inverse_kinematics(float vx, float vy, float vw, WheelOutput *out)
    {
        for (int i = 0; i < 4; i++)
        {
            out[i].speed_rpm = inverse_[i][0] * vx + inverse_[i][1] * vy + inverse_[i][2] * vw;
            out[i].angle_enc = 0.0f;
        }
    }

    void forward_kinematics(const float *wheel_speeds, ChassisOdometry &odom)
    {
        float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            vx_sum += wheel_speeds[i] * forward_[i][0];
            vy_sum += wheel_speeds[i] * forward_[i][1];
            vw_sum += wheel_speeds[i] * forward_[i][2];
        }

        odom.vx = vx_sum;
        odom.vy = vy_sum;
        odom.vw = vw_sum;
    }

    const char *get_solver_name() const { return "OmniSolver"; }

  private:
    float inverse_[4][3]{};
    float forward_[4][3]{};
};

/** @brief 差速底盘正、逆运动学求解器。 */
class DifferentialSolver : public IKinematicsSolver
{
  public:
    struct Config
    {
        float wheel_radius = 0.05f;
        float wheelbase = 0.4f;
        float gear_ratio = 1.0f;
    };

    explicit DifferentialSolver(const Config &config) : config_(config) {}

    uint8_t get_wheel_count() const { return 2; }

    void inverse_kinematics(float vx, float vy, float vw, WheelOutput *out)
    {
        (void)vy;

        float r = config_.wheel_radius;
        float L = config_.wheelbase;
        float k = config_.gear_ratio;

        float v_left = (vx - vw * L / 2.0f) / r * k;
        float v_right = (vx + vw * L / 2.0f) / r * k;

        out[0].speed_rpm = v_left;
        out[1].speed_rpm = v_right;
    }

    void forward_kinematics(const float *wheel_speeds, ChassisOdometry &odom)
    {
        float r = config_.wheel_radius;
        float L = config_.wheelbase;
        float k = config_.gear_ratio;

        float w_left = wheel_speeds[0] / k;
        float w_right = wheel_speeds[1] / k;

        odom.vx = (w_left + w_right) * r / 2.0f;
        odom.vy = 0.0f;
        odom.vw = (w_right - w_left) * r / L;
    }

    const char *get_solver_name() const { return "DifferentialSolver"; }

  private:
    Config config_;
};

/** @brief 转向轮底盘正、逆运动学求解器。 */
class StringWheelSolver : public IKinematicsSolver
{
  public:
    struct Config
    {
        float wheel_radius = 0.05f;
        float wheelbase_radius = 0.2f;
        float wheel_scaling = 0.04f;
        float wheel_azimuth[4] = {0.785f, 2.356f, 3.927f, 5.498f};
        float phase[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float gear_ratio = 1.0f;
        float chassis_direction[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    };

    explicit StringWheelSolver(const Config &config)
        : config_(config), current_steer_angles_{0.0f, 0.0f, 0.0f, 0.0f}
    {
    }

    uint8_t get_wheel_count() const { return 4; }

    void inverse_kinematics(float vx, float vy, float vw, WheelOutput *out)
    {
        float R = config_.wheelbase_radius;
        float S = config_.wheel_scaling;
        float k = config_.gear_ratio;

        for (int i = 0; i < 4; i++)
        {
            float tmp_speed_x = vx - vw * R * sinf(config_.wheel_azimuth[i]);
            float tmp_speed_y = vy + vw * R * cosf(config_.wheel_azimuth[i]);
            float tmp_speed_modulus =
                sqrtf(tmp_speed_x * tmp_speed_x + tmp_speed_y * tmp_speed_y) / S;

            float motor_wheel = tmp_speed_modulus * RAD_PER_SEC_TO_RPM * k;
            float motor_direction;

            if (tmp_speed_modulus < 0.05f)
            {
                motor_direction = current_steer_angles_[i];
                motor_wheel = 0.0f;
            }
            else
            {
                motor_direction = atan2f(tmp_speed_y, tmp_speed_x) + config_.phase[i];
            }

            float tmp_delta_angle =
                Sentry::Utils::normalize_angle(motor_direction - current_steer_angles_[i]);

            if (-HALF_PI <= tmp_delta_angle && tmp_delta_angle <= HALF_PI)
            {
                out[i].speed_rpm = motor_wheel * config_.chassis_direction[i];
                out[i].angle_enc = rad_to_encoder(tmp_delta_angle + current_steer_angles_[i]);
            }
            else
            {
                out[i].speed_rpm = -motor_wheel * config_.chassis_direction[i];
                out[i].angle_enc =
                    rad_to_encoder(Sentry::Utils::normalize_angle(tmp_delta_angle + PI) +
                                   current_steer_angles_[i]);
            }
        }
    }

    void forward_kinematics(const float *wheel_speeds, ChassisOdometry &odom)
    {
        float S = config_.wheel_scaling;

        float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            float w = wheel_speeds[i] * config_.chassis_direction[i];
            float angle = current_steer_angles_[i];

            vx_sum += w * S * cosf(angle - config_.phase[i]);
            vy_sum += w * S * sinf(angle - config_.phase[i]);
            vw_sum += (w * S * sinf((angle - config_.phase[i]) - config_.wheel_azimuth[i])) /
                      config_.wheelbase_radius;
        }

        odom.vx = vx_sum / 4.0f;
        odom.vy = vy_sum / 4.0f;
        odom.vw = vw_sum / 4.0f;
    }

    const char *get_solver_name() const { return "StringWheelSolver"; }

    void update_steer_angles(const float *encoders)
    {
        for (int i = 0; i < 4; i++)
        {
            current_steer_angles_[i] = encoder_to_rad(encoders[i]);
        }
    }

  private:
    static float encoder_to_rad(float enc) { return enc * (TWO_PI / 8192.0f); }

    static float rad_to_encoder(float rad)
    {
        float enc = rad * (8192.0f / TWO_PI);

        while (enc < 0.0f)
            enc += 8192.0f;
        while (enc >= 8192.0f)
            enc -= 8192.0f;
        return enc;
    }

    Config config_;
    float current_steer_angles_[4];
};

} // namespace Omni
} // namespace ALG
