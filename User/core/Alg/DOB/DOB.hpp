#ifndef DOB_HPP
#define DOB_HPP

namespace ALG::DOB
{
class StandardDOB
{
public:
    StandardDOB(float J_, float wc_, float dt_);

    void Reset(float init_speed);

    /**
     * @brief 标准 DOB 更新逻辑，直接使用电机的真实物理反馈扭矩。
     * @param v_real   当前真实角速度 (rad/s)
     * @param t_actual 当前真实的电机反馈扭矩 (Nm)
     */
    float Update(float v_real, float t_actual);

private:
    float J;
    float wc;
    float dt;
    float filter_state;
};
}  // namespace ALG::DOB

#endif  // DOB_HPP
