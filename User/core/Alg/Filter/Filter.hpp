#ifndef FILTER_HPP
#define FILTER_HPP


/*  =========================== 卡尔曼滤波器类 ===========================  */
class KalmanFilter
{
private:
    float X_last;   // 上一时刻的最优状态估计值
    float X_mid;    // 当前时刻的预测状态估计值
    float X_now;    // 当前时刻的最优状态估计值
    float P_mid;    // 预测状态的协方差
    float P_now;    // 最优状态的协方差
    float P_last;   // 上一时刻的协方差
    float kg;       // 卡尔曼增益，用于平衡预测值和观测值的权重
    float A;        // 状态转移系数（状态转移矩阵的元素）
    float Q;        // 预测过程噪声协方差
    float R;        // 观测噪声协方差
    float H;        // 观测矩阵系数

public:
    // 构造函数
    KalmanFilter(float T_Q = 0.0001f, float T_R = 0.0001f);
    
    // 滤波处理函数
    float filter(float dat);
    
    // 重新初始化参数
    void reinit(float T_Q, float T_R);

    float getState() const;        // 获取当前状态估计值
    float getPrediction() const;   // 获取当前预测值
    float getGain() const;         // 获取当前卡尔曼增益
};


// 梯形/s型位置规划器
class TrajectoryPlanner {
private:
    float v_max;      // 允许的最大速度 (rad/s)
    float a_max;      // 允许的最大加速度 (rad/s^2)
    float dt;         // 控制周期 (例如 0.002s)

    float current_pos;// 当前规划出的位置
    float current_vel;// 当前规划出的速度

public:
    TrajectoryPlanner(float _v_max, float _a_max, float _dt) 
        : v_max(_v_max), a_max(_a_max), dt(_dt), current_pos(0.0f), current_vel(0.0f) {}

    // 强行同步内部状态（用于 STOP 状态的无扰切换）
    void Reset(float init_pos);
    

    // 更新规划器，输入目标位置，返回当前应当到达的位置
    float Update(float target_pos);

    // 获取规划出的前馈速度
    float GetVelocity() { return current_vel; }
};

/*  =========================== TD跟踪微分器类 ===========================  */
class TDFilter
{
private:
    float v1, v2;   // 状态变量：v1为跟踪信号，v2为微分信号
    float R;        // 速度因子，决定跟踪速度的快慢
    float H;        // 积分步长

public:
    // 构造函数
    TDFilter(float init_R = 100.0f, float init_H = 0.01f);
    
    // 滤波处理函数
    float filter(float Input);
    
    // 重新设置参数
    void setParams(float new_R, float new_H);

    // 同步内部状态，常用于模式切换时避免跳变
    void reset(float value = 0.0f);

    float getDerivative() const;      // 获取微分信号
};




/*  =========================== 一阶低通滤波器类 ===========================  */
class LPFFilter
{
private:
    float Last_Out;  // 上次滤波输出值
    float Ratio;     // 滤波系数(0-1)：值越大响应越快，滤波效果越弱

public:
    // 构造函数
    LPFFilter(float ratio = 0.5f);
    
    // 滤波处理函数
    float filter(float Input);
    
    // 设置滤波系数
    void setRatio(float ratio);

    // 同步滤波器输出，常用于模式切换时避免历史值突跳
    void reset(float value = 0.0f);
    
    
    float getOutput() const;    // 获取当前输出值
    float getRatio() const;    // 获取当前滤波系数
};




/*  =========================== 限幅滤波器类 ===========================  */
class LMFFilter
{
private:
    float Last_Out;      // 上次滤波输出值
    float Limit_Ratio;   // 最大允许变化量（限制幅度）

public:
    // 构造函数
    LMFFilter(float limit_ratio = 1.0f);
    
    // 滤波处理函数
    float filter(float Input);
    
    // 设置限制幅度
    void setLimit(float limit_ratio);
    
    
    float getOutput() const;        // 获取当前输出值
    float getLimitRatio() const;    // 获取当前限制幅度
};

#endif
