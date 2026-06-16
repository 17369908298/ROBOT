"""
摩擦力前馈参数辨识脚本
拟合模型: τ = k * θ̇ + fc
输入: 各转速下的稳态均值 (速度RPM, 力矩Nm)
输出: 粘滞摩擦系数 k, 库伦摩擦力 fc, 拟合图像
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

def main():
    print("=" * 50)
    print("  摩擦力前馈参数辨识 (τ = k·θ̇ + fc)")
    print("=" * 50)

    # ---------- 输入数据 ----------
    n = int(input("\n请输入数据点数量(建议4个以上): "))

    speeds_rpm = []
    torques = []

    for i in range(n):
        print(f"\n--- 第 {i+1} 组数据 ---")
        v = float(input(f"  速度均值 (RPM): "))
        t = float(input(f"  力矩均值 (Nm):  "))
        speeds_rpm.append(v)
        torques.append(t)

    # RPM → rad/s
    speeds_rads = np.array(speeds_rpm) * 2 * np.pi / 60.0
    torques = np.array(torques)

    # ---------- 最小二乘线性拟合: τ = k * ω + fc * sgn(ω) ----------
    # 对于多向测试，库伦摩擦力 fc 应该随速度变向（sgn）。
    # 构造 A * [k, fc]^T = τ
    signs = np.sign(speeds_rads)
    # 如果所有点都是同号，fc 的含义等同于截距项
    A = np.vstack([speeds_rads, signs]).T
    
    # 检查输入是否包含 0 速度，如果有则去除或处理 sgn(0) == 0 的问题
    # 这里简单处理：让 sgn(0) 默认为 1（如果有特殊需求可再改）
    signs[signs == 0] = 1
    A = np.vstack([speeds_rads, signs]).T
    
    result = np.linalg.lstsq(A, torques, rcond=None)
    k, fc = result[0]

    # 拟合残差
    torques_fit = k * speeds_rads + fc * signs
    residuals = torques - torques_fit
    r_squared = 1 - np.sum(residuals**2) / np.sum((torques - np.mean(torques))**2)

    # ---------- 输出结果 ----------
    print("\n" + "=" * 50)
    print("  拟合结果 (模型: τ = k·ω + fc·sgn(ω))")
    print("=" * 50)
    print(f"  粘滞摩擦系数 k  = {k:.6f} Nm·s/rad")
    print(f"  库伦摩擦力   fc = {fc:.6f} Nm")
    print(f"  R² = {r_squared:.6f}")
    print("=" * 50)

    # ---------- 绘图 ----------
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 子图1: 拟合曲线
    v_min, v_max = min(speeds_rads), max(speeds_rads)
    margin = (v_max - v_min) * 0.1 if v_max > v_min else 1.0
    omega_plot = np.linspace(v_min - margin, v_max + margin, 400)
    # 计算拟合曲线时也要带 sgn
    tau_plot = k * omega_plot + fc * np.sign(omega_plot)
    # 修复 sgn(0) 导致的中断显示
    tau_plot[np.abs(omega_plot) < 1e-3] = np.nan 

    axes[0].scatter(speeds_rads, torques, color='#e74c3c', s=80, zorder=5, label='实测数据点')
    axes[0].plot(omega_plot, tau_plot, color='#3498db', linewidth=2, label='拟合曲线')
    axes[0].set_xlabel('角速度 ω (rad/s)', fontsize=12)
    axes[0].set_ylabel('力矩 τ (Nm)', fontsize=12)
    axes[0].set_title(f'双向摩擦力拟合 (R² = {r_squared:.4f})', fontsize=14)
    axes[0].legend(fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='-', alpha=0.2)
    axes[0].axvline(x=0, color='k', linestyle='-', alpha=0.2)

    # 子图2: 残差图
    axes[1].stem(speeds_rads, residuals, linefmt='#2ecc71', markerfmt='o', basefmt='k-')
    axes[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[1].set_xlabel('角速度 ω (rad/s)', fontsize=12)
    axes[1].set_ylabel('残差 (Nm)', fontsize=12)
    axes[1].set_title('拟合残差', fontsize=14)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('friction_fit_result.png', dpi=150)
    plt.show()
    print("\n图片已保存为 friction_fit_result.png")

if __name__ == '__main__':
    main()
