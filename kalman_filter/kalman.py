import matplotlib.pyplot as plt
import numpy as np


class kalman_filter:
    def __init__(self, x_0, p_0, Q_cov, R_cov):
        # 初始化卡尔曼滤波的参数
        self.x_last = x_0  # 第一个数据的估计值
        self.p_last = p_0
        self.Q_cov = Q_cov  # 过程噪声
        self.R_cov = R_cov  # 测量噪声

    # 滤波函数
    def predict(self):
        # 预测部分
        x_now = self.x_last
        p_now = self.p_last + self.Q_cov
        return x_now, p_now

    def filter(self, z_s, x_0):
        filtered_state_estimates = []
        for z in z_s:
            # 预测部分
            x_now, p_now = self.predict()

            # 更新部分
            kalman_gain = p_now / (p_now + self.R_cov)
            self.x_last = x_now + kalman_gain * (z - x_now)
            self.p_last = (1 - kalman_gain) * p_now

            filtered_state_estimates.append(self.x_last)

        return filtered_state_estimates


# 生成含有噪声的数据点
num_points = 1000
true_values = np.sin(np.linspace(0, 15 * np.pi, num_points)) + np.sin(np.linspace(0, 2 * np.pi, num_points))
noise = np.random.uniform(0.1, 0.3, num_points) # 由于噪声在1%-10%看不出来变化（按考核题分数说明文件中的图），故使用10%到30%的噪声
flag = np.random.choice((-1, 1), num_points)
values_with_noise = true_values + flag * true_values * noise

fig, axs = plt.subplots(3, 3, sharex=True, sharey=False, figsize=(14, 14))  # 创建合适的画布
plt.rcParams['font.family'] = 'Microsoft YaHei'
plt.suptitle("直线轨迹滤波效果图", fontsize=20)
for i in range(3):
    for j in range(3):
        # 初始化卡尔曼滤波的一些参数
        x_0 = 0
        p_0 = 0  # 初始化后验估计协方差
        Q_cov = 0.1 * 10 ** -i  # 初始化过程噪声方差
        R_cov = 0.5 * 10 ** -j  # 初始化观测噪声方差
        kalman = kalman_filter(x_0, p_0, Q_cov, R_cov)
        filtered_estimates = kalman.filter(values_with_noise, 0)

        axs[i, j].plot(values_with_noise, label='values_with_noise', alpha=0.5)
        axs[i, j].plot(filtered_estimates, label='filtered_estimates')
        axs[i, j].legend()
        axs[i, j].set_ylabel('Value')
        axs[i, j].set_title('Q_cov:%.3f, R_cov:%.3f' % (Q_cov, R_cov))

plt.savefig('直线轨迹滤波效果图')
plt.show()
