import cv2
import numpy as np


class kalman_filter:
    def __init__(self, x_0, p_0, Q_cov, R_cov):
        self.x_last = x_0  # 初始状态量
        self.p_last = p_0  # 上一时刻后验估计协方差
        self.Q_cov = Q_cov  # 过程噪声方差
        self.R_cov = R_cov  # 观测噪声方差

    def predict(self, speed):
        # 预测部分
        x_now = self.x_last + speed
        p_now = self.p_last + self.Q_cov
        return x_now, p_now

    def update(self, z, speed):
        x_now, p_now = self.predict(speed)
        # 更新部分
        kalman_gain = p_now @ np.linalg.inv(p_now + self.Q_cov)
        self.x_last = x_now + kalman_gain @ (z - x_now)
        self.p_last = (np.eye(len(self.x_last)) - kalman_gain) @ p_now

        return self.x_last

    def predict_next_x(self, z, speed): # 预测下一刻球的坐标
        x_now, p_now = self.predict(speed)
        return x_now + p_now @ np.linalg.inv(p_now + self.Q_cov) @ (z - x_now)


# 初始化卡尔曼滤波的一些参数
x_0 = np.array([0, 0])  # 初始化小球坐标
p_0 = np.eye(2)  # 初始化后验估计协方差
Q_cov = np.diag([0.001, 0.001])  # 初始化过程噪声方差
R_cov = np.diag([0.1, 0.1])  # 初始化观测噪声方差

kalman_filter = kalman_filter(x_0, p_0, Q_cov, R_cov)

cap = cv2.VideoCapture("KalmanFilter_test_video.mp4")
cv2.namedWindow("Frame", cv2.WINDOW_FREERATIO)
out = cv2.VideoWriter('kalman.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
last_circle = [0, 0]
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        frame_inrange = cv2.inRange(frame, (0, 0, 0), (255, 230, 255))
        frame_inrange = cv2.erode(frame_inrange, (3, 3))
        edges = cv2.Canny(frame_inrange, 50, 150)
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=100, param2=9, minRadius=10,
                                   maxRadius=100)
        arr1 = np.zeros([0, 2], dtype=int)  # 创建一个0行, 2列的空数组
        if circles is not None:
            circle = np.int16(np.around(circles))[0][0]  # 4舍5入, 然后转为uint16
            cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)  # 轮廓
            cv2.circle(frame, (circle[0], circle[1]), 2, (0, 0, 0), 6)  # 圆心
            if last_circle[0] != 0:
                z = np.array([circle[0], circle[1]])
                speed = [circle[0] - last_circle[0], circle[1] - last_circle[1]]    # 计算上一时间段球的平均速度
                kalman_filter.update(z, speed)
                predicted_next_position = kalman_filter.predict_next_x(z, speed)
                cv2.circle(frame, (int(predicted_next_position[0]), int(predicted_next_position[1])), 2, (0, 0, 0),
                           6)  # 圆心
                cv2.circle(frame, (int(predicted_next_position[0]), int(predicted_next_position[1])), circle[2],
                           (0, 255, 0), 2)  # 轮廓
            last_circle[0] = circle[0]
            last_circle[1] = circle[1]
    cv2.imshow('Frame', frame)
    cv2.waitKey(1)
    out.write(frame)
cap.release()
out.release()
cv2.destroyAllWindows()