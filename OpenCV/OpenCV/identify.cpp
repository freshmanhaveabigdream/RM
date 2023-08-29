#include "identify.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<fstream>
#define pi 3.1415926

identification::identification() {	// 初始化参数，用于区分每个不同颜色曝光的目标视频
	this->armored_plate_parameter_array = {
		std::make_pair(cv::Scalar(0, 0, 255), cv::Scalar(0, 5, 255)),
		std::make_pair(cv::Scalar(0, 0, 255), cv::Scalar(100, 150, 255)),
		std::make_pair(cv::Scalar(0, 30, 200), cv::Scalar(255, 255, 255)),
		std::make_pair(cv::Scalar(0, 70, 255), cv::Scalar(255, 255, 255)),
	};
	this->center_parameter_array = {
		std::make_pair(cv::Scalar(0, 0, 255), cv::Scalar(110, 55, 255)),
		std::make_pair(cv::Scalar(0, 0, 255), cv::Scalar(255, 160, 255)),
		std::make_pair(cv::Scalar(0, 0, 97), cv::Scalar(255, 255, 255)),
		std::make_pair(cv::Scalar(0, 70, 200), cv::Scalar(255, 255, 255)),
	};
}

cv::Point2f identification::rotatePoint(cv::Point2f cur_Point, cv::Point2f center, float angle) {
	float cosA = cos(angle);
	float sinA = sin(angle);
	float translatedX = cur_Point.x - center.x;
	float translatedY = center.y - cur_Point.y;  // 取反以适应数学坐标系
	return cv::Point2f(
		translatedX * cosA - translatedY * sinA + center.x,
		center.y - (translatedX * sinA + translatedY * cosA)  // 取反以适应数学坐标系
	);
}

void identification::save(std::string path, std::vector<float> info) {
	std::ofstream outFile;
	outFile.open(path, std::ios::out);
	for (auto iter : info)
	{
		outFile << iter << std::endl;
	}
	outFile.close();
}

void identification::identify(std::string path, int type_number) {
	type_number -= 1;
	cv::VideoCapture cap(path);	//打开视频
	cv::Mat frame, frame_temp, armored_plate_temp, center_temp;	//定义mat变量，分别为视频的一帧、一帧视频的临时变量、检测装甲板的临时变量、检测中心的临时变量
	cv::Mat kernel_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
	cv::Mat kernel_3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::Mat kernel_4 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));	//定义用于图像腐蚀扩张的kernel核
	cv::Point2f last_position(0, 0); //用于存储上一帧检测到的装甲板的中心点
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours_center;
	std::vector<cv::Vec4i> hierarchy_center;	//定义存储轮廓、层级信息的矩阵

	int flag = -1;	//定义标志，-1表示本轮未检测到未击打装甲板，0表示检测到新的未击打装甲板,1表示检测到上一帧检测到的未击打装甲板
	float last_angle = -10;	//存储上一帧检测到的装甲板中心位置和上一帧检测到的圆心的夹角，初始值为-10
	float linear_velocity = 0;	//存储当前时刻的矩形框中心线速度
	float angular_velocity = 0;	//存储当前时刻的矩形框角速度
	float angle;	//存储在此刻角速度下的单位时间旋转角度
	int number = 0;	//存储连续几帧没检测到矩形框

	std::vector<float> angular_velocity_record;	//记录装甲板的历史角速度
	int frame_rate = static_cast<int>(cap.get(cv::CAP_PROP_FPS));	// 获取帧率
	float time_per_frame = 1.0 / frame_rate;	// 获取每一帧的时长
	int frameCounter = 0;	//记录当前是第几帧
	int Frame_skipping = 0;	//是否跳帧的标志，因为经过验证，低曝光的视频存在每两帧相同的情况，所以跳过偶数帧解决这个问题
	if (type_number == 2 || type_number == 3) {	//如果是低曝光视频，则跳帧，并且把处理的每帧的时长乘2
		time_per_frame *= 2;
		Frame_skipping = 1;
		frame_rate /= 2;
	}
	int frameWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
	int frameHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
	std::stringstream video_id;
	video_id << type_number + 1;
	cv::VideoWriter outputVideo("videos/output" + video_id.str() + ".avi", cv::VideoWriter::fourcc('M', 'P', '4', '2'), frame_rate, cv::Size(frameWidth, frameHeight));
	while (cap.read(frame)) {
		frameCounter++;
		if (frameCounter % 2 == 0 && Frame_skipping != 0)continue;	//跳过偶数帧
		double time = static_cast<double>(cv::getTickCount());	//记录本轮循环起始时间
		frame_temp = frame.clone();

		//图像基础处理部分
		cv::cvtColor(frame_temp, frame_temp, cv::COLOR_BGR2HSV);	//将图像转换为HSV格式
		cv::inRange(frame_temp, this->armored_plate_parameter_array[type_number].first, this->armored_plate_parameter_array[type_number].second, armored_plate_temp);	//提取出目标颜色
		cv::inRange(frame_temp, this->center_parameter_array[type_number].first, this->center_parameter_array[type_number].second, center_temp);
		cv::dilate(armored_plate_temp, armored_plate_temp, kernel_4);
		cv::erode(armored_plate_temp, armored_plate_temp, kernel_2);
		cv::dilate(center_temp, center_temp, kernel_3);
		cv::erode(center_temp, center_temp, kernel_1);	//对图像进行扩张和腐蚀处理
		
		//检测图像部分
		cv::findContours(armored_plate_temp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(center_temp, contours_center, hierarchy_center, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		
		//定义检测到的装甲板最小外接圆的中心、半径以及检测到的圆心、半径（圆心附近矩形的最小外接圆半径）
		cv::Point2f armored_plate;
		float armored_plate_radius;
		cv::Point2f center;
		float center_radius;

		//定义存储多边形拟合后的多边形顶点的数组
		std::vector<std::vector<cv::Point>> armored_plate_conPloy(contours.size());
		std::vector<std::vector<cv::Point>> center_conPloy(contours_center.size());

		//定义目标轮廓在轮廓数组中的下标
		int aim;

		int armored_plate_area = 0;	//存储当前装甲板面积
		for (aim = 0; aim < contours.size(); aim++) {	//遍历检测到的所有轮廓
			int area = cv::contourArea(contours[aim]);	//计算轮廓所围面积
			float peri = cv::arcLength(contours[aim], true);	//计算图形轮廓周长
			cv::approxPolyDP(contours[aim], armored_plate_conPloy[aim], 0.02 * peri, true);	//把一个连续光滑曲线折线化，对图像轮廓点进行多边形拟合
			if (area > 1000) {	//剔除无关的点
				if (static_cast<int>(armored_plate_conPloy[aim].size()) == 4 && hierarchy[aim][3] != -1 && hierarchy[hierarchy[aim][3]][3] == -1 && hierarchy[aim][2] == -1) {	//判断条件：拟合多边形为四边形，有父轮廓且父轮廓为最外层轮廓，没有子轮廓
					if ((hierarchy[aim][0] > 0 && cv::contourArea(contours[hierarchy[aim][0]]) > 1000) || (hierarchy[aim][1] > 0 && cv::contourArea(contours[hierarchy[aim][1]]) > 1000)) {	//判断条件：没有面积大于1000的同级轮廓
						continue;
					}
					cv::drawContours(frame, contours, aim, cv::Scalar(0, 255, 255), 3);
					armored_plate_area = area;	//将当前检测到的装甲板面积存储起来
					cv::minEnclosingCircle(armored_plate_conPloy[aim], armored_plate, armored_plate_radius);	//找出装甲板中心点
					if (flag == -1)flag = 0;	//如果此前未检测到装甲板则flag变为0，表示检测到新的未击打装甲板
					else if (flag == 0)flag = 1;	//如果此前已检测到装甲板则flag变为1，表示检测到上一帧检测到的未击打装甲板
					break;
				}
			}
		}
		if (aim == contours.size()) {	//判断条件：本轮未检测到装甲板
			flag = -1;
			number++;	//如果未检测到，number加一，避免下一次计算的速度异常
		}
		for (int i = 0; i < contours_center.size(); i++) {	//遍历检测到的所有轮廓
			int area = cv::contourArea(contours_center[i]);
			std::vector<std::vector<cv::Point>> center_conPloy(contours_center.size());
			if (area < armored_plate_area / 5 && area > armored_plate_area / 10) {	//判断条件：当前轮廓所围面积在装甲板识别面积的1/10~1/5之间
				float peri = cv::arcLength(contours_center[i], true);
				cv::approxPolyDP(contours_center[i], center_conPloy[i], 0.09 * peri, true);
				if (static_cast<int>(center_conPloy[i].size()) == 4) {	//判断条件：拟合多边形为四边形
					cv::drawContours(frame, contours_center, i, cv::Scalar(0, 255, 255), 3);
					cv::minEnclosingCircle(center_conPloy[i], center, center_radius);
					cv::circle(frame, center, center_radius, cv::Scalar(255, 255, 0), 9);	//画出中心圆
					break;
				}
			}
		}
		if (flag == 0) {	//判断条件：检测到新的未击打装甲板或者
			last_position = armored_plate;	//将此刻的装甲板中心位置标为上一刻，不输出速度
			last_angle = atan((armored_plate.x - center.x) / (armored_plate.y - center.y));
			std::cout << "本轮未检测到未击打装甲板" << std::endl;
		}
		else if (flag == 1) {	//判断条件：检测到上一帧检测到的未击打装甲板
			angular_velocity = (atan((armored_plate.x - center.x) / (armored_plate.y - center.y)) - last_angle) / time_per_frame / (number + 1);	//计算角速度（s为单位）
			if (last_angle == -10 || abs(angular_velocity) < 2 * pi) {	//判断条件：这是第一次（用last_angle来判定）有同一装甲板的两个连续中心点记录或者本次测算的角速度小于约360度每秒
				number = 0;
				if (angular_velocity > 0) {
					std::cout << "当前旋转方向为逆时针" << std::endl;
				}
				else {
					std::cout << "当前旋转方向为顺时针" << std::endl;
				}
				//计算旋转的角度（一秒）
				if (type_number == 0 || type_number == 1) {
					// 如果是标准曝光视频则求出转速为10rpm下的预测角度
					angle = 10.0 / 60 * 2 * pi;
					if (angular_velocity <= 0) angle *= -1;	//若是顺时针旋转，则需要取角度的相反数
				}else {
					// 如果是低曝光视频则求出当前角速度下的预测角度
					angle = angular_velocity * 1;
				}
				armored_plate_conPloy[aim][0] = this->rotatePoint(armored_plate_conPloy[aim][0], center, angle);
				armored_plate_conPloy[aim][1] = this->rotatePoint(armored_plate_conPloy[aim][1], center, angle);
				armored_plate_conPloy[aim][2] = this->rotatePoint(armored_plate_conPloy[aim][2], center, angle);
				armored_plate_conPloy[aim][3] = this->rotatePoint(armored_plate_conPloy[aim][3], center, angle);
				std::cout << "目标装甲板矩形框四点的坐标信息为：("
					<< armored_plate_conPloy[aim][0].x << "," << armored_plate_conPloy[aim][0].y << "), ("
					<< armored_plate_conPloy[aim][1].x << "," << armored_plate_conPloy[aim][1].y << "), ("
					<< armored_plate_conPloy[aim][2].x << "," << armored_plate_conPloy[aim][2].y << "), ("
					<< armored_plate_conPloy[aim][3].x << "," << armored_plate_conPloy[aim][3].y << ")" << std::endl;
				angular_velocity_record.push_back(angular_velocity);	//记录角速度信息
				cv::drawContours(frame, armored_plate_conPloy, aim, cv::Scalar(255, 255, 0), 3);	//画出预测的装甲板轮廓
			}
			else {
				flag = 0;	//在两次距离相差大于50的情况下，认定检测到了新的装甲板
			}
			last_angle = atan((armored_plate.x - center.x) / (armored_plate.y - center.y));
		}
		std::cout << "本轮运行时间：" << (static_cast<double>(cv::getTickCount()) - time) / cv::getTickFrequency() * 1000 << "ms" << std::endl;	//运算模块耗时
		cv::imshow("result", frame);
		outputVideo.write(frame);
		cv::waitKey(1);
		std::cout << "---------------------------------------------------" << std::endl;	//运算模块耗时
	};
	std::stringstream ss;
	ss << type_number + 1;
	this->save("data" + ss.str() + ".csv", angular_velocity_record);	// 将历史角速度以csv文件保存下来，可以使用excel的图表功能画图
	outputVideo.release();
	cap.release();
}