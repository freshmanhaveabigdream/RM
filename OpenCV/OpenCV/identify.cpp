#include "identify.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<fstream>
#define pi 3.1415926

identification::identification() {	// ��ʼ����������������ÿ����ͬ��ɫ�ع��Ŀ����Ƶ
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
	float translatedY = center.y - cur_Point.y;  // ȡ������Ӧ��ѧ����ϵ
	return cv::Point2f(
		translatedX * cosA - translatedY * sinA + center.x,
		center.y - (translatedX * sinA + translatedY * cosA)  // ȡ������Ӧ��ѧ����ϵ
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
	cv::VideoCapture cap(path);	//����Ƶ
	cv::Mat frame, frame_temp, armored_plate_temp, center_temp;	//����mat�������ֱ�Ϊ��Ƶ��һ֡��һ֡��Ƶ����ʱ���������װ�װ����ʱ������������ĵ���ʱ����
	cv::Mat kernel_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
	cv::Mat kernel_3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::Mat kernel_4 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));	//��������ͼ��ʴ���ŵ�kernel��
	cv::Point2f last_position(0, 0); //���ڴ洢��һ֡��⵽��װ�װ�����ĵ�
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours_center;
	std::vector<cv::Vec4i> hierarchy_center;	//����洢�������㼶��Ϣ�ľ���

	int flag = -1;	//�����־��-1��ʾ����δ��⵽δ����װ�װ壬0��ʾ��⵽�µ�δ����װ�װ�,1��ʾ��⵽��һ֡��⵽��δ����װ�װ�
	float last_angle = -10;	//�洢��һ֡��⵽��װ�װ�����λ�ú���һ֡��⵽��Բ�ĵļнǣ���ʼֵΪ-10
	float linear_velocity = 0;	//�洢��ǰʱ�̵ľ��ο��������ٶ�
	float angular_velocity = 0;	//�洢��ǰʱ�̵ľ��ο���ٶ�
	float angle;	//�洢�ڴ˿̽��ٶ��µĵ�λʱ����ת�Ƕ�
	int number = 0;	//�洢������֡û��⵽���ο�

	std::vector<float> angular_velocity_record;	//��¼װ�װ����ʷ���ٶ�
	int frame_rate = static_cast<int>(cap.get(cv::CAP_PROP_FPS));	// ��ȡ֡��
	float time_per_frame = 1.0 / frame_rate;	// ��ȡÿһ֡��ʱ��
	int frameCounter = 0;	//��¼��ǰ�ǵڼ�֡
	int Frame_skipping = 0;	//�Ƿ���֡�ı�־����Ϊ������֤�����ع����Ƶ����ÿ��֡��ͬ���������������ż��֡����������
	if (type_number == 2 || type_number == 3) {	//����ǵ��ع���Ƶ������֡�����ҰѴ����ÿ֡��ʱ����2
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
		if (frameCounter % 2 == 0 && Frame_skipping != 0)continue;	//����ż��֡
		double time = static_cast<double>(cv::getTickCount());	//��¼����ѭ����ʼʱ��
		frame_temp = frame.clone();

		//ͼ�����������
		cv::cvtColor(frame_temp, frame_temp, cv::COLOR_BGR2HSV);	//��ͼ��ת��ΪHSV��ʽ
		cv::inRange(frame_temp, this->armored_plate_parameter_array[type_number].first, this->armored_plate_parameter_array[type_number].second, armored_plate_temp);	//��ȡ��Ŀ����ɫ
		cv::inRange(frame_temp, this->center_parameter_array[type_number].first, this->center_parameter_array[type_number].second, center_temp);
		cv::dilate(armored_plate_temp, armored_plate_temp, kernel_4);
		cv::erode(armored_plate_temp, armored_plate_temp, kernel_2);
		cv::dilate(center_temp, center_temp, kernel_3);
		cv::erode(center_temp, center_temp, kernel_1);	//��ͼ��������ź͸�ʴ����
		
		//���ͼ�񲿷�
		cv::findContours(armored_plate_temp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(center_temp, contours_center, hierarchy_center, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		
		//�����⵽��װ�װ���С���Բ�����ġ��뾶�Լ���⵽��Բ�ġ��뾶��Բ�ĸ������ε���С���Բ�뾶��
		cv::Point2f armored_plate;
		float armored_plate_radius;
		cv::Point2f center;
		float center_radius;

		//����洢�������Ϻ�Ķ���ζ��������
		std::vector<std::vector<cv::Point>> armored_plate_conPloy(contours.size());
		std::vector<std::vector<cv::Point>> center_conPloy(contours_center.size());

		//����Ŀ�����������������е��±�
		int aim;

		int armored_plate_area = 0;	//�洢��ǰװ�װ����
		for (aim = 0; aim < contours.size(); aim++) {	//������⵽����������
			int area = cv::contourArea(contours[aim]);	//����������Χ���
			float peri = cv::arcLength(contours[aim], true);	//����ͼ�������ܳ�
			cv::approxPolyDP(contours[aim], armored_plate_conPloy[aim], 0.02 * peri, true);	//��һ�������⻬�������߻�����ͼ����������ж�������
			if (area > 1000) {	//�޳��޹صĵ�
				if (static_cast<int>(armored_plate_conPloy[aim].size()) == 4 && hierarchy[aim][3] != -1 && hierarchy[hierarchy[aim][3]][3] == -1 && hierarchy[aim][2] == -1) {	//�ж���������϶����Ϊ�ı��Σ��и������Ҹ�����Ϊ�����������û��������
					if ((hierarchy[aim][0] > 0 && cv::contourArea(contours[hierarchy[aim][0]]) > 1000) || (hierarchy[aim][1] > 0 && cv::contourArea(contours[hierarchy[aim][1]]) > 1000)) {	//�ж�������û���������1000��ͬ������
						continue;
					}
					cv::drawContours(frame, contours, aim, cv::Scalar(0, 255, 255), 3);
					armored_plate_area = area;	//����ǰ��⵽��װ�װ�����洢����
					cv::minEnclosingCircle(armored_plate_conPloy[aim], armored_plate, armored_plate_radius);	//�ҳ�װ�װ����ĵ�
					if (flag == -1)flag = 0;	//�����ǰδ��⵽װ�װ���flag��Ϊ0����ʾ��⵽�µ�δ����װ�װ�
					else if (flag == 0)flag = 1;	//�����ǰ�Ѽ�⵽װ�װ���flag��Ϊ1����ʾ��⵽��һ֡��⵽��δ����װ�װ�
					break;
				}
			}
		}
		if (aim == contours.size()) {	//�ж�����������δ��⵽װ�װ�
			flag = -1;
			number++;	//���δ��⵽��number��һ��������һ�μ�����ٶ��쳣
		}
		for (int i = 0; i < contours_center.size(); i++) {	//������⵽����������
			int area = cv::contourArea(contours_center[i]);
			std::vector<std::vector<cv::Point>> center_conPloy(contours_center.size());
			if (area < armored_plate_area / 5 && area > armored_plate_area / 10) {	//�ж���������ǰ������Χ�����װ�װ�ʶ�������1/10~1/5֮��
				float peri = cv::arcLength(contours_center[i], true);
				cv::approxPolyDP(contours_center[i], center_conPloy[i], 0.09 * peri, true);
				if (static_cast<int>(center_conPloy[i].size()) == 4) {	//�ж���������϶����Ϊ�ı���
					cv::drawContours(frame, contours_center, i, cv::Scalar(0, 255, 255), 3);
					cv::minEnclosingCircle(center_conPloy[i], center, center_radius);
					cv::circle(frame, center, center_radius, cv::Scalar(255, 255, 0), 9);	//��������Բ
					break;
				}
			}
		}
		if (flag == 0) {	//�ж���������⵽�µ�δ����װ�װ����
			last_position = armored_plate;	//���˿̵�װ�װ�����λ�ñ�Ϊ��һ�̣�������ٶ�
			last_angle = atan((armored_plate.x - center.x) / (armored_plate.y - center.y));
			std::cout << "����δ��⵽δ����װ�װ�" << std::endl;
		}
		else if (flag == 1) {	//�ж���������⵽��һ֡��⵽��δ����װ�װ�
			angular_velocity = (atan((armored_plate.x - center.x) / (armored_plate.y - center.y)) - last_angle) / time_per_frame / (number + 1);	//������ٶȣ�sΪ��λ��
			if (last_angle == -10 || abs(angular_velocity) < 2 * pi) {	//�ж����������ǵ�һ�Σ���last_angle���ж�����ͬһװ�װ�������������ĵ��¼���߱��β���Ľ��ٶ�С��Լ360��ÿ��
				number = 0;
				if (angular_velocity > 0) {
					std::cout << "��ǰ��ת����Ϊ��ʱ��" << std::endl;
				}
				else {
					std::cout << "��ǰ��ת����Ϊ˳ʱ��" << std::endl;
				}
				//������ת�ĽǶȣ�һ�룩
				if (type_number == 0 || type_number == 1) {
					// ����Ǳ�׼�ع���Ƶ�����ת��Ϊ10rpm�µ�Ԥ��Ƕ�
					angle = 10.0 / 60 * 2 * pi;
					if (angular_velocity <= 0) angle *= -1;	//����˳ʱ����ת������Ҫȡ�Ƕȵ��෴��
				}else {
					// ����ǵ��ع���Ƶ�������ǰ���ٶ��µ�Ԥ��Ƕ�
					angle = angular_velocity * 1;
				}
				armored_plate_conPloy[aim][0] = this->rotatePoint(armored_plate_conPloy[aim][0], center, angle);
				armored_plate_conPloy[aim][1] = this->rotatePoint(armored_plate_conPloy[aim][1], center, angle);
				armored_plate_conPloy[aim][2] = this->rotatePoint(armored_plate_conPloy[aim][2], center, angle);
				armored_plate_conPloy[aim][3] = this->rotatePoint(armored_plate_conPloy[aim][3], center, angle);
				std::cout << "Ŀ��װ�װ���ο��ĵ��������ϢΪ��("
					<< armored_plate_conPloy[aim][0].x << "," << armored_plate_conPloy[aim][0].y << "), ("
					<< armored_plate_conPloy[aim][1].x << "," << armored_plate_conPloy[aim][1].y << "), ("
					<< armored_plate_conPloy[aim][2].x << "," << armored_plate_conPloy[aim][2].y << "), ("
					<< armored_plate_conPloy[aim][3].x << "," << armored_plate_conPloy[aim][3].y << ")" << std::endl;
				angular_velocity_record.push_back(angular_velocity);	//��¼���ٶ���Ϣ
				cv::drawContours(frame, armored_plate_conPloy, aim, cv::Scalar(255, 255, 0), 3);	//����Ԥ���װ�װ�����
			}
			else {
				flag = 0;	//�����ξ���������50������£��϶���⵽���µ�װ�װ�
			}
			last_angle = atan((armored_plate.x - center.x) / (armored_plate.y - center.y));
		}
		std::cout << "��������ʱ�䣺" << (static_cast<double>(cv::getTickCount()) - time) / cv::getTickFrequency() * 1000 << "ms" << std::endl;	//����ģ���ʱ
		cv::imshow("result", frame);
		outputVideo.write(frame);
		cv::waitKey(1);
		std::cout << "---------------------------------------------------" << std::endl;	//����ģ���ʱ
	};
	std::stringstream ss;
	ss << type_number + 1;
	this->save("data" + ss.str() + ".csv", angular_velocity_record);	// ����ʷ���ٶ���csv�ļ���������������ʹ��excel��ͼ���ܻ�ͼ
	outputVideo.release();
	cap.release();
}