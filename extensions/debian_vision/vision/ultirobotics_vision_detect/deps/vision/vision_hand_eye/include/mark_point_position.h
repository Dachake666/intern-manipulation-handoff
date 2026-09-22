#pragma once
#ifndef MARK_POINT_POSITION_H
#define MARK_POINT_POSITION_H
#include<iostream>
#include<vector>
#include <opencv2/opencv.hpp>
//#include "io.h"
#include<fstream>
#include"cal_logger.h"
#include <nlohmann/json.hpp>
#include <filesystem> 
namespace fs = std::filesystem;

class mark3dPosition {
public:
	mark3dPosition() {}
	~mark3dPosition() {}
	mark3dPosition& operator=(const mark3dPosition& tp) {
		if (this != &tp) {
			p1 = tp.p1;
			p2 = tp.p2;
			p3 = tp.p3;
			p4 = tp.p4;
			id = tp.id;
		}
		return *this;
	}
	cv::Point3f p1;
	cv::Point3f p2;
	cv::Point3f p3;
	cv::Point3f p4;
	int id;
};

class markPosition {
public:
	markPosition(){
	}
	void init(std::string config_dir) {
		col = 0;
		row = 0;
		mark_lenght = 0;
		side_lenght = 0;

		mark_file_path = config_dir + "/mark.json";
		LOG_INFO("local_img_path  {}",mark_file_path);
	}
	~markPosition(){}
	markPosition& operator=(const markPosition& tp) {	
		if (this != &tp) {
			mark_class_ = tp.mark_class_;
			marks3d = tp.marks3d;
		}
		return *this;
	}
	void get_pair_param(std::vector<std::string>& read_lines, std::vector<std::pair<std::string, int>>& pair_param) {
		if (read_lines.size() < 1)
			return;
		std::string str;
		std::string str_numb;
		int numb;
		int pos_m;
		int pos;
		for (int i = 0; i < read_lines.size(); ++i) {
			pos_m = read_lines[i].find(":");
			pos = read_lines[i].find("#");
			if (std::string::npos == pos_m || pos_m < 1)
				continue;
			if (std::string::npos == pos)
				pos = read_lines[i].length();
			if (pos - pos_m - 1 < 1)
				continue;
			str = read_lines[i].substr(0, pos_m);
			str_numb = read_lines[i].substr(pos_m + 1, pos - pos_m - 1);
			numb = std::stoi(str_numb);
			pair_param.push_back(std::pair<std::string, int>(str, numb));
		}
	}
	void read_param() {
		if (access("mark_param.txt", 0) != 0) {
			LOG_INFO("can_not_fount_mark_param.txt_file!!!!!!!!!!!!!!!!!!!!");
			return;
		}
		else {
			std::vector<std::string> read_lines;
			std::string s;
			std::ifstream inf;
			inf.open("mark_param.txt");
			if (!inf.is_open()) {
				return;
			}
			while (getline(inf, s)) {
				read_lines.push_back(s);
			}
			inf.close();

			if (read_lines.size() < 1) {
				LOG_INFO("mark_param.txt_numbers<1!!!!!!!!!!!!!!!!!!!!");
				return;
			}
			std::vector<std::pair<std::string, int>> pair_param;
			get_pair_param(read_lines, pair_param);
			for (int i = 0; i < pair_param.size(); ++i) {
				if ("col" == pair_param[i].first) {
					col = pair_param[i].second;
				}
				if ("row" == pair_param[i].first) {
					row = pair_param[i].second;
				}
				if ("mark_lenght" == pair_param[i].first) {
					mark_lenght = pair_param[i].second;
				}
				if ("side_lenght" == pair_param[i].first) {
					side_lenght = pair_param[i].second;
				}
				if ("id" == pair_param[i].first) {
					mark_class_.push_back(pair_param[i].second);
				}
			}
		}
		if (col > 0 && row > 0 && mark_lenght > 0 && side_lenght > 0) {
			if (col * row != mark_class_.size()) {
				LOG_INFO("mark_col*row!=mark_size!!!!!!!!!!!!!!!!!!!!");
				return;
			}

			float x1 = 0;
			float x2 = mark_lenght + side_lenght;
			float y1 = 0;
			float y2 = mark_lenght + side_lenght;
			mark3dPosition tmp_m;
			for (int i = 0; i < mark_class_.size(); ++i) {
				tmp_m.p1.x = x1;
				tmp_m.p1.y = y1;
				tmp_m.p2.x = x2;
				tmp_m.p2.y = y1;
				tmp_m.p3.x = x2;
				tmp_m.p3.y = y2;
				tmp_m.p4.x = x1;
				tmp_m.p4.y = y2;
				tmp_m.id = mark_class_[i];
				marks3d.push_back(tmp_m);

				x1 = x2 + side_lenght;
				x2 = x1 + mark_lenght + side_lenght;

				if ((i + 1) % row == 0) {
					x1 = 0;
					x2 = mark_lenght + side_lenght;
					y1 = y2 + side_lenght;
					y2 = y1 + mark_lenght + side_lenght;
				}

			}
		}
	}

	void read_param_json() {
		nlohmann::json config;

        std::ifstream ifs(mark_file_path);
		if (!ifs.is_open()) {
			LOG_INFO("open_mark_file_fail!!!!!!  {}",mark_file_path);
			return;
		}
		ifs >> config;
		ifs.close();
		float mark_length = config["lenght"];
		for (const auto& marker_json : config["markers"]) {
			int id = marker_json["id"];
			std::vector<float> pt1 = marker_json["corners_3d"];
			mark3dPosition tmp_m;
			tmp_m.id = id;
			tmp_m.p1 = cv::Point3f(pt1[0], pt1[1], 0);
			tmp_m.p2 = cv::Point3f(pt1[0], pt1[1] + mark_length, 0);
			tmp_m.p3 = cv::Point3f(pt1[0] - mark_length, pt1[1] + mark_length, 0);
			tmp_m.p4 = cv::Point3f(pt1[0] - mark_length, pt1[1], 0);
			marks3d.push_back(tmp_m);
		}
	}

	bool get_mark3d_position(int id, mark3dPosition& mark) {
		for (int i = 0; i < marks3d.size(); ++i) {
			if (marks3d[i].id == id) {
				mark = marks3d[i];
				return true;
			}
		}
		return false;
	}

private:
	int col;
	int row;
	float mark_lenght;
	float side_lenght;
	std::vector<int> mark_class_;
	std::vector<mark3dPosition> marks3d;
	std::string mark_file_path;
};

#endif // !MARK_POINT_POSITION_H
