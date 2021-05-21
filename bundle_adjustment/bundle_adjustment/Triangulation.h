#pragma once
#define  _CRT_SECURE_NO_WARNINGS
#include "worker.h"

class Triangulation: public Worker {
public:
	Triangulation(std::string target_image_name_);
	int DoWork(std::string work_path);
private:
	std::string input_image_pt_path;
};
