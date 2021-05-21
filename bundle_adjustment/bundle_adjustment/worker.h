#pragma once
#define  _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include "../include/rapidjson/rapidjson.h"
#include "../include/rapidjson/document.h"
#include "../include/rapidjson/filereadstream.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using std::cout;
using std::endl;

#define LEN_IMAGE_NAME 5

class Worker {
public:
	Worker();
	virtual int DoWork(std::string work_path);

private:
	
};
