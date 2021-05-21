#pragma once
#include "worker.h"

class BundleAdjustment6Dof : public Worker {
public:
	BundleAdjustment6Dof(const int skip_every_frames_);
	int DoWork(std::string work_path);
private:
	int skip_every_frames;
};
