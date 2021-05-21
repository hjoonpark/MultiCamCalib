#define MODE 0
#include "bundle_adjustment_6dof.h"
#include "Triangulation.h"

int main(int argc, char** argv) {
	/*
	example usage in cmd.exe:
		<NEW: 2020-10-30>
		"C:\Users\hjoon\Documents\Visual Studio 2019\Projects\CeresCamCalib\x64\Release\CeresTest.exe" 0 "D:\CalibrationData\CameraCalibration\2020_10_21_CalibTestSets\1" "D:\CalibrationData\CameraCalibration\2020_10_21_CalibTestSets\1\Triangulation_planar" 1
		"C:\Users\hjoon\Documents\Visual Studio 2019\Projects\CeresCamCalib\x64\Release\CeresTest.exe" 2 "D:\CalibrationData\CameraCalibration\2020_10_21_CalibTestSets\7" "" 1

		<OLD>
		"C:\Users\hjoon\Documents\Visual Studio 2019\Projects\CeresCamCalib\x64\Release\CeresTest.exe" 2 D:\CalibrationData\CameraCalibration\2019_12_24_Marianne_Capture
		"C:\Users\hjoon\Documents\Visual Studio 2019\Projects\CeresCamCalib\x64\Release\CeresTest.exe" 7 D:\CalibrationData\CameraCalibration\2019_12_13_Lada_Capture_k1k2k3p1p2
	*/

	/*
	how ceres computes loss:
	e.g., residual[3] = {1, 2, 3}
	L = 1^2 + 2^2 + 3^2
	*/
	int mode;
	std::string path;
	std::string input_dir;
	int skip_every_frames;
	if (argc < 3) {
		/*
		if (mode == 6) {
		}
		else {
			std::cerr << "[ERROR] type input parameters: (int)mode (string)path" << endl;
			std::cerr << "        e.g., 1 D:/CalibrationData/CameraCalibration/2019_12_09_capture" << endl;
			std::getchar();
			return 1;
		}
		*/
		mode = 12;
		path = "D:/CalibrationData/CameraCalibration/2019_06_05_NewSuitCapture2";
		//path = "D:/CalibrationData/CameraCalibration/2020_11_02_SynthChb_ThreeCalibMethods/IncludeOutliers";
		//input_dir = "D:/CalibrationData/CameraCalibration/2019_06_05_NewSuitCapture2/Triangulations";
		input_dir = "";
		skip_every_frames = 0;
	}
	else {
		mode = atoi(argv[1]);
		path = argv[2];
		input_dir = argv[3]; 
		skip_every_frames = atoi(argv[4]);
	}
	cout << "Mode: " << mode << " | Root path: " << path.c_str() << endl;
	
	Worker *worker = NULL;
	if (mode == 2) {
		cout << "Running: Bundle Adjustment w/ 6DoF" << endl;
		worker = new BundleAdjustment6Dof(skip_every_frames);
	}
	else if (mode == 12) {
		cout << "Running: Triangulation" << endl;
		// input_dir: D:\CalibrationData\CameraCalibration\2020_11_02_SynthChb_ThreeCalibMethods\ExcludeOutliers\Triangulation
		input_dir = "D:/CalibrationData/CameraCalibration/2020_11_02_SynthChb_ThreeCalibMethods/ExcludeOutliers/Triangulation";
		worker = new Triangulation(input_dir);
	}

	if (worker != NULL) {
		int result = worker->DoWork(path);
	}
	// cout << "Press Enter to continue..." << endl;
	// std::getchar();

	delete worker;
	return 0;
}