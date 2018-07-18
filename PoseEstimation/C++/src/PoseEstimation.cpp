//============================================================================
// Name        : PoseEstimation.cpp
// Author      : Marcelo Kaihara
// Version     :
// Copyright   : 
// Description : Pose Estimation algorithm implemented in C++
//============================================================================

#include <iostream>

#include "Mat_33.hpp"
#include "Points.hpp"
#include "Vec_Points.hpp"
#include "Estimation.hpp"
#include <time.h>
#include <chrono>
#include <unistd.h>

std::string GetCurrentWorkingDir( void ) {
  char buff[FILENAME_MAX]{};
  getcwd( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

int main() {

	// set path to input/output data
	std::string path2data = GetCurrentWorkingDir() + "/data/";
	std::string path_data1 = path2data + "p3d_1.txt";
	std::string path_data2 = path2data + "p3d_2.txt";
	std::string path_data3 = path2data + "p3d_3.txt";

	Vec_Points<double> p3d_1 { };
	Vec_Points<double> p3d_2 { };
	Vec_Points<double> p3d_3 { };

	// Load data from file
	if (p3d_1.load_vecpoints(path_data1)) {
		// Error opening the file
		return 1;
	}
	if (p3d_2.load_vecpoints(path_data2)) {
		// Error opening the file
		return 1;
	}
	if (p3d_3.load_vecpoints(path_data3)) {
		// Error opening the file
		return 1;
	}

	std::vector<double> sv_u(p3d_1.size(),1);
	std::vector<double> sv_v(p3d_2.size(),1);
	std::vector<double> sv_w(p3d_3.size(),1);

	Mat_33<double> sv_r_12{};
	Mat_33<double> sv_r_23{};
	Mat_33<double> sv_r_31{};

	Points<double> sv_t_12{};
	Points<double> sv_t_23{};
	Points<double> sv_t_31{};

	Vec_Points<double> sv_scene{p3d_1.size()};

	int iterations {50};

	// For timing measurements
	std::chrono::high_resolution_clock::time_point t1{};
	std::chrono::high_resolution_clock::time_point t2{};

	t1 = std::chrono::high_resolution_clock::now();

	pose_estimation (p3d_1, p3d_2, p3d_3,
					 iterations,
					 sv_scene,
					 sv_r_12, sv_r_23, sv_r_31,
					 sv_t_12, sv_t_23, sv_t_31);

	t2 = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
	std::cout << "execution time: " << duration << " microseconds" << std::endl;

	std::cout << "sv_scene==================================" << std::endl;
	std::cout << sv_scene;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_r_12===================================" << std::endl;
	std::cout << sv_r_12;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_r_23===================================" << std::endl;
	std::cout << sv_r_23;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_r_31===================================" << std::endl;
	std::cout << sv_r_31;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_t_12===================================" << std::endl;
	std::cout << sv_t_12;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_t_23===================================" << std::endl;
	std::cout << sv_t_23;
	std::cout << "==========================================" << std::endl;

	std::cout << "sv_t_31===================================" << std::endl;
	std::cout << sv_t_31;
	std::cout << "==========================================" << std::endl;

	std::string path_out_data1 = path2data + "sv_scene.txt";

	if (sv_scene.save_vecpoints(path_out_data1)) {
		// Error opening the file
		return 1;
	}

	return 0;
}
