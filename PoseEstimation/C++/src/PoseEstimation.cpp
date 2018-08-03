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
#include <chrono>

std::string GetCurrentWorkingDir( void ) {
// gets the current working directory
  char buff[FILENAME_MAX]{};
  if (getcwd( buff, FILENAME_MAX )==nullptr) {
	  throw (std::runtime_error("The directory could not be determined."));
  }
  std::string current_working_dir(buff);
  return current_working_dir;
}

int main() {

	// Set path to input data
	std::string path2data = GetCurrentWorkingDir() + "/data/";
	std::string path_data1 = path2data + "p3d_1.txt";
	std::string path_data2 = path2data + "p3d_2.txt";
	std::string path_data3 = path2data + "p3d_3.txt";

	// Input vector of points
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

	// Rotation matrices
	Mat_33<double> sv_r_12{};
	Mat_33<double> sv_r_23{};
	Mat_33<double> sv_r_31{};

	// Translation vectors
	Points<double> sv_t_12{};
	Points<double> sv_t_23{};
	Points<double> sv_t_31{};

	// Output result, vector of points
	Vec_Points<double> sv_scene{p3d_1.size()};

	// This sets the number of iterations in the algorithm
	int iterations {50};

	// For timing measurements
	std::chrono::high_resolution_clock::time_point t1{};
	std::chrono::high_resolution_clock::time_point t2{};

	// start measuring time
	t1 = std::chrono::high_resolution_clock::now();

	// main algorithm
	pose_estimation (p3d_1, p3d_2, p3d_3,
					 iterations,
					 sv_scene,
					 sv_r_12, sv_r_23, sv_r_31,
					 sv_t_12, sv_t_23, sv_t_31);

	// stop measuring time
	t2 = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
	std::cout << "Number of points: " << p3d_1.size() << std::endl;
	std::cout << "Number of iterations: " << iterations << std::endl;
	std::cout << "Execution time: " << duration << " microseconds" << std::endl;

	// setup path to save the output result
	std::string path_out_data1 = path2data + "sv_scene.txt";

	if (sv_scene.save_vecpoints(path_out_data1)) {
		// Error opening the file
		return 1;
	}

	return 0;
}
