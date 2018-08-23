#ifndef CAMERA_HPP_
#define CAMERA_HPP_

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Images.hpp"

#include <algorithm>

class Camera {
private:
	Pylon::IGigETransportLayer *pTL{};
	// Limits the amount of cameras used for grabbing.
	// It is important to manage the available bandwidth when grabbing with multiple
	// cameras. This applies, for instance, if two GigE cameras are connected to the
	// same network adapter via a switch. To manage the bandwidth, the GevSCPD
	// interpacket delay parameter and the GevSCFTD transmission delay parameter can
	// be set for each GigE camera device. The "Controlling Packet Transmission Timing
	// with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
	// Application Note (AW000649xx000) provides more information about this topic.
	uint32_t c_maxCamerasToUse = 2;
	Pylon::CBaslerGigEInstantCameraArray cameras{};
	uint32_t DeviceKey = 0;
	// For this sample we configure all cameras to be in the same group.
    uint32_t GroupKey = 0x112233;
    Pylon::String_t subnet {};

    double exposureTime = 13057;// exposure time
    int64_t gain = 23;		// gain

    size_t height = 3008;
    size_t width = 3008;

    size_t offsetX = 0;
    size_t offsetY = 0;

    size_t aoi_height = (1520 - 595);
	size_t aoi_width = (3131 - 958);

	size_t aoi_offsetX = 958;
	size_t aoi_offsetY = 595;

	int autoTargetVal = 100;

	bool autoExpTimeCont = true;
	bool autoGainCont = true;

	std::vector<size_t> sortedCameraIdx {};

	std::string config_path = {"./config/"}; // default location of the configuration files of the cameras
	bool loadParam = true; // when true, it will load the configuration files to the cameras

	void Init();

public:
	Camera();
	Camera(std::string path_to_config_files);
	size_t GetNumCam();
	void GrabImages();
	void SaveParameters();
	void LoadParameters();
	virtual ~Camera();
};

#endif /* CAMERA_HPP_ */
