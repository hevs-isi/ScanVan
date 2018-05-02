// AutoAdjustmentOnce.cpp
/*
  This program is make the auto adjustment of the exposure time and the gain once and get the parameters for further tuning.
*/

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// To measure the time
#include <time.h>
#include <chrono>
// For rand & srand
#include <stdlib.h>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>

#include <iostream>
#include <fstream>

#include <unistd.h>

// To manage threads
#include <pthread.h>

// Settings to use Basler GigE cameras.
using namespace Basler_GigECameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

#ifndef USE_GIGE
#error This example is usable for GigE cameras only.
#endif

// Namespace for using OpenCV objects.
using namespace cv;

// Limits the amount of cameras used for grabbing.
// It is important to manage the available bandwidth when grabbing with multiple
// cameras. This applies, for instance, if two GigE cameras are connected to the
// same network adapter via a switch. To manage the bandwidth, the GevSCPD
// interpacket delay parameter and the GevSCFTD transmission delay parameter can
// be set for each GigE camera device. The "Controlling Packet Transmission Timing
// with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
// Application Note (AW000649xx000) provides more information about this topic.
static const uint32_t c_maxCamerasToUse = 2;

// Parameters for the image
const int imageWidth = 3004;
const int imageHeight = 3004;
const int offsetX = 502;
const int offsetY = 2;

// Prototypes
void AutoExposureOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet);
void AutoGainOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet);
void PlotHistogram(const Mat &src);

int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try
    {
        // Get the GigE transport layer.
        // We'll need it later to issue the action commands.
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        IGigETransportLayer *pTL = dynamic_cast<IGigETransportLayer*>(tlFactory.CreateTl(BaslerGigEDeviceClass));
        if (pTL == NULL)
        {
            throw RUNTIME_EXCEPTION("No GigE transport layer available.");
        }

        // In this sample we use the transport layer directly to enumerate cameras.
        // By calling EnumerateDevices on the TL we get get only GigE cameras.
        // You could also accomplish this by using a filter and
        // let the Transport Layer Factory enumerate.
        DeviceInfoList_t allDeviceInfos;
        if (pTL->EnumerateDevices(allDeviceInfos) == 0)
        {
            throw RUNTIME_EXCEPTION("No GigE cameras present.");
        }

        // Only use cameras in the same subnet as the first one.
        DeviceInfoList_t usableDeviceInfos;
        usableDeviceInfos.push_back(allDeviceInfos[0]);
        const String_t subnet(static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[0]).GetSubnetAddress());

        // Start with index 1 as we have already added the first one above.
        // We will also limit the number of cameras to c_maxCamerasToUse.
        for (size_t i = 1; i < allDeviceInfos.size() && usableDeviceInfos.size() < c_maxCamerasToUse; ++i)
        {
            const CBaslerGigEDeviceInfo& gigeinfo = static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[i]);
            if (subnet == gigeinfo.GetSubnetAddress())
            {
                // Add this deviceInfo to the ones we will be using.
                usableDeviceInfos.push_back(gigeinfo);
            }
            else
            {
                cerr << "Camera will not be used because it is in a different subnet " << subnet << "!" << endl;
            }

        }

        // In this sample we'll use an CBaslerGigEInstantCameraArray to access multiple cameras.
        CBaslerGigEInstantCameraArray cameras(usableDeviceInfos.size());

        // Seed the random number generator and generate a random device key value.
        srand((unsigned)time(NULL));
        const uint32_t DeviceKey = rand();

        // For this sample we configure all cameras to be in the same group.
        const uint32_t GroupKey = 0x112233;

        // For the following sample we use the CActionTriggerConfiguration to configure the camera.
        // It will set the DeviceKey, GroupKey and GroupMask features. It will also
        // configure the camera FrameTrigger and set the TriggerSource to the action command.
        // You can look at the implementation of CActionTriggerConfiguration in <pylon/gige/ActionTriggerConfiguration.h>
        // to see which features are set.

        // Create all GigE cameras and attach them to the InstantCameras in the array.
        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach(tlFactory.CreateDevice(usableDeviceInfos[i]));
            // We'll use the CActionTriggerConfiguration, which will set up the cameras to wait for an action command.
            cameras[i].RegisterConfiguration(new CActionTriggerConfiguration(DeviceKey, GroupKey, AllGroupMask), RegistrationMode_Append, Cleanup_Delete);
            // Set the context. This will help us later to correlate the grab result to a camera in the array.
            cameras[i].SetCameraContext(i);

            const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();

            // Print the model name of the camera.
            cout << "Using camera " << i << ": " << di.GetModelName() << " (" << di.GetIpAddress() << ")" << endl;
        }

		// Open all cameras.
		// This will apply the CActionTriggerConfiguration specified above.
		cameras.Open();

		for (size_t i = 0; i < cameras.GetSize(); ++i) {
			// This sets the transfer pixel format to BayerRG8
			cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG8);

			// This sets the value of the packet size on the camera to 9000
			// On the host size, change the MTU to 9014 and the maximum UDP receive buffer size to 2097152
			cameras[i].GevSCPSPacketSize.SetValue(8000);
			cameras[i].GevSCPD.SetValue(2500);

			cameras[i].ExposureTimeAbs.SetValue(cameras[i].ExposureTimeAbs.GetMin());
			cameras[i].GainRaw.SetValue(80);

			cameras[i].Width.SetValue(imageWidth);
			cameras[i].Height.SetValue(imageHeight);
			if (IsWritable(cameras[i].OffsetX)) {
				cameras[i].OffsetX.SetValue(offsetX);
			}
			if (IsWritable(cameras[i].OffsetY)) {
				cameras[i].OffsetY.SetValue(offsetY);
			}

		}

		// Auto exposure time adjustment once
		// Only area scan cameras support auto functions.
		if (cameras[0].DeviceScanType.GetValue() == DeviceScanType_Areascan) {
			AutoExposureOnce(cameras, pTL, DeviceKey, GroupKey, subnet);
		}

		// Auto gain adjustment once
		cameras[0].GainRaw.SetValue(cameras[0].GainRaw.GetMin());
		// Carry out luminance control by using the "once" gain auto function.
		// Only area scan cameras support auto functions.
		if (cameras[0].DeviceScanType.GetValue() == DeviceScanType_Areascan) {
			AutoGainOnce(cameras, pTL, DeviceKey, GroupKey, subnet);
		}

		cout << "Press 'e' to increase the exposure time on camera 0." << endl;
		cout << "Press 'd' to reduce the exposure time on camera 0." << endl;
		cout << "Press 'u' to increase the gain on camera 0." << endl;
		cout << "Press 'j' to reduce the gain on camera 0." << endl;

		if (usableDeviceInfos.size() == 2) {
			cout << "Press 'r' to increase the exposure time on camera 1." << endl;
			cout << "Press 'f' to reduce the exposure time on camera 1." << endl;
			cout << "Press 'i' to increase the gain on camera 1." << endl;
			cout << "Press 'k' to reduce the gain on camera 1." << endl;
		}

		cout << "Press 'q' to save and quit." << endl;

		// Starts grabbing for all cameras.
		// The cameras won't transmit any image data, because they are configured to wait for an action command.
		cameras.StartGrabbing();

		bool terminateLoop = false;
		int key = 0;

		// Create a pylon ImageFormatConverter object.
		CImageFormatConverter formatConverter;
		// Specify the output pixel format.
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;
		// Create a PylonImage that will be used to create OpenCV images later.
		CPylonImage pylonImage;
		// Create an OpenCV image.
		Mat openCvImage;

		while (cameras.IsGrabbing() && !terminateLoop) {
			// Now we issue the action command to all devices in the subnet.
			// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
			pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

			// This smart pointer will receive the grab result data.
			CBaslerGigEGrabResultPtr ptrGrabResult;

			// Retrieve images from all cameras.
			const int DefaultTimeout_ms = 5000;

			// Retrieve images from all cameras.
			for (size_t i = 0; i < usableDeviceInfos.size() && cameras.IsGrabbing(); ++i) {
				// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
				cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult, TimeoutHandling_ThrowException);

				intptr_t cameraIndex = ptrGrabResult->GetCameraContext();

				// Image grabbed successfully?
				if (ptrGrabResult->GrabSucceeded()) {

					// Convert the grabbed buffer to a pylon image.
					formatConverter.Convert(pylonImage, ptrGrabResult);

					// Create an OpenCV image from a pylon image.
					openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

					// Create an OpenCV display window.
					string windowTitle = "Camera ";
					windowTitle.append (to_string(cameraIndex));
					namedWindow(windowTitle, CV_WINDOW_NORMAL); // other options: // CV_AUTOSIZE, CV_FREERATIO

					// Display the current image in the OpenCV display window.
					imshow(windowTitle, openCvImage);
					// Define a timeout for customer's input in ms.
					// '0' means indefinite, i.e. the next image will be displayed after closing the window.
					// '1' means live stream
					key = waitKey(1);

					double expTime;
					int gain;
					switch (key) {
					case 113:
						terminateLoop = true;  //if 'q' key is pressed
						break;

					case 101: 	// if 'e' key is pressed
						expTime = cameras[0].ExposureTimeAbs.GetValue() + 1000;

						if (expTime < cameras[0].ExposureTimeAbs.GetMax()) {
							cout << "Camera 0 - Exposure time: " << expTime << " us" << endl;
							cameras[0].ExposureTimeAbs.SetValue(expTime);
						}
						break;

					case 100: // if 'd' key is pressed
						expTime = cameras[0].ExposureTimeAbs.GetValue() - 1000;

						if (expTime > cameras[0].ExposureTimeAbs.GetMin()) {
							cout << "Camera 0 - Exposure time: " << expTime << " us" << endl;
							cameras[0].ExposureTimeAbs.SetValue(expTime);
						}
						break;

					case 117: 	// if 'u' key is pressed
						gain = cameras[0].GainRaw.GetValue() + 10;

						if (gain < cameras[0].GainRaw.GetMax()) {
							cout << "Camera 0 - Gain: " << gain << endl;
							cameras[0].GainRaw.SetValue(gain);
						}
						break;

					case 106: // if 'j' key is pressed
						gain = cameras[0].GainRaw.GetValue() - 10;

						if (gain > cameras[0].GainRaw.GetMin()) {
							cout << "Camera 0 - Gain: " << gain << endl;
							cameras[0].GainRaw.SetValue(gain);
						}
						break;

					case 114: 	// if 'r' key is pressed
						if (usableDeviceInfos.size() == 2) {
							expTime = cameras[1].ExposureTimeAbs.GetValue()
									+ 1000;

							if (expTime < cameras[1].ExposureTimeAbs.GetMax()) {
								cout << "Camera 1 - Exposure time: " << expTime
										<< " us" << endl;
								cameras[1].ExposureTimeAbs.SetValue(expTime);
							}
						}
						break;

					case 102: // if 'f' key is pressed
						if (usableDeviceInfos.size() == 2) {
							expTime = cameras[1].ExposureTimeAbs.GetValue()
									- 1000;

							if (expTime > cameras[1].ExposureTimeAbs.GetMin()) {
								cout << "Camera 1 - Exposure time: " << expTime
										<< " us" << endl;
								cameras[1].ExposureTimeAbs.SetValue(expTime);
							}
						}
						break;

					case 105: 	// if 'i' key is pressed
						if (usableDeviceInfos.size() == 2) {
							gain = cameras[1].GainRaw.GetValue() + 10;

							if (gain < cameras[1].GainRaw.GetMax()) {
								cout << "Camera 1 - Gain: " << gain << endl;
								cameras[1].GainRaw.SetValue(gain);
							}
						}
						break;

					case 107: // if 'k' key is pressed
						if (usableDeviceInfos.size() == 2) {
							gain = cameras[1].GainRaw.GetValue() - 10;

							if (gain > cameras[1].GainRaw.GetMin()) {
								cout << "Camera 1 - Gain: " << gain << endl;
								cameras[1].GainRaw.SetValue(gain);
							}
						}
						break;

					}


					// Plot the histogram
					PlotHistogram(openCvImage);

				} else {
							// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
							// multiple images simultaneously. See note above c_maxCamerasToUse.
							cout << "Error: " << ptrGrabResult->GetErrorCode() << " "
									<< ptrGrabResult->GetErrorDescription() << endl;
				}

			}

		}

		cameras.StopGrabbing();

		// Save camera configuration to file
		string configFileName = "cameraparam.cfg";
		ofstream myFile;
		myFile.open (configFileName);;
		if (myFile.is_open()) {
			if (cameras.GetSize() == 2) {
				myFile << "Num Cameras: 2" << "\n";
			} else {
				myFile << "Num Cameras: 1" << "\n";
			}
			myFile << "Camera 0 - Exposure time: " << cameras[0].ExposureTimeAbs.GetValue() << "\n";
			myFile << "Camera 0 - Gain: " << cameras[0].GainRaw.GetValue() << "\n";
			if (cameras.GetSize() == 2) {
				myFile << "Camera 1 - Exposure time: " << cameras[1].ExposureTimeAbs.GetValue() << "\n";
				myFile << "Camera 1 - Gain: " << cameras[1].GainRaw.GetValue() << "\n";
			}
			myFile.close();
			cout << "Saved camera configuration to file \"" << configFileName << "\"" << endl;
		} else {
			cout << "Error: Unable to open the file \"" << configFileName << "\"" << endl;
		}

		destroyAllWindows();

		for (size_t i = 0; i < cameras.GetSize(); ++i) {
			cameras[i].GainAuto.SetValue(GainAuto_Off);
			cameras[i].ExposureAuto.SetValue(ExposureAuto_Off);
			cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG8);
			cameras[i].DeviceReset();
		}

		// Close all cameras.
		cameras.Close();

    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}

void AutoExposureOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet) {

	// Create a pylon ImageFormatConverter object.
	CImageFormatConverter formatConverter;
	// Specify the output pixel format.
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	// Create a PylonImage that will be used to create OpenCV images later.
	CPylonImage pylonImage;
	// Create an OpenCV image.
	Mat openCvImage;

	// Check whether auto exposure is available
	if (!IsWritable(cameras[0].ExposureAuto)) {
		cout << "The camera does not support Exposure Auto." << endl << endl;
		return;
	}

	cameras[0].Width.SetValue(imageWidth);
	cameras[0].Height.SetValue(imageHeight);

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY);
	}

	// Set the Auto Function AOI for luminance statistics.
	// Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
	// luminance statistics.
	cameras[0].AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
	cameras[0].AutoFunctionAOIWidth.SetValue(imageWidth);
	cameras[0].AutoFunctionAOIHeight.SetValue(imageHeight);
	cameras[0].AutoFunctionAOIOffsetX.SetValue(offsetX);
	cameras[0].AutoFunctionAOIOffsetY.SetValue(offsetY);

	// Set the target value for luminance control. The value is always expressed
	// as an 8 bit value regardless of the current pixel data output format,
	// i.e., 0 -> black, 255 -> white.
	cameras[0].AutoTargetValue.SetValue(128);

	// Try ExposureAuto = Once.
	cout << "Trying 'ExposureAuto = Once'." << endl;
	cout << "Initial exposure time = ";
	cout << cameras[0].ExposureTimeAbs.GetValue() << " us" << endl;

	// Set the exposure time ranges for luminance control.
	cameras[0].AutoExposureTimeAbsLowerLimit.SetValue(cameras[0].AutoExposureTimeAbsLowerLimit.GetMin());
	cameras[0].AutoExposureTimeAbsUpperLimit.SetValue(cameras[0].AutoExposureTimeAbsLowerLimit.GetMax());

	cameras[0].ExposureAuto.SetValue(ExposureAuto_Once);

	// Starts grabbing for all cameras.
	// The cameras won't transmit any image data, because they are configured to wait for an action command.
	cameras[0].StartGrabbing();

	// When the "once" mode of operation is selected,
	// the parameter values are automatically adjusted until the related image property
	// reaches the target value. After the automatic parameter value adjustment is complete, the auto
	// function will automatically be set to "off", and the new parameter value will be applied to the
	// subsequently grabbed images.
	int n = 0;
	while (cameras[0].ExposureAuto.GetValue() != ExposureAuto_Off) {

		// Now we issue the action command to all devices in the subnet.
		// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
		pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

		// This smart pointer will receive the grab result data.
		CBaslerGigEGrabResultPtr ptrGrabResult;

		// Retrieve images from all cameras.
		const int DefaultTimeout_ms = 5000;

		// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
		cameras[0].RetrieveResult(DefaultTimeout_ms, ptrGrabResult,	TimeoutHandling_ThrowException);

		// Image grabbed successfully?
		if (ptrGrabResult->GrabSucceeded()) {

			// Convert the grabbed buffer to a pylon image.
			formatConverter.Convert(pylonImage, ptrGrabResult);

			// Create an OpenCV image from a pylon image.
			openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

			// Create an OpenCV display window.
			namedWindow("Camera 0", CV_WINDOW_NORMAL); // other options: // CV_AUTOSIZE, CV_FREERATIO

			// Display the current image in the OpenCV display window.
			imshow("Camera 0", openCvImage);
			// Define a timeout for customer's input in ms.
			// '0' means indefinite, i.e. the next image will be displayed after closing the window.
			// '1' means live stream
			waitKey(1);

			// Plot the histogram
			PlotHistogram(openCvImage);

		} else {
			// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
			// multiple images simultaneously. See note above c_maxCamerasToUse.
			cout << "Error: " << ptrGrabResult->GetErrorCode() << " "
					<< ptrGrabResult->GetErrorDescription() << endl;
		}
		++n;

		cameras[0].WaitForFrameTriggerReady(500, TimeoutHandling_ThrowException);

		//Make sure the loop is exited.
		if (n > 100) {
			throw TIMEOUT_EXCEPTION( "The adjustment of auto exposure did not finish.");
		}
	}

	cameras[0].StopGrabbing();

	cout << "ExposureAuto went back to 'Off' after " << n << " frames." << endl;
	cout << "Final exposure time = ";
	cout << cameras[0].ExposureTimeAbs.GetValue() << " us" << endl << endl;
}


void AutoGainOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet)  {

	// Create a pylon ImageFormatConverter object.
	CImageFormatConverter formatConverter;
	// Specify the output pixel format.
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	// Create a PylonImage that will be used to create OpenCV images later.
	CPylonImage pylonImage;
	// Create an OpenCV image.
	Mat openCvImage;

	// Check whether the gain auto function is available.
	if (!IsWritable(cameras[0].GainAuto)) {
		cout << "The camera does not support Gain Auto." << endl << endl;
		return;
	}

	cameras[0].Width.SetValue(imageWidth);
	cameras[0].Height.SetValue(imageHeight);

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY);
	}

	// Set the Auto Function AOI for luminance statistics.
	// Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
	// luminance statistics.
	cameras[0].AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
	cameras[0].AutoFunctionAOIWidth.SetValue(imageWidth);
	cameras[0].AutoFunctionAOIHeight.SetValue(imageHeight);
	cameras[0].AutoFunctionAOIOffsetX.SetValue(offsetX);
	cameras[0].AutoFunctionAOIOffsetY.SetValue(offsetY);

	// Set the target value for luminance control. The value is always expressed
	// as an 8 bit value regardless of the current pixel data output format,
	// i.e., 0 -> black, 255 -> white.
	cameras[0].AutoTargetValue.SetValue(80);

	// We are going to try GainAuto = Once.
	cout << "Trying 'GainAuto = Once'." << endl;
	cout << "Initial Gain = " << cameras[0].GainRaw.GetValue() << endl;

	// Set the gain ranges for luminance control.
	cameras[0].AutoGainRawLowerLimit.SetValue(cameras[0].GainRaw.GetMin());
	cameras[0].AutoGainRawUpperLimit.SetValue(cameras[0].GainRaw.GetMax());

	cameras[0].GainAuto.SetValue(GainAuto_Once);
	// When the "once" mode of operation is selected,
	// the parameter values are automatically adjusted until the related image property
	// reaches the target value. After the automatic parameter value adjustment is complete, the auto
	// function will automatically be set to "off" and the new parameter value will be applied to the
	// subsequently grabbed images.

	int n = 0;
	// Starts grabbing for all cameras.
	// The cameras won't transmit any image data, because they are configured to wait for an action command.
	cameras[0].StartGrabbing();

	while (cameras[0].GainAuto.GetValue() != GainAuto_Off) {

		// Now we issue the action command to all devices in the subnet.
		// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
		pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

		// This smart pointer will receive the grab result data.
		CBaslerGigEGrabResultPtr ptrGrabResult;

		// Retrieve images from all cameras.
		const int DefaultTimeout_ms = 5000;

		// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
		cameras[0].RetrieveResult(DefaultTimeout_ms, ptrGrabResult,
				TimeoutHandling_ThrowException);

		// Image grabbed successfully?
		if (ptrGrabResult->GrabSucceeded()) {

			// Convert the grabbed buffer to a pylon image.
			formatConverter.Convert(pylonImage, ptrGrabResult);

			// Create an OpenCV image from a pylon image.
			openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

			// Create an OpenCV display window.
			namedWindow("Camera 0", CV_WINDOW_NORMAL);// other options: // CV_AUTOSIZE, CV_FREERATIO

			// Display the current image in the OpenCV display window.
			imshow("Camera 0", openCvImage);
			// Define a timeout for customer's input in ms.
			// '0' means indefinite, i.e. the next image will be displayed after closing the window.
			// '1' means live stream
			waitKey(1);

			// Plot the histogram
			PlotHistogram(openCvImage);

		} else {
				// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
				// multiple images simultaneously. See note above c_maxCamerasToUse.
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " "
						<< ptrGrabResult->GetErrorDescription() << endl;
		}

		++n;

		cameras[0].WaitForFrameTriggerReady(500, TimeoutHandling_ThrowException);

		//For demonstration purposes only. Wait until the image is shown.
		//WaitObject::Sleep(100);

		//Make sure the loop is exited.
		if (n > 100) {
			throw TIMEOUT_EXCEPTION( "The adjustment of auto gain did not finish.");
		}
	}

	cameras[0].StopGrabbing();

	cout << "GainAuto went back to 'Off' after " << n << " frames." << endl;
	cout << "Final Gain = " << cameras[0].GainRaw.GetValue() << endl << endl;
}

void PlotHistogram(const Mat &src) {

	/// Separate the image in 3 places ( B, G and R )
	vector<Mat> bgr_planes;
	split(src, bgr_planes);

	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 };
	const float* histRange = { range };

	bool uniform = true;
	bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	/// Compute the histograms:
	calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange,	uniform, accumulate);
	calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange,	uniform, accumulate);
	calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange,	uniform, accumulate);

	// Draw the histograms for B, G and R
	int hist_w = 512;
	int hist_h = 400;
	int bin_w = cvRound((double) hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	/// Normalize the result to [ 0, histImage.rows ]
	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	/// Draw for each channel
	for (int i = 1; i < histSize; i++) {
		line(histImage,	Point(bin_w * (i - 1),
						hist_h - cvRound(b_hist.at<float>(i - 1))),
				Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
		line(histImage,
				Point(bin_w * (i - 1),
						hist_h - cvRound(g_hist.at<float>(i - 1))),
				Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
				Scalar(0, 255, 0), 2, 8, 0);
		line(histImage,
				Point(bin_w * (i - 1),
						hist_h - cvRound(r_hist.at<float>(i - 1))),
				Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
				Scalar(0, 0, 255), 2, 8, 0);
	}

	/// Display
	namedWindow("Histogram", CV_WINDOW_AUTOSIZE);
	imshow("Histogram", histImage);

	waitKey(1);
}

