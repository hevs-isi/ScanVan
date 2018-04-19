// Grab_UsingActionCommand.cpp
/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample shows how to issue a GigE Vision ACTION_CMD to multiple cameras.
    By using an action command multiple cameras can be triggered at the same time
    compared to software triggering, which must be triggered individually.

    To make the configuration of multiple cameras easier this sample uses the CInstantCameraArray class.
    It also uses a CActionTriggerConfiguration to set up the basic action command features.
*/

// Define as 1 if images are to be saved.
#define SAVE_IMAGES 1

// Define as 1 if images are to be displayed in a window
#define SHOW_IMAGES 1

// Define as 1 to do an auto gain adjustment on the camera 0 in the beginning
#define AUTO_GAIN_0 0

// Define as 1 to do an auto exposure time adjustment on the camera 0 in the beginning
#define AUTO_EXPOSURE_TIME_0 0

// Define as 1 to do an auto gain adjustment on the camera 0 continuously
#define AUTO_GAIN_CONTINUOUS_0 1

// Define as 1 to do an auto exposure time adjustment on the camera 0 continuously
#define AUTO_EXPOSURE_TIME_CONTINUOUS_0 1

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>   // for time
#include <stdlib.h> // for rand & srand

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>

#include <iostream>
#include <fstream>

#include <time.h>
#include <chrono>

#include <unistd.h>
#include <pthread.h>
#include <mutex>
#include <sched.h>

// Settings to use Basler GigE cameras.
using namespace Basler_GigECameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

#ifndef USE_GIGE
#error This example is usable for GigE cameras only.
#endif

// Namespace for using the timing functions
using namespace std::chrono;

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

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

// This smart pointer will receive the grab result data.
// Prepare two buffers
CBaslerGigEGrabResultPtr ptrGrabResult1;
CBaslerGigEGrabResultPtr ptrGrabResult2;

//Additional buffers
uint8_t buffer1[3004*3004*12/8];
uint8_t buffer2[3004*3004*12/8];

// Flags to control access to the buffers
bool buffer1_writable = true;
bool buffer2_writable = true;
bool buffer1_readable = false;
bool buffer2_readable = false;
bool buffer1_iswriting = false;
bool buffer2_iswriting = false;
bool buffer1_isdisplaying = false;
bool buffer2_isdisplaying = false;

// Barrier to synchronize the threads
pthread_barrier_t mybarrier;

// Passing arguments to the thread
struct thread_data {
	int thread_id;
	CBaslerGigEInstantCameraArray *cameras;
	IGigETransportLayer *pTL;
	uint32_t DeviceKey;
	uint32_t GroupKey;
	String_t subnet;
	DeviceInfoList_t *usableDeviceInfos;
};

// Boolean to exit loops in thread
bool terminateLoop = false;

int numImages = 0;

// Prototypes
void AutoExposureOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet);
void AutoGainOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet);
void AutoAdjustOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet);
double PlotHistogram(const Mat &src);

void* thCapture (void *threadarg) {

	struct thread_data *my_data;
	my_data = (struct thread_data *) threadarg;

	int thread_id = my_data->thread_id;

	// Starts grabbing for all cameras.
	// The cameras won't transmit any image data, because they are configured to wait for an action command.
	my_data->cameras->StartGrabbing();

	const int DefaultTimeout_ms = 5000;

	// Counter of the number of images grabbed
	int grabbedImages = 0;

	while ((!terminateLoop)&&(my_data->cameras->IsGrabbing())) {

		if (buffer1_writable) {

			cout << "thread " << thread_id << ": Start capturing pic number "
					<< grabbedImages << ". Writing buffer 1." << endl;

			buffer1_writable = false;

			//////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////
			// Use an Action Command to Trigger Multiple Cameras at the Same Time.
			//////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////

			cout << endl << "Issuing an action command." << endl;

			// Now we issue the action command to all devices in the subnet.
			// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
			my_data->pTL->IssueActionCommand(my_data->DeviceKey,
					my_data->GroupKey, AllGroupMask, my_data->subnet);

			// Retrieve images from all cameras.

			for (size_t i = 0;
					i < my_data->usableDeviceInfos->size()
							&& my_data->cameras->IsGrabbing(); ++i) {

				//g_Mutex.lock();
				// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
				my_data->cameras->RetrieveResult(DefaultTimeout_ms,
						ptrGrabResult1, TimeoutHandling_ThrowException);
				//g_Mutex.unlock();

				// When the cameras in the array are created the camera context value
				// is set to the index of the camera in the array.
				// The camera context is a user-settable value.
				// This value is attached to each grab result and can be used
				// to determine the camera that produced the grab result.
				intptr_t cameraIndex = ptrGrabResult1->GetCameraContext();

				// Image grabbed successfully?
				if (ptrGrabResult1->GrabSucceeded()) {

					// Print the index and the model name of the camera.
					cout << "Camera " << cameraIndex << ": "
							<< (*(my_data->cameras))[cameraIndex].GetDeviceInfo().GetModelName()
							<< " ("
							<< (*(my_data->cameras))[cameraIndex].GetDeviceInfo().GetIpAddress()
							<< ")" << endl;

					// You could process the image here by accessing the image buffer.
					cout << "GrabSucceeded: " << ptrGrabResult1->GrabSucceeded()
							<< endl;
					const uint8_t *pImageBuffer =
							(uint8_t *) ptrGrabResult1->GetBuffer();

					cout << "Gray value of first pixel: "
							<< (uint32_t) pImageBuffer[0] << endl;
					cout << "Image number: " << grabbedImages << endl << endl;

				} else {
					// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
					// multiple images simultaneously. See note above c_maxCamerasToUse.
					cout << "Error: " << ptrGrabResult1->GetErrorCode() << " "
							<< ptrGrabResult1->GetErrorDescription() << endl;
				}
			}

			numImages++;

			// In case you want to trigger again you should wait for the camera
			// to become trigger-ready before issuing the next action command.
			// To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
			// (see Grab_UsingGrabLoopThread sample for details).
			for (size_t i = 0; i < my_data->cameras->GetSize(); ++i) {
				(*(my_data->cameras))[i].WaitForFrameTriggerReady(1000,
						TimeoutHandling_ThrowException);
			}

			buffer1_readable = true;
			cout << "thread " << thread_id << ": Finished capturing." << endl;


		} else if (buffer2_writable) {

			cout << "thread " << thread_id << ": Start capturing pic number "
					<< grabbedImages << ". Writing buffer 2." << endl;

			buffer2_writable = false;

			//////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////
			// Use an Action Command to Trigger Multiple Cameras at the Same Time.
			//////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////

			cout << endl << "Issuing an action command." << endl;

			// Now we issue the action command to all devices in the subnet.
			// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
			my_data->pTL->IssueActionCommand(my_data->DeviceKey,
					my_data->GroupKey, AllGroupMask, my_data->subnet);

			// Retrieve images from all cameras.

			for (size_t i = 0;
					i < my_data->usableDeviceInfos->size()
							&& my_data->cameras->IsGrabbing(); ++i) {

				// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
				my_data->cameras->RetrieveResult(DefaultTimeout_ms,
						ptrGrabResult2, TimeoutHandling_ThrowException);

				// When the cameras in the array are created the camera context value
				// is set to the index of the camera in the array.
				// The camera context is a user-settable value.
				// This value is attached to each grab result and can be used
				// to determine the camera that produced the grab result.
				intptr_t cameraIndex = ptrGrabResult2->GetCameraContext();

				// Image grabbed successfully?
				if (ptrGrabResult2->GrabSucceeded()) {

					// Print the index and the model name of the camera.
					cout << "Camera " << cameraIndex << ": "
							<< (*(my_data->cameras))[cameraIndex].GetDeviceInfo().GetModelName()
							<< " ("
							<< (*(my_data->cameras))[cameraIndex].GetDeviceInfo().GetIpAddress()
							<< ")" << endl;

					// You could process the image here by accessing the image buffer.
					cout << "GrabSucceeded: " << ptrGrabResult2->GrabSucceeded()
							<< endl;
					const uint8_t *pImageBuffer =
							(uint8_t *) ptrGrabResult2->GetBuffer();

					cout << "Gray value of first pixel: "
							<< (uint32_t) pImageBuffer[0] << endl;
					cout << "Image number: " << grabbedImages << endl << endl;

				} else {
					// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
					// multiple images simultaneously. See note above c_maxCamerasToUse.
					cout << "Error: " << ptrGrabResult2->GetErrorCode() << " "
							<< ptrGrabResult2->GetErrorDescription() << endl;
				}
			}

			numImages++;

			// In case you want to trigger again you should wait for the camera
			// to become trigger-ready before issuing the next action command.
			// To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
			// (see Grab_UsingGrabLoopThread sample for details).
			for (size_t i = 0; i < my_data->cameras->GetSize(); ++i) {
				(*(my_data->cameras))[i].WaitForFrameTriggerReady(1000,
						TimeoutHandling_ThrowException);
			}

			buffer2_readable = true;
			cout << "thread " << thread_id << ": Finished capturing." << endl;

		}

		pthread_barrier_wait(&mybarrier);

	}

	my_data->cameras->StopGrabbing();

	pthread_exit(NULL);
}

void* thSave (void *threadarg) {

	struct thread_data *my_data;
	my_data = (struct thread_data *) threadarg;

	int thread_id = my_data->thread_id;

	int picNum = 0;

	while (!terminateLoop) {

		if (buffer1_readable) {
			buffer1_readable = false;
			cout << "thread " << thread_id << ": Start saving pic number " << picNum << ". Reading buffer 1." << endl;

			// When the cameras in the array are created the camera context value
			// is set to the index of the camera in the array.
			// The camera context is a user-settable value.
			// This value is attached to each grab result and can be used
			// to determine the camera that produced the grab result.
			intptr_t cameraIndex = ptrGrabResult1->GetCameraContext();

			// Save image using Pylon interface
			std::ostringstream s1;
			// Create image name files with ascending grabbed image numbers.
			s1 << "./img/image_" << cameraIndex << "_" << picNum	<< ".raw";
			std::string imageName(s1.str());

			CImagePersistence::Save( ImageFileFormat_Raw, String_t(imageName.c_str()), ptrGrabResult1);

			cout << "thread " << thread_id << ": Finished saving." << endl;
			picNum++;


			buffer1_writable = true;

		} else if (buffer2_readable) {
			buffer2_readable = false;
			cout << "thread " << thread_id << ": Start saving pic number " << picNum << ". Reading buffer 2." << endl;

			// When the cameras in the array are created the camera context value
			// is set to the index of the camera in the array.
			// The camera context is a user-settable value.
			// This value is attached to each grab result and can be used
			// to determine the camera that produced the grab result.
			intptr_t cameraIndex = ptrGrabResult2->GetCameraContext();

			// Save image using Pylon interface
			std::ostringstream s1;
			// Create image name files with ascending grabbed image numbers.
			s1 << "./img/image_" << cameraIndex << "_" << picNum << ".raw";
			std::string imageName(s1.str());

			CImagePersistence::Save(ImageFileFormat_Raw, String_t(imageName.c_str()), ptrGrabResult2);

			cout << "thread " << thread_id << ": Finished saving." << endl;
			picNum++;

			buffer2_writable = true;
		}

		pthread_barrier_wait(&mybarrier);
	}


	pthread_exit(NULL);
}

int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

	// Declare an integer variable to count the number of grabbed images
	// and create image file names with ascending number.
	int grabbedImages = 0;

	// Set up the threads and barrier
	const int NUM_THREAD = 3;
	const int THREAD_BARRIER_COUNT = 2;

	int rc;
	int i;
	pthread_t ids[NUM_THREAD];

	struct thread_data td[NUM_THREAD];

	pthread_barrier_init(&mybarrier, NULL, THREAD_BARRIER_COUNT);


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
                cerr << "Camera will not be used because it is in a different subnet "
                     << subnet << "!" << endl;
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

		const int width_reduced(3004);
		const int height_reduced(3004);


		for (size_t i = 0; i < cameras.GetSize(); ++i) {
			// This sets the transfer pixel format to BayerRG8
			//cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG8);
			cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG12Packed);
			//cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG12);

			// This sets the value of the packet size on the camera to 9000
			// On the host size, change the MTU to 9014 and the maximum UDP receive buffer size to 2097152
			cameras[i].GevSCPSPacketSize.SetValue(9000);

			cameras[i].ExposureTimeAbs.SetValue(cameras[i].ExposureTimeAbs.GetMin());
			//cameras[i].ExposureTimeAbs.SetValue(6352);
			cameras[i].GainRaw.SetValue(cameras[i].GainRaw.GetMin());
			//cameras[i].GainRaw.SetValue(79);
			cameras[i].Width.SetValue(width_reduced);
			cameras[i].Height.SetValue(height_reduced);


		}

#if AUTO_EXPOSURE_TIME_0==1
		cameras[0].ExposureTimeAbs.SetValue(cameras[0].ExposureTimeAbs.GetMin());
		cameras[0].GainRaw.SetValue((cameras[0].GainRaw.GetMax()-cameras[0].GainRaw.GetMin())/2);
		// Only area scan cameras support auto functions.
		if (cameras[0].DeviceScanType.GetValue() == DeviceScanType_Areascan) {
			AutoExposureOnce(cameras, pTL, DeviceKey, GroupKey, subnet);
		}
#endif

#if AUTO_GAIN_0==1
		cameras[0].GainRaw.SetValue(cameras[0].GainRaw.GetMin());
		// Carry out luminance control by using the "once" gain auto function.
		// Only area scan cameras support auto functions.
		if (cameras[0].DeviceScanType.GetValue() == DeviceScanType_Areascan) {
			AutoGainOnce(cameras, pTL, DeviceKey, GroupKey, subnet);
		}
#endif


#if AUTO_GAIN_CONTINUOUS_0==1
		cameras[0].GainAuto.SetValue(GainAuto_Continuous);
#endif

#if AUTO_EXPOSURE_TIME_CONTINUOUS_0==1
		cameras[0].ExposureAuto.SetValue(ExposureAuto_Continuous);
#endif

		// For measuring the grabbing time
		high_resolution_clock::time_point t1;
		high_resolution_clock::time_point t2;

		td[0].thread_id = 0;
		td[0].DeviceKey = DeviceKey;
		td[0].GroupKey = GroupKey;
		td[0].cameras = &cameras;
		td[0].pTL = pTL;
		td[0].subnet = subnet;
		td[0].usableDeviceInfos = &usableDeviceInfos;

//		pthread_attr_t tattr;
//		int ret;
//		int newprio = 80;
//		sched_param param;
//
//		ret = pthread_attr_init (&tattr);
//		ret = pthread_attr_getschedparam (&tattr, &param);
//		param.sched_priority = newprio;
//		ret = pthread_attr_setschedparam (&tattr, &param);

		rc = pthread_create(&ids[0], NULL, thCapture, (void *)&td[0]);
		if (rc) {
			cout << "Error: Unable to create thread, " << rc << endl;
			exit(-1);
		}

//		td[1].thread_id = 1;
//		td[1].DeviceKey = DeviceKey;
//		td[1].GroupKey = GroupKey;
//		td[1].cameras = &cameras;
//		td[1].pTL = pTL;
//		td[1].subnet = subnet;
//		td[1].usableDeviceInfos = &usableDeviceInfos;
//
//		rc = pthread_create(&ids[1], NULL, thSave, (void *)&td[1]);
//		if (rc) {
//			cout << "Error: Unable to create thread, " << rc << endl;
//			exit(-1);
//		}

//		int id = 2;
//
//		rc = pthread_create(&ids[2], NULL, thDetectKeyStroke, (void *)&id);
//		if (rc) {
//			cout << "Error: Unable to create thread, " << rc << endl;
//			exit(-1);
//		}

		// Create a pylon ImageFormatConverter object.
		CImageFormatConverter formatConverter;
		// Specify the output pixel format.
		formatConverter.OutputPixelFormat = PixelType_BGR8packed;
		// Create a PylonImage that will be used to create OpenCV images later.
		CPylonImage pylonImage;
		// Create an OpenCV image.
		Mat openCvImage;


		// Measure the starting of grabbing
		t1 = high_resolution_clock::now();

		int thread_id = 1;
		int picNum = 0;
		int key = 0;

		while (!terminateLoop) {
			if (buffer1_readable) {

				buffer1_readable = false;

				cout << "thread " << thread_id << ": Start displaying pic number "
						<< picNum << ". Reading buffer 1." << endl;


				intptr_t cameraIndex = ptrGrabResult1->GetCameraContext();

				// Save image using Pylon interface
				std::ostringstream s1;
				// Create image name files with ascending grabbed image numbers.
				s1 << "./img/image_" << cameraIndex << "_" << picNum	<< ".bmp";
				std::string imageName(s1.str());


//				try {
//					CImagePersistence::Save( ImageFileFormat_Raw, String_t(imageName.c_str()), ptrGrabResult1);
//				}
//				catch (const GenericException &e)
//				{
//					// Error handling
//					cerr << "An exception occurred." << endl
//							<< e.GetDescription() << endl;
//					exitCode = 1;
//				}

				// Convert the grabbed buffer to a pylon image.
				formatConverter.Convert(pylonImage, ptrGrabResult1);

				// Create an OpenCV image from a pylon image.
				openCvImage = cv::Mat(ptrGrabResult1->GetHeight(),
						ptrGrabResult1->GetWidth(), CV_8UC3,
						(uint8_t *) pylonImage.GetBuffer());

				//imwrite(imageName, openCvImage);

				ofstream myFile (imageName, ios::out | ios::binary);
				myFile.write ((char*)buffer1,3004*3004*2);
				myFile.close();


				// Specify the name of the window to show
				String windowTitle = "Camera " + to_string(cameraIndex);

				// Create an OpenCV display window.
				namedWindow(windowTitle, CV_WINDOW_NORMAL);	// other options: // CV_AUTOSIZE, CV_FREERATIO

				// Display the current image in the OpenCV display window.
				imshow(windowTitle, openCvImage);

				// Define a timeout for customer's input in ms.
				// '0' means indefinite, i.e. the next image will be displayed after closing the window.
				// '1' means live stream
				key=waitKey(1);

				if (key == 113 )
					terminateLoop = true;

				cout << "thread " << thread_id << ": Finished saving." << endl;
				picNum++;

				buffer1_writable = true;


			} else if (buffer2_readable) {

				buffer2_readable = false;

				cout << "thread " << thread_id << ": Start saving pic number "
						<< picNum << ". Reading buffer 2." << endl;

				intptr_t cameraIndex = ptrGrabResult2->GetCameraContext();

				// Save image using Pylon interface
				std::ostringstream s1;
				// Create image name files with ascending grabbed image numbers.
				s1 << "./img/image_" << cameraIndex << "_" << picNum << ".bmp";
				std::string imageName(s1.str());

//				try {
//					CImagePersistence::Save( ImageFileFormat_Raw, String_t(imageName.c_str()), ptrGrabResult2);
//				}
//				catch (const GenericException &e)
//				{
//					// Error handling
//					cerr << "An exception occurred." << endl
//							<< e.GetDescription() << endl;
//					exitCode = 1;
//				}

				// Convert the grabbed buffer to a pylon image.
				formatConverter.Convert(pylonImage, ptrGrabResult2);

				// Create an OpenCV image from a pylon image.
				openCvImage = cv::Mat(ptrGrabResult2->GetHeight(),
						ptrGrabResult2->GetWidth(), CV_8UC3,
						(uint8_t *) pylonImage.GetBuffer());


				//imwrite(imageName, openCvImage);
				ofstream myFile (imageName, ios::out | ios::binary);
				myFile.write ((char*)buffer1,3004*3004*2);
				myFile.close();


				// Specify the name of the window to show
				String windowTitle = "Camera " + to_string(cameraIndex);

				// Create an OpenCV display window.
				namedWindow(windowTitle, CV_WINDOW_NORMAL);	// other options: // CV_AUTOSIZE, CV_FREERATIO

				// Display the current image in the OpenCV display window.
				imshow(windowTitle, openCvImage);

				// Define a timeout for customer's input in ms.
				// '0' means indefinite, i.e. the next image will be displayed after closing the window.
				// '1' means live stream
				key = waitKey(1);

				if (key == 113 )
					terminateLoop = true;

				cout << "thread " << thread_id << ": Finished saving." << endl;
				picNum++;

				buffer2_writable = true;



			}

			pthread_barrier_wait(&mybarrier);
		}

		// Measure the end of the grabbing
		t2 = high_resolution_clock::now();

		destroyAllWindows();

		rc = pthread_join(ids[0], NULL);
		if (rc) {
			cout  << "Error: Unable to join," << rc << endl;
		}
//		rc = pthread_join(ids[1], NULL);
//		if (rc) {
//			cout  << "Error: Unable to join," << rc << endl;
//		}
//		rc = pthread_join(ids[2], NULL);
//		if (rc) {
//			cout  << "Error: Unable to join," << rc << endl;
//		}


		// Measure duration of grabbing
		auto duration = duration_cast<microseconds>(t2 - t1).count();
		cout << "time for grabbing: " << duration << " microseconds" << endl << flush;
		cout << "fps: " << double(1000000) / duration * numImages << endl;


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

    pthread_barrier_destroy(&mybarrier);

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while (cin.get() != '\n');

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}

void AutoExposureOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet) {

#if SHOW_IMAGES ==1
	// Create a pylon ImageFormatConverter object.
	CImageFormatConverter formatConverter;
	// Specify the output pixel format.
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	// Create a PylonImage that will be used to create OpenCV images later.
	CPylonImage pylonImage;
	// Create an OpenCV image.
	Mat openCvImage;
#endif

	// Check whether auto exposure is available
	if (!IsWritable(cameras[0].ExposureAuto)) {
		cout << "The camera does not support Exposure Auto." << endl << endl;
		return;
	}

	// Region for adjustment
	int width_reduced(3004);
	int height_reduced(3004);
	int offsetX_reduced(552);
	int offsetY_reduced(0);

	// Full region
	int width_full(3004);
	int height_full(3004);
	int offsetX_full(552);
	int offsetY_full(0);

	cameras[0].Width.SetValue(width_reduced);
	cameras[0].Height.SetValue(height_reduced);

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_reduced);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_reduced);
	}

	// Set the Auto Function AOI for luminance statistics.
	// Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
	// luminance statistics.
	cameras[0].AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
	cameras[0].AutoFunctionAOIWidth.SetValue(width_reduced);
	cameras[0].AutoFunctionAOIHeight.SetValue(height_reduced);
	cameras[0].AutoFunctionAOIOffsetX.SetValue(offsetX_reduced);
	cameras[0].AutoFunctionAOIOffsetY.SetValue(offsetY_reduced);

	// Set the target value for luminance control. The value is always expressed
	// as an 8 bit value regardless of the current pixel data output format,
	// i.e., 0 -> black, 255 -> white.
	cameras[0].AutoTargetValue.SetValue(cameras[0].AutoTargetValue.GetMax()*80/255);

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

#ifdef PYLON_WIN_BUILD
			Pylon::DisplayImage(1, ptrGrabResult);
#endif

#if SHOW_IMAGES==1
			// Convert the grabbed buffer to a pylon image.
			formatConverter.Convert(pylonImage, ptrGrabResult);

			// Create an OpenCV image from a pylon image.
			openCvImage = cv::Mat(ptrGrabResult->GetHeight(),
					ptrGrabResult->GetWidth(), CV_8UC3,
					(uint8_t *) pylonImage.GetBuffer());

			// Create an OpenCV display window.
			namedWindow("Camera 0", CV_WINDOW_NORMAL); // other options: // CV_AUTOSIZE, CV_FREERATIO

			// Display the current image in the OpenCV display window.
			imshow("Camera 0", openCvImage);
			// Define a timeout for customer's input in ms.
			// '0' means indefinite, i.e. the next image will be displayed after closing the window.
			// '1' means live stream
			waitKey(1);

			// Plot the histogram
			//PlotHistogram(openCvImage);
#endif

		}
		++n;

		cameras[0].WaitForFrameTriggerReady(500, TimeoutHandling_ThrowException);

		//For demonstration purposes only. Wait until the image is shown.
		//WaitObject::Sleep(100);

		//Make sure the loop is exited.
		if (n > 100) {
			throw TIMEOUT_EXCEPTION( "The adjustment of auto exposure did not finish.");
		}
	}

	cameras[0].StopGrabbing();

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_full);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_full);
	}
	cameras[0].Width.SetValue(width_full);
	cameras[0].Height.SetValue(height_full);

	cout << "ExposureAuto went back to 'Off' after " << n << " frames." << endl;
	cout << "Final exposure time = ";
	cout << cameras[0].ExposureTimeAbs.GetValue() << " us" << endl << endl;
}


void AutoGainOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet)  {

#if SHOW_IMAGES ==1
	// Create a pylon ImageFormatConverter object.
	CImageFormatConverter formatConverter;
	// Specify the output pixel format.
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	// Create a PylonImage that will be used to create OpenCV images later.
	CPylonImage pylonImage;
	// Create an OpenCV image.
	Mat openCvImage;
#endif

	// Check whether the gain auto function is available.
	if (!IsWritable(cameras[0].GainAuto)) {
		cout << "The camera does not support Gain Auto." << endl << endl;
		return;
	}

	// Region for adjustment
	int width_reduced(3004);
	int height_reduced(3004);
	int offsetX_reduced(552);
	int offsetY_reduced(0);

	// Full region
	int width_full(3004);
	int height_full(3004);
	int offsetX_full(552);
	int offsetY_full(0);

	cameras[0].Width.SetValue(width_reduced);
	cameras[0].Height.SetValue(height_reduced);

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_reduced);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_reduced);
	}

	// Set the Auto Function AOI for luminance statistics.
	// Currently, AutoFunctionAOISelector_AOI1 is predefined to gather
	// luminance statistics.
	cameras[0].AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
	cameras[0].AutoFunctionAOIWidth.SetValue(width_reduced);
	cameras[0].AutoFunctionAOIHeight.SetValue(height_reduced);
	cameras[0].AutoFunctionAOIOffsetX.SetValue(offsetX_reduced);
	cameras[0].AutoFunctionAOIOffsetY.SetValue(offsetY_reduced);

	// Set the target value for luminance control. The value is always expressed
	// as an 8 bit value regardless of the current pixel data output format,
	// i.e., 0 -> black, 255 -> white.
	cameras[0].AutoTargetValue.SetValue(cameras[0].AutoTargetValue.GetMax()*80/255);

	// We are going to try GainAuto = Once.

	cout << "Trying 'GainAuto = Once'." << endl;
	cout << "Initial Gain = " << cameras[0].GainRaw.GetValue() << endl;

	// Set the gain ranges for luminance control.
	cameras[0].AutoGainRawLowerLimit.SetValue(cameras[0].GainRaw.GetMin());
	cameras[0].AutoGainRawUpperLimit.SetValue(cameras[0].GainRaw.GetMax());

	cameras[0].GainAuto.SetValue(GainAuto_Once);
	//cameras[0].GainAuto.SetValue(GainAuto_Continuous);

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

#ifdef PYLON_WIN_BUILD
			Pylon::DisplayImage(1, ptrGrabResult);
#endif

#if SHOW_IMAGES ==1

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
			//PlotHistogram(openCvImage);
#endif
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

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_full);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_full);
	}
	cameras[0].Width.SetValue(width_full);
	cameras[0].Height.SetValue(height_full);

	cout << "GainAuto went back to 'Off' after " << n << " frames." << endl;
	cout << "Final Gain = " << cameras[0].GainRaw.GetValue() << endl << endl;
}

void AutoAdjustOnce(CBaslerGigEInstantCameraArray& cameras, IGigETransportLayer *pTL, uint32_t DeviceKey, uint32_t GroupKey, String_t subnet)  {

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

	// Region for adjustment
	int width_reduced(3004);
	int height_reduced(3004);
	int offsetX_reduced(552);
	int offsetY_reduced(0);

	// Full region
	int width_full(3004);
	int height_full(3004);
	int offsetX_full(552);
	int offsetY_full(0);

	cameras[0].Width.SetValue(width_reduced);
	cameras[0].Height.SetValue(height_reduced);

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_reduced);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_reduced);
	}

	// We are going to try GainAuto = Once.

	cout << "Trying 'GainAuto = Once'." << endl;
	cout << "Initial Gain = " << cameras[0].GainRaw.GetValue() << endl;

	int n = 0;
	// Starts grabbing for all cameras.
	// The cameras won't transmit any image data, because they are configured to wait for an action command.
	cameras[0].StartGrabbing();

	bool auto_adjust_off(false);
	double med(128);

	while (!auto_adjust_off) {

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

#ifdef PYLON_WIN_BUILD
			Pylon::DisplayImage(1, ptrGrabResult);
#endif

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
			med = PlotHistogram(openCvImage);


		}
		++n;

		cameras[0].WaitForFrameTriggerReady(500, TimeoutHandling_ThrowException);

		if (med > 138) {
			//cameras[0].ExposureTimeAbs.SetValue(cameras[0].ExposureTimeAbs.GetValue()-10);
			cameras[0].GainRaw.SetValue(cameras[0].GainRaw.GetValue()-10);
		} else if (med < 118) {
			//cameras[0].ExposureTimeAbs.SetValue(cameras[0].ExposureTimeAbs.GetValue()+10);
			cameras[0].GainRaw.SetValue(cameras[0].GainRaw.GetValue()+10);
		} else {
			auto_adjust_off = true;
		}

		//For demonstration purposes only. Wait until the image is shown.
		//WaitObject::Sleep(100);

		//Make sure the loop is exited.
		if (n > 100) {
			throw TIMEOUT_EXCEPTION( "The adjustment of auto gain did not finish.");
		}
	}

	cameras[0].StopGrabbing();

	// Maximize the grabbed area of interest (Image AOI).
	if (IsWritable(cameras[0].OffsetX)) {
		cameras[0].OffsetX.SetValue(offsetX_full);
	}
	if (IsWritable(cameras[0].OffsetY)) {
		cameras[0].OffsetY.SetValue(offsetY_full);
	}
	cameras[0].Width.SetValue(width_full);
	cameras[0].Height.SetValue(height_full);

	cout << "GainAuto went back to 'Off' after " << n << " frames." << endl;
	cout << "Final Gain = " << cameras[0].GainRaw.GetValue() << endl << endl;
}


double PlotHistogram(const Mat &src) {

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

	// Calculate the median
	double med = -1.0;
	int bin = 0;
	double m = (bgr_planes[1].rows * bgr_planes[1].cols) / 2;

	for (int i = 0; i < histSize && med < 0.0; ++i) {
		bin += cvRound(g_hist.at<float>(i));
		if (bin > m && med < 0.0)
			med = i;
	}

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

	line(histImage, Point(bin_w * med , 0), Point(bin_w * med, hist_h), Scalar(0,255,0), 2, 8, 0);

	/// Display
	namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
	imshow("calcHist Demo", histImage);

	waitKey(1);

	return med;
}

