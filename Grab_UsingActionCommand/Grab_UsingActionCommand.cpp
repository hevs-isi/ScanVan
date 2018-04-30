// Grab_UsingActionCommand.cpp
/*
   This program grabs images from the camera and saves in raw format.
   It displays the images on the screen.
   There are three threads:
   One that grabs the image and saves in a buffer.
   Another saves the buffer to a file.
   The third one displays the image.
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

// Parameters for the image
const int imageWidth = 3004;
const int imageHeight = 3004;
const int offsetX = 502;
const int offsetY = 2;

// Image buffers
const unsigned long int imageBufferSize = imageWidth * imageHeight; // Unpacked 8-bit per pixel
uint8_t rawImageBuffer[imageBufferSize];

// Flags to control access to the buffers
bool bufferOpenCvReadable = false;
bool bufferRawReadable = false;

// Barrier to synchronize the threads
pthread_barrier_t thBarrier;
// Markers for the barrier
int barrier_0 = 0;
int barrier_1 = 0;
int barrier_2 = 0;

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

struct image_data {
	size_t cameraIdx; 		// camera index
	time_t captureTime;		// capture time
	double exposureTime;	// exposure time
	int64_t gain;			// gain
	double balanceR; 		// white balance R
	double balanceG; 		// white balance G
	double balanceB;		// white balance B
	int autoExpTime;		// Auto Exposure Time
	int autoGain;
};

image_data imageData[2] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},  // Struct where information of the images are stored
						   {0, 0, 0, 0, 0, 0, 0, 0, 0}};

// Boolean to exit loops in thread
bool terminateLoop = false;
// Counts the number images grabbed
int numGrabbedImages = 0;
// Counts number of grab errors
int numGrabError = 0;

// Auto exposure time continuous on-off
bool autoExpTimeCont = false;
// Auto gain continuous on-off
bool autoGainCont = false;
// Folder where images will be stored
string imageFolder = "./img";

// Prototypes
double PlotHistogram(const Mat &src);
int readConfigFile (double &expTime0, int &gain0, double &expTime1, int &gain1);
void* thSave (void *threadarg);
void* thCapture (void *threadarg);

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
	const int THREAD_BARRIER_COUNT = 3;

	int rc;
	int i;
	pthread_t ids[NUM_THREAD];

	struct thread_data td[NUM_THREAD];

	pthread_barrier_init(&thBarrier, NULL, THREAD_BARRIER_COUNT);

	double expTime[2] {0, 0};
	int gain[2] {0, 0};

	if (readConfigFile (expTime[0], gain[0], expTime[1], gain[1])) {
		return 1; // Error reading the configuration file
	}

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
			cameras[i].GevSCPD.SetValue(0);
			cameras[i].MaxNumBuffer.SetValue(20);

			cameras[i].ExposureTimeAbs.SetValue(expTime[i]);
			cameras[i].GainRaw.SetValue(gain[i]);

			cameras[i].Width.SetValue(imageWidth);
			cameras[i].Height.SetValue(imageHeight);
			if (IsWritable(cameras[i].OffsetX)) {
				cameras[i].OffsetX.SetValue(offsetX);
			}
			if (IsWritable(cameras[i].OffsetY)) {
				cameras[i].OffsetY.SetValue(offsetY);
			}

		}

		// Sets auto adjustments continuous
		if (autoExpTimeCont)
			cameras[0].ExposureAuto.SetValue(ExposureAuto_Continuous);
		if (autoGainCont)
			cameras[0].GainAuto.SetValue(GainAuto_Continuous);

		// For measuring the grabbing time
		high_resolution_clock::time_point t1;
		high_resolution_clock::time_point t2;

		td[0].thread_id = 1;
		td[0].DeviceKey = DeviceKey;
		td[0].GroupKey = GroupKey;
		td[0].cameras = &cameras;
		td[0].pTL = pTL;
		td[0].subnet = subnet;
		td[0].usableDeviceInfos = &usableDeviceInfos;

		rc = pthread_create(&ids[0], NULL, thCapture, (void *)&td[0]);
		if (rc) {
			cout << "Error: Unable to create thread, " << rc << endl;
			exit(-1);
		}

		td[1].thread_id = 2;
		td[1].DeviceKey = DeviceKey;
		td[1].GroupKey = GroupKey;
		td[1].cameras = &cameras;
		td[1].pTL = pTL;
		td[1].subnet = subnet;
		td[1].usableDeviceInfos = &usableDeviceInfos;

		rc = pthread_create(&ids[1], NULL, thSave, (void *)&td[1]);
		if (rc) {
			cout << "Error: Unable to create thread, " << rc << endl;
			exit(-1);
		}

		// Create an OpenCV image.
		Mat openCvImage;
		Mat openCvImageRG8;

		// Measure the starting of grabbing
		t1 = high_resolution_clock::now();

		int thread_id = 0;
		int imageNum = 0;
		int key = 0;

		while (!terminateLoop) {
			if (bufferOpenCvReadable) {

				cout << "thread " << thread_id << ": Start displaying image number " << imageNum << endl;

				//intptr_t cameraIndex = ptrGrabResult1->GetCameraContext();
				const int cameraIndex = 0;

				openCvImageRG8 = cv::Mat(imageHeight, imageWidth, CV_8UC1, rawImageBuffer);

				cvtColor(openCvImageRG8, openCvImage, COLOR_BayerRG2RGB);

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

				if (key == 113) // if 'q' key is pressed
					terminateLoop = true;

				cout << "thread " << thread_id << ": Finished displaying buffer." << endl;
				imageNum++;


			}

			cout << "thread " << thread_id << ": Barrier 1" << endl;
			barrier_0 = 1;
			pthread_barrier_wait(&thBarrier);
			cout << "thread " << thread_id << ": Barrier 2" << endl;
			barrier_0 = 2;
			pthread_barrier_wait(&thBarrier);
		}

		// Measure the end of the grabbing
		t2 = high_resolution_clock::now();

		destroyAllWindows();

		rc = pthread_join(ids[0], NULL);
		if (rc) {
			cout  << "Error: Unable to join," << rc << endl;
		}
		rc = pthread_join(ids[1], NULL);
		if (rc) {
			cout  << "Error: Unable to join," << rc << endl;
		}

		// Measure duration of grabbing
		auto duration = duration_cast<microseconds>(t2 - t1).count();
		cout << "time for grabbing: " << duration << " microseconds" << endl << flush;
		cout << "fps: " << double(1000000) / duration * numGrabbedImages << endl;
		cout << "Number of grabbing errors: " << numGrabError << endl;

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

    pthread_barrier_destroy(&thBarrier);

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while (cin.get() != '\n');

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}

int readConfigFile (double &expTime0, int &gain0, double &expTime1, int &gain1) {

	expTime0 = 0;
	expTime1 = 0;
	gain0 = 0;
	gain1 = 0;

	// Reads the camera configuration parameters
	// The exposure time and the gain for camera 0 and camera 1
	ifstream myConfigFile;
	string configFileName = "cameraparam.cfg";
	string str(""), str1(""), str2(""), str3(""), str4("");
	int numCam = 0;
	string token;

	myConfigFile.open (configFileName);
	if (myConfigFile.is_open()){
		getline (myConfigFile, str);
		token = str.substr(str.find(":") + 1,str.length());
		numCam = stoi(token);
		getline (myConfigFile, str1);
		getline (myConfigFile, str2);
		if (numCam == 2) {
			getline (myConfigFile, str3);
			getline (myConfigFile, str4);
		}
		myConfigFile.close();
	} else {
		cout << "Error: Unable to open the file \"" << configFileName << "\"";
		return 1;
	}

	token = str1.substr(str1.find(":") + 1,str1.length());
	expTime0 = stoi(token);
	token = str2.substr(str2.find(":") + 1,str2.length());
	gain0 = stoi(token);

	if (numCam == 2) {
		token = str3.substr(str3.find(":") + 1,str3.length());
		expTime1 = stoi(token);
		token = str4.substr(str4.find(":") + 1,str4.length());
		gain1 = stoi(token);
	}

	// Reads the configuration parameteres
	configFileName = "genparam.cfg";

	myConfigFile.open (configFileName);
	if (myConfigFile.is_open()){
		getline (myConfigFile, str1);	// Reads the main img folder
		getline (myConfigFile, str2);	// Auto exposure time continuous
		getline (myConfigFile, str3);	// Auto gain continuous
		myConfigFile.close();
	} else {
		cout << "Error: Unable to open the file \"" << configFileName << "\"";
		return 1;
	}

	imageFolder = str1.substr(str1.find(":") + 1,str1.length());
	token = str2.substr(str2.find(":") + 1,str2.length());
	autoExpTimeCont = (bool)stoi(token);
	token = str3.substr(str3.find(":") + 1,str3.length());
	autoGainCont = (bool)stoi(token);

	return 0;
}

void* thCapture (void *threadarg) {

	struct thread_data *passedData;
	passedData = (struct thread_data *) threadarg;

	int thread_id = passedData->thread_id;

	const int DefaultTimeout_ms = 10000;

	passedData->cameras->StartGrabbing();

	image_data imData[2]; // Struct where information of the images are stored

	cout << "thread " << thread_id << ": Start getting parameters from camera " << endl;

	// Get the information about the cameras and store temporarily in a struct
	for (size_t i = 0; i < (*(passedData->cameras)).GetSize(); ++i) {
		imData[i].cameraIdx = i;
		imData[i].exposureTime =	(*(passedData->cameras))[i].ExposureTimeAbs.GetValue();
		imData[i].gain = (*(passedData->cameras))[i].GainRaw.GetValue();
		(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
		imData[i].balanceR = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
		(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
		imData[i].balanceG = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
		(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
		imData[i].balanceB = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
		imData[i].autoExpTime = (int) autoExpTimeCont;
		imData[i].autoGain = (int) autoGainCont;
	}

	// Wait for trigger ready
	((*(passedData->cameras))[0].WaitForFrameTriggerReady(10000, TimeoutHandling_ThrowException));
	// Isuue an action command
	passedData->pTL->IssueActionCommand(passedData->DeviceKey, passedData->GroupKey, AllGroupMask, passedData->subnet);

	while (!terminateLoop) {

		// This smart pointer will receive the grab result data.
		CBaslerGigEGrabResultPtr ptrGrabResult;

		// Retrieve images from all cameras.
		for (size_t i = 0; i < passedData->usableDeviceInfos->size() && passedData->cameras->IsGrabbing(); ++i) {

			cout << "thread " << thread_id << ": Retrieve result" << endl;
			// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
			passedData->cameras->RetrieveResult(DefaultTimeout_ms, ptrGrabResult, TimeoutHandling_ThrowException);
			cout << "thread " << thread_id << ": End retrieve result" << endl;

			cout << "thread " << thread_id << ": Get camera index" << endl;
			intptr_t cameraIndex = ptrGrabResult->GetCameraContext();
			cout << "thread " << thread_id << ": Got camera index" << endl;

			auto tnow = std::chrono::system_clock::now();
			imData[cameraIndex].captureTime = std::chrono::system_clock::to_time_t(tnow);

			cout << "thread " << thread_id << ": Barrier 1" << endl;
			barrier_1 = 1;
			pthread_barrier_wait(&thBarrier);

			// Image grabbed successfully?
			if (ptrGrabResult->GrabSucceeded()) {

				const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();

				cout << "thread " << thread_id << ": Start writing image number " << numGrabbedImages << endl;

				// Copy image to raw image buffer1
				memcpy(rawImageBuffer, pImageBuffer, imageBufferSize);

				bufferOpenCvReadable = true;
				bufferRawReadable = true;
				cout << "thread " << thread_id << ": Finished writing buffer." << endl;

				numGrabbedImages++;

				cout << "thread " << thread_id << ": Start getting parameters from camera " << endl;

				// Save the attributes into a global struct so that it can be saved from a different thread
				imageData[i].cameraIdx = imData[i].cameraIdx;
				imageData[i].exposureTime =	imData[i].exposureTime;
				imageData[i].gain = imData[i].gain;
				imageData[i].balanceR = imData[i].balanceR;
				imageData[i].balanceG = imData[i].balanceG;
				imageData[i].balanceB = imData[i].balanceB;
				imageData[i].autoExpTime = imData[i].autoExpTime;
				imageData[i].autoGain = imData[i].autoGain;
				imageData[i].captureTime = imData[i].captureTime;

				cout << "thread " << thread_id << ": Barrier 2" << endl;
				barrier_1 = 2;
				pthread_barrier_wait(&thBarrier);
				cout << "-----------------------------------------------------------------" << endl;

			} else {
				// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
				// multiple images simultaneously. See note above c_maxCamerasToUse.
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " "
						<< ptrGrabResult->GetErrorDescription() << endl;
				numGrabError++;
				bufferRawReadable = false;
			}

		}

		// Get the information about the cameras and store temporarily in a struct
		for (size_t i = 0; i < (*(passedData->cameras)).GetSize(); ++i) {
			imData[i].cameraIdx = i;
			imData[i].exposureTime =	(*(passedData->cameras))[i].ExposureTimeAbs.GetValue();
			imData[i].gain = (*(passedData->cameras))[i].GainRaw.GetValue();
			(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
			imData[i].balanceR = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
			(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
			imData[i].balanceG = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
			(*(passedData->cameras))[i].BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
			imData[i].balanceB = (*(passedData->cameras))[i].BalanceRatioAbs.GetValue();
			imData[i].autoExpTime = (int) autoExpTimeCont;
			imData[i].autoGain = (int) autoGainCont;
		}

		// Wait for trigger ready
		((*(passedData->cameras))[0].WaitForFrameTriggerReady(10000, TimeoutHandling_ThrowException));
		// Issue an action command
		passedData->pTL->IssueActionCommand(passedData->DeviceKey, passedData->GroupKey, AllGroupMask, passedData->subnet);

	}

	// This is to put the barriers when the terminate loop is set to one while reading the camera parameters
	if ((barrier_0 == 1)&(barrier_2 == 1)){
		pthread_barrier_wait(&thBarrier);
		pthread_barrier_wait(&thBarrier);
	}

	passedData->cameras->StopGrabbing();

	pthread_exit(NULL);
}

void* thSave (void *threadarg) {

	struct thread_data *passedData;
	passedData = (struct thread_data *) threadarg;

	int thread_id = passedData->thread_id;

	int imageNum = 0;

	while (!terminateLoop) {

		if (bufferRawReadable) {

			cout << "thread " << thread_id << ": Start saving image number " << imageNum << endl;

			int cameraIndex = 0;

			// Write image file
			ostringstream s1;
			// Create image name files with ascending grabbed image numbers.
			s1 << imageFolder << "/img_" << cameraIndex << "_" << imageNum	<< ".raw";
			string imageName(s1.str());

			ostringstream s2;
			// Create image name files with ascending grabbed image numbers.
			s2 << imageFolder << "/img_" << cameraIndex << "_" << imageNum << ".txt";
			string imageCfgName(s2.str());

			// Save the raw image into file
			ofstream myFile (imageName, ios::out | ios::binary);
			if (myFile.is_open()) {
				myFile.write ((char*)rawImageBuffer,imageBufferSize);
				myFile.close();
			} else {
				cout << "Error writing image file " << imageName << endl;
				terminateLoop = true;
			}

			if (!terminateLoop) {
				// Write camera parameters
				ofstream myFile2(imageCfgName);
				if (myFile2.is_open()) {
					myFile2 << "Camera Index: "
							<< (imageData[cameraIndex]).cameraIdx << "\n";
					time_t my_time = (imageData[cameraIndex]).captureTime;
					myFile2 << "Capture Time: " << ctime(&my_time);
					myFile2 << "Exposure Time: "
							<< (imageData[cameraIndex]).exposureTime << "\n";
					myFile2 << "Gain: " << (imageData[cameraIndex]).gain
							<< "\n";
					myFile2 << "Balance Red  : "
							<< (imageData[cameraIndex]).balanceR << "\n";
					myFile2 << "Balance Green: "
							<< (imageData[cameraIndex]).balanceG << "\n";
					myFile2 << "Balance Blue : "
							<< (imageData[cameraIndex]).balanceB << "\n";
					myFile2 << "Auto Exposure Time Continuous: "
							<< (imageData[cameraIndex]).autoExpTime << "\n";
					myFile2 << "Auto Gain Continuous: "
							<< (imageData[cameraIndex]).autoGain << "\n";
					myFile2.close();

					cout << "thread " << thread_id << ": Finished saving from buffer." << endl;
					imageNum++;

				} else {
					cout << "Error writing image file " << imageCfgName << endl;
					terminateLoop = true;
				}
			}

		}

		cout << "thread " << thread_id << ": Barrier 1" << endl;
		barrier_2 = 1;
		pthread_barrier_wait(&thBarrier);
		cout << "thread " << thread_id << ": Barrier 2" << endl;
		barrier_2 = 2;
		pthread_barrier_wait(&thBarrier);
	}


	pthread_exit(NULL);
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
	namedWindow("Histogram", CV_WINDOW_AUTOSIZE);
	imshow("Histogram", histImage);

	waitKey(1);

	return med;
}

