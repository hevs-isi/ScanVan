#include "Camera.hpp"

// Settings to use Basler GigE cameras.
using namespace Basler_GigECameraParams;
// Namespace for using pylon objects.
using namespace Pylon;
// Namespace for using cout.
using namespace std;

Camera::Camera() {
	// It will not load the configuration file to the camera
	loadParam = false;
	Init();
}

Camera::Camera(std::string path_to_config_files): config_path { path_to_config_files } {
	// It will load the configuration file to the cameras
	// Files are located under the folder path_to_config_files
	// The names of the files are the serial number of the camera .pfs
	loadParam = true;
	Init();
}

void Camera::Init() {

	CTlFactory& tlFactory = CTlFactory::GetInstance();
	pTL = dynamic_cast<IGigETransportLayer*>(tlFactory.CreateTl(BaslerGigEDeviceClass));
	if (pTL == nullptr) {
		throw RUNTIME_EXCEPTION("No GigE transport layer available.");
	}

	// In this sample we use the transport layer directly to enumerate cameras.
	// By calling EnumerateDevices on the TL we get get only GigE cameras.
	// You could also accomplish this by using a filter and
	// let the Transport Layer Factory enumerate.
	DeviceInfoList_t allDeviceInfos { };
	if (pTL->EnumerateDevices(allDeviceInfos) == 0) {
		throw RUNTIME_EXCEPTION("No GigE cameras present.");
	}

	// Only use cameras in the same subnet as the first one.
	DeviceInfoList_t usableDeviceInfos { };
	usableDeviceInfos.push_back(allDeviceInfos[0]);
	subnet = static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[0]).GetSubnetAddress();

	// Start with index 1 as we have already added the first one above.
	// We will also limit the number of cameras to c_maxCamerasToUse.
	for (size_t i = 1; i < allDeviceInfos.size() && usableDeviceInfos.size() < c_maxCamerasToUse; ++i) {
		const CBaslerGigEDeviceInfo& gigeinfo = static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[i]);
		if (subnet == gigeinfo.GetSubnetAddress()) {
			// Add this deviceInfo to the ones we will be using.
			usableDeviceInfos.push_back(gigeinfo);
		} else {
			cerr << "Camera will not be used because it is in a different subnet " << subnet << "!" << endl;
		}
	}

	cameras.Initialize(usableDeviceInfos.size());

	// Seed the random number generator and generate a random device key value.
	srand((unsigned) time(nullptr));
	DeviceKey = rand();

	// For the following sample we use the CActionTriggerConfiguration to configure the camera.
	// It will set the DeviceKey, GroupKey and GroupMask features. It will also
	// configure the camera FrameTrigger and set the TriggerSource to the action command.
	// You can look at the implementation of CActionTriggerConfiguration in <pylon/gige/ActionTriggerConfiguration.h>
	// to see which features are set.

	// vector of serial number to sort and order the camera idx
	struct SerialNumIdx {
		String_t number;
		size_t index;
	};
	std::vector<SerialNumIdx> sn{};

	// Create all GigE cameras and attach them to the InstantCameras in the array.
	for (size_t i = 0; i < cameras.GetSize(); ++i) {
		cameras[i].Attach(tlFactory.CreateDevice(usableDeviceInfos[i]));
		// We'll use the CActionTriggerConfiguration, which will set up the cameras to wait for an action command.
		cameras[i].RegisterConfiguration(new CActionTriggerConfiguration { DeviceKey, GroupKey, AllGroupMask }, RegistrationMode_Append,
				Cleanup_Delete);
		// Set the context. This will help us later to correlate the grab result to a camera in the array.
		cameras[i].SetCameraContext(i);

		const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();

		// Print the model name of the camera.
		cout << "Using camera " << i << ": " << di.GetModelName() << " (" << di.GetIpAddress() << ")" << " - (SN:" << di.GetSerialNumber() << ")" << endl;

		SerialNumIdx elem {di.GetSerialNumber(), i};
		// push the serial numbers and the idx positions into vectors
		sn.push_back(elem);
	}

	// Sort the serial numbers in increasing value
	std::sort(sn.begin(), sn.end(), [](const auto &e1, const auto &e2) {
		return e1.number < e2.number;
	});

	// Store the indices of the camera corresponding to the increasing value of serial numbers
	for (const auto &x : sn) {
		sortedCameraIdx.push_back(x.index);
	}

	// Open all cameras.
	// This will apply the CActionTriggerConfiguration specified above.
	cameras.Open();

	// Reads the camera parameters from file
	if (loadParam) {
		try {
			LoadParameters();
		} catch (const GenericException &e) {
			// Error handling
			cerr << "Error loading the parameters of the camera." << std::endl;
			cerr <<  e.GetDescription() << endl;
		}
	}

	for (size_t i = 0; i < cameras.GetSize(); ++i) {
		// This sets the transfer pixel format to BayerRG8
		cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG8);

		cameras[i].GevSCPSPacketSize.SetValue(8192);
		cameras[i].GevSCPD.SetValue(50);
		cameras[i].GevSCFTD.SetValue(50);
		cameras[i].GevSCBWRA.SetValue(cameras[i].GevSCBWRA.GetMax());

		cameras[i].GainAuto.SetValue(GainAuto_Off);
		cameras[i].ExposureAuto.SetValue(ExposureAuto_Off);

		cameras[i].ExposureTimeAbs.SetValue(exposureTime);
		cameras[i].GainRaw.SetValue(gain);

		cameras[i].Width.SetValue(100);
		cameras[i].Height.SetValue(100);

		if (IsWritable(cameras[i].OffsetX)) {
			cameras[i].OffsetX.SetValue(offsetX);
		}
		if (IsWritable(cameras[i].OffsetY)) {
			cameras[i].OffsetY.SetValue(offsetY);
		}

		cameras[i].Width.SetValue(width);
		cameras[i].Height.SetValue(height);

		cameras[i].AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
		cameras[i].AutoFunctionAOIWidth.SetValue(aoi_width);
		cameras[i].AutoFunctionAOIHeight.SetValue(aoi_height);
		cameras[i].AutoFunctionAOIOffsetX.SetValue(aoi_offsetX);
		cameras[i].AutoFunctionAOIOffsetY.SetValue(aoi_offsetY);

		cameras[i].AutoTargetValue.SetValue(autoTargetVal);

		// Sets auto adjustments continuous
		if (autoExpTimeCont)
			cameras[i].ExposureAuto.SetValue(ExposureAuto_Continuous);
		if (autoGainCont)
			cameras[i].GainAuto.SetValue(GainAuto_Continuous);
	}
}


void Camera::GrabImages() {
	//////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////
	// Use an Action Command to Trigger Multiple Cameras at the Same Time.
	//////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////

	cout << endl << "Issuing an action command." << endl;

	// Starts grabbing for all cameras.
	// The cameras won't transmit any image data, because they are configured to wait for an action command.
	cameras.StartGrabbing();

	// Now we issue the action command to all devices in the subnet.
	// The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
	pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

	// This smart pointer will receive the grab result data.
	CBaslerGigEGrabResultPtr ptrGrabResult {};

	// Retrieve images from all cameras.
	const int DefaultTimeout_ms  { 5000 };

	for (size_t i = 0; i < cameras.GetSize() && cameras.IsGrabbing(); ++i) {
		// CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
		cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult, TimeoutHandling_ThrowException);

		// When the cameras in the array are created the camera context value
		// is set to the index of the camera in the array.
		// The camera context is a user-settable value.
		// This value is attached to each grab result and can be used
		// to determine the camera that produced the grab result.
		intptr_t cameraIndex = ptrGrabResult->GetCameraContext();

		// Image grabbed successfully?
		if (ptrGrabResult->GrabSucceeded()) {
			// Print the index and the model name of the camera.
			cout << "Camera " << sortedCameraIdx[cameraIndex] << ": " << cameras[cameraIndex].GetDeviceInfo().GetModelName() << " ("
					<< cameras[cameraIndex].GetDeviceInfo().GetIpAddress() << ") (SN:" <<
					cameras[cameraIndex].GetDeviceInfo().GetSerialNumber() << ")"
					<< endl;
			// You could process the image here by accessing the image buffer.
			cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;
			uint8_t *pImageBuffer = static_cast<uint8_t *> (ptrGrabResult->GetBuffer());

			// Create an Image object with the grabbed data
			Images img1 {height, width, reinterpret_cast<char *> (pImageBuffer)};
			img1.show(to_string(sortedCameraIdx[cameraIndex]));
			cv::waitKey(1);


			cout << "Gray value of first pixel: " << static_cast<uint32_t> (pImageBuffer[0]) << endl << endl;
		} else {
			// If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
			// multiple images simultaneously. See note above c_maxCamerasToUse.
			cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
		}
	}

	// In case you want to trigger again you should wait for the camera
	// to become trigger-ready before issuing the next action command.
	// To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
	// (see Grab_UsingGrabLoopThread sample for details).

	cameras.StopGrabbing();
}

void Camera::SaveParameters(){

	std::vector<String_t> sn{};

	for (size_t i = 0; i < cameras.GetSize(); ++i) {
		sn.push_back(cameras[i].GetDeviceInfo().GetSerialNumber());
		std::stringstream ss{};
		ss << sn[i].c_str();
		ss << ".pfs";
		std::string filename = config_path + "/" + ss.str();

		//const char Filename[name.size()+1] { name.c_str() };
		cout << "Saving camera's node map to file..." << endl;
		// Save the content of the camera's node map into the file.
		CFeaturePersistence::Save(filename.c_str(), &cameras[i].GetNodeMap());
	}

}

void Camera::LoadParameters(){

	std::vector<String_t> sn{};

	for (size_t i = 0; i < cameras.GetSize(); ++i) {
		sn.push_back(cameras[i].GetDeviceInfo().GetSerialNumber());
		std::stringstream ss { };
		ss << sn[i].c_str();
		ss << ".pfs";
		std::string filename = config_path + "/" + ss.str();

	    std::cout << "Reading file back to camera's node map for camera with SN:"<< cameras[i].GetDeviceInfo().GetSerialNumber() << " ..." << std::endl;
	    CFeaturePersistence::Load(filename.c_str(), &cameras[i].GetNodeMap(), true );
	}
}

size_t Camera::GetNumCam() {
	return cameras.GetSize();
}

Camera::~Camera() {


	for (size_t i = 0; i < cameras.GetSize(); ++i) {
		cameras[i].DeviceReset();
	}


	// Close all cameras.
	cameras.Close();
}

