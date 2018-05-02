// ResetCamera.cpp
/*
   This program resets the camera.
*/

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


// Limits the amount of cameras used for grabbing.
// It is important to manage the available bandwidth when grabbing with multiple
// cameras. This applies, for instance, if two GigE cameras are connected to the
// same network adapter via a switch. To manage the bandwidth, the GevSCPD
// interpacket delay parameter and the GevSCFTD transmission delay parameter can
// be set for each GigE camera device. The "Controlling Packet Transmission Timing
// with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
// Application Note (AW000649xx000) provides more information about this topic.
static const uint32_t c_maxCamerasToUse = 2;

int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

	// Declare an integer variable to count the number of grabbed images
	// and create image file names with ascending number.
	int grabbedImages = 0;

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
			cameras[i].GainAuto.SetValue(GainAuto_Off);
			cameras[i].ExposureAuto.SetValue(ExposureAuto_Off);
			cameras[i].PixelFormat.SetValue(PixelFormat_BayerRG8);
			cout << "Reseting the camera ..." << endl;
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

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while (cin.get() != '\n');

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}
