#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "VzenseNebula_api.h"
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

#include <Eigen/Dense>
#include <cstring>

using namespace std;
using namespace cv;

VzDeviceInfo* g_pDeviceListInfo = NULL;
VzDeviceHandle g_DeviceHandle = 0;
Point g_Pos(-1, -1);

Point g_TransPos(800, 600);
int g_Slope = 7495;					//dpeth color_Max

bool g_IsSavePointCloud = false;
static bool isTransformColorImgToDepthSensorEnabled = true;

VzVector3f* g_worldv = NULL;

uint16_t g_frameWith = 0;
uint16_t g_frameHeight = 0;

VzVector3f* g_transDepthv = NULL;
uint16_t g_transDepthWith = 0;

struct CallbackData
{
	ros::Publisher pub;
};

struct cameraExtrinsic{
	double rotation[9];
	double translation[3];
};
cameraExtrinsic ce;

cv::Point2d rgb_point;
image_geometry::PinholeCameraModel cam_model;


ros::Publisher pub;
ros::Publisher pub_body;



bool InitDevice(const int deviceCount);
// void ShowMenu();
static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg, Point point);
void HotPlugStateCallback(const VzDeviceInfo* pInfo, int state, void* pUserData);
void SavePointCloud(const char* pFileName, const VzFrame& depthFrame);

void PrintPointCloudInfo(int x , int y , int width , ros::Publisher &pub);
void PrintPointCloudInfo(cv::Point2d  ,int width,ros::Publisher &pub);

void printMatrix(const cv::Mat& mat) ;

void on_MouseHandle(int event, int x, int y, int flags, void * userdata )
{
	CallbackData* data = static_cast<CallbackData*>(userdata);
	if (EVENT_LBUTTONDOWN == event)
	{
		g_Pos.x = x;
		g_Pos.y = y;		
		PrintPointCloudInfo(x,y,g_frameWith,data->pub);

	}
}

void PrintPointCloudInfo(cv::Point2d rgb_point ,int width,ros::Publisher &pub_body)
{
	if (g_transDepthv != NULL)
    {
        int index = rgb_point.y * width + rgb_point.x;

		std::cout << "index" << index << std::endl;

		std::stringstream ss;
		ss << g_transDepthv[index].x << " "<< g_transDepthv[index].y <<" "<< g_transDepthv[index].z ;

		std_msgs::String msg;
		msg.data = ss.str();

		pub.publish(msg);

        /* std::cout << "Point Cloud at (" << rgb_point.x << ", " << rgb_point.y << "): ("
             << g_transDepthv[index].x << ", " 
             << g_transDepthv[index].y << ", " 
             << g_transDepthv[index].z << ")" << endl; */
	}
}


// Add a function to print point cloud information of the clicked point
void PrintPointCloudInfo(int x, int y, int width, ros::Publisher &pub)
{
    if (g_worldv != NULL)
    {
        int index = y * width + x;

		std::stringstream ss;
		ss << g_worldv[index].x << " "<< g_worldv[index].y <<" "<< g_worldv[index].z ;

		std_msgs::String msg;
		msg.data = ss.str();

		pub.publish(msg);

        /* std::cout << "Point Cloud at (" << x << ", " << y << "): ("
             << g_worldv[index].x << ", " 
             << g_worldv[index].y << ", " 
             << g_worldv[index].z << ")" << endl; */
    }
    else
    {
        std::cout << "Point Cloud data is not available." << endl;
    }
}


void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg )
{

	cv::Mat trs(3,1,CV_64F,ce.translation);
	cv::Mat rot(3,3,CV_64F,ce.rotation);
	cv::Mat rgb_point_mat = (cv::Mat_<double>(3,1) << msg->point.x,msg->point.y,0);

	printMatrix(trs);	
	printMatrix(rot);
	
	std::cout << rgb_point_mat << endl;
	
	cv::Mat depth_point = (rgb_point_mat * rot) + trs;

	std::cout << depth_point << endl;

	rgb_point.x = depth_point.at<double>(0,0);

	rgb_point.y = depth_point.at<double>(1,0);


	PrintPointCloudInfo(rgb_point,g_transDepthWith,pub_body);

}

void printMatrix(const cv::Mat& mat) {
    std::cout << "Matrix:" << std::endl;
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            std::cout << mat.at<double>(i, j) << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char *argv[])
{
	// initialize the ROS system
	ros::init(argc,argv,"point_cloud_publisher");
	ros::NodeHandle nh;
	pub = nh.advertise<std_msgs::String>("camera_coord_pos",1000);
	pub_body = nh.advertise<std_msgs::String>("Car_BoC",1000);
	image_transport::ImageTransport trc(nh);
	image_transport::Publisher pub_color = trc.advertise("camera/color_image", 1);
	
	ros::Subscriber clicked_point_sub = nh.subscribe("clicked_point",1000, &clickedPointCallback);

	CallbackData data;
	data.pub = pub;

	uint32_t deviceCount = 0;
	
    VzReturnStatus status = VZ_Initialize();
	if (status != VzReturnStatus::VzRetOK)
	{
		std::cout << "VzInitialize failed!" << endl;
		system("pause");
		return -1;
	}


GET:
	status = VZ_GetDeviceCount(&deviceCount);
	if (status != VzReturnStatus::VzRetOK)
	{
		std::cout << "VzGetDeviceCount failed!" << endl;
		system("pause");
		return -1;
	}
	std::cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}

	InitDevice(deviceCount);


	VzSensorExtrinsicParameters camera_exchange_depth_Parameters;
	status = VZ_GetSensorExtrinsicParameters(g_DeviceHandle, &camera_exchange_depth_Parameters);
	
	memcpy(ce.rotation,camera_exchange_depth_Parameters.rotation,9 * sizeof(double));
	
	memcpy(ce.translation,camera_exchange_depth_Parameters.translation,3 * sizeof(double));

	// VzReturnStatus VZ_GetExposureTime(VzDeviceHandle g_DeviceHandle,VzSensorType VzDepthFrame,VzExposureTimeParams* pExposureTime);
	
	// cout << "Get VZ_GetSensorIntrinsicParameters status: " << status << endl;
	// cout << "ToF Sensor Intinsic: " << endl;
	// cout << "Fx: " << cameraParameters.fx << endl;
	// cout << "Cx: " << cameraParameters.cx << endl;
	// cout << "Fy: " << cameraParameters.fy << endl;
	// cout << "Cy: " << cameraParameters.cy << endl;
	// cout << "ToF Sensor Distortion Coefficient: " << endl;
	// cout << "K1: " << cameraParameters.k1 << endl;
	// cout << "K2: " << cameraParameters.k2 << endl;
	// cout << "P1: " << cameraParameters.p1 << endl;
	// cout << "P2: " << cameraParameters.p2 << endl;
	// cout << "K3: " << cameraParameters.k3 << endl;
	// cout << "K4: " << cameraParameters.k4 << endl;
	// cout << "K5: " << cameraParameters.k5 << endl;
	// cout << "K6: " << cameraParameters.k6 << endl;

	// ShowMenu();

	cv::Mat imageMat;
	// const string irImageWindow = "IR Image";
	// const string depthImageWindow = "Depth Image";
    const string rgbImageWindow = "RGB Image";
	// const string transformedDepthWindow = "TransformedDepth";
	const string transformedColorwindow = "TransformedColor";

	// cv::namedWindow(depthImageWindow, cv::WINDOW_AUTOSIZE);
	// cv::namedWindow(irImageWindow, cv::WINDOW_AUTOSIZE);
	// cv::namedWindow(rgbImageWindow,cv::WINDOW_AUTOSIZE);
	cv::namedWindow(transformedColorwindow,cv::WINDOW_AUTOSIZE);
	// cv::namedWindow(transformedDepthWindow,cv::WINDOW_AUTOSIZE);

	// setMouseCallback(depthImageWindow, on_MouseHandle, &data);
	// setMouseCallback(irImageWindow, on_MouseHandle, &data);
	setMouseCallback(rgbImageWindow, on_MouseHandle, &data);
	setMouseCallback(transformedColorwindow,on_MouseHandle, &data);
	// setMouseCallback(transformedDepthWindow,on_MouseHandle, &data);


	bool isTransformedDepthPointToColorPointEnable = false;
	VzVector2u16 rgbFrameWH = {640,480};
	VZ_SetColorResolution(g_DeviceHandle, 640, 640);

	VZ_SetTransformColorImgToDepthSensorEnabled(g_DeviceHandle, isTransformColorImgToDepthSensorEnabled);

	static bool isTransformDepthImgToColorSensorEnabled = true;			
	VZ_SetTransformDepthImgToColorSensorEnabled(g_DeviceHandle, isTransformDepthImgToColorSensorEnabled);

	for (;;)
	{
		VzFrame depthFrame = { 0 };
		VzFrame irFrame = { 0 };
		VzFrame rgbFrame = { 0 };
		VzFrame transformedDepthFrame = { 0 };
		VzFrame transformedRgbFrame = { 0 };
		VzDepthVector3 TransformedDepthDepthVector[4] = {};

		// Read one frame before call VzGetFrame
		VzFrameReady frameReady = {0};
		status = VZ_GetFrameReady(g_DeviceHandle, 1200, &frameReady);
		
		//Get depth frame, depth frame only output in following data mode
		if (1 == frameReady.depth)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzDepthFrame, &depthFrame);

			if (depthFrame.pFrameData != NULL)
			{
				g_frameWith = depthFrame.width;
				g_frameHeight = depthFrame.height;

				if (g_worldv == NULL)
				{
					g_worldv = new VzVector3f[g_frameWith * g_frameHeight];
				}
				
				// realtime to updata pointcloud data
				VZ_ConvertDepthFrameToPointCloudVector(g_DeviceHandle, &depthFrame, g_worldv);

				// save the pointcloud data if need
				if (true == g_IsSavePointCloud)
				{
					g_IsSavePointCloud = false;
					SavePointCloud("pointCloud.txt",depthFrame);
				}

                static int index = 0;
                static float fps = 0;
                static int64 start = cv::getTickCount();

                int64 current = cv::getTickCount();
                int64 diff = current - start;
                index++;
                if (diff > cv::getTickFrequency())
                {
                    fps = index * cv::getTickFrequency() / diff;
                    index = 0;
                    start = current;
                }

				

				//Display the Depth Image
				Opencv_Depth(g_Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat, g_Pos);
                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));


				// cv::imshow(depthImageWindow, imageMat);
			}
			else
			{
				std::cout << "VZ_GetFrame VzDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		// if (1 == frameReady.transformedDepth)
		// {
		// 	status = VZ_GetFrame(g_DeviceHandle, VzTransformDepthImgToColorSensorFrame, &transformedDepthFrame);

		// 	if (transformedDepthFrame.pFrameData != NULL)
		// 	{

		// 		static int index = 0;
		// 		static float fps = 0;
		// 		static int64 start = cv::getTickCount();

		// 		int64 current = cv::getTickCount();
		// 		int64 diff = current - start;
		// 		index++;
		// 		if (diff > cv::getTickFrequency())
		// 		{
		// 			fps = index * cv::getTickFrequency() / diff;
		// 			index = 0;
		// 			start = current;
		// 		}

		// 		if (g_transDepthv == NULL)
		// 		{
		// 			g_transDepthv = new VzVector3f[transformedDepthFrame.width * transformedDepthFrame.height];
		// 		}

		// 		VZ_ConvertDepthFrameToPointCloudVector(g_DeviceHandle, &transformedDepthFrame, g_worldv);

		// 		g_transDepthWith = transformedDepthFrame.width;

		// 		//Display the Depth Image
		// 		// Opencv_Depth(g_Slope, transformedDepthFrame.height, transformedDepthFrame.width, transformedDepthFrame.pFrameData, imageMat,g_TransPos);
		// 		imageMat = cv::Mat(transformedDepthFrame.height, transformedDepthFrame.width, CV_8UC3, transformedDepthFrame.pFrameData);
		// 		// char text[30] = "";
		// 		// sprintf(text, "%.2f", fps);
		// 		// putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
		// 		cv::imshow(transformedDepthWindow, imageMat);				
		// 	}
		// 	else
		// 	{
		// 		std::cout << "VZ_GetFrame VzDepthFrame status:" << status << " pFrameData is NULL " << endl;
		// 	}
		// }
		
        //Get RGB frame, RGB frame only output in following data mode
        if (1 == frameReady.color)
        {
            status = VZ_GetFrame(g_DeviceHandle, VzColorFrame, &rgbFrame);

            if (rgbFrame.pFrameData != NULL)
            {

                static int index = 0;
                static float fps = 0;
                static int64 start = cv::getTickCount();

                int64 current = cv::getTickCount();
                int64 diff = current - start;
                index++; 
                if (diff > cv::getTickFrequency())
                {
                    fps = index * cv::getTickFrequency() / diff;
                    index = 0;
                    start = current;
                }
				

                //Display the RGB Image
                imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
				
				rgbFrameWH.x = rgbFrame.width;
				rgbFrameWH.y = rgbFrame.height;
                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));


				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",imageMat).toImageMsg();

				pub_color.publish(msg);


				/* cv::imshow(rgbImageWindow, imageMat); */
            }
            else
            {
                std::cout << "VZ_GetFrame VzRGBFrame status:" << status << " pFrameData is NULL " << endl;
            }
        }
		if (1 == frameReady.transformedColor)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzTransformColorImgToDepthSensorFrame, &transformedRgbFrame);

			if (transformedRgbFrame.pFrameData != NULL)
			{

				static int index = 0;
				static float fps = 0;
				static int64 start = cv::getTickCount();

				int64 current = cv::getTickCount();
				int64 diff = current - start;
				index++;
				if (diff > cv::getTickFrequency())
				{
					fps = index * cv::getTickFrequency() / diff;
					index = 0;
					start = current;
				}

				//Display the RGB Image
				imageMat = cv::Mat(transformedRgbFrame.height, transformedRgbFrame.width, CV_8UC3, transformedRgbFrame.pFrameData);
				circle(imageMat, g_Pos, 2, Scalar(0, 0, 0), -1, 4, 0);
				char text[30] = "";
				sprintf(text, "%.2f", fps);
				putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

				cv::imshow(transformedColorwindow, imageMat);
			}
			else
			{
				std::cout << "VZ_GetFrame VzRGBFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		ros::spinOnce();

		unsigned char key = waitKey(1);
        if (key == 'R' || key == 'r')
        {
            std::cout << "please select RGB resolution to set: 0:640*480; 1:800*600; 2:1600*1200" << endl;
            int index = 0;
            cin >> index;
            if (cin.fail())
            {
                std::cout << "Unexpected input"<< endl;
                cin.clear();
                cin.ignore(1024, '\n');
                continue;
            }
            else
            {
                cin.clear();
                cin.ignore(1024, '\n');
            }

            switch (index)
            {
            case 0:
				g_TransPos.x = g_TransPos.x *640 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y* 480 /rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 640, 480);
                break;
            case 1:
				g_TransPos.x = g_TransPos.x * 800 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y * 600 / rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 800, 600);
                break;
            case 2:
				g_TransPos.x = g_TransPos.x * 1600 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y * 1200 / rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 1600, 1200);
                break;
            default:
                std::cout << "input is invalid." << endl;
                break;
            }
            
        }

        else if (key == 'Q' || key == 'q')
        {
			
			VZ_SetTransformColorImgToDepthSensorEnabled(g_DeviceHandle, isTransformColorImgToDepthSensorEnabled);
			std::cout << "SetTransformColorImgToDepthSensorEnabled " << ((true == isTransformColorImgToDepthSensorEnabled) ? "enable" : "disable") << endl;
			isTransformColorImgToDepthSensorEnabled = !isTransformColorImgToDepthSensorEnabled;
        }
        
		else if (key == 27)	//ESC Pressed
		{
			break;
		}
	}

	status = VZ_StopStream(g_DeviceHandle);
    std::cout << "VZ_StopStream status: " << status << endl;

    status = VZ_CloseDevice(&g_DeviceHandle);
    std::cout << "CloseDevice status: " << status << endl;

	if (g_worldv != NULL)
	{
		delete[] g_worldv;
		g_worldv = NULL;
	}
	
    status = VZ_Shutdown();
    std::cout << "Shutdown status: " << status << endl;
	cv::destroyAllWindows();

	delete[] g_pDeviceListInfo;
	g_pDeviceListInfo = NULL;

    return 0;
}

bool InitDevice(const int deviceCount)
{
	VZ_SetHotPlugStatusCallback(HotPlugStateCallback, nullptr);

	g_pDeviceListInfo = new VzDeviceInfo[deviceCount];
    VzReturnStatus status = VZ_GetDeviceInfoList(deviceCount, g_pDeviceListInfo);
	g_DeviceHandle = 0;
	status = VZ_OpenDeviceByUri(g_pDeviceListInfo[0].uri, &g_DeviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		std::cout << "OpenDevice failed!" << endl;
		system("pause");
		return false;
	}

	VzConfidenceFilterParams confidenceFilterParams = { 0, true };
	status = VZ_GetConfidenceFilterParams(g_DeviceHandle, &confidenceFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		std::cout << "VZ_GetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}

	status = VZ_SetConfidenceFilterParams(g_DeviceHandle, confidenceFilterParams);

	if (status != VzReturnStatus::VzRetOK)
	{
		std::cout << "VZ_SetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}

    // cout << "sn  ==  " << g_pDeviceListInfo[0].serialNumber << endl;


    const int BufLen = 64;
	char fw[BufLen] = { 0 };
	VZ_GetFirmwareVersion(g_DeviceHandle, fw, BufLen);
	// cout << "fw  ==  " << fw << endl;

    VZ_StartStream(g_DeviceHandle);
    //Wait for the device to upload image data
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

 	return true;
}

// void ShowMenu()
// {
// 	cout << "\n--------------------------------------------------------------------" << endl;
// 	cout << "--------------------------------------------------------------------" << endl;
// 	cout << "Press following key to set corresponding feature:" << endl;
//     cout << "R/r: Change the RGB resolution: input corresponding index in terminal:" << endl;
//     cout << "                             0: 640 * 480" << endl;
//     cout << "                             1: 800 * 600" << endl;
//     cout << "                             2: 1600 * 1200" << endl;
// 	cout << "P/p: Save point cloud data into PointCloud.txt in current directory" << endl;
//     cout << "Q/q: Enables or disables transforms a color image into the geometry of the depth camera" << endl;
//     cout << "L/l: Enables or disables transforms the depth map into the geometry of the color camera" << endl;
//     cout << "T/t: Enables or disables transforms the depth point into the geometry of the color camera" << endl;
// 	cout << "Esc: Program quit " << endl;
// 	cout << "--------------------------------------------------------------------" << endl;
// 	cout << "--------------------------------------------------------------------\n" << endl;
// }

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg,Point point)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy = point;	
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 3, Scalar(color, color, color), -1, 4, 0);
	putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}


void HotPlugStateCallback(const VzDeviceInfo* pInfo, int state, void* pUserData)
{
	std::cout << pInfo->uri<<" " << (state ==0? "add":"remove" )<<endl ;
}

void SavePointCloud(const char* pFileName, const VzFrame& depthFrame)
{
	// if (depthFrame.pFrameData != NULL && pFileName != NULL)
	// {
		ofstream PointCloudWriter;
		PointCloudWriter.open(pFileName);
		const int len = depthFrame.width * depthFrame.height;

		// //VzVector3f *worldV = new VzVector3f[len];
		// if (g_worldv == NULL)
		// {
		// 	g_worldv = new VzVector3f[len];
		// }

		// if (VZ_ConvertDepthFrameToPointCloudVector(g_DeviceHandle, &depthFrame, g_worldv)) // Convert Depth frame to World vectors.
		// {
		// if (g_worldv != NULL)
		// {
		for (int i = 0; i < len; i++)
		{
			if (100 < g_worldv[i].z && g_worldv[i].z < g_Slope)
			{
				PointCloudWriter << g_worldv[i].x << "\t" << g_worldv[i].y << "\t" << g_worldv[i].z << std::endl;
			}
		}

		PointCloudWriter.close();

}
