#include "smartrunner_node.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>

#include <cstdlib>
#include <ctime>
#include <string>
#include <iostream>
#include <fstream>

namespace pepperl_fuchs {

   //Common Parameters
    std::string frame_id_               ="";
    std::string device_ip_              ="";
    std::string message_type_           ="";
    const char* IP                      = NULL;
    bool retValue                       = false;
    std::string sensor                  = ""; 
    const char* sensor_type             = NULL;
    time_t now;
    //Parameters for Sensor Data
    int linePitch                       = 0;
    long frameCounter                   = 0;
    double coordinateScale              = 0.0;
    double coordinateOffset             = 0.0;
    double axisMin                      = 0.0;
    double axisMax                      = 0.0;
    double invalidDataValue             = 0.0;
    int rowIndex                        = 0;
    int colIndex                        = 0;
    int index                           = 0;
    int width                           = 0;
    int height                          = 0;
   //Parameters for Vsx-Interface
    VsxStatusCode status;
    VsxSystemHandle* ptr_vsx            = NULL;
    VsxDataContainerHandle* dch         = NULL;
    VsxDevice* deviceData               = NULL;
    VsxParameterList* deviceParameters  = NULL;
    VsxImage* imageA                    = NULL;
    VsxImage* imageB                    = NULL;
    VsxImage* imageC                    = NULL;
    VsxLineData* lineData	            = NULL;
    VsxLineCoordinate* lineCoordinates  = NULL; 
    VsxImageData2Format format;
    //Parameters for SmartRunner(3D TOF)
    std::string tof_trigger_source_             ="";
    int tof_auto_trigger_rate_                  = 0;
    int tof_trigger_enable_                     = 0;
    int tof_exposure_time_                      = 0;
    int tof_range_mode_                         = 0;
    std::string tof_output_mode_                ="";
    //Parameters for SmartRunner(3D Stereo)
    std::string stereo_trigger_source_          ="";
    int stereo_auto_trigger_rate_               = 0;
    int stereo_trigger_enable_                  = 0;
    int stereo_exposure_time_                   = 0;
    int stereo_gain_                            = 0;
    int stereo_uniqueness_                      = 0;
    std::string stereo_output_mode_             ="";
    //Parameters for SmartRunner(Explorer)
    int smartrunner_autotrigger_enable_         = 0;
    int smartrunner_exposure_time_              = 0;
    int smartrunner_use_manual_exposure_time_   = 0;
    int smartrunner_flash_time_                 = 0;
    int smartrunner_object_contrast_            = 0;	
    int smartrunner_roi_min_x_                  = 0;	
    int smartrunner_roi_max_x_                  = 0;	
    int smartrunner_roi_min_z_                  = 0;	
    int smartrunner_roi_max_z_                  = 0;	
    int smartrunner_image_transfer_active_      = 0;
    
//-----------------------------------------------------------------------------
smartrunner_node::smartrunner_node():nh_("~")
{
    // Reading and checking parameters from launch-file
    nh_.param("frame_id", frame_id_, std::string(""));
    nh_.param("device_ip",device_ip_,std::string(""));
    nh_.param("message_type",message_type_,std::string(""));

    nh_.getParam("tof_trigger_source", tof_trigger_source_); 
    nh_.getParam("tof_output_mode", tof_output_mode_);
    nh_.getParam("tof_auto_trigger_rate", tof_auto_trigger_rate_);
    nh_.getParam("tof_trigger_enable", tof_trigger_enable_);
    nh_.getParam("tof_exposure_time", tof_exposure_time_);
    nh_.getParam("tof_range_mode", tof_range_mode_);
  
    nh_.getParam("stereo_trigger_source", stereo_trigger_source_); 
    nh_.getParam("stereo_output_mode", stereo_output_mode_);
    nh_.getParam("stereo_auto_trigger_rate", stereo_auto_trigger_rate_);
    nh_.getParam("stereo_trigger_enable", stereo_trigger_enable_);
    nh_.getParam("stereo_exposure_time", stereo_exposure_time_);
    nh_.getParam("stereo_gain", stereo_gain_);
    nh_.getParam("stereo_uniqueness", stereo_uniqueness_);
 
    nh_.getParam("smartrunner_autotrigger_enable", smartrunner_autotrigger_enable_);
    nh_.getParam("smartrunner_exposure_time", smartrunner_exposure_time_);
    nh_.getParam("smartrunner_use_manual_exposure_time", smartrunner_use_manual_exposure_time_);
    nh_.getParam("smartrunner_flash_time", smartrunner_flash_time_);
    nh_.getParam("smartrunner_object_contrast", smartrunner_object_contrast_);
    nh_.getParam("smartrunner_roi_min_x", smartrunner_roi_min_x_);
    nh_.getParam("smartrunner_roi_max_x", smartrunner_roi_max_x_);
    nh_.getParam("smartrunner_roi_min_z", smartrunner_roi_min_z_);
    nh_.getParam("smartrunner_roi_max_z", smartrunner_roi_max_z_);
    nh_.getParam("smartrunner_image_transfer_active", smartrunner_image_transfer_active_);
 
    printf("ip: %s\n", device_ip_.c_str());
    printf("message type: %s\n", message_type_.c_str());
    printf("id: %s\n", frame_id_.c_str());
    
    printf("tof_trigger_source: %s\n", tof_trigger_source_.c_str());
    printf("tof_auto_trigger_rate: %d\n", tof_auto_trigger_rate_);
    printf("tof_trigger_enable: %d\n", tof_trigger_enable_);
    printf("tof_exposure_time: %d\n", tof_exposure_time_);
    printf("tof_range_mode: %d\n", tof_range_mode_);
    printf("tof_output_mode: %s\n", tof_output_mode_.c_str());

    printf("stereo_trigger_source: %s\n", stereo_trigger_source_.c_str());
    printf("stereo_auto_trigger_rate: %d\n", stereo_auto_trigger_rate_);
    printf("stereo_trigger_enable: %d\n", stereo_trigger_enable_);
    printf("stereo_exposure_time: %d\n", stereo_exposure_time_);
    printf("stereo_gain: %d\n", stereo_gain_);
    printf("stereo_uniqueness: %d\n", stereo_uniqueness_); 
    printf("stereo_output_mode: %s\n", stereo_output_mode_.c_str());

    printf("smartrunner_trigger_enable: %d\n", smartrunner_autotrigger_enable_);
    printf("smartrunner_exposure_time_: %d\n", smartrunner_exposure_time_);
    printf("smartrunner_use_manual_exposure_time_: %d\n", smartrunner_use_manual_exposure_time_);
    printf("smartrunner_flash_time_: %d\n", smartrunner_flash_time_);
    printf("smartrunner_object_contrast_: %d\n", smartrunner_object_contrast_);
    printf("smartrunner_roi_min_x_: %d\n", smartrunner_roi_min_x_);
    printf("smartrunner_roi_max_x_: %d\n", smartrunner_roi_max_x_);
    printf("smartrunner_roi_min_z_: %d\n", smartrunner_roi_min_z_);
    printf("smartrunner_roi_max_z_: %d\n", smartrunner_roi_max_z_);
    printf("smartrunner_image_transfer_active_: %d\n", smartrunner_image_transfer_active_);
 
    if( device_ip_ == "" )
    {
        printf("IP of scanner not set!\n");
        return;
    }
    else
    {
        IP = device_ip_.c_str();
        
        retValue = connect();
        
        if(retValue == true)
        {         
            if(message_type_ == "PointCloud")
            {
                // Declare publisher and create timer
                scan_publisher_         = nh_.advertise<sensor_msgs::PointCloud>("scan",100);
                cmd_subscriber_         = nh_.subscribe("control_command",100,&smartrunner_node::cmdMsgCallback,this);
                get_scan_data_timer_    = nh_.createTimer(ros::Duration(0.1), &smartrunner_node::getScanDataPointCloud, this);
            }
            if(message_type_ == "PointCloud2")
            { 
                // Declare publisher and create timer
                scan_publisher_         = nh_.advertise<sensor_msgs::PointCloud2>("scan",100);
                cmd_subscriber_         = nh_.subscribe("control_command",100,&smartrunner_node::cmdMsgCallback,this);
                get_scan_data_timer_    = nh_.createTimer(ros::Duration(0.1), &smartrunner_node::getScanDataPointCloud2, this);
            }           
        }
    }
}

smartrunner_node::~smartrunner_node()
{
    printf("#######################################################\n");
    status =  vsx_Disconnect(ptr_vsx);
    printf("Disconnect Status: %d\n", status);
    if (status == VSX_STATUS_SUCCESS)
    {      
        printf("disconnected...\n");
        status = vsx_ReleaseSensor(&ptr_vsx);
    }
}

bool smartrunner_node::connect()
{  
    const char* version = NULL;
    const char* PlugIn = "";
   
    printf("#######################################################\n");
    status = vsx_GetLibraryVersion(&version);
    if (status == VSX_STATUS_SUCCESS)
    {      
        printf("Version %s\n", version);
        status = vsx_ReleaseString(&version);
    }
  
    printf("#######################################################\n");
    status = vsx_InitTcpSensor(&ptr_vsx, IP, PlugIn);
    printf("InitTcpSensor Status: %d\n", status);
    if (status == VSX_STATUS_SUCCESS)
    {      
        printf("initialize sensor...\n");
    }

    printf("#######################################################\n");
    status =  vsx_Connect(ptr_vsx);
    printf("Connect Status: %d\n", status);
    if (status == VSX_STATUS_SUCCESS)
    {      
        printf("connected...\n");
    }

    printf("#######################################################\n");
    status =  vsx_GetDeviceInformation(ptr_vsx,&deviceData);
    printf("GetCurrentDeviceInformation Status: %d\n", status);

    if (status == VSX_STATUS_SUCCESS)
    {   
        std::string s = "";   
        printf("Sensor: %s\n",deviceData->sensorType);
        printf("Firmware: %s\n",deviceData->firmwareVersion);
        printf("MAC: %s\n",deviceData->macAddress);
        printf("IP: %s\n",deviceData->ipAddress);

        sensor = std::string(deviceData->sensorType);
        if( sensor == "SR3D_TOF")
        {           
            printf("#######################################################\n");
            printf("Settings for SR3D TOF\n");
        	status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerSource", tof_trigger_source_.c_str());
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            s = std::to_string(tof_auto_trigger_rate_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "AutoTriggerFrameRate", s.c_str());
            printf("SetSingleParameterValue AutoTriggerFrameRate Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "0");
            s = std::to_string(tof_trigger_enable_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", s.c_str());
            printf("SetSingleParameterValue TriggerEnabled Status: %d\n", status);
            s = std::to_string(tof_range_mode_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "RangeMode", s.c_str());
            printf("SetSingleParameterValue RangeMode Status: %d\n", status);
            s = std::to_string(tof_exposure_time_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "ExposureTime", s.c_str());
            printf("SetSingleParameterValue ExposureTime Status: %d\n", status);       
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "OutputMode", tof_output_mode_.c_str());
            printf("SetSingleParameterValue OutputMode Status: %d\n", status);
        }
        if(sensor == "SR3D_STEREO")
        {         
            printf("#######################################################\n");
            printf("Settings for SR3D STEREO\n");
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerSource", stereo_trigger_source_.c_str());
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            s = std::to_string(stereo_auto_trigger_rate_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "AutoTriggerFrameRate", s.c_str());
            printf("SetSingleParameterValue AutoTriggerFrameRate Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "0");
            s = std::to_string(stereo_trigger_enable_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", s.c_str());
            printf("SetSingleParameterValue TriggerEnabled Status: %d\n", status);
            s = std::to_string(stereo_exposure_time_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "ExposureTime", s.c_str());
            printf("SetSingleParameterValue ExposureTime Status: %d\n", status);       
            s = std::to_string(stereo_gain_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "Gain", s.c_str());
            printf("SetSingleParameterValue Gain Status: %d\n", status);       
            s = std::to_string(stereo_uniqueness_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "SGBMUniqueness", s.c_str());
            printf("SetSingleParameterValue Uniqueness Status: %d\n", status);       
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "OutputMode", stereo_output_mode_.c_str());
            printf("SetSingleParameterValue Trigger OutputMode: %d\n", status);
        }
        
        if(sensor == "SMARTRUNNER")
        {         
            printf("#######################################################\n");
            printf("Settings for SMARTRUNNER_2D\n");

            s = std::to_string(smartrunner_autotrigger_enable_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0x510001", s.c_str());
            printf("SetSingleParameterValue AutoTrigger Status: %d\n", status);
            s = std::to_string(smartrunner_exposure_time_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0x680001", s.c_str());
            printf("SetSingleParameterValue ExposureTime Status: %d\n", status);
            s = std::to_string(smartrunner_use_manual_exposure_time_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0xBE0001", s.c_str());
            printf("SetSingleParameterValue UseManualExposureTime Status: %d\n", status);
            s = std::to_string(smartrunner_flash_time_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0x100001", s.c_str());
            printf("SetSingleParameterValue FlashTime Status: %d\n", status);
            s = std::to_string(smartrunner_object_contrast_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0x9F0001", s.c_str());
            printf("SetSingleParameterValue ObjectContrast Status: %d\n", status);
            s = std::to_string(smartrunner_roi_min_x_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0xC90001", s.c_str());
            printf("SetSingleParameterValue ROIMinX Status: %d\n", status);
            s = std::to_string(smartrunner_roi_max_x_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0xC90002", s.c_str());
            printf("SetSingleParameterValue ROIMaxX Status: %d\n", status);
            s = std::to_string(smartrunner_roi_min_z_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0xC90003", s.c_str());
            printf("SetSingleParameterValue ROIMinZ Status: %d\n", status);
            s = std::to_string(smartrunner_roi_max_z_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"0xC90004", s.c_str());
            printf("SetSingleParameterValue ROIMaxZ Status: %d\n", status);    
            s = std::to_string(smartrunner_image_transfer_active_);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "General", 1,"ImageTransferActive", s.c_str());
            printf("SetSingleParameterValue ImageTransferActive Status: %d\n", status);
            
        }       
        status = vsx_ReleaseDevice(&deviceData);
    }

    printf("#######################################################\n");
    status = vsx_ResetDynamicContainerGrabber(ptr_vsx, 1, VSX_STRATEGY_DROP_OLDEST);
    if (status == VSX_STATUS_SUCCESS)
    {
        printf("receiving data...\n");       
    }    
    return true;
}

//-----------------------------------------------------------------------------
void smartrunner_node::getScanDataPointCloud2(const ros::TimerEvent &e)
{    
    sensor_msgs::PointCloud2 scanmsg2;  
    sensor_msgs::PointCloud scanmsg; 
  
    if(sensor == "SMARTRUNNER")
    {   
        status = vsx_GetDataContainer(ptr_vsx, &dch, 10000);
        if(status==VSX_STATUS_SUCCESS)
        {
            status = vsx_GetLine(dch, "Line", &lineData);
            if(status==VSX_STATUS_SUCCESS)
            {           
                scanmsg.header.frame_id = frame_id_;
                scanmsg.header.stamp = ros::Time::now();
                scanmsg.points.resize(lineData->width);

                for( std::size_t i=0; i<lineData->width; i++ )
                {
                    scanmsg.points[i].x = lineData->lines[0][i].x / 1000.0;
                    scanmsg.points[i].y = 0;
                    scanmsg.points[i].z = lineData->lines[0][i].z / 1000.0; 
                }

                status = vsx_ReleaseLine(&lineData);
                status = vsx_ReleaseDataContainer(&dch);
             
                sensor_msgs::convertPointCloudToPointCloud2(scanmsg,scanmsg2);  
                scan_publisher_.publish(scanmsg2);
            }
        }
    }

  if(sensor == "SR3D_STEREO" || sensor == "SR3D_TOF" )
  {
    scanmsg.header.frame_id     = frame_id_;
    scanmsg.header.stamp        = ros::Time::now(); 
    status = vsx_GetDataContainer(ptr_vsx, &dch, 10000);
    if (status == VSX_STATUS_SUCCESS)
    {
        status = vsx_GetImage(dch, "CalibratedA", &imageA);
        if (status == VSX_STATUS_SUCCESS)
        {
            status = vsx_GetImage(dch, "CalibratedB", &imageB);
            if (status == VSX_STATUS_SUCCESS)
            {
                status = vsx_GetImage(dch, "CalibratedC", &imageC);
                if (status == VSX_STATUS_SUCCESS)
                {
                    width               = imageA->width;
                    height              = imageA->height;
                    format              = imageA->format;
                    linePitch           = imageA->linePitch;
                    frameCounter        = imageA->frameCounter;
                    coordinateScale     = imageA->coordinateScale;
                    coordinateOffset    = imageA->coordinateOffset;
                    axisMin             = imageA->axisMin;
                    axisMax             = imageA->axisMax;
                    invalidDataValue    = imageA->invalidDataValue;

                    float xPixelf   = 0;        float yPixelf   = 0;        float zPixelf   = 0;
                    float* xf       = NULL;     float* yf       = NULL;     float* zf       = NULL;
                    int16_t* x      = NULL;     int16_t* y      = NULL;     int16_t* z      = NULL;
                    int16_t xPixel  = 0;        int16_t yPixel  = 0;        int16_t zPixel  = 0;
                    float x_tmp     = 0.0;      float y_tmp     = 0.0;      float z_tmp     = 0.0;
                    
                    int point_id = 0;
                    int max_size = 0;
                    int rowIndex = 0, colIndex = 0;

                    max_size = imageA->height*imageA->width;
                    scanmsg.points.resize(max_size);
                   
                    if( sensor == "SR3D_TOF")
                    {                        
                        for(int rowCentered = 0;rowCentered < (imageA->height);rowCentered++)
                        {
                            rowIndex = 0;
                            colIndex = rowCentered * imageA->linePitch;                               
                            for(int colCentered = 0;colCentered < (imageA->width);colCentered++)
                            {
                                index = colIndex + rowIndex;         
                                z = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                                zPixel = *z;
                                x = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                                xPixel = *x;
                                y = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);
                                yPixel = *y;
                                
                                if(zPixel > 0 && point_id < max_size)
                                {
                                    x_tmp = float(xPixel)/1000.0;
                                    y_tmp = float(yPixel)/1000.0;
                                    z_tmp = float(zPixel)/1000.0;
        
                                    scanmsg.points[point_id].x = x_tmp;
                                    scanmsg.points[point_id].y = y_tmp;
                                    scanmsg.points[point_id].z = z_tmp; 
                            
                                    point_id++;                      
                                }
                            rowIndex +=2;
                            }      
                        }
                    }

                    if( sensor == "SR3D_STEREO")
                    {                          
                        for(int rowCentered = 0;rowCentered < (imageA->height);rowCentered++)
                        {
                            rowIndex = 0;
                            colIndex = rowCentered * imageA->linePitch;
                            for(int colCentered = 0;colCentered < (imageA->width);colCentered++)
                            {
                                index = colIndex + rowIndex;
       
                                zf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                                zPixelf = *zf;
                                xf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                                xPixelf = *xf;
                                yf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);
                                yPixelf = *yf;
                                
                                x_tmp = (xPixelf)/1000.0;
                                y_tmp = (yPixelf)/1000.0;
                                z_tmp = (zPixelf)/1000.0;

                                if(z_tmp > 0.1 && point_id < max_size)
                                {                                 
                                    scanmsg.points[point_id].x = x_tmp;
                                    scanmsg.points[point_id].y = y_tmp;
                                    scanmsg.points[point_id].z = z_tmp; 
                            
                                    point_id++;        
                                }                           
                                rowIndex +=4;
                            }      
                        }
                    }
                    status = vsx_ReleaseImage(&imageA);
                    status = vsx_ReleaseImage(&imageB);
                    status = vsx_ReleaseImage(&imageC);     
                }
            }
        }
        status = vsx_ReleaseDataContainer(&dch);
    } 
    sensor_msgs::convertPointCloudToPointCloud2(scanmsg,scanmsg2);  
    scan_publisher_.publish(scanmsg2);
  }
}

void smartrunner_node::getScanDataPointCloud(const ros::TimerEvent &e)
{  
    sensor_msgs::PointCloud scanmsg; 
    
    if(sensor == "SMARTRUNNER")
    {
        status = vsx_GetDataContainer(ptr_vsx, &dch, 10000);
        if(status==VSX_STATUS_SUCCESS)
        {
            status = vsx_GetLine(dch, "Line", &lineData);
            if(status==VSX_STATUS_SUCCESS)
            {          
                scanmsg.header.frame_id = frame_id_;
                scanmsg.header.stamp = ros::Time::now();
                scanmsg.points.resize(lineData->width);
    
                for( std::size_t i=0; i<lineData->width; i++ )
                {
                    scanmsg.points[i].x = lineData->lines[0][i].x / 1000.0;
                    scanmsg.points[i].y = 0;
                    scanmsg.points[i].z = lineData->lines[0][i].z / 1000.0; 
                }

                status = vsx_ReleaseLine(&lineData);
                status = vsx_ReleaseDataContainer(&dch);
             
                scan_publisher_.publish(scanmsg);
            }
        }
    }

  if(sensor == "SR3D_STEREO" || sensor == "SR3D_TOF" )
  {
    scanmsg.header.frame_id     = frame_id_;
    scanmsg.header.stamp        = ros::Time::now(); 
    status = vsx_GetDataContainer(ptr_vsx, &dch, 10000);
    if (status == VSX_STATUS_SUCCESS)
    {
        status = vsx_GetImage(dch, "CalibratedA", &imageA);
        if (status == VSX_STATUS_SUCCESS)
        {
            status = vsx_GetImage(dch, "CalibratedB", &imageB);
            if (status == VSX_STATUS_SUCCESS)
            {
                status = vsx_GetImage(dch, "CalibratedC", &imageC);
                if (status == VSX_STATUS_SUCCESS)
                {
                    width               = imageA->width;
                    height              = imageA->height;
                    format              = imageA->format;
                    linePitch           = imageA->linePitch;
                    frameCounter        = imageA->frameCounter;
                    coordinateScale     = imageA->coordinateScale;
                    coordinateOffset    = imageA->coordinateOffset;
                    axisMin             = imageA->axisMin;
                    axisMax             = imageA->axisMax;
                    invalidDataValue    = imageA->invalidDataValue;

                    float xPixelf   = 0;        float yPixelf   = 0;        float zPixelf   = 0;
                    float* xf       = NULL;     float* yf       = NULL;     float* zf       = NULL;
                    int16_t* x      = NULL;     int16_t* y      = NULL;     int16_t* z      = NULL;
                    int16_t xPixel  = 0;        int16_t yPixel  = 0;        int16_t zPixel  = 0;
                    float x_tmp     = 0.0;      float y_tmp     = 0.0;      float z_tmp     = 0.0;
                    
                    int point_id = 0;
                    int max_size = 0;
                    int rowIndex = 0, colIndex = 0;

                    max_size = imageA->height*imageA->width;
                    scanmsg.points.resize(max_size);
                   
                    if( sensor == "SR3D_TOF")
                    {                        
                        for(int rowCentered = 0;rowCentered < (imageA->height);rowCentered++)
                        {
                            rowIndex = 0;
                            colIndex = rowCentered * imageA->linePitch;                               
                            for(int colCentered = 0;colCentered < (imageA->width);colCentered++)
                            {
                                index = colIndex + rowIndex;         
                                z = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                                zPixel = *z;
                                x = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                                xPixel = *x;
                                y = reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);
                                yPixel = *y;
                                
                                if(zPixel > 0 && point_id < max_size)
                                {
                                    x_tmp = float(xPixel)/1000.0;
                                    y_tmp = float(yPixel)/1000.0;
                                    z_tmp = float(zPixel)/1000.0;
        
                                    scanmsg.points[point_id].x = x_tmp;
                                    scanmsg.points[point_id].y = y_tmp;
                                    scanmsg.points[point_id].z = z_tmp; 
                            
                                    point_id++;                      
                                }

                            rowIndex +=2;
                            }      
                        }
                    }

                    if( sensor == "SR3D_STEREO")
                    {                          
                        for(int rowCentered = 0;rowCentered < (imageA->height);rowCentered++)
                        {
                            rowIndex = 0;
                            colIndex = rowCentered * imageA->linePitch;
                            for(int colCentered = 0;colCentered < (imageA->width);colCentered++)
                            {
                                index = colIndex + rowIndex;
       
                                zf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                                zPixelf = *zf;
                                xf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                                xPixelf = *xf;
                                yf = reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);
                                yPixelf = *yf;
                                
                                x_tmp = (xPixelf)/1000.0;
                                y_tmp = (yPixelf)/1000.0;
                                z_tmp = (zPixelf)/1000.0;

                                if(z_tmp > 0.1 && point_id < max_size)
                                {                                 
                                    scanmsg.points[point_id].x = x_tmp;
                                    scanmsg.points[point_id].y = y_tmp;
                                    scanmsg.points[point_id].z = z_tmp; 
                            
                                    point_id++;        
                                }                           
                                rowIndex +=4;
                            }      
                        }
                    }
                    status = vsx_ReleaseImage(&imageA);
                    status = vsx_ReleaseImage(&imageB);
                    status = vsx_ReleaseImage(&imageC);     
                }
            }
        }
        status = vsx_ReleaseDataContainer(&dch);
    } 
    scan_publisher_.publish(scanmsg);
  }
}

void smartrunner_node::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
   printf("############### MsgCallBack ####################\n");
}
}

int main(int argc, char **argv)
{
    printf("main -> calling ros::init()\n");
    ros::init(argc, argv, "sr3d_node", ros::init_options::AnonymousName);
    printf("main -> creating sr3d object\n");
    pepperl_fuchs::smartrunner_node* ptr_sr3d = new pepperl_fuchs::smartrunner_node();
    printf("main -> calling ros::spin()\n");
    ros::spin();
    printf("main -> destroying sr3d object\n");
    delete ptr_sr3d;
    printf("main -> leavingh main()\n");
    return 0;
}
