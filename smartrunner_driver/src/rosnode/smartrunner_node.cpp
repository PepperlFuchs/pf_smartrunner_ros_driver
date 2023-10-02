#include "smartrunner_node.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>

namespace pepperl_fuchs {
 
//-----------------------------------------------------------------------------
smartrunner_node::smartrunner_node():nh_("~")
{
    running_ = false;
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
 
}

bool smartrunner_node::init()
{
    if( device_ip_ == "" )
    {
        printf("IP of scanner not set!\n");
        return false;
    }
    else
    {      
        if(connect())
        {         
            if(message_type_ == "PointCloud")
            {
                // Declare publisher
                scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("scan", 5);
            }
            if(message_type_ == "PointCloud2")
            { 
                // Declare publisher
                scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("scan", 5);
            }

            cmd_subscriber_ = nh_.subscribe("control_command",100,&smartrunner_node::cmdMsgCallback,this);

            // set up grabbing loop
            running_ = true;
            grab_thread_ = std::move(std::thread(&pepperl_fuchs::smartrunner_node::grab_loop, this));
            return true;
        }
    }
    return false;
}

smartrunner_node::~smartrunner_node()
{
    printf("#######################################################\n");
    printf("Destructing smartrunner_node\n");
    // Stop grab thread if it was started
    if (running_)
    {
        running_ = false;
        grab_thread_.join();
    }

    auto status =  vsx_Disconnect(ptr_vsx);
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
    VsxStatusCode status = vsx_GetLibraryVersion(&version);
    if (status == VSX_STATUS_SUCCESS)
    {      
        printf("Version %s\n", version);
        status = vsx_ReleaseString(&version);
    }
  
    printf("#######################################################\n");
    status = vsx_InitTcpSensor(&ptr_vsx, device_ip_.c_str(), PlugIn);
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
    VsxDevice* deviceData = nullptr;
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
    } else {
        return false;
    }


    printf("#######################################################\n");
    status = vsx_ResetDynamicContainerGrabber(ptr_vsx, 2, VSX_STRATEGY_DROP_OLDEST);
    if (status == VSX_STATUS_SUCCESS)
    {
        printf("receiving data...\n");       
    }         
    return true;
}

void smartrunner_node::grab_loop()
{
    while (running_) 
    {
        VsxDataContainerHandle* dch = NULL;
        VsxStatusCode status = vsx_GetDataContainer(ptr_vsx, &dch, 500);
        if (status == VSX_STATUS_SUCCESS)
        {
            if (message_type_ == "PointCloud")
            {
                auto scanmsg = getScanDataPointCloud(dch);
                scan_publisher_.publish(scanmsg);

            }
            if (message_type_ == "PointCloud2")
            {
                auto scanmsg2 = getScanDataPointCloud2(dch);
                scan_publisher_.publish(scanmsg2);                
            }
            vsx_ReleaseDataContainer(&dch);
            continue;
        }
        if (status != VSX_STATUS_ERROR_DRIVER_TIMEOUT)
        {
            printf("Exiting grab loop due to error!\n");
            break;
        }
    }
}

sensor_msgs::PointCloud smartrunner_node::getScanDataPointCloud(VsxDataContainerHandle* dch)
{  
    sensor_msgs::PointCloud scanmsg; 
    scanmsg.header.frame_id     = frame_id_;
    scanmsg.header.stamp        = ros::Time::now();
 
    if(sensor == "SMARTRUNNER")
    {
        VsxLineData* lineData = nullptr;
        auto status = vsx_GetLine(dch, "Line", &lineData);
        if(status==VSX_STATUS_SUCCESS)
        {          
            scanmsg.points.resize(lineData->width);

            for( std::size_t i=0; i<lineData->width; i++ )
            {
                scanmsg.points[i].x = lineData->lines[0][i].x / 1000.0;
                scanmsg.points[i].y = 0;
                scanmsg.points[i].z = lineData->lines[0][i].z / 1000.0; 
            }
            status = vsx_ReleaseLine(&lineData);
        }
    }

    if(sensor == "SR3D_STEREO" || sensor == "SR3D_TOF" )
    {
        VsxImage* imageA = nullptr;
        VsxImage* imageB = nullptr;
        VsxImage* imageC = nullptr;

        auto status_A = vsx_GetImage(dch, "CalibratedA", &imageA);
        auto status_B = vsx_GetImage(dch, "CalibratedB", &imageB);
        auto status_C = vsx_GetImage(dch, "CalibratedC", &imageC);


        if (status_A == VSX_STATUS_SUCCESS && status_B == VSX_STATUS_SUCCESS && status_C == VSX_STATUS_SUCCESS)
        {
            int width               = imageA->width;
            int height              = imageA->height;
            auto format             = imageA->format;
            int linePitch           = imageA->linePitch;

            int point_id = 0;
            scanmsg.points.resize(width * height);

            if( format == VSX_IMAGE_DATA2_FORMAT_COORD3D_A16 ) // TOF
            {                        
                for(int row = 0;row < height;row++)
                {
                    int index = row * linePitch;
                    for(int col = 0;col < width;col++)
                    {
                        int16_t z = *reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                        
                        if(z > 0)
                        {
                            int16_t x = *reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                            int16_t y = *reinterpret_cast<int16_t*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);

                            scanmsg.points[point_id].x = float(x)/1000.0;
                            scanmsg.points[point_id].y = float(y)/1000.0;
                            scanmsg.points[point_id].z = float(z)/1000.0; 
                            point_id++;                      
                        }
                        index += sizeof(uint16_t);
                    }      
                }
            }

            if( format == VSX_IMAGE_DATA2_FORMAT_COORD3D_A32F) // Stereo
            {                          
                for(int row = 0;row < height;row++)
                {
                    int index = row * linePitch;
                    for(int col = 0;col < width; col++)
                    {
                        float z = *reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageC->rawdata) + index);
                        
                        if(z > 100.0)
                        {                                 
                            float x = *reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageA->rawdata) + index);
                            float y = *reinterpret_cast<float*>(reinterpret_cast<int8_t*>(imageB->rawdata) + index);

                            scanmsg.points[point_id].x = x/1000.0;
                            scanmsg.points[point_id].y = y/1000.0;
                            scanmsg.points[point_id].z = z/1000.0;                   
                            point_id++;        
                        }                           
                        index += sizeof(float);
                    }      
                }
            }
        }
        if (imageA != nullptr)
            vsx_ReleaseImage(&imageA);
        if (imageB != nullptr)
            vsx_ReleaseImage(&imageB);
        if (imageC != nullptr)
            vsx_ReleaseImage(&imageC);     
    }
    return scanmsg;
}

sensor_msgs::PointCloud2 smartrunner_node::getScanDataPointCloud2(VsxDataContainerHandle* dch)
{
    // TODO: create native PointCloud2 message
    auto scanmsg = getScanDataPointCloud(dch);
    sensor_msgs::PointCloud2 scanmsg2;
    sensor_msgs::convertPointCloudToPointCloud2(scanmsg,scanmsg2);  
    return scanmsg2;
}

void smartrunner_node::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
   printf("############### MsgCallBack ####################\n");
}
}

int main(int argc, char **argv)
{
    printf("main -> calling ros::init()\n");
    ros::init(argc, argv, "smartrunner_node", ros::init_options::AnonymousName);

    {
        printf("main -> creating sr3d object\n");
        pepperl_fuchs::smartrunner_node node{};

        if (node.init())
        {
            printf("main -> calling ros::spin()\n");
            ros::spin();
        } else {
            printf("main -> failed to connect to SmartRunner sensor\n");
        }

        printf("main -> destroying sr3d object\n");
        // node is automatically destructed when leaving scope
    }
    printf("main -> leaving main()\n");
    return 0;
}
