#include "sr3d_node.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud_conversion.h>

extern "C"{
    #include "PF.VsxProtocolDriver.WrapperNE.h"
}

#include <cstdlib>
#include <ctime>
#include <string>
#include <iostream>
#include <fstream>

namespace pepperl_fuchs {

//-----------------------------------------------------------------------------
sr3dnode::sr3dnode():nh_("~")
{
    // Reading and checking parameters from launch-file
    nh_.param("frame_id", frame_id_, std::string(""));
    nh_.param("scanner_ip",scanner_ip_,std::string(""));
    nh_.param("message_type",message_type_,std::string(""));
    
    printf("ip: %s\n", scanner_ip_.c_str());
    printf("message type: %s\n", message_type_.c_str());
    printf("id: %s\n", frame_id_.c_str());

    if( scanner_ip_ == "" )
    {
        printf("IP of scanner not set!\n");
        return;
    }
    else
    {
        IP = scanner_ip_.c_str();
        
        retValue = connect();
        
        if(retValue == true)
        {         
            if(message_type_ == "PointCloud")
            {
                // Declare publisher and create timer
                scan_publisher_         = nh_.advertise<sensor_msgs::PointCloud>("scan",100);
                cmd_subscriber_         = nh_.subscribe("control_command",100,&sr3dnode::cmdMsgCallback,this);
                get_scan_data_timer_    = nh_.createTimer(ros::Duration(0.1), &sr3dnode::getScanDataPointCloud, this);
            }
            if(message_type_ == "PointCloud2")
            { 
                // Declare publisher and create timer
                scan_publisher_         = nh_.advertise<sensor_msgs::PointCloud2>("scan",100);
                cmd_subscriber_         = nh_.subscribe("control_command",100,&sr3dnode::cmdMsgCallback,this);
                get_scan_data_timer_    = nh_.createTimer(ros::Duration(0.1), &sr3dnode::getScanDataPointCloud2, this);
            }           
        }
    }
}

sr3dnode::~sr3dnode()
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

bool sr3dnode::connect()
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
        printf("Sensor: %s\n",deviceData->sensorType);
        printf("Firmware: %s\n",deviceData->firmwareVersion);
        printf("MAC: %s\n",deviceData->macAddress);
        printf("IP: %s\n",deviceData->ipAddress);

        sensor = std::string(deviceData->sensorType);
        if( sensor == "SR3D_TOF")
        {          
            printf("#######################################################\n");
            printf("Settings for SR3D TOF\n");
        	status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerSource", "AutoTrigger");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "0");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "1");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "OutputMode", "CalibratedABC_Grid_Amp");
            printf("SetSingleParameterValue Trigger OutputMode: %d\n", status);
        }
        if(sensor == "SR3D_STEREO")
        {         
            printf("#######################################################\n");
            printf("Settings for SR3D STEREO\n");
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerSource", "AutoTrigger");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "0");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "TriggerEnabled", "1");
            printf("SetSingleParameterValue Trigger Status: %d\n", status);
            status = vsx_SetSingleParameterValue(ptr_vsx, 1, "Base", 1, "OutputMode", "DisparityC");
            printf("SetSingleParameterValue Trigger OutputMode: %d\n", status);
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
void sr3dnode::getScanDataPointCloud2(const ros::TimerEvent &e)
{  
    sensor_msgs::PointCloud2 scanmsg2;   
    sensor_msgs::PointCloud scanmsg;   
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

void sr3dnode::getScanDataPointCloud(const ros::TimerEvent &e)
{  
    sensor_msgs::PointCloud scanmsg;   
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


void sr3dnode::cmdMsgCallback(const std_msgs::StringConstPtr &msg)
{
   printf("############### MsgCallBack ####################\n");
}

}

int main(int argc, char **argv)
{
    printf("main -> calling ros::init()\n");
    ros::init(argc, argv, "sr3d_node", ros::init_options::AnonymousName);
    printf("main -> creating sr3d object\n");
    pepperl_fuchs::sr3dnode* ptr_sr3d = new pepperl_fuchs::sr3dnode();
    printf("main -> calling ros::spin()\n");
    ros::spin();
    printf("main -> destroying sr3d object\n");
    delete ptr_sr3d;
    printf("main -> leavingh main()\n");
    return 0;
}
