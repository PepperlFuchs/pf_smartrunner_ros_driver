#ifndef SR3D_NODE_H
#define SR3D_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

extern "C"{
    #include "PF.VsxProtocolDriver.WrapperNE.h"
    #include "dnne.h"
}


namespace pepperl_fuchs {
//class sr3ddriver;

//! \class sr3dnode
//! \brief ROS driver node for the Pepperl+Fuchs SmartRunner3D
class sr3dnode
{
public:
    //! Initialize and connect to laser range finder
    sr3dnode();
    ~sr3dnode();

    //! Callback function for control commands
    void cmdMsgCallback( const std_msgs::StringConstPtr& msg );

private:
   
    const char* IP              = NULL;
    bool retValue               = false;
    std::string sensor          = ""; 
    const char* sensor_type     = NULL;
    int linePitch               = 0;
    long frameCounter           = 0;
    double coordinateScale      = 0.0;
    double coordinateOffset     = 0.0;
    double axisMin              = 0.0;
    double axisMax              = 0.0;
    double invalidDataValue     = 0.0;
    int rowIndex                = 0;
    int colIndex                = 0;
    int index                   = 0;
    int width                   = 0;
    int height                  = 0;
    time_t now;
    VsxStatusCode status;
    VsxSystemHandle* ptr_vsx    = NULL;
    VsxDataContainerHandle* dch = NULL;
    VsxDevice* deviceData       = NULL;
    VsxImage* imageA            = NULL;
    VsxImage* imageB            = NULL;
    VsxImage* imageC            = NULL;
    VsxImageData2Format format;



    //! Connect to the laser range finder
    //! @returns True on success, false otherwise
    bool connect();

    //! Time callback function for getting data from the driver and sending them out
    void getScanDataPointCloud( const ros::TimerEvent& e);
    void getScanDataPointCloud2( const ros::TimerEvent& e);

    //! Internal ROS node handle
    ros::NodeHandle nh_;

    //! Callback timer for getScanData(...)
    ros::Timer get_scan_data_timer_;

    //! ROS publisher for publishing scan data
    ros::Publisher scan_publisher_;

    //! ROS subscriber for receiving control commands
    ros::Subscriber cmd_subscriber_;

    //! frame_id of sensor_msgs/Laserscan messages
    std::string frame_id_;

    //! IP or hostname of laser range finder
    std::string scanner_ip_;

    std::string message_type_;

};
}

#endif // SR3D_NODE_H
