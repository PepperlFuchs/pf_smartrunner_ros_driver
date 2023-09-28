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

//! \class smartrunner_node
//! \brief ROS driver node for the Pepperl+Fuchs SmartRunner (2-D & 3-D)
class smartrunner_node
{
public:
    //! Initialize and connect to laser range finder
    smartrunner_node();
    ~smartrunner_node();

    //! Callback function for control commands
    void cmdMsgCallback( const std_msgs::StringConstPtr& msg );

private:

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

    //Common Parameters
    std::string frame_id_               	="";
    std::string device_ip_              	="";
    std::string message_type_           	="";
    float data_repetition_rate_           	=0.0;
    std::string sensor                  	=""; 
    const char* IP 				= NULL;
    const char* sensor_type             	= NULL;
    bool retValue                       	= false;
    //Parameters for Vsx-Interface
    VsxStatusCode status;
    VsxSystemHandle* ptr_vsx            	= NULL;
    VsxDataContainerHandle* dch         	= NULL;
    VsxDevice* deviceData               	= NULL;
    VsxParameterList* deviceParameters  	= NULL;
    VsxImage* imageA                    	= NULL;
    VsxImage* imageB                    	= NULL;
    VsxImage* imageC                    	= NULL;
    VsxLineData* lineData	            	= NULL;
    VsxLineCoordinate* lineCoordinates  	= NULL; 
    VsxImageData2Format format;
    //Parameters for SmartRunner(3-D TOF)
    std::string tof_trigger_source_             ="";
    int tof_auto_trigger_rate_                  = 0;
    int tof_trigger_enable_                     = 0;
    int tof_exposure_time_                      = 0;
    int tof_range_mode_                         = 0;
    std::string tof_output_mode_                ="";
    //Parameters for SmartRunner(3-D Stereo)
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
   
};
}

#endif // SMARTRUNNER_NODE_H
