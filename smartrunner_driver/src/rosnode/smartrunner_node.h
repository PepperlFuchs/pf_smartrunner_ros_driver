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

};
}

#endif // SMARTRUNNER_NODE_H
