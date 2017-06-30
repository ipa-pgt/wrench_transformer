#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>

#include "wrench_transformer/GetForceTorque.h"

class WrenchCollector
{
protected:
    ros::NodeHandle _nh;
    ros::Subscriber _wrenchSub;
    ros::ServiceServer _service;
    tf::TransformListener _tfListener;

    std::vector<geometry_msgs::WrenchStamped> _transWrench;
    boost::shared_mutex _wrenchDataAccess;

    std::string _subTopic;
    tf::TransformListener _listener;
    std::string _transFrameId;
    int _wrenchCount, _maxCount;

    bool collect(wrench_transformer::GetForceTorque::Request  &req,
             wrench_transformer::GetForceTorque::Response &res)
    {
        ROS_INFO("Got Command to collect %d samples transformed to frame_id %s", req.numberOfSamples, req.frame_id.c_str());
        _transFrameId = req.frame_id;
        _transWrench.clear();
        _maxCount = req.numberOfSamples; 

        while(!isWrenchCollected(req.numberOfSamples))
        {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            ROS_INFO("Collecting...");
        }
        res.forceTorques = _transWrench;
        return true;
    }

    void kmsCallback(const geometry_msgs::WrenchStampedConstPtr& data)
    {

        if (_wrenchCount < _maxCount)
        {
            geometry_msgs::WrenchStamped transWrench;
            transWrench.header.stamp = data->header.stamp;
            transWrench.header.frame_id = _transFrameId;

            if (data->header.frame_id.compare(_transFrameId) != 0)
            {

                tf::Matrix3x3 transRotMatrix, M_transMatrix;

                tf::Vector3 transTranslation, F_measured, M_measured , F_target , M_target;

                F_measured[0] = data->wrench.force.x;
                F_measured[1] = data->wrench.force.y;
                F_measured[2] = data->wrench.force.z;

                M_measured[0] = data->wrench.torque.x;
                M_measured[1] = data->wrench.torque.y;
                M_measured[2] = data->wrench.torque.z;


                tf::StampedTransform transform;
                // Listen to kms40 to the tool frame
                try{
    //                _listener.waitForTransform(data->header.frame_id, _transFrameId, ros::Time(0), ros::Duration(0.1));
                    _listener.lookupTransform(_transFrameId, data->header.frame_id, ros::Time(0), transform);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                    return;
                }

                // Finding the rotation Matrix from kms40 to tool_tcp
                transRotMatrix = transform.getBasis();

                //Finding the translation vector from kms40 to tool_tcp
                transTranslation = transform.getOrigin();

                // Calculating the Force in the new tool_tcp
                F_target = transRotMatrix * F_measured;


                // Define the Omega matrix
                M_transMatrix[0][0] = 0;
                M_transMatrix[0][1] = -transTranslation[2];
                M_transMatrix[0][2] = transTranslation[1];
                M_transMatrix[1][0] = transTranslation[2];
                M_transMatrix[1][1] = 0;
                M_transMatrix[1][2] = -transTranslation[0];
                M_transMatrix[2][0] = -transTranslation[1];
                M_transMatrix[2][1] = transTranslation[0];
                M_transMatrix[2][2] = 0;

                //Calculating the moment at the tool_tcp

                M_target = (M_transMatrix * transRotMatrix) * F_measured + transRotMatrix * M_measured;

                transWrench.wrench.force.x = F_target[0];
                transWrench.wrench.force.y = F_target[1];
                transWrench.wrench.force.z = F_target[2];
                transWrench.wrench.torque.x = M_target[0];
                transWrench.wrench.torque.y = M_target[1];
                transWrench.wrench.torque.z = M_target[2];

            }
            else
            {
                transWrench.wrench = data->wrench;
            }

            _transWrench.push_back(transWrench);
            ROS_INFO("Sample Count: %d", _wrenchCount);

            _wrenchCount++;
        }


    }

    bool isWrenchCollected(int maxCount)
    {
        return _wrenchCount >= maxCount;
    }


public:
    WrenchCollector(std::string name)
    {

        if( !ros::param::get("~subscribeTopic", _subTopic) )
        {
            ROS_ERROR("Cannot find %s/subscribeTopic @ parameterServer", name.c_str());
            return;
        }

        _wrenchCount = 0;
        _maxCount = 0;
        
        _nh = ros::NodeHandle("~");
         _service = _nh.advertiseService(name+"/collect", &WrenchCollector::collect, this);
        _wrenchSub = _nh.subscribe(_subTopic, 1000, &WrenchCollector::kmsCallback, this);
        ros::spin();

    }

};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "WrenchCollector");

    //    ROS_ERROR("Node not working as hoped. Math failed");
    WrenchCollector WrenchCollector(ros::this_node::getName());

    return 0;
}
