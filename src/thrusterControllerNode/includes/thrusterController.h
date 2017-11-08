#ifndef THRUSTER_CONTROLLER_NODE_H
#define THRUSTER_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <custom_msgs/stampedCartesianTrack.h>
#include <std_msgs/Float64.h>
#include <QObject>

struct commands{
    float yaw;
    float speed;
};

struct cubeState{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float speed;
};

class ThrusterController : public QObject
{
    Q_OBJECT
public:
    ThrusterController(ros::NodeHandle& _nh,QObject *parent = 0);
    void publishCmd();

    ~ThrusterController();

protected:
    void onNewNavigationState(const custom_msgs::stampedCartesianTrackPtr  & _msg);
    void onNewTransform(const geometry_msgs::PosePtr & _msg);
    commands computeCmd();



private:
    ros::NodeHandle m_nh;
    ros::Publisher m_yawCmdPublisher;
    ros::Publisher m_velocityCmdPublisher;
    ros::Subscriber m_usvTransformSubscriber;
    ros::Subscriber m_cartNavStateSubscriber;

    float m_kpYaw=0.1;
    float m_kiSpeed=5;
    float m_dt=0.1;
    float m_integral;
    float m_angleMax;
    float m_thrustMax;
    double m_rhoPrec;
    int m_shipID;
    cubeState m_currentState;
};

#endif // THRUSTER_CONTROLLER_NODE_H
