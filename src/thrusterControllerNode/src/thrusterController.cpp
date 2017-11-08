#include "includes/thrusterController.h"
#include <angles/angles.h>

ThrusterController::ThrusterController(ros::NodeHandle& _nh,QObject *parent) :
    QObject(parent)
{
    m_nh=_nh;

    if(m_nh.getParam("shipID",m_shipID)){
    }else{
        ROS_ERROR("No ownShipID");
        exit(-1);
    }


    m_nh.param<float>("angleMax", m_angleMax, 30);
    m_angleMax=angles::from_degrees(m_angleMax);
    m_nh.param<float>("thrustMax", m_thrustMax, 100);

    m_yawCmdPublisher = m_nh.advertise<std_msgs::Float64>("/drone"+std::to_string(m_shipID)+"/yawCmd",10);
    m_velocityCmdPublisher = m_nh.advertise<std_msgs::Float64>("/drone"+std::to_string(m_shipID)+"/velocityCmd",10);

    m_usvTransformSubscriber =  m_nh.subscribe("/drone"+std::to_string(m_shipID)+"/usvTransform",10,&ThrusterController::onNewTransform,this);
    m_cartNavStateSubscriber =  m_nh.subscribe("/drone"+std::to_string(m_shipID)+"/cartNavState",10,&ThrusterController::onNewNavigationState,this);

    m_integral=0;
}

ThrusterController::~ThrusterController()
{
}


void ThrusterController::onNewNavigationState(const custom_msgs::stampedCartesianTrackPtr  & _msg)
{
    cubeState newState;

    newState.speed=_msg->sog;
    newState.yaw=_msg->cog;

    newState.x=_msg->cart_track.point.x;
    newState.y=_msg->cart_track.point.y;
    newState.z=_msg->cart_track.point.z;

    newState.pitch=m_currentState.pitch;
    newState.roll=m_currentState.roll;

    m_currentState=newState;

    commands cmd =computeCmd();
    std_msgs::Float64 speedCmd_msg, yawCmd_msg;
    speedCmd_msg.data=cmd.speed;
    yawCmd_msg.data=cmd.yaw;
    m_velocityCmdPublisher.publish(speedCmd_msg);
    m_yawCmdPublisher.publish(yawCmd_msg);
    ROS_INFO_STREAM("Position :  "<<m_currentState.x<<"   "<<m_currentState.y<<"    "<<m_currentState.z);
    ROS_INFO_STREAM("Orientation :  "<<m_currentState.roll<<"   "<<m_currentState.pitch<<"    "<<m_currentState.yaw);
    ROS_INFO_STREAM("Speed :  "<<m_currentState.speed);
}

void ThrusterController::onNewTransform(const geometry_msgs::PosePtr & _msg)
{
    m_currentState.roll=_msg->orientation.x;
    m_currentState.pitch=_msg->orientation.y;

}


commands ThrusterController::computeCmd()
{
    float steeringCmd, thrustCmd;

    double desiredYaw = 45.0; 
    float desiredSpeed = 10.0f; 
    double shortestangle = angles::shortest_angular_distance(angles::from_degrees(m_currentState.yaw),angles::from_degrees(desiredYaw));
    float errSpeed = desiredSpeed-m_currentState.speed;
    m_integral+=errSpeed*m_dt;
    m_integral=std::min(m_integral,20.0f);
    thrustCmd = m_integral*m_kiSpeed;

    if(fabs(thrustCmd)>m_thrustMax){
            thrustCmd = m_thrustMax*copysign(1,thrustCmd);
    }

    steeringCmd=shortestangle*m_kpYaw;
    if(fabs(steeringCmd)>m_angleMax){
            steeringCmd = m_angleMax*copysign(1,steeringCmd);
    }

    commands cmd;
    cmd.speed=thrustCmd;
    cmd.yaw=angles::to_degrees(steeringCmd);
    return cmd;
}



