#include <QCoreApplication>
#include <librosqt/QRosCallBackQueue.h>
#include "includes/thrusterController.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "thrusterControllerNode",ros::init_options::NoSigintHandler);
    QCoreApplication app(_argc, _argv);
    QCoreApplication::setApplicationName("thrusterControllerN");
    QCoreApplication::setApplicationVersion("0.1");

    QRosCallBackQueue cbQueue;

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cbQueue);

    ThrusterController controller(nh);

    app.exec();
    return 0;
}
