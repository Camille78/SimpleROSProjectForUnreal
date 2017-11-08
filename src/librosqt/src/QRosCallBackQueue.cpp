#include <librosqt/QRosCallBackQueue.h>
#include <QMetaObject>
#include <QCoreApplication>
#include <signal.h>
#include <sys/socket.h>
#include <QDebug>

namespace ros{
    extern CallbackQueuePtr g_global_queue;
}

int QRosCallBackQueue::m_sigtermFd[2];

QRosCallBackQueue::QRosCallBackQueue()
{
    // On installe le handler du Ctrl+C via un QSocketNotifier :
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, QRosCallBackQueue::m_sigtermFd)) qFatal("Couldn't create TERM socketpair");
    m_snTerm = new QSocketNotifier(QRosCallBackQueue::m_sigtermFd[1], QSocketNotifier::Read, this);
    connect(m_snTerm, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));

    struct sigaction term;
    term.sa_handler = QRosCallBackQueue::termSignalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;
    if (sigaction(SIGINT, &term, 0) > 0)
        qFatal("Can't perform sigaction for INT Signal.");
    if (sigaction(SIGTERM, &term, 0) > 0)
        qFatal("Can't perform sigaction for TERM Signal.");

}

QRosCallBackQueue::~QRosCallBackQueue()
{

}

void QRosCallBackQueue::addCallback(const ros::CallbackInterfacePtr &callback, uint64_t owner_id)
{
    ros::CallbackQueue::addCallback(callback, owner_id);
    QMetaObject::invokeMethod(this, "processCallbacks", Qt::QueuedConnection);
}

void QRosCallBackQueue::processCallbacks()
{
    callAvailable();
}

void QRosCallBackQueue::termSignalHandler(int)
{
    // Cette fonction est appelée dans le contexte du "SIGNAL" posix, donc pas dans le thread QT... On écrit dans la
    // socket pair, et cela déclenche une notification su SocketNotifier.
    char a = 1;
    ::write(QRosCallBackQueue::m_sigtermFd[0], &a, sizeof(a));
}

void QRosCallBackQueue::replaceGlobalQueue()
{
    ros::g_global_queue.reset(new QRosCallBackQueue());
}

void QRosCallBackQueue::handleSigTerm()
{
    // Quand le socket notifier se déclenche, on passe ici, donc on quitte l'application.
    m_snTerm->setEnabled(false);
    char tmp;
    ::read(QRosCallBackQueue::m_sigtermFd[1], &tmp, sizeof(tmp));

    // do Qt stuff
    qDebug() << "Properly quit QT";
    QCoreApplication::quit();
    m_snTerm->setEnabled(true);
}
