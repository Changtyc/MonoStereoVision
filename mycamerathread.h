#ifndef MYCAMERATHREAD_H
#define MYCAMERATHREAD_H

#include <QObject>
#include <QThread>

class MyCameraThread: public QThread
{
    Q_OBJECT
public:
    MyCameraThread();
    void run() override;
    void stop();
private:
    bool m_start;
};

#endif // MYCAMERATHREAD_H
