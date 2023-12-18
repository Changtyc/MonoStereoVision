#ifndef MYTHREAD_H
#define MYTHREAD_H
#include <QObject>
#include <QWidget>
#include <QThread>
#include <QtCore/QCoreApplication>
#include <QDebug>

class MyThread : public QThread
{
    Q_OBJECT
public:
    MyThread();
//    void run() override;
//    void stop();
//private:
//    bool m_start;
};

#endif // MYTHREAD_H
