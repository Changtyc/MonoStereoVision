#include "mycamerathread.h"
#include <QDebug>

MyCameraThread::MyCameraThread()
{
    m_start=true;
}


void MyCameraThread::run(){
    m_start=true;
    while(m_start){
        qDebug()<<"hello world"<<endl;
        sleep(1);
    }
}

void MyCameraThread::stop(){
    m_start=false;
}

