QT       += core gui
QT       += serialport
QT       += concurrent
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include($$PWD/opencv_lib.pri) #include包含的文件会显示在工程中


SOURCES += \
    MvCamera.cpp \
    calibrate_method.cpp \
    main.cpp \
    mycamerathread.cpp \
    widget.cpp

HEADERS += \
    MvCamera.h \
    calibrate_method.h \
    mycamerathread.h \
    widget.h

FORMS += \
    widget.ui


RC_ICONS = sanwei.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES +=

RESOURCES += \
    res.qrc


# 链接相机SDK
win32: LIBS += -L$$PWD/'../../../../../Program Files (x86)/MVS/MVS/Development/Libraries/win64/' -lMvCameraControl

INCLUDEPATH += $$PWD/'../../../../../Program Files (x86)/MVS/MVS/Development/Includes'
DEPENDPATH += $$PWD/'../../../../../Program Files (x86)/MVS/MVS/Development/Includes'

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/'../../../../../Program Files (x86)/MVS/MVS/Development/Libraries/win64/MvCameraControl.lib'
else:win32-g++: PRE_TARGETDEPS += $$PWD/'../../../../../Program Files (x86)/MVS/MVS/Development/Libraries/win64/libMvCameraControl.a'
