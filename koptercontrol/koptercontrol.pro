TEMPLATE = app
TARGET =
DEPENDPATH += .
INCLUDEPATH += ./ ../
CONFIG += qt debug
QT += network

# Include the posix stuff for QExtSerialPort, not the windows headers
unix:DEFINES   = _TTY_POSIX_
win32:DEFINES  = _TTY_WIN_

# Input
HEADERS +=  koptercontrol.h \
	    qextserialport/src/qextserialport.h \
	    kopter.h \
	    gpsdevice.h \
	    laserscanner.h \
	    koptermessage.h

SOURCES +=  koptercontrol.cpp \
	    qextserialport/src/qextserialport.cpp \
	    qextserialport/src/posix_qextserialport.cpp \
	    gpsdevice.cpp \
	    laserscanner.cpp \
	    kopter.cpp \
	    koptermessage.cpp
