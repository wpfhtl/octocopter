#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QDebug>
#include <QString>
#include <QTimer>
#include <QObject>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/joystick.h>

#define MAX_DEVICE_NUM 4

/*
 This class wraps a joystick. When getValues() is called, it reads
 all joystick-events since its last invocation, updates internal state
 and sets all three axes to values in [-1,+1]
 Buttons are easy to implement, but aren't yet.
 */

class Joystick : public QObject
{
    Q_OBJECT

private:
    QTimer* mPollTimer;
    struct js_event event;
    struct timespec timeout;
    bool valid;
    qint16 x, y, z, r;
    int fd, deviceNum, numAxis, numButtons;
    QString deviceFile;
    QString mJoystickName;
    char* mButtons;


private slots:
    bool updateValues();
//    void slotPollButtons();

signals:
    void buttonStateChanged(quint8 buttonNumber, bool pressed);

    // used to set the motor-speeds
    void motion(const quint8& thrust, const qint8& pitch, const qint8& roll, const qint8& yaw, const qint8& height);

public:
    Joystick();
    ~Joystick();

    QString getJoystickName(void);
    bool isValid(void) const;


//    void getAxisValues(float &axisX, float &axisY, float &axisZ, float &axisR);
    void emitMotionCommands();
    bool isButtonPressed(const unsigned short number);
};

#endif
