#include "joystick.h"

Joystick::Joystick() : QObject()
{
    x = y = z = numButtons = numAxis = 0;
    deviceFile = "/dev/input/js%1";
    valid = false;

    // Set a short timeout for select()ing the fd
    timeout.tv_sec = 0;
    timeout.tv_nsec = 10;

    // Try opening all js-device-files before failing.
    int currentDeviceIndex = 0;
    while((fd = open(deviceFile.arg(currentDeviceIndex).toLocal8Bit().constData(), O_RDONLY)) == -1)
    {
        qDebug() << "Joystick::Joystick(): Couldn't open joystick at" << deviceFile.arg(currentDeviceIndex);
        currentDeviceIndex++;

        if(currentDeviceIndex > MAX_DEVICE_NUM)
        {
            qDebug() << "Joystick::Joystick(): tried every device file, failed.";
            return;
        }
    }

    valid = true;

    char joystickName[255];

    ioctl(fd, JSIOCGAXES, &numAxis);
    ioctl(fd, JSIOCGBUTTONS, &numButtons);
    ioctl(fd, JSIOCGNAME(255), &joystickName);

    mJoystickName = QString(joystickName);

    qDebug() << "Joystick detected:" << mJoystickName << numAxis << "axis and" << numButtons << "buttons.";

    mButtons = (char *) calloc( numButtons, sizeof( char ) );

    mPollTimer = new QTimer;
    connect(mPollTimer, SIGNAL(timeout()), SLOT(updateValues()));
    mPollTimer->setInterval(200);
    mPollTimer->start();

    // use non-blocking mode
    fcntl(fd, F_SETFL, O_NONBLOCK);
}

Joystick::~Joystick()
{
    mPollTimer->stop();
    mPollTimer->deleteLater();
    close(fd);
}

//void Joystick::slotPollButtons()
//{
//}

bool Joystick::isValid(void) const
{
    return valid;
}

bool Joystick::updateValues()
{
    if(!valid) return false;

    // get all events from joystick and update our private values
    while(read(fd, &event, sizeof(struct js_event)) > 0)
    {
        /* see what to do with the event */
        if(mJoystickName == "WAILLY PPM") // the USB adaptor for the kopter's RC. Sucks, don't use it!
        {
            switch (event.type & ~JS_EVENT_INIT)
            {
            case JS_EVENT_AXIS:
                qDebug() << "JOYSTICK:" << event.number;
                switch(event.number)
                {
                case 0:
//                     = event.value;
                    break;
                case 1:
                    x = -event.value;
                    break;
                case 2:
                    y = -event.value;
                    break;
                case 3:
                    r = event.value;
                    break;
                case 4:
                    qDebug() << "CTRL 7 know turned to" << event.value;
                    break;
                case 5:
                    z = -event.value;
                    break;
                }
                break;

            case JS_EVENT_BUTTON:
                // button pressed is 1, released 0
                mButtons[event.number] = event.value;
                emit buttonStateChanged(event.number, event.value == 1);
                qDebug() << "button" << event.number << "is now" << (event.value == 1);
            break;
            }
        }
        else
        {
            switch(event.type & ~JS_EVENT_INIT)
            {
            case JS_EVENT_AXIS:
                switch(event.number)
                {
                case 0:
                    x = event.value;
                    break;
                case 1:
                    y = event.value;
                    break;
                case 2:
                    z = event.value;
                    break;
                case 3:
                    r = event.value;
                    break;
                case 4:
                    qDebug() << "coolie l/r!";
                    break;
                case 5:
                    qDebug() << "coolie u/d!";
                    break;
                }
                break;

            case JS_EVENT_BUTTON:
                // button pressed is 1, released 0
                mButtons[event.number] = event.value;
                emit buttonStateChanged(event.number, event.value == 1);
                //            qDebug() << "button" << event.number << "is now" << (event.value == 1);
            break;
            }
        }
    }

    // print the results
//    qDebug() << "xyzr:" << x << y << z << r;

    if(errno != EAGAIN)
    {
        qDebug() << "Joystick::getValues(): received error" << errno;
        return false;
    }

    return true;
}
/*
void Joystick::getAxisValues(float &axisX, float &axisY, float &axisZ, float &axisR)
{
    updateValues();

    axisX = ((float)x)/32768.0;
    axisY = ((float)y)/32768.0;
    axisZ = ((float)z)/32768.0;
    axisR = ((float)r)/32768.0;
}*/

void Joystick::slotEmitMotionCommands()
{
    updateValues();

//    qDebug() << "Joystick::slotEmitMotionCommands(): going to emit joystick values"
//             << "thrust" << ((((float)-r)/32768.0) + 1.0) * 128.0
//             << "pitch" << (((float)-y)/32768.0) * 127.0
//             << "roll" << (((float)-x)/32768.0) * 127.0
//             << "yaw" << (((float)-z)/32768.0) * 127.0;

    emit motion(
                ((((float)-r)/32768.0) + 1.0) * 128.0, // thrust 0-255
                (((float)-z)/32768.0) * 127.0, // qint8 yaw   -127-+127
                (((float)y)/32768.0) * 127.0 * 0.8, // qint8 pitch -127-+127
                (((float)-x)/32768.0) * 127.0 * 0.8, // qint8 roll  -127-+127
                128 // height is dummy, unused
                );
}

bool Joystick::isButtonPressed(const unsigned short number)
{
    Q_ASSERT(number < numButtons);

    updateValues();
    return mButtons[number] == 1;
}
