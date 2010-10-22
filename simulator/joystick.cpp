#include "joystick.h"

Joystick::Joystick() : QObject()
{
    x = y = z = numButtons = numAxis = 0;
    deviceFile = "/dev/input/js";
    valid = false;

    // Set a short timeout for select()ing the fd
    timeout.tv_sec = 0;
    timeout.tv_nsec = 10;

    // Try opening all js-device-files before failing.
    int currentDeviceIndex = 0;
    while((fd = open(deviceFile.append(QString::number(currentDeviceIndex)).toLocal8Bit().constData(), O_RDONLY)) == -1)
    {
        qDebug() << "Joystick::Joystick(): Couldn't open joystick at" << deviceFile << currentDeviceIndex;
        currentDeviceIndex++;

        if(currentDeviceIndex > MAX_DEVICE_NUM)
        {
            qDebug() << "Joystick::Joystick(): tried every device file, failed.";
            return;
        }
    }

    valid = true;

    ioctl(fd, JSIOCGAXES, &numAxis);
    ioctl(fd, JSIOCGBUTTONS, &numButtons);
    ioctl(fd, JSIOCGNAME(255), &joystickName);

    qDebug() << "Joystick detected:" << joystickName << numAxis << "axis and" << numButtons << "buttons.";

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
        switch (event.type & ~JS_EVENT_INIT)
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

    // print the results
//    qDebug() << "xyzr:" << x << y << z << r;

    if(errno != EAGAIN)
    {
        qDebug() << "Joystick::getValues(): received error" << errno;
        return false;
    }

    return true;
}

void Joystick::getAxisValues(float &axisX, float &axisY, float &axisZ, float &axisR)
{
    updateValues();

    axisX = ((float)x)/32768.0;
    axisY = ((float)y)/32768.0;
    axisZ = ((float)z)/32768.0;
    axisR = ((float)r)/32768.0;
}

bool Joystick::isButtonPressed(const unsigned short number)
{
    Q_ASSERT(number < numButtons);

    updateValues();
    return mButtons[number] == 1;
}
