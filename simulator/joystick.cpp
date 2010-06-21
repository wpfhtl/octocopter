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

    // use non-blocking mode
    fcntl(fd, F_SETFL, O_NONBLOCK);
}

Joystick::~Joystick()
{
    close(fd);
}

bool Joystick::isValid(void) const
{
    return valid;
}

bool Joystick::getValues(float &axisX, float &axisY, float &axisZ, float &axisR)
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
            }
            break;

        case JS_EVENT_BUTTON:
            // do nothing for now, we don't use buttons yet
            //button [ js.number ] = js.value;
            break;
        }


//        for(x=0; x < numButtons; ++x)
//            printf("B%d: %d  ", x, button[x] );
    }

    axisX = ((float)x)/32768.0;
    axisY = ((float)y)/32768.0;
    axisZ = ((float)z)/32768.0;
    axisR = ((float)r)/32768.0;

    // print the results
//    qDebug() << "xyzr:" << axisX << axisY << axisZ << axisR;

    if(errno != EAGAIN)
    {
        qDebug() << "Joystick::getValues(): received error" << errno;
        return false;
    }

    return true;
}
