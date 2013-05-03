#include <QCoreApplication>
#include <QFile>
#include <flightcontrollervalues.h>

int main(int argc, char **argv)
{
    QCoreApplication app(argc,argv);

    QByteArray logData;

    if(app.arguments().size() < 2)
    {
        qDebug() << "please specify file to open!";
        exit(0);
    }

    QFile logFileFlightController(app.arguments().at(1));
    if(!logFileFlightController.open(QIODevice::ReadOnly))
    {
        qFatal("couldn't open %s, exiting.", qPrintable(logFileFlightController.fileName()));
    }
    else
    {
        logData = logFileFlightController.readAll();
        logFileFlightController.close();
    }

    QDataStream ds(logData);

    printf("Time PoseAge HeightIs HeightShould WeightP OutP WeightD OutD Out OutClamped\n");

    while(!ds.atEnd())
    {
        ds.skipRawData(6); // FLTCLR
        FlightControllerValues fcv;
        ds >> fcv;

        //if(fcv.flightState.state != FlightState::Value::Hover) continue;

        QString line = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10")
                    .arg(fcv.timestamp)
                    .arg(fcv.timestamp - fcv.lastKnownPose.timestamp)
                    .arg(fcv.lastKnownPose.getPosition().y()*100)
                    .arg(fcv.hoverPosition.y()*100)
                    .arg(fcv.controllerThrust.getWeightP())
                    .arg(fcv.controllerThrust.getLastOutputP())
                    .arg(fcv.controllerThrust.getWeightD())
                    .arg(fcv.controllerThrust.getLastOutputD())
                    .arg(MotionCommand::thrustHover + fcv.controllerThrust.getLastOutput())
                    .arg(qBound(50, (int)(MotionCommand::thrustHover + fcv.controllerThrust.getLastOutput()), 180));

        printf("%s\n", qPrintable(line));
    }
}
