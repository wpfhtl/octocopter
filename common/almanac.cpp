#include "almanac.h"
#include <QNetworkRequest>
#include <QNetworkReply>

Almanac::Almanac(QObject *parent) : QObject(parent)
{
    mTow = -1.0f;

    mReceiverPosition.longitude = 9.98;
    mReceiverPosition.latitude = 53.5;
    mReceiverPosition.elevation = 0.01;

    mNetworkAccessManager = new QNetworkAccessManager(this);
    connect(mNetworkAccessManager, SIGNAL(finished(QNetworkReply*)), SLOT(slotAlmanacReceived(QNetworkReply*)));

}

bool Almanac::addTleAlmanac(const QString& twoLineElmentsUrl, const Satellite::Constellation& constellation)
{
    QNetworkRequest request;
    request.setUrl(QUrl(twoLineElmentsUrl));
    request.setAttribute(QNetworkRequest::User, QVariant(static_cast<quint8>(constellation)));
    mNetworkAccessManager->get(request);
}

void Almanac::slotAlmanacReceived(QNetworkReply* reply)
{
    // Parse the data, create satellites.
    quint8 constellationNumber = reply->request().attribute(
                QNetworkRequest::User,
                QVariant(static_cast<quint8>(Satellite::Constellation::ConstellationUnknown))).toInt();

    Satellite::Constellation constellation = static_cast<Satellite::Constellation>(constellationNumber);

    QStringList stringListTle = QString(reply->readAll()).split("\n", QString::SkipEmptyParts);

    for(int i=0;i<=stringListTle.size()-3;i+=3)
    {
        Satellite s(this, stringListTle.at(i+0).simplified(), stringListTle.at(i+1), stringListTle.at(i+2));
        s.setConstellation(constellation);
        s.computeOrbit();
        mSatellites.append(s);
    }

    emit dataChanged();
}

void Almanac::slotComputeOrbits()
{
    for(int i=0;i<mSatellites.size();i++)
    {
        mSatellites[i].computeOrbit();
    }

    emit dataChanged();
}

void Almanac::slotClear()
{
    mSatellites.clear();
    emit dataChanged();
}
