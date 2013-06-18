#ifndef SATELLITEWIDGET_H
#define SATELLITEWIDGET_H

#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QTimer>
#include "ui_satellitewidget.h"
#include <common.h>
#include <almanac.h>

class SatelliteWidget : public QDockWidget, public Ui::SatelliteWidget
{
Q_OBJECT

    Almanac* mAlmanac;
    QTimer* mTimerUpdate;
    QGraphicsScene* mGraphicsScene;

    QPointF getPolarCoordinate(const Satellite& sat) const;
    QPointF transformToView(const QPointF& in) const;
    QRectF transformToView(const QRectF& in) const;
    float transformToView(const float& in) const;

public:
    SatelliteWidget(QWidget *widget);
    ~SatelliteWidget();

    void resizeEvent(QResizeEvent*);

public slots:
    void slotFetchTles();
    void slotUpdateTable();
    void slotUpdatePolarPlot();
    void slotSetReceiverPosition(const PositionGeodetic& pos);

private slots:
    void slotShowMessage(const QString text) {mLabelSatelliteInfo->setText(text);}
signals:
};


class GraphicSatelliteItem : public QGraphicsTextItem
{
    Q_OBJECT

private:
    Satellite* mSatellite;
    SatelliteWidget* mSatelliteWidget;
public:

    GraphicSatelliteItem(Satellite* sat, SatelliteWidget* widget, QGraphicsItem * parent = 0) : QGraphicsTextItem(sat->getShortName(), parent)
    {
        mSatellite = sat;
        mSatelliteWidget = widget;
        setDefaultTextColor(mSatellite->getTextColor());
        setAcceptHoverEvents(true);
    }

    void hoverEnterEvent(QGraphicsSceneHoverEvent * event)
    {
        emit message(QString("%1 Az %2 El %3").arg(mSatellite->getName()).arg(mSatellite->getAzimuth(), 0, 'f', 2).arg(mSatellite->getElevation(), 0, 'f', 2));
    }

    void hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
    {
        emit message(QString());
    }
signals:
    void message(QString);
};

#endif
