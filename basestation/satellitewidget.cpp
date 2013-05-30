#include "satellitewidget.h"
#include <QTableWidgetItem>

SatelliteWidget::SatelliteWidget(QWidget* widget) : QDockWidget(widget)
{
    setupUi(this);

    mTableWidget->setColumnCount(3);

    mGraphicsScene = new QGraphicsScene(this);

    mPolarPlot->setScene(mGraphicsScene);
//    mPolarPlot->setMouseTracking(true);
//    setMouseTracking(true);
    mPolarPlot->setBackgroundBrush(QBrush());
//    mPolarPlot->setSizeIncrement(1,1);
//    mPolarPlot->setBaseSize(150,150);

    mAlmanac = new Almanac(this);

    mTimerUpdate = new QTimer(this);
    connect(mTimerUpdate, SIGNAL(timeout()), SLOT(slotUpdateTable()));
    connect(mTimerUpdate, SIGNAL(timeout()), SLOT(slotUpdatePolarPlot()));
    mTimerUpdate->start(10000);

    connect(mBtnFetchTles, SIGNAL(clicked()), SLOT(slotFetchTles()));

    connect(mAlmanac, SIGNAL(dataChanged()), SLOT(slotUpdateTable()));
    connect(mAlmanac, SIGNAL(dataChanged()), SLOT(slotUpdatePolarPlot()));

    connect(mBtnEnableGps, SIGNAL(clicked()), SLOT(slotUpdateTable()));
    connect(mBtnEnableGps, SIGNAL(clicked()), SLOT(slotUpdatePolarPlot()));
    connect(mBtnEnableGlo, SIGNAL(clicked()), SLOT(slotUpdateTable()));
    connect(mBtnEnableGlo, SIGNAL(clicked()), SLOT(slotUpdatePolarPlot()));
    connect(mBtnEnableGal, SIGNAL(clicked()), SLOT(slotUpdateTable()));
    connect(mBtnEnableGal, SIGNAL(clicked()), SLOT(slotUpdatePolarPlot()));

    slotFetchTles();
}

SatelliteWidget::~SatelliteWidget()
{

}

void SatelliteWidget::resizeEvent(QResizeEvent*)
{
    mPolarPlot->fitInView(mGraphicsScene->sceneRect(), Qt::KeepAspectRatio);
}

void SatelliteWidget::slotFetchTles()
{
    qDebug() << "fetching TLEs from celestrak.com...";
    mAlmanac->slotClear();
    mAlmanac->addTleAlmanac("http://celestrak.com/NORAD/elements/gps-ops.txt", Satellite::Constellation::ConstellationGps);
    mAlmanac->addTleAlmanac("http://celestrak.com/NORAD/elements/glo-ops.txt", Satellite::Constellation::ConstellationGlonass);
    mAlmanac->addTleAlmanac("http://celestrak.com/NORAD/elements/galileo.txt", Satellite::Constellation::ConstellationGalileo);
}

void SatelliteWidget::slotUpdateTable()
{
    mTableWidget->reset();

    QFontMetrics fm(fontMetrics());
//    mTableWidget->setRowHeight(0, fm.height());
//    mTableWidget->setColumnWidth(0, fm.width("GALILEO-PFM (GSAT0101)__"));
//    mTableWidget->setColumnWidth(1, fm.width("360.00__"));
//    mTableWidget->setColumnWidth(2, fm.width("-90.00__"));


    const PositionGeodetic pos = mAlmanac->getReceiverPosition();
    mLabelReceiverPosLon->setText(QString::number(pos.longitude, 'f', 2));
    mLabelReceiverPosLat->setText(QString::number(pos.latitude, 'f', 2));
    mLabelReceiverPosHeight->setText(QString::number(pos.elevation, 'f', 2));

    QList<Satellite>* satellites = mAlmanac->getSatellites();
    quint32 satelliteNumber = 0;
    quint32 satelliteCounter = 0;

    for(int i=0;i<satellites->size();i++)
    {
        Satellite& s = (*satellites)[i];

        if(!mBtnEnableGps->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGps) continue;
        if(!mBtnEnableGlo->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGlonass) continue;
        if(!mBtnEnableGal->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGalileo) continue;

        satelliteCounter++;
    }

    mTableWidget->setRowCount(satelliteCounter);

    for(int i=0;i<satellites->size();i++)
    {
        Satellite& s = (*satellites)[i];

        if(!mBtnEnableGps->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGps) continue;
        if(!mBtnEnableGlo->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGlonass) continue;
        if(!mBtnEnableGal->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGalileo) continue;

        s.computeOrbit();
        QTableWidgetItem *item;
        item = new QTableWidgetItem(s.getShortName());
        if(s.getElevation() < 0.0f) item->setForeground(QBrush(QColor(128,128,128)));
        mTableWidget->setItem(satelliteNumber, 0, item);
        item = new QTableWidgetItem(QString::number(s.getAzimuth(), 'f', 2));
        item->setTextAlignment(Qt::AlignRight);
        mTableWidget->setItem(satelliteNumber, 1, item);
        item = new QTableWidgetItem(QString::number(s.getElevation(), 'f', 2));
        item->setTextAlignment(Qt::AlignRight);
        mTableWidget->setItem(satelliteNumber, 2, item);

        satelliteNumber++;
    }
    resizeEvent(0);
    mTableWidget->resizeColumnsToContents();
    mTableWidget->resizeRowsToContents();
}

float SatelliteWidget::transformToView(const float& in) const
{
    // in.x and in.y have range [-1,1]. Expand that to mPolarPlot's range
    return 0.99 * in * mPolarPlot->viewport()->size().width() / 2.0f;
}

QPointF SatelliteWidget::transformToView(const QPointF& in) const
{
    return QPointF(transformToView(in.x()), transformToView(in.y()));
}

QRectF SatelliteWidget::transformToView(const QRectF& in) const
{
    return QRectF(transformToView(in.topLeft()), transformToView(in.bottomRight()));
}

void SatelliteWidget::slotUpdatePolarPlot()
{
    mGraphicsScene->clear();

    mGraphicsScene->addEllipse(transformToView(QRectF(-1.00, -1.00, 2.00, 2.00)))->setPen(QPen(QColor(200,200,200)));
    mGraphicsScene->addEllipse(transformToView(QRectF(-0.66, -0.66, 1.33, 1.33)))->setPen(QPen(QColor(200,200,200)));
    mGraphicsScene->addEllipse(transformToView(QRectF(-0.33, -0.33, 0.66, 0.66)))->setPen(QPen(QColor(200,200,200)));
//    mGraphicsScene->addEllipse(transformToView(QRectF(-0.25, -0.25, 0.5, 0.5)))->setPen(QPen(QColor(200,200,200)));

    mGraphicsScene->addLine(
                transformToView(0),
                transformToView(-1),
                transformToView(0),
                transformToView(1))->setPen(QPen(QColor(200,200,200)));

    mGraphicsScene->addLine(
                transformToView(-1),
                transformToView(0),
                transformToView(1),
                transformToView(0))->setPen(QPen(QColor(200,200,200)));

    QList<Satellite>* satellites = mAlmanac->getSatellites();
    for(int i=0;i<satellites->size();i++)
    {
        Satellite& s = (*satellites)[i];

        if(!mBtnEnableGps->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGps) continue;
        if(!mBtnEnableGlo->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGlonass) continue;
        if(!mBtnEnableGal->isChecked() && s.getConstellation() == Satellite::Constellation::ConstellationGalileo) continue;

        if(s.getElevation() > 0.0f)
        {
            GraphicSatelliteItem* item = new GraphicSatelliteItem(&s, this);
            connect(item, SIGNAL(message(QString)), SLOT(slotShowMessage(QString)));
            mGraphicsScene->addItem(item);
//            QGraphicsTextItem* satName = mGraphicsScene->addText(s.getShortName());

            QPointF textPos = transformToView(getPolarCoordinate(s));
            textPos.setX(textPos.x() - item->boundingRect().width()*0.5f);
            textPos.setY(textPos.y() - item->boundingRect().height()*0.55f);

            textPos.setX(
                        qBound(
                            -mPolarPlot->viewport()->size().width()/2.0,
                            textPos.x(),
                            mPolarPlot->viewport()->size().width()/2 - item->boundingRect().width()
                            )
                        );

            textPos.setY(
                        qBound(
                            -mPolarPlot->viewport()->size().height() - 15.0,
                            textPos.y(),
                            mPolarPlot->viewport()->size().height()/2 - item->boundingRect().height()
                            )
                        );

            item->setPos(textPos);
        }
    }

    resizeEvent(0);
}

QPointF SatelliteWidget::getPolarCoordinate(const Satellite& sat) const
{
    Q_ASSERT(sat.getElevation() > 0.0f && sat.getElevation() <= 90.0f);

    float distanceFromCenter = 1 - (sat.getElevation() / 90.0f);

    float x = sin(DEG2RAD(sat.getAzimuth())) * distanceFromCenter;
    float y = cos(DEG2RAD(sat.getAzimuth())) * distanceFromCenter;
    return QPointF(x, -y);
}

void SatelliteWidget::slotSetReceiverPosition(const PositionGeodetic& pos)
{
    mAlmanac->setReceiverPosition(pos);
    slotUpdateTable();
}

