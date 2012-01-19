#include "plotwidget.h"

PlotWidget::PlotWidget(QWidget *parent) // : QDockWidget(parent)
{
    mPlot = new QwtPlot(this);
    mPlot->enableAxis(2, false);
    mPlot->setMinimumSize(160, 120);
    setMinimumSize(160, 120);
    setWidget(mPlot);

    mPlot->setAxisFont(0, QApplication::font());
    mPlot->setAxisFont(1, QApplication::font());

    // initialize X
    mCurveDataX = new QVector<double>(VECTOR_SIZE);
    for(int i=0;i<VECTOR_SIZE;i++) (*mCurveDataX)[i] = i;

    mPlot->canvas()->setLineWidth(0);
    QPalette p;
    p.setColor(QPalette::Window, QColor(60, 60, 60));
    mPlot->canvas()->setPalette(p);

    mPlot->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
    mPlot->legend()->setItemMode(QwtLegend::CheckableItem);

    connect(mPlot, SIGNAL(legendChecked(QwtPlotItem*,bool)), SLOT(slotLegendChecked(QwtPlotItem*,bool)));

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->enableX(false);
//    grid->enableXMin(true);
//    grid->enableYMin(true);
    QList<double> ticks[3];
    ticks[0] << 127;
    ticks[1] << 10;
    ticks[2] << 150;
    grid->setYDiv(QwtScaleDiv(-360.0, 360.0, ticks));
    grid->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    grid->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    grid->attach(mPlot);

    mNumberOfRowsStored = 0;

//    mFile = new QFile(QString("plotwidget-data-%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmss")));
//    if(!mFile->open(QIODevice::WriteOnly | QIODevice::Text))
//        qFatal("Cannot open plotwidget data file for writing");

//    mFileStream = new QTextStream(mFile);

    mPlot->replot();
}

PlotWidget::~PlotWidget()
{
//    mFileStream->flush();
//    delete mFileStream;
//    mFile->close();


//    QMapIterator<QString, QVector<double>* > x(mCurveDataX);
//    while(x.hasNext())
//    {
//        x.next();
//        delete x.value();
//    }

    QMapIterator<QString, QVector<double>* > y(mCurveDataY);
    while(y.hasNext())
    {
        y.next();
        delete y.value();
    }

    QMapIterator<QString, QwtPlotCurve* > c(mCurves);
    while(c.hasNext())
    {
        c.next();
        delete c.value();
    }

    delete mPlot;
}

QwtPlotCurve* PlotWidget::createCurve(const QString& name)
{
    mCurves.insert(name, new QwtPlotCurve(name));
//    mCurveDataX.insert(name, new QVector<double>(VECTOR_SIZE, 0.0));
    mCurveDataY.insert(name, new QVector<double>(VECTOR_SIZE, 0.0));



    QwtPlotCurve* c = mCurves.value(name);

    c->setPaintAttribute(QwtPlotCurve::CacheSymbols);
    c->setPaintAttribute(QwtPlotCurve::ClipPolygons);
    c->setRenderHint(QwtPlotItem::RenderAntialiased, true);

    QMapIterator<QString, QwtPlotCurve* > i(mCurves);
    quint8 currentCurve = 0;
    while(i.hasNext())
    {
         i.next();
         i.value()->setPen(QPen(QColor::fromHsv(currentCurve*255/mCurves.size(), 255, 255)));
         currentCurve++;
    }

    c->setRawSamples(
                mCurveDataX->constData(),
                mCurveDataY.value(name)->constData(),
                VECTOR_SIZE
                );

    c->attach(mPlot);


//    QwtLegendItem* l = (QwtLegendItem*)mPlot->legend()->find(c);
//    if(l != NULL) l->setChecked(true);

    return c;
}

QwtPlotCurve* PlotWidget::getCurve(const QString& name)
{
    if(mCurves.contains(name))
        return mCurves.value(name);
    else
        return createCurve(name);
}

void PlotWidget::slotAppendData(const QVector<float>& data)
{
//    qDebug() << "PlotWidget::slotAppendData(): number of values to append:" << data.size();

    if(data.size() != mCurves.size())
    {
        qWarning() << "PlotWidget::slotAppendData(): plot has" << mCurves.size() << "curves, data contains only" << data.size() << "values.";
        return;
    }

    int column = 0;

//    *mFileStream << QString::number(mNumberOfRowsStored) << ":";

    QMapIterator<QString, QwtPlotCurve* > i(mCurves);
    while(i.hasNext())
    {
         i.next();

//         qDebug() << "now appending to curve" << i.key() << "row" << mNumberOfRowsStored << "value" << data.at(column);

//         QVector<double>* x = mCurveDataX.value(i.key());
         QVector<double>* y = mCurveDataY.value(i.key());

//         (*x)[mNumberOfRowsStored % VECTOR_SIZE] = mNumberOfRowsStored % VECTOR_SIZE;
         (*y)[mNumberOfRowsStored % VECTOR_SIZE] = data.at(column);

//         i.value()->setSamples(*(mCurveDataX.value(i.key())), *(mCurveDataY.value(i.key())));

//         qDebug() << "x:" << mCurveDataX->at(5);
//         qDebug() << "y:" << *y;

         i.value()->setRawSamples(
                     mCurveDataX->constData(),
                     mCurveDataY.value(i.key())->constData(),
                     VECTOR_SIZE
                     );

//         *mFileStream << " " << data.at(column);

         column++;
    }

    mNumberOfRowsStored++;

//    *mFileStream << "\n";

//    mFileStream->flush();
//    mFile->flush();

    if(isVisible()) mPlot->replot();
}

void PlotWidget::slotLegendChecked(QwtPlotItem* plotItem, bool checked)
{
    const QString plotName = plotItem->title().text();

    if(checked)
        mCurves.value(plotName)->hide();
    else
        mCurves.value(plotName)->show();

    mPlot->replot();
}
