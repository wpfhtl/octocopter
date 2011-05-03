#include "plotwidget.h"

PlotWidget::PlotWidget(QWidget *parent)// : QDockWidget(parent)
{
    mPlot = new QwtPlot(this);
    mPlot->enableAxis(2, false);
    setWidget(mPlot);

    mNumberOfRowsStored = 0;

    mFile = new QFile(QString("plotwidget-data-%1.txt").arg(QDateTime::currentDateTime().toString("YYYYmmdd-hhiiss")));
    if(!mFile->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("Cannot open plotwidget data file for writing");

    mFileStream = new QTextStream(mFile);
}

PlotWidget::~PlotWidget()
{
    mFileStream->flush();
    delete mFileStream;
    mFile->close();


    QMapIterator<QString, QVector<double>* > x(mCurveDataX);
    while(x.hasNext())
    {
        x.next();
        delete x.value();
    }

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
    mCurveDataX.insert(name, new QVector<double>(1000, 0.0));
    mCurveDataY.insert(name, new QVector<double>(1000, 0.0));

    QwtPlotCurve* c = mCurves.value(name);

    c->attach(mPlot);

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
    if(data.size() != mCurves.size())
    {
        qWarning() << "PlotWidget::slotAppendData(): plot has" << mCurves.size() << "curves, data contains only" << data.size() << "values.";
        return;
    }

    int column = 0;

    *mFileStream << QString::number(mNumberOfRowsStored) << ":";

    QMapIterator<QString, QwtPlotCurve* > i(mCurves);
    while(i.hasNext())
    {
         i.next();

         mCurveDataX.value(i.key())->append(mNumberOfRowsStored);
         mCurveDataY.value(i.key())->append(data.at(column));

         i.value()->setRawSamples(
                     mCurveDataX.value(i.key())->data(),
                     mCurveDataY.value(i.key())->data(),
                     mNumberOfRowsStored
                     );

         *mFileStream << " " << data.at(column);

         column++;
         mNumberOfRowsStored++;
    }

    *mFileStream << "\n";

    mPlot->replot();
}
