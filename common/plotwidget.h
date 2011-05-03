#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QMap>
#include <QFile>
#include <QLayout>
#include <QString>
#include <QDateTime>
#include <QDockWidget>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

class PlotWidget : public QDockWidget
{
    Q_OBJECT

public:
    PlotWidget(QWidget *parent = 0);
    ~PlotWidget();

    QwtPlotCurve* createCurve(const QString& name = "unknown");
    QwtPlotCurve* getCurve(const QString& name);

private:
    QFile* mFile;
    QTextStream* mFileStream;
    QwtPlot* mPlot;
    quint32 mNumberOfRowsStored;
    QMap<QString, QwtPlotCurve*> mCurves;
    QMap<QString, QVector<double>* > mCurveDataX, mCurveDataY;

public slots:
    void slotAppendData(const QVector<float>&);
};

#endif
