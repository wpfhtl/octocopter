#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QApplication>
#include <QMap>
#include <QFile>
#include <QLayout>
#include <QString>
#include <QTextStream>
#include <QPalette>
#include <QDateTime>
#include <QDockWidget>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_legend_item.h>

#define VECTOR_SIZE 500

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
    QMap<QString, QVector<double>* > mCurveDataY;//, mCurveDataY;
    QVector<double>* mCurveDataX;

private slots:
    void slotLegendChecked(QwtPlotItem* plotItem, bool checked);

public slots:
    void slotAppendData(const QVector<float>&);
};

#endif
