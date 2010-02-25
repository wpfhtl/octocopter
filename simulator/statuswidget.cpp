#include "statuswidget.h"

StatusWidget::StatusWidget(QWidget *parent, Battery* battery) : QDockWidget(parent)
{
    setupUi(this);

    mBattery = battery;

    connect(mBattery, SIGNAL(chargeStatusChanged(int)), SLOT(slotUpdateBattery(int)));

    mLabelBatteryVoltage->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
    mLabelBatteryEnergy->setText(QString::number(mBattery->capacity(), 'g', 2) + " Ah");
}

void StatusWidget::slotUpdateBattery(const int chargeStateInPercent)
{
    mLabelBatteryVoltageCurrent->setText(QString::number(mBattery->voltage(), 'g', 2) + " V");
    mLabelBatteryEnergyCurrent->setText(QString::number(mBattery->energy(), 'g', 2) + " Ah");
}

void StatusWidget::slotUpdateVisualization(QSize windowSize, int triangles, float fps)
{
    mLabelVisualizationSize->setText(QString::number(windowSize.width()) + " x " + QString::number(windowSize.height()));
    mLabelVisualizationTriangles->setText(QString::number(triangles));
    mLabelVisualizationFramerate->setText(QString::number(fps, 'g', 2));
}
