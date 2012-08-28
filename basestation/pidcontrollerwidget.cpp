#include "pidcontrollerwidget.h"
#include "ui_pidcontrollerwidget.h"

PidControllerWidget::PidControllerWidget(QWidget *parent) :
    QDockWidget(parent),
    mParent(parent),
    ui(new Ui::PidControllerWidget)
{
    ui->setupUi(this);
    ui->mTableControllerWeights->setSortingEnabled(false);

    mPopulated = false;
}

PidControllerWidget::~PidControllerWidget()
{
    delete ui;
}

void PidControllerWidget::setControllers(const FlightControllerValues* const fcv)
{
    mControllers.clear();
    mControllers.insert("thrust", &fcv->controllerThrust);
    mControllers.insert("yaw", &fcv->controllerYaw);
    mControllers.insert("pitch", &fcv->controllerPitch);
    mControllers.insert("roll", &fcv->controllerRoll);

    mPopulated = false;
}

void PidControllerWidget::slotRebuild()
{
    disconnect(ui->mTableControllerWeights, SIGNAL(cellChanged(int,int)), this, SLOT(slotWeightChanged(int,int)));

    ui->mTableControllerWeights->clear();

    if(mControllers.size())
    {
        ui->mTableControllerWeights->setRowCount(mControllers.size());
        ui->mTableControllerWeights->setColumnCount(3);

        ui->mTableControllerWeights->setHorizontalHeaderItem(0, new QTableWidgetItem("p"));
        ui->mTableControllerWeights->setHorizontalHeaderItem(1, new QTableWidgetItem("i"));
        ui->mTableControllerWeights->setHorizontalHeaderItem(2, new QTableWidgetItem("d"));

        QMapIterator<QString, const PidController*> i(mControllers);

        int row = 0;
        while(i.hasNext())
        {
            i.next();

            ui->mTableControllerWeights->setVerticalHeaderItem(row, new QTableWidgetItem(i.key()));

            ui->mTableControllerWeights->setItem(row, 0, new QTableWidgetItem(QString::number(i.value()->getWeightP(), 'f', 2)));
            ui->mTableControllerWeights->setItem(row, 1, new QTableWidgetItem(QString::number(i.value()->getWeightI(), 'f', 2)));
            ui->mTableControllerWeights->setItem(row, 2, new QTableWidgetItem(QString::number(i.value()->getWeightD(), 'f', 2)));
            row++;
        }
    }

    connect(ui->mTableControllerWeights, SIGNAL(cellChanged(int,int)), SLOT(slotWeightChanged(int,int)));

    mPopulated = true;
}

void PidControllerWidget::slotWeightChanged(int row, int column)
{
    bool success = true;

    const QString name = ui->mTableControllerWeights->verticalHeaderItem(row)->text();

    const float p = ui->mTableControllerWeights->item(row,0)->text().toFloat(&success);

    if(!success)
    {
        QMessageBox::warning(mParent, "Value is not a number!", "The value could not be converted to a number!");
        return;
    }

    const float i = ui->mTableControllerWeights->item(row,1)->text().toFloat(&success);

    if(!success)
    {
        QMessageBox::warning(mParent, "Value is not a number!", "The value could not be converted to a number!");
        return;
    }

    const float d = ui->mTableControllerWeights->item(row,2)->text().toFloat(&success);

    if(!success)
    {
        QMessageBox::warning(mParent, "Value is not a number!", "The value could not be converted to a number!");
        return;
    }

    QMap<QString, float> weights;

    weights.insert("p", p);
    weights.insert("i", i);
    weights.insert("d", d);

//    qDebug() << "PidControllerWidget::slotWeightChanged(): controller" << name << "changed to" << p << i << d;

    emit controllerWeight(name, weights);

    // So that the table will be rebuilt/acknowledged
//    mPopulated = false;
}
