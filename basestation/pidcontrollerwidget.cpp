#include "pidcontrollerwidget.h"
#include "ui_pidcontrollerwidget.h"

PidControllerWidget::PidControllerWidget(QWidget *parent) :
    QDockWidget(parent),
    mParent(parent),
    ui(new Ui::PidControllerWidget)
{
    ui->setupUi(this);
//    ui->mTableControllerWeights->setSortingEnabled(false);
    ui->mTableControllerValues->setSortingEnabled(false);

    connect(ui->mTableControllerValues, SIGNAL(cellActivated(int,int)), SLOT(slotCellActivated(int,int)));
    connect(ui->mTableControllerValues, SIGNAL(cellChanged(int,int)), SLOT(slotCellChanged(int,int)));

    mActiveItem = 0;

    mPopulated = false;
}

PidControllerWidget::~PidControllerWidget()
{
    delete ui;
}

//void PidControllerWidget::setEnabled(const bool enabled)
//{
//    ui->mTableControllerWeights->setEnabled(enabled);
//
//}

void PidControllerWidget::setControllers(const FlightControllerValues* const fcv)
{
    mControllers.clear();
    mControllers.insert("thrust", &fcv->controllerThrust);
    mControllers.insert("yaw", &fcv->controllerYaw);
    mControllers.insert("pitch", &fcv->controllerPitch);
    mControllers.insert("roll", &fcv->controllerRoll);
/*
    // Build the weights table
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
*/
    // Build the controllervalues table

    disconnect(ui->mTableControllerValues, SIGNAL(cellChanged(int,int)), this, SLOT(slotCellChanged(int,int)));

    ui->mTableControllerValues->clear();

    ui->mTableControllerValues->setRowCount(mControllers.size());
    ui->mTableControllerValues->setColumnCount(14);

    ui->mTableControllerValues->setHorizontalHeaderItem(0, new QTableWidgetItem("p"));
    ui->mTableControllerValues->setHorizontalHeaderItem(1, new QTableWidgetItem("i"));
    ui->mTableControllerValues->setHorizontalHeaderItem(2, new QTableWidgetItem("d"));

    ui->mTableControllerValues->setHorizontalHeaderItem(3, new QTableWidgetItem("tim"));
    ui->mTableControllerValues->setHorizontalHeaderItem(4, new QTableWidgetItem("des"));
    ui->mTableControllerValues->setHorizontalHeaderItem(5, new QTableWidgetItem("val"));
    ui->mTableControllerValues->setHorizontalHeaderItem(6, new QTableWidgetItem("err"));
    ui->mTableControllerValues->setHorizontalHeaderItem(7, new QTableWidgetItem("prverr"));
    ui->mTableControllerValues->setHorizontalHeaderItem(8, new QTableWidgetItem("der"));
    ui->mTableControllerValues->setHorizontalHeaderItem(9, new QTableWidgetItem("int"));
    ui->mTableControllerValues->setHorizontalHeaderItem(10, new QTableWidgetItem("outp"));
    ui->mTableControllerValues->setHorizontalHeaderItem(11, new QTableWidgetItem("outi"));
    ui->mTableControllerValues->setHorizontalHeaderItem(12, new QTableWidgetItem("outd"));
    ui->mTableControllerValues->setHorizontalHeaderItem(13, new QTableWidgetItem("out"));

    QMapIterator<QString, const PidController*> i(mControllers);

    int row = 0;
    while(i.hasNext())
    {
        i.next();
        ui->mTableControllerValues->setVerticalHeaderItem(row, new QTableWidgetItem(i.key()));

        ui->mTableControllerValues->setItem(row, 0, new QTableWidgetItem(QString::number(i.value()->getWeightP(), 'f', 2)));
        ui->mTableControllerValues->setItem(row, 1, new QTableWidgetItem(QString::number(i.value()->getWeightI(), 'f', 2)));
        ui->mTableControllerValues->setItem(row, 2, new QTableWidgetItem(QString::number(i.value()->getWeightD(), 'f', 2)));

        ui->mTableControllerValues->setItem(row, 3, new QTableWidgetItem(QString::number(i.value()->getTimeDiff(), 'f', 2)));
        ui->mTableControllerValues->item(row, 3)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 4, new QTableWidgetItem(QString::number(i.value()->getValueDesired(), 'f', 2)));
        ui->mTableControllerValues->item(row, 4)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 5, new QTableWidgetItem(QString::number(i.value()->getValue(), 'f', 2)));
        ui->mTableControllerValues->item(row, 5)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 6, new QTableWidgetItem(QString::number(i.value()->getError(), 'f', 2)));
        ui->mTableControllerValues->item(row, 6)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 7, new QTableWidgetItem(QString::number(i.value()->getErrorPrevious(), 'f', 2)));
        ui->mTableControllerValues->item(row, 7)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 8, new QTableWidgetItem(QString::number(i.value()->getDerivative(), 'f', 2)));
        ui->mTableControllerValues->item(row, 8)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 9, new QTableWidgetItem(QString::number(i.value()->getIntegral(), 'f', 2)));
        ui->mTableControllerValues->item(row, 9)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

        ui->mTableControllerValues->setItem(row, 10, new QTableWidgetItem(QString::number(i.value()->getOutputP(), 'f', 2)));
        ui->mTableControllerValues->item(row, 10)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 11, new QTableWidgetItem(QString::number(i.value()->getOutputI(), 'f', 2)));
        ui->mTableControllerValues->item(row, 11)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 12, new QTableWidgetItem(QString::number(i.value()->getOutputD(), 'f', 2)));
        ui->mTableControllerValues->item(row, 12)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        ui->mTableControllerValues->setItem(row, 13, new QTableWidgetItem(QString::number(i.value()->getLastOutput(), 'f', 2)));
        ui->mTableControllerValues->item(row, 13)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

        row++;
    }

    connect(ui->mTableControllerValues, SIGNAL(cellChanged(int,int)), this, SLOT(slotCellChanged(int,int)));

//    ui->mTableControllerValues->resizeColumnsToContents();
//    ui->mTableControllerValues->resizeRowsToContents();

    slotUpdateValues();

    mPopulated = false;
}

void PidControllerWidget::slotUpdateValues()
{
    QMapIterator<QString, const PidController*> i(mControllers);

    disconnect(ui->mTableControllerValues, SIGNAL(cellChanged(int,int)), this, SLOT(slotCellChanged(int,int)));

    int row = 0;
    while(i.hasNext())
    {
        i.next();
        QTableWidgetItem* item;
        item = ui->mTableControllerValues->item(row, 0);
        if(mActiveItem != item) item->setText(QString::number(i.value()->getWeightP(), 'f', 2));
        item = ui->mTableControllerValues->item(row, 1);
        if(mActiveItem != item) item->setText(QString::number(i.value()->getWeightI(), 'f', 2));
        item = ui->mTableControllerValues->item(row, 2);
        if(mActiveItem != item) item->setText(QString::number(i.value()->getWeightD(), 'f', 2));

        ui->mTableControllerValues->item(row, 3)->setText(QString::number(i.value()->getTimeDiff(), 'f', 2));
        ui->mTableControllerValues->item(row, 4)->setText(QString::number(i.value()->getValueDesired(), 'f', 2));
        ui->mTableControllerValues->item(row, 5)->setText(QString::number(i.value()->getValue(), 'f', 2));
        ui->mTableControllerValues->item(row, 6)->setText(QString::number(i.value()->getError(), 'f', 2));
        ui->mTableControllerValues->item(row, 7)->setText(QString::number(i.value()->getErrorPrevious(), 'f', 2));
        ui->mTableControllerValues->item(row, 8)->setText(QString::number(i.value()->getDerivative(), 'f', 2));
        ui->mTableControllerValues->item(row, 9)->setText(QString::number(i.value()->getIntegral(), 'f', 2));

        ui->mTableControllerValues->item(row, 10)->setText(QString::number(i.value()->getOutputP(), 'f', 2));
        ui->mTableControllerValues->item(row, 11)->setText(QString::number(i.value()->getOutputI(), 'f', 2));
        ui->mTableControllerValues->item(row, 12)->setText(QString::number(i.value()->getOutputD(), 'f', 2));
        ui->mTableControllerValues->item(row, 13)->setText(QString::number(i.value()->getLastOutput(), 'f', 2));
        row++;
    }

    connect(ui->mTableControllerValues, SIGNAL(cellChanged(int,int)), this, SLOT(slotCellChanged(int,int)));
}

/*
void PidControllerWidget::slotUpdateWeights()
{
    disconnect(ui->mTableControllerWeights, SIGNAL(cellChanged(int,int)), this, SLOT(slotWeightChanged(int,int)));

//    ui->mTableControllerWeights->clear();

    if(mControllers.size())
    {
//        ui->mTableControllerWeights->setRowCount(mControllers.size());
//        ui->mTableControllerWeights->setColumnCount(3);

//        ui->mTableControllerWeights->setHorizontalHeaderItem(0, new QTableWidgetItem("p"));
//        ui->mTableControllerWeights->setHorizontalHeaderItem(1, new QTableWidgetItem("i"));
//        ui->mTableControllerWeights->setHorizontalHeaderItem(2, new QTableWidgetItem("d"));

        QMapIterator<QString, const PidController*> i(mControllers);

        int row = 0;
        while(i.hasNext())
        {
            i.next();

//            ui->mTableControllerWeights->setVerticalHeaderItem(row, new QTableWidgetItem(i.key()));

            ui->mTableControllerWeights->item(row, 0)->setText(QString::number(i.value()->getWeightP(), 'f', 2));
            ui->mTableControllerWeights->item(row, 1)->setText(QString::number(i.value()->getWeightI(), 'f', 2));
            ui->mTableControllerWeights->item(row, 2)->setText(QString::number(i.value()->getWeightD(), 'f', 2));
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
}*/

void PidControllerWidget::slotCellActivated(const int& row, const int& column)
{
    mActiveItem = ui->mTableControllerValues->item(row, column);
}

void PidControllerWidget::slotCellChanged(const int& row, const int& column)
{
    Q_ASSERT(mActiveItem = ui->mTableControllerValues->item(row, column));

    bool success = true;

    const QString name = ui->mTableControllerValues->verticalHeaderItem(row)->text();

    const float p = ui->mTableControllerValues->item(row,0)->text().toFloat(&success);

    if(!success)
    {
        QMessageBox::warning(mParent, "Value is not a number!", "The value could not be converted to a number!");
        return;
    }

    const float i = ui->mTableControllerValues->item(row,1)->text().toFloat(&success);

    if(!success)
    {
        QMessageBox::warning(mParent, "Value is not a number!", "The value could not be converted to a number!");
        return;
    }

    const float d = ui->mTableControllerValues->item(row,2)->text().toFloat(&success);

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
}
