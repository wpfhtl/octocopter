#ifndef PIDCONTROLLERWIDGET_H
#define PIDCONTROLLERWIDGET_H

#include <QDockWidget>
#include <QMessageBox>
#include <QString>
#include <QMap>

#include <pidcontroller.h>

namespace Ui {
class PidControllerWidget;
}

class PidControllerWidget : public QDockWidget
{
    Q_OBJECT
    
public:
    PidControllerWidget(QWidget *parent = 0);
    ~PidControllerWidget();

    QWidget* mParent;
    QMap<QString, PidController> mControllers;

    void setWeights(QMap<QString, PidController> controllers);

    bool isPopulated() const {return mPopulated;}

signals:
    void controllerWeight(QString, QMap<QString, float>);
    
private:
    Ui::PidControllerWidget *ui;
    bool mPopulated;
    void buildTable();

private slots:
    void slotWeightChanged(int row, int column);
};

#endif
