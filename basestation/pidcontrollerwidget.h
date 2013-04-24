#ifndef PIDCONTROLLERWIDGET_H
#define PIDCONTROLLERWIDGET_H

#include <QDockWidget>
#include <QMessageBox>
#include <QString>
#include <QResizeEvent>
#include <QTableWidgetItem>
#include <QMap>

#include <flightcontrollervalues.h>

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
    QMap<QString, const PidController*> mControllers;

    void setControllers(const FlightControllerValues *const fcv);
//    bool isPopulated() const {return mPopulated;}
    void setEnabled(bool enabled);

signals:
    // signals that a weight for a single controller was changed. Transmits all the weights.
    void controllerWeight(QString, QMap<QChar, float>);
    
private:
    Ui::PidControllerWidget *ui;
    bool mPopulated;

    // We don't want to overwrite the values of a cell thats currently being edited by the user.
    QTableWidgetItem* mActiveItem;
    void resizeEvent(QResizeEvent * event);

private slots:
//    void slotWeightChanged(int row, int column);
    void slotCellActivated(const int& row, const int& column);
    void slotCellChanged(const int& row, const int& column);

public slots:
//    void setEnabled(const bool enabled);
//    void slotUpdateWeights();
    void slotUpdateValues();
};

#endif
