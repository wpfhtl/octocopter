#ifndef CONNECTIONDIALOG_H
#define CONNECTIONDIALOG_H

#include <QDialog>
#include <QKeyEvent>

namespace Ui {
    class ConnectionDialog;
}

class ConnectionDialog : public QDialog
{
    Q_OBJECT

public:
    ConnectionDialog(QWidget *parent = 0);
    ~ConnectionDialog();

    QString getRoverHostName() const;
    quint16 getRoverPort() const;
    QString getRtkBaseHostName() const;
    quint16 getRtkBasePort() const;

private:
    Ui::ConnectionDialog *ui;

    void keyPressEvent(QKeyEvent* event);
};

#endif // CONNECTIONDIALOG_H
