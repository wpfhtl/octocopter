#include "connectiondialog.h"
#include <QCompleter>
#include "ui_connectiondialog.h"

ConnectionDialog::ConnectionDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectionDialog)
{
    ui->setupUi(this);

    QStringList hostNames;
    hostNames << "kopter" << "localhost" << "192.168.1.1" << "tams9" << "134.100.13.169" << "80.153.240.55";

    QCompleter *completer = new QCompleter(hostNames, this);
    completer->setCaseSensitivity(Qt::CaseInsensitive);

    ui->mLineEditHostNameRover->setCompleter(completer);
    ui->mLineEditHostNameRtkBase->setCompleter(completer);

    ui->mLineEditHostNameRover->selectAll();
}

ConnectionDialog::~ConnectionDialog()
{
    delete ui;
}

void ConnectionDialog::keyPressEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
        ui->mPushButtonConnect->click();
    else if(event->key() == Qt::Key_Escape)
        ui->mPushButtonWorkOffline->click();
}

QString ConnectionDialog::getRoverHostName() const
{
    return ui->mLineEditHostNameRover->text();
}

QString ConnectionDialog::getRtkBaseHostName() const
{
    return ui->mLineEditHostNameRtkBase->text();
}

quint16 ConnectionDialog::getRoverPort() const
{
    return ui->mSpinBoxPortRover->value();
}

quint16 ConnectionDialog::getRtkBasePort() const
{
    return ui->mSpinBoxPortRtkBase->value();
}
