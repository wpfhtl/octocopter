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
}

ConnectionDialog::~ConnectionDialog()
{
    delete ui;
}

QString ConnectionDialog::getHostNameRover() const
{
    return ui->mLineEditHostNameRover->text();
}

QString ConnectionDialog::getHostNameRtkBase() const
{
    return ui->mLineEditHostNameRtkBase->text();
}
