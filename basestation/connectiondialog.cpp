#include "connectiondialog.h"
#include <QCompleter>
#include "ui_connectiondialog.h"

ConnectionDialog::ConnectionDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectionDialog)
{
    ui->setupUi(this);

    QStringList hostNames;
    hostNames << "atomboard.dyndns.org" << "localhost" << "192.168.1.2";

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
