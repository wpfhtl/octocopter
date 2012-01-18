#include "logplayer.h"
#include "ui_logplayer.h"

LogPlayer::LogPlayer(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::LogPlayer)
{
    ui->setupUi(this);
    resize(minimumSizeHint());
}

LogPlayer::~LogPlayer()
{
    delete ui;
}
