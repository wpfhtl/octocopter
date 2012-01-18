#ifndef LOGPLAYER_H
#define LOGPLAYER_H

#include <QDockWidget>

namespace Ui {
    class LogPlayer;
}

class LogPlayer : public QDockWidget
{
    Q_OBJECT

public:
    explicit LogPlayer(QWidget *parent = 0);
    ~LogPlayer();

private:
    Ui::LogPlayer *ui;
};

#endif // LOGPLAYER_H
