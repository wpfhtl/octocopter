#ifndef LOGWIDGET_H
#define LOGWIDGET_H

#include <QtGui>
#include <common.h>
#include "ui_logwidget.h"

class LogWidget : public QDockWidget, public Ui::LogWidget
{

    Q_OBJECT

private:
    QWidget* mWidget;

public:
    LogWidget(QWidget* widget);
//    ~LogWidget();

public slots:
    void log(const LogImportance& importance, const QString& source, const QString& text);
    void save();
    void clear();

private slots:

signals:

};

#endif
