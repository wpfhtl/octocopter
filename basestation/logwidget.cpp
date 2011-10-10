#include "logwidget.h"

LogWidget::LogWidget(QWidget* widget)
{
    setupUi(this);
    mWidget = widget;
    //setTitleBarWidget(new QWidget());
}

void LogWidget::save()
{
    QString fileName = QFileDialog::getSaveFileName(mWidget, tr("Save log"), QString(), tr("Log Files (*.txt *.log)"));

    if(fileName.isNull()) return;

    QFile file(fileName);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::critical(mWidget, "Cannot write to file", "File is not writable, failed.");
        return;
    }

    QTextStream out(&file);
    out << mTextEdit->toPlainText();
}

void LogWidget::clear()
{
    mTextEdit->clear();
}

void LogWidget::log(const LogImportance& importance, const QString& source, const QString& text)
{
    mTextEdit->append(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz") + QString(" %1 %2\t%3").arg(importance).arg(source).arg(text));
}
