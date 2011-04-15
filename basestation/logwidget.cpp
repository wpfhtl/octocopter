#include "logwidget.h"

LogWidget::LogWidget(QWidget* widget)
{
    setupUi(this);
    mWidget = widget;
    mTextEdit->setFontPointSize(7.5);
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

void LogWidget::log(const QString &text)
{
    mTextEdit->append(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz") + "\t" + text);
}
