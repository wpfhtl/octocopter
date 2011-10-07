#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QtGui>

class CameraWidget : public QDockWidget
{
    Q_OBJECT

    QLabel* mLabel;
    QPixmap* mPixmap;

public:
    CameraWidget(QWidget *parent = 0, const QString &title = QString());

private slots:
    void slotSetDimensions(const QSize size);
public slots:
    void slotSetPixmapData(const QByteArray &data);

};

#endif
