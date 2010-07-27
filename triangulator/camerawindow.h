#ifndef CAMERAWINDOW_H
#define CAMERAWINDOW_H

#include <QtGui>

class CameraWindow : public QDialog
{
    Q_OBJECT

    QLabel* mLabel;
    QPixmap* mPixmap;

public:
    CameraWindow(QWidget *parent = 0);

private slots:
    void slotSetDimensions(const QSize size);
public slots:
    void slotSetPixmapData(const QByteArray &data);

};

#endif
