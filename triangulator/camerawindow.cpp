#include "camerawindow.h"

CameraWindow::CameraWindow(QWidget *parent) :
        QDialog(parent), mPixmap(0), mLabel(0)
{
    setLayout(new QVBoxLayout);
    mLabel = new QLabel(this);
    layout()->addWidget(mLabel);
}

void CameraWindow::slotSetPixmapData(const QByteArray &data)
{
    if(!mPixmap) mPixmap = new QPixmap;
    mPixmap->loadFromData(data);
    slotSetDimensions(mPixmap->size());
    mLabel->setPixmap(*mPixmap);
}

void CameraWindow::slotSetDimensions(const QSize size)
{
    mLabel->setMinimumSize(size);
    mLabel->setMaximumSize(size);

    setFixedSize(size);

    if(mPixmap) delete mPixmap;
    mPixmap = new QPixmap(size);
}
