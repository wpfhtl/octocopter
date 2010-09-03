#include "camerawindow.h"

CameraWindow::CameraWindow(QWidget *parent, const QString &windowTitle) :
        QDialog(parent), mPixmap(0), mLabel(0)
{
    setLayout(new QVBoxLayout);
    setWindowTitle(windowTitle);
    layout()->setContentsMargins(0, 0, 0, 0);
    setContentsMargins(0, 0, 0, 0);
    mLabel = new QLabel(this);
    layout()->addWidget(mLabel);
    mPixmap = new QPixmap;

    // if we do that, we need to delete this window from Triangulator::mCameraWindows, too.
//    connect(this, SIGNAL(finished(int)), SLOT(deleteLater()));
}

void CameraWindow::slotSetPixmapData(const QByteArray &data)
{
    delete mPixmap;
    mPixmap = new QPixmap;
    mPixmap->loadFromData(data);
//    mPixmap->save("/tmp/target.jpg", "JPG", 100);
    slotSetDimensions(mPixmap->size());
    mLabel->setPixmap(*mPixmap);
}

void CameraWindow::slotSetDimensions(const QSize size)
{
    mLabel->setMinimumSize(size);
    mLabel->setMaximumSize(size);

    setFixedSize(size);

//    if(mPixmap) delete mPixmap;
//    mPixmap = new QPixmap(size);
}
