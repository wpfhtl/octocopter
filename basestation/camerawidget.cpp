#include "camerawidget.h"

CameraWidget::CameraWidget(QWidget *parent, const QString &windowTitle) :
        QDockWidget(parent), mPixmap(0), mLabel(0)
{
    setLayout(new QVBoxLayout);
    setWindowTitle(windowTitle);
    layout()->setContentsMargins(0, 0, 0, 0);
    setContentsMargins(0, 0, 0, 0);
    mLabel = new QLabel(this);
    layout()->addWidget(mLabel);
    mPixmap = new QPixmap;

    // if we do that, we need to delete this window from Triangulator::mCameraWidgets, too.
//    connect(this, SIGNAL(finished(int)), SLOT(deleteLater()));
}

void CameraWidget::slotSetPixmapData(const QByteArray &data)
{
    delete mPixmap;
    mPixmap = new QPixmap;
    mPixmap->loadFromData(data);
//    mPixmap->save("/tmp/target.jpg", "JPG", 100);
    slotSetDimensions(mPixmap->size());
    mLabel->setPixmap(*mPixmap);
}

void CameraWidget::slotSetDimensions(const QSize size)
{
    mLabel->setMinimumSize(size);
    mLabel->setMaximumSize(size);

    setFixedSize(size);

//    if(mPixmap) delete mPixmap;
//    mPixmap = new QPixmap(size);
}
