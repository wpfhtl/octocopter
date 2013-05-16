#include "camerawidget.h"
#include <QVBoxLayout>

CameraWidget::CameraWidget(QWidget *parent, const QString &windowTitle) :
        QDockWidget(parent), mPixmap(0), mLabel(0)
{
    setLayout(new QVBoxLayout);
    setWindowTitle(windowTitle);
    layout()->setContentsMargins(0, 0, 0, 0);
    setContentsMargins(0, 0, 0, 0);
    mLabel = new QLabel(this);
//    mLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout()->addWidget(mLabel);
    setWidget(mLabel);
    mPixmap = new QPixmap;
    mLabel->setPixmap(*mPixmap);
    mLabel->show();

}

void CameraWidget::slotSetPixmapData(const QByteArray &data)
{
//    delete mPixmap;
//    mPixmap = new QPixmap;
    mPixmap->loadFromData(data);
//    mPixmap->save("/tmp/target.jpg", "JPG", 100);
    slotSetDimensions(mPixmap->size());
    mLabel->setPixmap(*mPixmap);
//    mLabel->setPixmap(mPixmap->scaled(mLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

void CameraWidget::slotSetDimensions(const QSize size)
{
//    mLabel->setMinimumSize(size);
//    mLabel->setMaximumSize(size);

//    setFixedSize(size.width()+5, size.height()+5);
}
