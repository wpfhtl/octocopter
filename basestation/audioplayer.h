#ifndef AUDIOPLAYER_H
#define AUDIOPLAYER_H

#include <phonon/phonon>
#include <QPointer>
#include <QDebug>

class AudioPlayer : public QObject
{
    Q_OBJECT

private:
    QPointer<Phonon::MediaObject> mMediaObject;
    QString mCurrentlyPlaying;

public:
    bool playSound(const QString& soundFile);
};

#endif
