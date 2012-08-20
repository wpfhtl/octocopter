#include "audioplayer.h"

bool AudioPlayer::playSound(const QString& soundFile)
{
    if(!mMediaObject)
    {
        qDebug() << "AudioPlayer::playSound(): ending silence, playing sound" << soundFile;
        mMediaObject = Phonon::createPlayer(Phonon::NoCategory, Phonon::MediaSource(soundFile));
        mMediaObject->play();
        connect(mMediaObject, SIGNAL(finished()), mMediaObject.data(), SLOT(stop()));
        connect(mMediaObject, SIGNAL(finished()), mMediaObject.data(), SLOT(deleteLater()));
//        connect(mMediaObject, SIGNAL(aboutToFinish()), mMediaObject.data(), SLOT(stop()));
//        connect(mMediaObject, SIGNAL(aboutToFinish()), mMediaObject.data(), SLOT(deleteLater()));
        mCurrentlyPlaying = soundFile;
    }
    else if(soundFile != mCurrentlyPlaying)
    {
        qDebug() << "AudioPlayer::playSound(): cancelling sound" << mCurrentlyPlaying << "to play new sound" << soundFile;
        mMediaObject->stop();
        mMediaObject->deleteLater();

        mMediaObject = Phonon::createPlayer(Phonon::NoCategory, Phonon::MediaSource(soundFile));
        mMediaObject->play();
        connect(mMediaObject, SIGNAL(finished()), mMediaObject.data(), SLOT(deleteLater()));
        mCurrentlyPlaying = soundFile;
    }
    else
    {
        qDebug() << "AudioPlayer::playSound(): will not play sound" << soundFile << ": playing same sound is in progress.";
        return false;
    }
}
