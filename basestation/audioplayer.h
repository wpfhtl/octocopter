#ifndef AUDIOPLAYER_H
#define AUDIOPLAYER_H

#include <QFileInfo>
#include <QDebug>
#include <QTimer>

#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>

class AudioPlayer : public QObject
{
    Q_OBJECT

private:
    QTimer mTimerStopPlaying;
    Mix_Music* mSound;
    QString mCurrentlyPlaying;


    void playSound();

public:
    AudioPlayer();
    ~AudioPlayer();
    bool setSound(const QString& soundFile);

private slots:
    void slotStopPlaying();
};

#endif
