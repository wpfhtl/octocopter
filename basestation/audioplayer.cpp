#include "audioplayer.h"

AudioPlayer::AudioPlayer()
{
    int audio_rate = 16000;
    Uint16 audio_format = AUDIO_S16; /* 16-bit stereo */
    int audio_channels = 1;
    int audio_buffers = 4096;

    SDL_Init(SDL_INIT_AUDIO);

    /* This is where we open up our audio device.  Mix_OpenAudio takes
         as its parameters the audio format we'd /like/ to have. */
    if(Mix_OpenAudio(audio_rate, audio_format, audio_channels, audio_buffers)) {
        printf("Unable to open audio!\n");
        exit(1);
    }

    /* If we actually care about what we got, we can ask here.  In this
         program we don't, but I'm showing the function call here anyway
         in case we'd want to know later. */
    Mix_QuerySpec(&audio_rate, &audio_format, &audio_channels);

    // After three seconds of no setSound(), quit playing.
    mTimerStopPlaying.setInterval(3000);
    connect(&mTimerStopPlaying, SIGNAL(timeout()), SLOT(slotStopPlaying()));
}

AudioPlayer::~AudioPlayer()
{
    Mix_CloseAudio();
    SDL_Quit();
}

bool AudioPlayer::setSound(const QString& soundFile)
{
    mTimerStopPlaying.start();

    if(mCurrentlyPlaying.isEmpty())
    {
//        qDebug() << "AudioPlayer::playSound(): ending silence, playing sound" << soundFile;
        mCurrentlyPlaying = soundFile;
        playSound();
    }
    else if(soundFile != mCurrentlyPlaying)
    {
//        qDebug() << "AudioPlayer::playSound(): cancelling sound" << mCurrentlyPlaying << "to play new sound" << soundFile;

        slotStopPlaying();

        mCurrentlyPlaying = soundFile;

        playSound();
    }
    else
    {
//        qDebug() << "AudioPlayer::playSound(): will not play sound" << soundFile << ": playing same sound is in progress.";
        return false;
    }
}

void AudioPlayer::playSound()
{
    QFileInfo fi(mCurrentlyPlaying);
    mSound = Mix_LoadMUS(qPrintable(fi.absoluteFilePath()));
    printf("playin sound %s\n", qPrintable(fi.absoluteFilePath()));
    Mix_PlayMusic(mSound, -1);
//    Mix_HookMusicFinished(playingFinished);
}

void AudioPlayer::slotStopPlaying()
{
//    qDebug() << "AudioPlayer::slotStopPlaying(): finished playing" << mCurrentlyPlaying;
    Mix_HaltMusic();
    Mix_FreeMusic(mSound);
    mSound = NULL;

    mCurrentlyPlaying = QString();
}
