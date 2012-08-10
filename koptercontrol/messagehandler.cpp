#include "messagehandler.h"

QFile* mMasterLogFile = 0;
QTextStream* mMasterLogStream = 0;

MessageHandler::MessageHandler(const QString& logFilePrefix)
{
    qDebug() << "MessageHandler::installMessageHandler(): setting up console logging...";

    mNumPunct = new comma_numpunct();
    mLocale = new std::locale(std::locale(), mNumPunct);

    // tell cout to use our new locale.
    std::cout.imbue(*mLocale);
    std::cout << std::setprecision(3) << std::fixed;

    mMasterLogFile = new QFile(logFilePrefix + QString("console.txt"));
    if(!mMasterLogFile->open(QIODevice::WriteOnly | QIODevice::Append))
    {
        qFatal("MessageHandler::MessageHandler(): cannot open logfile: %s, exiting", qPrintable(mMasterLogFile->fileName()));
    }

    mMasterLogStream = new QTextStream(mMasterLogFile);

    // To get a thousand group separator of "."
    QLocale german(QLocale::German);
    mMasterLogStream->setLocale(german);

    qInstallMsgHandler(MessageHandler::handleMessage);
    qDebug() << "MessageHandler::MessageHandler(): successfully set up console logging.";
}

MessageHandler::~MessageHandler()
{
    qDebug() << "MessageHandler::~MessageHandler(): shutting down logmessage handler...";
    delete mNumPunct;
    delete mLocale;

    // Re-set logging to the default handler.
    qInstallMsgHandler(0);

    if(mMasterLogStream)
    {
        qDebug() << "MessageHandler::~MessageHandler(): shutting down logstream";
        mMasterLogStream->flush();
        delete mMasterLogStream;
    }
    if(mMasterLogFile)
    {
        qDebug() << "MessageHandler::~MessageHandler(): closing logfile";
        mMasterLogFile->flush();
        mMasterLogFile->close();
        delete mMasterLogFile;
    }
}

void MessageHandler::handleMessage(QtMsgType type, const char *msg)
{
    Q_ASSERT(mMasterLogStream != 0 && "masterLogSteram is not set!");

    const qint32 tow = GnssTime::currentTow();

    std::cout << tow << ' ';
    std::cout << msg << std::endl;

    // Don't use endl, as that would flush the file to sd-card, which is sloooooow
    (*mMasterLogStream) << tow << ' ' << msg << '\n';

    if(type == QtFatalMsg) abort();
}
