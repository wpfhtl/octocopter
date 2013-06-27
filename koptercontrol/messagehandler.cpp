#include "messagehandler.h"

LogFile* mMasterLogFile = 0;
QTextStream* mMasterLogStream = 0;
QByteArray* mLogMessage = 0;
QMutex* mMutexMessageHandler = 0;

MessageHandler::MessageHandler(const QString& logFilePrefix)
{
    qDebug() << "MessageHandler::installMessageHandler(): setting up console logging...";

    mNumPunct = new comma_numpunct();
    mLocale = new std::locale(std::locale(), mNumPunct);
    mLogMessage = new QByteArray;

    // tell cout to use our new locale.
    std::cout.imbue(*mLocale);
    std::cout << std::setprecision(3) << std::fixed;

    mMasterLogFile = new LogFile(logFilePrefix + QString(".txt"), LogFile::Encoding::Text);
    mMasterLogStream = new QTextStream(mLogMessage);

    // To get a thousand group separator of "."
    QLocale german(QLocale::German);
    mMasterLogStream->setLocale(german);

    mMutexMessageHandler = new QMutex;

    qInstallMessageHandler(MessageHandler::handleMessage);
    qDebug() << "MessageHandler::MessageHandler(): successfully set up console logging.";
}

MessageHandler::~MessageHandler()
{
    qDebug() << "MessageHandler::~MessageHandler(): shutting down logmessage handler...";
    delete mNumPunct;
    delete mLocale;

    // Re-set logging to the default handler.
    qInstallMessageHandler(0);

    if(mMasterLogStream)
    {
        qDebug() << "MessageHandler::~MessageHandler(): shutting down logstream";
        mMasterLogStream->flush();
        delete mMasterLogStream;
    }
    if(mMasterLogFile)
    {
        qDebug() << "MessageHandler::~MessageHandler(): closing logfile";
        delete mMasterLogFile;
    }

    delete mLogMessage;
}

void MessageHandler::handleMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QMutexLocker l(mMutexMessageHandler);

    Q_ASSERT(mMasterLogStream != 0 && "masterLogSteram is not set!");

    const qint32 tow = GnssTime::currentTow();

    std::cout << tow << ' ' << qPrintable(msg) << std::endl;

    // Don't use endl, as that would flush the line/file to sd-card, which is sloooooow
    (*mMasterLogStream) << tow << ' ' << msg << '\n';
    mMasterLogStream->flush();
    mMasterLogStream->seek(0);
    mMasterLogFile->write(mLogMessage);
    mLogMessage->clear();

    if(type == QtFatalMsg)
    {
        fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", qPrintable(msg), context.file, context.line, context.function);
        abort();
    }
}
