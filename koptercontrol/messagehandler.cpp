#include "messagehandler.h"

QFile* mMasterLogFile = 0;
QTextStream* mMasterLogStream = 0;

MessageHandler::MessageHandler(const QString& logFilePrefix)
{
    qDebug() << "KopterControl::installMessageHandler(): setting up console logging...";

    mNumPunct = new comma_numpunct();
    mLocale = new std::locale(std::locale(), mNumPunct);

    // tell cout to use our new locale.
    std::cout.imbue(*mLocale);
    std::cout << std::setprecision(3) << std::fixed;

    mMasterLogFile = new QFile(logFilePrefix + QString("console.txt"));
    if(!mMasterLogFile->open(QIODevice::WriteOnly | QIODevice::Append))
    {
        qFatal("Cannot open logfile: %s, exiting", qPrintable(mMasterLogFile->fileName()));
    }

    mMasterLogStream = new QTextStream(mMasterLogFile);

    // To get a thousand group separator of "."
    QLocale german(QLocale::German);
    mMasterLogStream->setLocale(german);

    qInstallMsgHandler(MessageHandler::handleMessage);
    qDebug() << "KopterControl::installMessageHandler(): successfully set up console logging.";
}

MessageHandler::~MessageHandler()
{
    delete mNumPunct;
    delete mLocale;

    if(mMasterLogStream)
    {
        mMasterLogStream->flush();
        delete mMasterLogStream;
    }
    if(mMasterLogFile)
    {
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

    (*mMasterLogStream) << tow << ' ' << msg << endl;

    if(type == QtFatalMsg) abort();
}
