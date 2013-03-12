#ifndef MESSAGEHANDLER_H
#define MESSAGEHANDLER_H

#include <QFile>
#include <QTextStream>

#include <iostream>
#include <iomanip>

#include "logfile.h"
#include <gnsstime.h>

class MessageHandler
{
private:

    class comma_numpunct : public std::numpunct<char>
    {
      protected:
        virtual char do_thousands_sep() const
        {
            return '.';
        }

        virtual std::string do_grouping() const
        {
            return "\03";
        }
    };

    comma_numpunct* mNumPunct;
    std::locale* mLocale;

public:
    MessageHandler(const QString& logFilePrefix);

    ~MessageHandler();

    static void handleMessage(QtMsgType type, const char *msg);
};

#endif // MESSAGEHANDLER_H
