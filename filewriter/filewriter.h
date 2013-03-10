#ifndef FILEWRITER_H
#define FILEWRITER_H

#include "filetowrite.h"

#include <QCoreApplication>
#include <QVector>
#include <QDebug>
#include <QStringList>

class FileWriter : public QCoreApplication
{
    Q_OBJECT

public:
    FileWriter(int argc, char **argv);
    ~FileWriter(void);

private:
    QVector<FileToWrite*> mFileVector;
    void printUsage();

};

#endif
