#ifndef TESTPTU_HPP
#define TESTPTU_HPP

#include <QObject>

class TestPtu : public QObject
{
    Q_OBJECT
private slots:
    void testFromTop();
    void testFromSide();
    void testBasePoseAlignment();
};

#endif // TESTPTU_HPP
