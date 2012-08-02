#include "testptu.hpp"

#include <QTest>

int main(int argc, char* argv[])
{
    QApplication(argc, argv);

    TestPtu testptu;
    QTest::qExec(&testptu, argc, argv);

    return 0;
}
