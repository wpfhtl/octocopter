#include "urg/UrgCtrl.h"
#include <cstdlib>
#include <iostream>

using namespace qrk;
using namespace std;


int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        cout << "usage: " << argv[0] << " /dev/ttyACM?" << endl;
        exit(1);
    }

    //const char device[] = "COM3";
    const char device[] = "/dev/ttyACM0";

    UrgCtrl urg;
    if (!urg.connect(argv[1]))
    {
        cerr << "error: connect():" << urg.what();
        exit(1);
    }

    vector<string> lines;

    // Receive version information
    urg.versionLines(lines);
    if(lines.empty())
    {
        cerr << "UrgCtrl::versionLines: " << urg.what() << endl;
        exit(1);
    }

    // Output
    for(vector<string>::iterator it = lines.begin(); it != lines.end(); ++it)
    {
	if(it->find(std::string("SERI")) != string::npos)
	{
          cout << it->substr(5, 8) << endl;
	}
    }

    return 0;
}
