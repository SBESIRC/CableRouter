#include "../include/CableRouteEngine.h"
#include <fstream>
#include <sstream>
#include <crtdbg.h>

using namespace CableRouter;
using namespace std;

int main()
{
	CableRouteEngine cre;
	ifstream f("err_json/ID1001078.geojson");
	stringstream ss;
	ss << f.rdbuf();
	f.close();
	string datastr = ss.str();
	string res = cre.routing(datastr);
	printf("%s", res.c_str());
	return 0;
}