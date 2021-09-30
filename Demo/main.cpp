#include "../include/CableRouteEngine.h"
#include <fstream>
#include <sstream>

using namespace CableRouter;
using namespace std;

int main()
{
	CableRouteEngine cre;
	ifstream f("test1.geojson");
	stringstream ss;
	ss << f.rdbuf();
	f.close();
	string datastr = ss.str();
	string res = cre.routing(datastr);
	printf("%s", res.c_str());
	return 0;
}