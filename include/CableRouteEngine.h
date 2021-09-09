#include <string>

#ifndef _CABLEROUTER_INTERFACE_H_
#define _CABLEROUTER_INTERFACE_H_

namespace CableRouter
{
	using namespace std;

	//string grouping(string datastr);
	//string inter_routing(string datastr);

	class CableRouteEngine
	{
	public:
		string routing(string datastr, int loop_max_count = 25);
	};

}

#endif