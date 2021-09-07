#include <string>

#ifdef THMODULE_DLL
#  define THMODULE_EXPORT __declspec(dllexport)
#else
#  define THMODULE_EXPORT __declspec(dllimport)
#endif

#ifndef _CABLEROUTER_INTERFACE_H_
#define _CABLEROUTER_INTERFACE_H_

namespace CableRouter
{
	using namespace std;

	//string grouping(string datastr);
	//string inter_routing(string datastr);

	class THMODULE_EXPORT CableRouteEngine
	{
	public:
		string routing(string datastr);
	};

}

#endif