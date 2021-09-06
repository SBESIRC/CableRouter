#include "parse.h"
#include "MapUtils.h"

#ifndef _CABLEROUTER_IO_H_
#define _CABLEROUTER_IO_H_

namespace CableRouter
{
	Point cgal_point(CableRouterParse::point& p);
	vector<Point> cgal_points(vector<CableRouterParse::point>& pts);
	MapInfo read_blocks(vector<CableRouterParse::Block>& data);
}

#endif