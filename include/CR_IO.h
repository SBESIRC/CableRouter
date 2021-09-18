#include "MapUtils.h"

#ifndef _CABLEROUTER_IO_H_
#define _CABLEROUTER_IO_H_

namespace CableRouter
{
	//Point cgal_point(CableRouterParse::point& p);
	//vector<Point> cgal_points(vector<CableRouterParse::point>& pts);

	//CableRouterParse::point block_point(Point& p);
	//vector<CableRouterParse::point> block_points(vector<Point>& pts);

	//MapInfo read_blocks(vector<CableRouterParse::Block>& data);

	MapInfo read_from_geojson_string(const string& datastr);
	MapInfo read_from_geojson_file(const string& filename);

	string  write_to_geojson_string(const vector<Polyline>& paths);
	void	write_to_geojson_file(const string& filename, const vector<Polyline>& paths);
}

#endif