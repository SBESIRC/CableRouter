#include "CR_CGALUtils.h"
#include "MapUtils.h"

#ifndef _CABLEROUTER_ROUTE_ENGINE_H_
#define _CABLEROUTER_ROUTE_ENGINE_H_

namespace CableRouter
{
	struct ASNode
	{
		int id;
		Face_handle face;
		Point mid;

		set<int> neighbor;
		int parent = -1;

		bool is_start = false;
		bool is_end = false;
		bool open = false;
		bool close = false;

		double g;
		double h;
		double f()
		{
			return g + h;
		}
	};

	// General
	vector<Point>	manhattan_connect(MapInfo* const map, Point s, Point t, Vector pre_dir, vector<Segment>& lines);
	vector<Point>	funnel_smooth(CDT& dt, ASNode* nodes, Point s, Point t, int end_node_id);

	// Point to Point
	vector<Point>	obstacle_avoid_connect_p2p(MapInfo* const map, Point s, Point t, vector<Segment>& lines);
	vector<Point>	a_star_connect_p2p(MapInfo* const map, Point s, Point t, vector<Segment>& lines);
	vector<Point>	manhattan_smooth_p2p(MapInfo* const map, vector<Point>& path, vector<Segment>& exist_lines);

	// Point to Segment
	vector<Point>	obstacle_avoid_connect_p2s(MapInfo* const map, Point s, Segment t, vector<Segment>& lines);
	vector<Point>	a_star_connect_p2s(MapInfo* const map, Point s, Segment t, vector<Segment>& lines);
	vector<Point>	manhattan_smooth_p2s(MapInfo* const map, vector<Point>& path, Segment des, vector<Segment>& exist_lines);

	// Smooth
	vector<Point>	line_break(vector<Point>& line, const double gap);
	vector<Point>	line_simple(vector<Point>& line);
	vector<Point>	manhattan_smooth_basic(MapInfo* const map, vector<Point>& path, vector<Segment>& exist_lines);

	double			tooCloseToSun(MapInfo* const map, const Point p, const Point q, vector<Segment>& exist_lines, bool is_end = false);
}

#endif