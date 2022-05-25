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

		int cross_num = 0;

		double g;
		double h;
		double f()
		{
			return g + h;
		}
	};

	struct ASPath
	{
		Polyline path;
		vector<int> cross_num;
	};

	// General
	Polyline	manhattan_connect(MapInfo* const map, Point s, Point t, Vector pre_dir, vector<Segment>& lines, Transformation rotate = Transformation());
	Polyline	funnel_smooth(CDTP& dt, ASNode* nodes, Point s, Point t, int end_node_id);

	// Point to Point
	Polyline	obstacle_avoid_connect_p2p(MapInfo* const map, Point s, Point t, vector<Segment>& lines);
	ASPath		a_star_connect_p2p(MapInfo* const map, Point s, Point t, vector<Segment>& lines);
	Polyline	manhattan_smooth_p2p(MapInfo* const map, Polyline& path, vector<Segment>& exist_lines);
	Polyline	center_connect_p2p(MapInfo* const map, Polyline center, Point s, Point t, vector<Segment>& lines);

	// Point to Segment
	Polyline	obstacle_avoid_connect_p2s(MapInfo* const map, Point s, Segment t, vector<Segment>& lines);
	Polyline	a_star_connect_p2s(MapInfo* const map, Point s, Segment t, vector<Segment>& lines);
	Polyline	manhattan_smooth_p2s(MapInfo* const map, Polyline& path, Segment des, vector<Segment>& exist_lines);

	// Smooth
	Polyline	line_break(Polyline line, const double gap);
	Polyline	line_simple(Polyline line);
	Polyline	manhattan_smooth_basic(MapInfo* const map, ASPath& path, vector<Segment>& exist_lines, Transformation rotate = Transformation());

	double		tooCloseToSun(MapInfo* const map, const Point p, const Point q, vector<Segment>& exist_lines, bool is_end = false);
}

#endif