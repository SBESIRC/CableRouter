#include "CR_CGALUtils.h"

#ifndef _CABLEROUTER_MAPUTILS_H_
#define _CABLEROUTER_MAPUTILS_H_

#define FARWAY 10000

namespace CableRouter
{
	// Polygon Element
	struct PElement
	{
		Polygon boundary;
		double weight;
	};

	typedef rbush::RBush<Segment> SegBush;
	typedef rbush::RBush<PElement> PEBush;

	struct Power
	{
		int id;
		vector<Point> points;
		Power(vector<Point> pts)
		{
			points = pts;
		}
		Power(Point p)
		{
			points.push_back(p);
		}
		Power(Segment s)
		{
			points.push_back(s.source());
			points.push_back(s.target());
		}
		bool is_point()
		{
			return points.size() == 1;
		}
		bool is_segment()
		{
			return points.size() == 2;
		}
		bool is_valid()
		{
			return is_point() || is_segment();
		}
	};

	struct Device
	{
		int id;
		Point coord;
		Device(Point p, int i)
			: coord(p), id(i) {}
		Device(Point p)
			: coord(p), id(-1) {}
	};

	struct FireArea
	{
		PElement info;
		SegBush* area_edge_tree;
	};

	struct MapInfo
	{
		vector<Power> powers;
		vector<Device> devices;
		vector<Segment> centers;
		vector<Polygon> holes;
		vector<Polygon> rooms;
		FireArea area;
		SegBush* cen_line_tree;
		PEBush* hole_tree;
		PEBush* room_tree;
	};

	void deleteMapInfo(MapInfo& map);

	CDT			buildTriangulation	(MapInfo* const map);
	double**	buildGraphAll		(MapInfo* const map, const CDT& cdt, int n, bool center_weighted = false, bool room_weighted = false);
	void		addDeviceEdges		(MapInfo* const map, double** G, bool center_weighted = false, bool room_weighted = false);
	void		addPowerEdges		(MapInfo* const map, const CDT& cdt, double** G, bool center_weighted = false, bool room_weighted = false);
	void		removeObstacles		(MapInfo* const map, double** G, int n);

	void		addWeightCenters	(MapInfo* const map, const Point p, const Point q, double& w);
	void		addWeightRooms		(MapInfo* const map, const Point p, const Point q, double& w);
	bool		crossObstacle		(MapInfo* const map, const Segment s);
	bool		crossObstacle		(MapInfo* const map, const Point p, const Point q);
	bool		touchObstacle		(MapInfo* const map, const Point p, const Point q);
}

#endif