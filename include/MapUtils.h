#include "CR_CGALUtils.h"

#ifndef _CABLEROUTER_MAPUTILS_H_
#define _CABLEROUTER_MAPUTILS_H_

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
		int id = -1;
		int region_id = -1;
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
		int region_id;
		Point coord;
		Device(Point p, int i)
			: coord(p), id(i) , region_id(-1) {}
		Device(Point p)
			: coord(p), id(-1), region_id(-1) {}
	};

	struct FireArea
	{
		PElement info;
		SegBush* area_edge_tree;
	};

	struct Region
	{
		Polygon boundary;
		vector<Polygon> holes;

		// if ucs
		Direction align = Direction(1, 0);

		// if center
		bool align_center = false;
		vector<Segment> centers;

		bool has_on_unbounded_side(Point pos);
	};

	struct MapInfo
	{
		vector<Power> powers;
		vector<Device> devices;
		vector<Segment> centers;
		vector<Polygon> holes;
		vector<Polygon> rooms;
		vector<Region> regions;
		FireArea area;
		SegBush* cen_line_tree;
		PEBush* hole_tree;
		PEBush* room_tree;
	};

	void preprocess(MapInfo& map);
	void deleteMapInfo(MapInfo& map);
	void deleteInvalidDevice(MapInfo& map);
	void deleteInvalidPower(MapInfo& map);
	void correctInvalidPower(MapInfo& map);
	bool isValidPoint(MapInfo& map, Point pos);

	CDT			buildTriangulation	(MapInfo* const map);
	double**	buildGraphAll		(MapInfo* const map, const CDT& cdt, int n, bool center_weighted = false, bool room_weighted = false);
	void		addDeviceEdges		(MapInfo* const map, double** G, bool center_weighted = false, bool room_weighted = false);
	void		addPowerEdges		(MapInfo* const map, const CDT& cdt, double** G, bool center_weighted = false, bool room_weighted = false);
	void		removeObstacles		(MapInfo* const map, double** G, int n);

	void		addWeightCenters	(MapInfo* const map, const Point p, const Point q, double& w);
	void		addWeightRooms		(MapInfo* const map, const Point p, const Point q, double& w);
	bool		crossObstacle		(MapInfo* const map, const Segment s);
	bool		crossObstacle		(MapInfo* const map, const Point p, const Point q);
	int			crossRoom			(MapInfo* const map, const Segment s);
	int			crossRoom			(MapInfo* const map, const Point p, const Point q);
	bool		touchObstacle		(MapInfo* const map, const Point p, const Point q);

	MapInfo		rotateMap			(MapInfo* const map, Direction align);
	vector<Polyline> getBoundaryOf(const Region& r1, const Region& r2);
	vector<Polyline> getBoundaryOf(Polygon p1, Polygon p2);
}

#endif