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

	struct SElement
	{
		Segment seg;
		double weight;
	};

	typedef rbush::RBush<SElement> SegBush;
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

	enum LayoutType {
		Hoisting = 0,
		WallMounted,
		Ground
	};

	double priority_between(LayoutType a, LayoutType b);

	struct Device
	{
		int id;
		int region_id = -1;
		Point coord;
		int mass;
		LayoutType layout;
		Device(Point p)
			: coord(p), id(-1), mass(1), layout(Hoisting) {}
		Device(Point p, int i)
			: coord(p), id(i), mass(1), layout(Hoisting) {}
		Device(Point p, int i, int m)
			: coord(p), id(i), mass(m), layout(Hoisting) {}
		Device(Point p, int i, int m, LayoutType l)
			: coord(p), id(i), mass(m), layout(l) {}
	};

	struct FireArea
	{
		PElement info;
		SegBush* area_edge_tree;
		Direction align = Direction(1, 0);
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
		vector<Segment> borders;
		FireArea area;
		SegBush* cen_line_tree;
		PEBush* hole_tree;
		PEBush* room_tree;
	};

	int	 getRegionId(MapInfo& map, Point pos);
	void preprocess(MapInfo& map);
	void deleteMapInfo(MapInfo& map);
	void deleteInvalidDevice(MapInfo& map);
	void deleteInvalidPower(MapInfo& map);
	void correctInvalidPower(MapInfo& map);
	bool isValidPoint(MapInfo& map, Point pos);

	CDTP		buildTriangulation	(MapInfo* const map);
	double**	buildGraphAll		(MapInfo* const map, const CDTP& cdt, int n, bool center_weighted = false, bool room_weighted = false);
	void		addDeviceEdges		(MapInfo* const map, double** G, bool center_weighted = false, bool room_weighted = false);
	void		addPowerEdges		(MapInfo* const map, const CDTP& cdt, double** G, bool center_weighted = false, bool room_weighted = false);
	void        adjustByLayoutType	(MapInfo* const map, double** G);
	void		removeObstacles		(MapInfo* const map, double** G, int n);

	void		addWeightCenters	(MapInfo* const map, const Point p, const Point q, double& w);
	void		addWeightRooms		(MapInfo* const map, const Point p, const Point q, double& w);
	bool		crossObstacle		(MapInfo* const map, const Segment s);
	bool		crossObstacle		(MapInfo* const map, const Point p, const Point q);
	int			crossRoom			(MapInfo* const map, const Segment s);
	int			crossRoom			(MapInfo* const map, const Point p, const Point q);
	bool		touchObstacle		(MapInfo* const map, const Point p, const Point q);

	MapInfo		rotateMap			(MapInfo* const map, Direction align);

	vector<Polyline> getBoundaryOf	(const Region& r1, const Region& r2);
	vector<Polyline> getBoundaryOf	(Polygon p1, Polygon p2);

	vector<vector<int>>		getFittingLines		(vector<Device> dev, double gap, Direction align = Direction(1, 0));
	vector<vector<Device>>	getFittingLines		(MapInfo* const map, double gap);
	vector<vector<Device>>	breakFittingLine	(const vector<Device>& input, MapInfo* const map);
	vector<vector<Device>>	breakFittingLine	(const vector<Device>& input, vector<Segment>& lines, vector<int>& vis);
	vector<vector<Device>>	breakFittingLines	(MapInfo* const map, vector<vector<Device>> groups);

	vector<Segment>	rearrangeCenters(const vector<Segment> centers);

	void expandRooms(vector<Polygon>& rooms, const Polygon& area);
}

#endif