#include "CR_IO.h"

using namespace CableRouter;

Point CableRouter::cgal_point(CableRouterParse::point& p)
{
	return Point(p.x, p.y);
}

vector<Point> CableRouter::cgal_points(vector<CableRouterParse::point>& pts)
{
	vector<Point> res;
	for (int i = 0; i < pts.size(); i++)
	{
		res.push_back(cgal_point(pts[i]));
	}
	return res;
}

MapInfo CableRouter::read_blocks(vector<CableRouterParse::Block>& data)
{
	MapInfo map;
	vector<rbush::TreeNode<PElement>*> hole_nodes;
	vector<rbush::TreeNode<PElement>*> room_nodes;
	vector<rbush::TreeNode<Segment>* > area_nodes;
	vector<rbush::TreeNode<Segment>* > center_nodes;

	PElement e;

	for (int i = 0; i < data.size(); i++)
	{
		string category = data[i].category;
		string area_id = data[i].area_id;
		if (category.compare("Device") == 0)
		{
			map.devices.push_back(Device(cgal_point(data[i].coords[0]), (int)map.devices.size()));
		}
		if (category.compare("Power") == 0)
		{
			map.powers.push_back(Power(cgal_points(data[i].coords)));
		}
		else if (category.compare("Hole") == 0)
		{
			e.boundary = construct_polygon(&cgal_points(data[i].coords));
			e.weight = CR_INF;
			hole_nodes.push_back(get_rtree_node(&e));
		}
		else if (category.compare("Area") == 0)
		{
			e.boundary = construct_polygon(&cgal_points(data[i].coords));
			e.weight = CR_INF;
			map.area.info = e;
			for (auto eit = e.boundary.edges_begin(); eit != e.boundary.edges_end(); eit++)
			{
				area_nodes.push_back(get_seg_rtree_node(&(*eit)));
			}
		}
		else if (category.compare("CenterLine") == 0)
		{
			center_nodes.push_back(get_seg_rtree_node(&Segment(cgal_point(data[i].coords[0]), cgal_point(data[i].coords[1]))));
		}
		else if (category.compare("RoomLine") == 0)
		{
			e.boundary = construct_polygon(&cgal_points(data[i].coords));
			e.weight = 10;
			room_nodes.push_back(get_rtree_node(&e));
		}
	}

	map.cen_line_tree = new SegBush(center_nodes);
	map.area.area_edge_tree = new SegBush(area_nodes);
	map.hole_tree = new PEBush(hole_nodes);
	map.room_tree = new PEBush(room_nodes);

	return map;
}
