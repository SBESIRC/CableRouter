#include "MapUtils.h"

#define FARWAY 10000
using namespace CableRouter;

bool CableRouter::touchObstacle(MapInfo* const data, const Point p, const Point q)
{
	Segment s(p, q);

	// area
	auto edges = segment_search(data->area.area_edge_tree, s.source(), s.target());
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		Segment seg = (*eit)->data->seg;
		if (CGAL::do_intersect(s, seg)) return true;
	}

	// holes
	auto holes = segment_search(data->hole_tree, s.source(), s.target());
	for (auto oit = holes.begin(); oit != holes.end(); oit++)
	{
		Polygon boundary = (*oit)->data->boundary;
		for (auto eit = boundary.edges_begin(); eit != boundary.edges_end(); eit++)
		{
			if (CGAL::do_intersect(s, *eit)) return true;
		}
	}

	// devices
	//for (auto d = data->devices.begin(); d != data->devices.begin(); d++)
	//{
	//	vector<Point> box = point_box(d->coord, LINE_GAP);
	//	for (int i = 0; i < box.size(); i++)
	//	{
	//		if (CGAL::do_intersect(s, Segment(box[i], box[(i + 1) % box.size()]))) return true;
	//	}
	//}

	return false;
}

MapInfo CableRouter::rotateMap(MapInfo* const data, Direction align)
{
	MapInfo map;
	PElement pe;
	SElement se;

	Transformation rotate = get_tf_from_dir(align).inverse();

	vector<rbush::TreeNode<PElement>*> hole_nodes;
	vector<rbush::TreeNode<PElement>*> room_nodes;
	vector<rbush::TreeNode<SElement>* > area_nodes;
	vector<rbush::TreeNode<SElement>* > center_nodes;

	for (auto d : data->devices)
	{
		Device dev(d.coord.transform(rotate), d.id);
		map.devices.push_back(dev);
	}

	for (auto p : data->powers)
	{
		vector<Point> pts;
		for (auto q : p.points)
			pts.push_back(q.transform(rotate));
		Power pwr(pts);
		pwr.id = p.id;
		map.powers.push_back(pwr);
	}

	for (auto h : data->holes)
	{
		pe.boundary = CGAL::transform(rotate, h);
		pe.weight = CR_INF;
		hole_nodes.push_back(get_rtree_node(&pe));
		map.holes.push_back(pe.boundary);
	}

	pe.boundary = CGAL::transform(rotate, data->area.info.boundary);
	pe.weight = data->area.info.weight;
	map.area.info = pe;
	for (auto eit = pe.boundary.edges_begin(); eit != pe.boundary.edges_end(); eit++)
	{
		se.seg = *eit;
		se.weight = data->area.info.weight;
		area_nodes.push_back(get_seg_rtree_node(&se));
	}

	for (auto c : data->centers)
	{
		Segment s = c.transform(rotate);
		se.seg = s;
		se.weight = 8000;
		center_nodes.push_back(get_seg_rtree_node(&se));
		map.centers.push_back(s);
	}

	for (auto r : data->rooms)
	{
		pe.boundary = CGAL::transform(rotate, r);
		pe.weight = 16000;
		room_nodes.push_back(get_rtree_node(&pe));
		map.rooms.push_back(pe.boundary);
	}

	map.cen_line_tree = new SegBush(center_nodes);
	map.area.area_edge_tree = new SegBush(area_nodes);
	map.hole_tree = new PEBush(hole_nodes);
	map.room_tree = new PEBush(room_nodes);

	return map;
}

CDT CableRouter::buildTriangulation(MapInfo* const data)
{
	printf("Triangulation begin\n");

	CDT dt;

	vector<std::pair<Point, VertexInfo>> vec;
	set<Constraint> constraints;

	// insert device, id from 0 to dev_n - 1
	for (auto v = data->devices.begin(); v != data->devices.end(); v++)
	{
		vec.push_back(std::make_pair(v->coord, VertexInfo(v->id, true)));
	}
	if (vec.size() > 0)
	{
		dt.insert(vec.begin(), vec.end());
	}

	// insert area point
	Polygon area_boundary = data->area.info.boundary;
	for (int i = 0; i < area_boundary.size(); i++)
	{
		constraints.insert(Constraint(area_boundary.vertex(i), area_boundary.vertex((i + 1) % area_boundary.size())));
	}

	// insert holes point
	for (auto h = data->holes.begin(); h != data->holes.end(); h++)
	{
		for (int i = 0; i < h->size(); i++)
		{
			constraints.insert(Constraint(h->vertex(i), h->vertex((i + 1) % h->size())));
		}
	}

	for (auto c = constraints.begin(); c != constraints.end(); c++)
	{
		dt.insert_constraint(c->source, c->target);
	}

	int id = (int)data->devices.size();
	// parse_groups other points' id
	for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); v++)
	{
		if (v->info().id < 0) {
			v->info().id = id++;
		}
	}
	mark_domains(dt);

	printf("Triangulation over\n");

	return dt;
}

double** CableRouter::buildGraphAll(MapInfo* const data, const CDT& cdt, int n, bool center_weighted, bool room_weighted)
{
	printf("Point Size: %d\n", n);

	printf("Init G begin\n");
	double** G = newDoubleGraph(n, CR_INF);
	for (auto feit = cdt.finite_edges_begin(); feit != cdt.finite_edges_end(); feit++)
	{
		auto vi = feit->first->vertex(cdt.cw(feit->second));
		auto vj = feit->first->vertex(cdt.ccw(feit->second));
		int i = vi->info().id;
		int j = vj->info().id;
		Point p = vi->point();
		Point q = vj->point();

		// hole
		if (feit->first->info().not_reach() &&
			feit->first->neighbor(feit->second)->info().not_reach())
		{
			G[i][j] = G[j][i] = CR_INF;
		}
		else
		{
			double w = DIST(p, q);
			if (center_weighted)
				addWeightCenters(data, p, q, w);
			if (room_weighted)
				addWeightRooms(data, p, q, w);
			G[i][j] = G[j][i] = w;
			//test_result->push_back(Segment(p, q));
		}
	}
	printf("Init G over\n");

	return G;
}

void CableRouter::addDeviceEdges(MapInfo* const data, double** G, bool center_weighted, bool room_weighted)
{
	printf("Make up dev-to-dev edge begin\n");
	for (int i = 0; i < data->devices.size(); i++)
	{
		for (int j = i + 1; j < data->devices.size(); j++)
		{
			int pi = data->devices[i].id;
			int qi = data->devices[j].id;
			if (G[pi][qi] >= CR_INF)
			{
				Point p = data->devices[i].coord;
				Point q = data->devices[j].coord;
				double w = DIST(p, q);
				if (w < FARWAY && !crossObstacle(data, p, q))
				{
					if (center_weighted)
						addWeightCenters(data, p, q, w);
					if (room_weighted)
						addWeightRooms(data, p, q, w);
					G[pi][qi] = G[qi][pi] = w;
					//test_result->push_back(Segment(p, q));
				}
			}
		}
	}
	printf("Make up dev-to-dev edge end\n");
}

void CableRouter::addPowerEdges(MapInfo* const data, const CDT& dt, double** G, bool center_weighted, bool room_weighted)
{
	printf("Make up power-to-point edge begin\n");
	for (int i = 0; i < data->powers.size(); i++)
	{
		data->powers[i].id = (int)dt.number_of_vertices() + i;
		printf("power[%d].id = %d\n", i, data->powers[i].id);
		for (auto j = dt.finite_vertices_begin(); j != dt.finite_vertices_end(); j++)
		{
			int pi = data->powers[i].id;
			int qi = j->info().id;
			Power power = data->powers[i];
			// p = dist of point q and point/segment p
			Point q = j->point();
			Point p;
			if (power.is_point())
				p = power.points[0];
			else if (power.is_segment())
				p = project_point_to_segment(q, Segment(power.points[0], power.points[1]));
			q = Point(
				(1.0 - 1.0 / DIST(p, q)) * DOUBLE(q.hx() - p.hx()) + DOUBLE(p.hx()),
				(1.0 - 1.0 / DIST(p, q)) * DOUBLE(q.hy() - p.hy()) + DOUBLE(p.hy()));
			double w = DIST(p, q);
			if (w < FARWAY && !crossObstacle(data, p, q))
			{
				if (center_weighted)
					addWeightCenters(data, p, q, w);
				if (room_weighted)
					addWeightRooms(data, p, q, w);
				G[pi][qi] = G[qi][pi] = w;
				//test_result->push_back(Segment(p, q));
			}
		}
	}
	printf("Make up power-to-point edge end\n");
}

void CableRouter::removeObstacles(MapInfo* const data, double** G, int n)
{
	printf("Shifting obstacle points begin\n");
	int dn = (int)data->devices.size();
	int pn = (int)data->powers.size();
	int on = n - dn - pn;
	double* dis = new double[n];
	bool* vis = new bool[n];
	for (int start = 0; start < dn + pn; start++)
	{
		fill(dis, dis + n, CR_INF);
		fill(vis, vis + n, false);
		int sid = start < dn ? start : start + on;
		dis[sid] = 0;
		for (int i = 0; i < n; i++)
		{
			double MIN = CR_INF;
			int u = -1;
			for (int j = 0; j < n; j++)
			{
				if (!vis[j] && dis[j] < MIN)
				{
					MIN = dis[j];
					u = j;
				}
			}
			if (u == -1) break;
			vis[u] = true;

			for (int v = 0; v < n; v++)
			{
				if (!vis[v] && G[u][v] != CR_INF)
				{
					if (dis[v] > dis[u] + G[u][v])
					{
						dis[v] = dis[u] + G[u][v];
					}
				}
			}
		}

		for (int i = 0; i < dn + pn; i++)
		{
			int id = i < dn ? i : i + on;
			if (sid == id) continue;
			G[sid][id] = G[id][sid] = dis[id];
		}

	}
	delete[] dis;
	delete[] vis;
	printf("Shifting obstacle points end\n");
}


void CableRouter::addWeightCenters(MapInfo* const data, const Point p, const Point q, double& w)
{
	vector<rbush::TreeNode<SElement>* > centers = segment_search(data->cen_line_tree, p, q);

	for (auto eit = centers.begin(); eit != centers.end(); eit++)
	{
		if (CGAL::do_intersect(Segment(p, q), (*eit)->data->seg))
		{
			w += (*eit)->data->weight;
		}
	}
}

void CableRouter::addWeightRooms(MapInfo* const data, const Point p, const Point q, double& w)
{
	vector<rbush::TreeNode<PElement>* > rooms = segment_search(data->room_tree, p, q);

	for (auto oit = rooms.begin(); oit != rooms.end(); oit++)
	{
		Polygon boundary = (*oit)->data->boundary;

		for (auto eit = boundary.edges_begin(); eit != boundary.edges_end(); eit++)
		{
			if (CGAL::do_intersect(Segment(p, q), *eit))
			{
				w += (*oit)->data->weight;
			}
		}
	}
}

bool CableRouter::crossObstacle(MapInfo* const data, const Point p, const Point q)
{
	return crossObstacle(data, Segment(p, q));
}

bool CableRouter::crossObstacle(MapInfo* const data, const Segment s)
{
	// area
	auto edges = segment_search(data->area.area_edge_tree, s.source(), s.target());
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		Segment seg = (*eit)->data->seg;
		if (!CGAL::do_intersect(s, seg)) continue;
		CGAL::Object result = CGAL::intersection(s, seg);
		Point pt;
		if (CGAL::assign(pt, result))
		{
			return true;
		}
	}

	// holes
	auto holes = segment_search(data->hole_tree, s.source(), s.target());
	for (auto oit = holes.begin(); oit != holes.end(); oit++)
	{
		Polygon boundary = (*oit)->data->boundary;
		for (auto eit = boundary.edges_begin(); eit != boundary.edges_end(); eit++)
		{
			if (!CGAL::do_intersect(s, *eit)) continue;
			CGAL::Object result = CGAL::intersection(s, *eit);
			Point pt;
			if (CGAL::assign(pt, result))
			{
				return true;
			}
		}
	}

	return false;
}

int CableRouter::crossRoom(MapInfo* const data, const Point p, const Point q)
{
	return crossRoom(data, Segment(p, q));
}

int CableRouter::crossRoom(MapInfo* const data, const Segment s)
{
	int res = 0;
	auto rooms = segment_search(data->room_tree, s.source(), s.target());
	for (auto oit = rooms.begin(); oit != rooms.end(); oit++)
	{
		Polygon boundary = (*oit)->data->boundary;
		for (auto eit = boundary.edges_begin(); eit != boundary.edges_end(); eit++)
		{
			if (!CGAL::do_intersect(s, *eit)) continue;
			CGAL::Object result = CGAL::intersection(s, *eit);
			Point pt; Segment seg;
			if (CGAL::assign(pt, result))
			{
				res++;
			}
			else if (CGAL::assign(seg, result))
			{
				res += 1000;
			}
		}
	}
	return res;
}

void CableRouter::preprocess(MapInfo& map)
{
	deleteInvalidDevice(map);
	//deleteInvalidPower(data);
	correctInvalidPower(map);
	for (int i = 0; i < map.devices.size(); i++)
	{
		for (int rid = 0; rid < map.regions.size(); rid++)
		{
			if (!map.regions[rid].has_on_unbounded_side(map.devices[i].coord))
			{
				map.devices[i].region_id = rid;
				break;
			}
		}
	}
	for (int i = 0; i < map.powers.size(); i++)
	{
		for (int rid = 0; rid < map.regions.size(); rid++)
		{
			if (!map.regions[rid].has_on_unbounded_side(map.powers[i].points[0]))
			{
				map.powers[i].region_id = rid;
				break;
			}
		}
	}
	for (int i = 0; i < map.centers.size(); i++)
	{
		Segment c = shrink_segment(map.centers[i]);
		for (int rid = 0; rid < map.regions.size(); rid++)
		{
			if (!map.regions[rid].has_on_unbounded_side(c.source()) &&
				!map.regions[rid].has_on_unbounded_side(c.target()))
			{
				map.regions[rid].centers.push_back(map.centers[i]);
				break;
			}
		}
	}

	for (int i = 0; i < map.regions.size(); i++)
	{
		map.regions[i].align = Direction(1, 0);
	}
}

void CableRouter::deleteMapInfo(MapInfo& map)
{
	auto cen_all = map.cen_line_tree->all();
	for (int i = 0; i < (*cen_all).size(); i++)
	{
		delete (*cen_all)[i];
	}
	reset(*cen_all);
	delete cen_all;

	auto hole_all = map.hole_tree->all();
	for (int i = 0; i < (*hole_all).size(); i++)
	{
		delete (*hole_all)[i];
	}
	reset(*hole_all);
	delete hole_all;

	auto area_all = map.area.area_edge_tree->all();
	for (int i = 0; i < (*area_all).size(); i++)
	{
		delete (*area_all)[i];
	}
	reset(*area_all);
	delete area_all;

	auto room_all = map.room_tree->all();
	for (int i = 0; i < (*room_all).size(); i++)
	{
		delete (*room_all)[i];
	}
	reset(*room_all);
	delete room_all;
}

void CableRouter::deleteInvalidDevice(MapInfo& map)
{
	vector<Device> valid_dev;
	set<Point> exist;
	for (int i = 0; i < map.devices.size(); i++)
	{
		Point pos = map.devices[i].coord;

		if (exist.find(pos) != exist.end())
			continue;

		if (!isValidPoint(map, pos))
			continue;

		valid_dev.push_back(Device(pos, (int)valid_dev.size()));
		exist.insert(pos);
	}
	map.devices.swap(valid_dev);
}

void CableRouter::deleteInvalidPower(MapInfo& map)
{
	vector<Power> valid_pwr;
	set<Point> exist_pt;
	// set<Segment> exist_seg;
	for (int i = 0; i < map.powers.size(); i++)
	{
		if (!map.powers[i].is_valid())
			continue;

		if (map.powers[i].is_point())
		{
			Point pos = map.powers[i].points[0];

			if (exist_pt.find(pos) != exist_pt.end())
				continue;

			if (!isValidPoint(map, pos))
				continue;

			valid_pwr.push_back(Power(pos));
			exist_pt.insert(pos);
		}

		else if (map.powers[i].is_segment())
		{
			// To do
		}
	}
	map.powers.swap(valid_pwr);
}

void CableRouter::correctInvalidPower(MapInfo& map)
{
	vector<Power> valid_pwr;
	set<Point> exist_pt;
	// set<Segment> exist_seg;
	for (int i = 0; i < map.powers.size(); i++)
	{
		if (!map.powers[i].is_valid())
			continue;

		if (map.powers[i].is_point())
		{
			Point pos = map.powers[i].points[0];

			if (exist_pt.find(pos) != exist_pt.end())
				continue;

			if (!map.area.info.boundary.has_on_bounded_side(pos))
			{
				Segment closet;
				double MIN = -1;
				for (auto eit = map.area.info.boundary.edges_begin(); eit != map.area.info.boundary.edges_end(); eit++)
				{
					double d = DIST(*eit, pos);
					if (MIN < 0 || d < MIN)
					{
						MIN = d;
						closet = *eit;
					}
				}
				Point project = project_point_to_segment(pos, closet);
				pos = shrink_segment(Segment(pos, project), 1.0 + 0.01 / MIN).target();
			}

			if (!isValidPoint(map, pos))
				continue;

			valid_pwr.push_back(Power(pos));
			exist_pt.insert(pos);
		}

		else if (map.powers[i].is_segment())
		{
			// To do
		}
	}
	map.powers.swap(valid_pwr);
}

bool CableRouter::isValidPoint(MapInfo& map, Point pos)
{
	if (!map.area.info.boundary.has_on_bounded_side(pos))
		return false;

	auto holes = point_search(map.hole_tree, pos);
	bool valid = true;

	for (auto hit = holes.begin(); hit != holes.end(); hit++)
	{
		auto h = *hit;
		Polygon boundary = h->data->boundary;
		if (!boundary.has_on_unbounded_side(pos))
		{
			valid = false;
			break;
		}
	}

	return valid;
}

bool CableRouter::Region::has_on_unbounded_side(Point pos)
{
	bool out = boundary.has_on_unbounded_side(pos);
	for (auto& h : holes)
		out |= h.has_on_bounded_side(pos);
	return out;
}

vector<Polyline> CableRouter::getBoundaryOf(const Region& r1, const Region& r2)
{
	vector<Polyline> res;

	vector<Polygon> pygs1 = r1.holes;
	pygs1.push_back(r1.boundary);
	vector<Polygon> pygs2 = r2.holes;
	pygs2.push_back(r2.boundary);
	for (auto& pg1 : pygs1)
	{
		for (auto& pg2 : pygs2)
		{
			auto pls = getBoundaryOf(pg1, pg2);
			res.insert(res.end(), pls.begin(), pls.end());
		}
	}
	return res;
}

vector<Polyline> CableRouter::getBoundaryOf(Polygon p1, Polygon p2)
{
	vector<Polyline> res;
	if (p1.size() < 3 || p2.size() < 3) return res;
	p2.reverse_orientation();
	int i = 0, j = 0;
	int n1 = p1.size(), n2 = p2.size();
	int start = 0;
	while (start < n1)
	{
		Polyline pl;
		Point pt = p1.vertex(start);
		j = 0;
		while (j < n2 && !POINT_EQUAL(p2.vertex(j), pt)) j++;
		if (j == n2)
		{
			start++;
			continue;
		}
		pl.push_back(pt);
		i = (start + 1) % n1;
		j = (j + 1) % n2;
		while (i != start && POINT_EQUAL(p1.vertex(i), p2.vertex(j)))
		{
			pl.push_back(p1.vertex(i));
			i = (i + 1) % n1;
			j = (j + 1) % n2;
		}
		if (i == start) pl.push_back(pt);
		if (pl.size() > 1) res.push_back(pl);
		start += pl.size();
	}
	return res;
}