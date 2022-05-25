#include "MapUtils.h"

#define FARWAY 12000
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
		Device dev(d.coord.transform(rotate), d.id, d.mass, d.layout);
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

CDTP CableRouter::buildTriangulation(MapInfo* const data)
{
	printf("Triangulation begin\n");

	CDTP dt;
	set<Cid> obstacle_cid;
	set<Constraint> constraints;

	// insert device, id from 0 to dev_n - 1
	for (auto v = data->devices.begin(); v != data->devices.end(); v++)
	{
		auto vh = dt.insert(v->coord);
		vh->info() = VertexInfo(v->id, true);
	}

	// insert area point
	Polygon area_boundary = data->area.info.boundary;
	for (int i = 0; i < area_boundary.size(); i++)
		constraints.insert(Constraint(area_boundary.vertex(i), area_boundary.vertex((i + 1) % area_boundary.size())));

	// insert holes point
	for (auto& h : data->holes)
		for (int i = 0; i < h.size(); i++)
			constraints.insert(Constraint(h.vertex(i), h.vertex((i + 1) % h.size())));

	for (auto& c : constraints)
		obstacle_cid.insert(dt.insert_constraint(c.source, c.target));

	int id = (int)data->devices.size();
	// parse_groups other points' id
	for (auto v = dt.finite_vertices_begin(); v != dt.finite_vertices_end(); v++)
	{
		if (v->info().id < 0) {
			v->info().id = id++;
		}
	}
	mark_domains(dt, obstacle_cid);

	printf("Triangulation over\n");

	return dt;
}

double** CableRouter::buildGraphAll(MapInfo* const data, const CDTP& cdt, int n, bool center_weighted, bool room_weighted)
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

void CableRouter::addPowerEdges(MapInfo* const data, const CDTP& dt, double** G, bool center_weighted, bool room_weighted)
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

void CableRouter::adjustByLayoutType(MapInfo* const data, double** G)
{
	int dn = (int)data->devices.size();
	for (int u = 0; u < dn; u++)
	{
		for (int v = u + 1; v < dn; v++)
		{
			if (G[u][v] != CR_INF)
			{
				double prior = priority_between(data->devices[u].layout, data->devices[v].layout);
				if (EQUAL(prior, 0))
					G[u][v] = G[v][u] = CR_INF;
				else
					G[u][v] = G[v][u] = G[u][v] / prior;
			}
		}
	}
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

	auto fl = getFittingLines(data, 100);
	for (auto l : fl)
	{
		for (int i = 0; i + 1 < l.size(); i++)
		{
			int u = l[i].id, v = l[i + 1].id;
			double w = G[u][v];
			if (APPRO_EQUAL(w, DIST(data->devices[u].coord, data->devices[v].coord)))
			{
				G[u][v] = G[v][u] = w / 1.5;
			}
		}
	}
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
	vector<Point> inter;
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
				inter.push_back(pt);
			}
			else if (CGAL::assign(seg, result))
			{
				res += 1000;
			}
		}
	}
	sort(inter.begin(), inter.end(), compare_point_by_x_y);
	res += points_simple(inter).size();
	return res;
}

int CableRouter::getRegionId(MapInfo& map, Point pos)
{
	for (int rid = 0; rid < map.regions.size(); rid++)
		if (!map.regions[rid].has_on_unbounded_side(pos))
			return rid;
	return -1;
}

void CableRouter::preprocess(MapInfo& map)
{
	deleteInvalidDevice(map);
	//deleteInvalidPower(data);
	correctInvalidPower(map);
	for (int i = 0; i < map.devices.size(); i++)
	{
		map.devices[i].region_id = getRegionId(map, map.devices[i].coord);
	}
	for (int i = 0; i < map.powers.size(); i++)
	{
		map.powers[i].region_id = getRegionId(map, map.powers[i].points[0]);
	}

	for (int i = 0; i < map.regions.size(); i++)
	{
		map.regions[i].align = to_first_quadrant(map.regions[i].align);
		reset(map.regions[i].centers);
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
		if (map.regions[i].align_center)
			map.regions[i].centers = rearrangeCenters(map.regions[i].centers);
	}

	reset(map.borders);
	for (int i = 0; i < map.regions.size(); i++)
	{
		//for (int j = i + 1; j < map.regions.size(); j++)
		//{
		//	auto res = getBoundaryOf(map.regions[i], map.regions[j]);
		//	for (auto pl : res)
		//		for (int k = 0; k < pl.size() - 1; k++)
		//			map.borders.push_back(Segment(pl[k], pl[k + 1]));
		//}
		auto pygs = map.regions[i].holes;
		pygs.push_back(map.regions[i].boundary);
		for (auto pyg : pygs)
			for (auto eit = pyg.edges_begin(); eit != pyg.edges_end(); eit++)
				map.borders.push_back(*eit);
	}
}

void CableRouter::deleteMapInfo(MapInfo& map)
{
	auto cen_all = map.cen_line_tree->all();
	set<rbush::TreeNode<SElement>*> cen_set(cen_all->begin(), cen_all->end());
	for (auto it = cen_set.begin(); it != cen_set.end(); it++)
	{
		delete *it;
	}
	reset(*cen_all);
	delete cen_all;

	auto hole_all = map.hole_tree->all();
	set<rbush::TreeNode<PElement>*> hole_set(hole_all->begin(), hole_all->end());
	for (auto it = hole_set.begin(); it != hole_set.end(); it++)
	{
		delete* it;
	}
	reset(*hole_all);
	delete hole_all;

	auto area_all = map.area.area_edge_tree->all();
	set<rbush::TreeNode<SElement>*> area_set(area_all->begin(), area_all->end());
	for (auto it = area_set.begin(); it != area_set.end(); it++)
	{
		delete* it;
	}
	reset(*area_all);
	delete area_all;

	auto room_all = map.room_tree->all();
	set<rbush::TreeNode<PElement>*> room_set(room_all->begin(), room_all->end());
	for (auto it = room_set.begin(); it != room_set.end(); it++)
	{
		delete* it;
	}
	reset(*room_all);
	delete room_all;

	delete map.cen_line_tree;
	map.cen_line_tree = NULL;
	delete map.area.area_edge_tree;
	map.area.area_edge_tree = NULL;
	delete map.hole_tree;
	map.hole_tree = NULL;
	delete map.room_tree;
	map.room_tree = NULL;
}

void CableRouter::deleteInvalidDevice(MapInfo& map)
{
	vector<Device> valid_dev;
	set<Point> exist;
	for (int i = 0; i < map.devices.size(); i++)
	{
		Device dev = map.devices[i];

		if (exist.find(dev.coord) != exist.end())
			continue;

		if (!isValidPoint(map, dev.coord))
			continue;

		valid_dev.push_back(Device(dev.coord, (int)valid_dev.size(), dev.mass, dev.layout));
		exist.insert(dev.coord);
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

			//if (!isValidPoint(map, pos))
			//	continue;

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

bool compare_device_by_x(Device a, Device b)
{
	return a.coord.hx() < b.coord.hx();
}

bool compare_device_by_y(Device a, Device b)
{
	return a.coord.hy() < b.coord.hy();
}

vector<vector<int>> CableRouter::getFittingLines(vector<Device> devices, double gap, Direction align)
{
	vector<vector<int>> res;

	if (devices.size() < 2) return res;

	Transformation rotate = get_tf_from_dir(align).inverse();
	for (auto& d : devices)
		d.coord = d.coord.transform(rotate);
	vector<Device> line;
	double last;

	sort(devices.begin(), devices.end(), compare_device_by_x);
	last = devices[0].coord.hx();
	line = { devices[0] };
	for (int i = 1; i < devices.size(); i++)
	{
		double now = devices[i].coord.hx();
		if (now - last < gap)
		{
			last = (last * line.size() + now) / (line.size() + 1);
			line.push_back(devices[i]);
		}
		else
		{
			if (line.size() > 1) {
				sort(line.begin(), line.end(), compare_device_by_y);
				vector<int> ll;
				for (auto d : line)
					ll.push_back(d.id);
				res.push_back(ll);
			}
			last = now;
			line = { devices[i] };
		}
	}
	if (line.size() > 1) {
		sort(line.begin(), line.end(), compare_device_by_y);
		vector<int> ll;
		for (auto d : line)
			ll.push_back(d.id);
		res.push_back(ll);
	}

	sort(devices.begin(), devices.end(), compare_device_by_y);
	last = devices[0].coord.hy();
	line = { devices[0] };
	for (int i = 1; i < devices.size(); i++)
	{
		double now = devices[i].coord.hy();
		if (now - last < gap)
		{
			last = (last * line.size() + now) / (line.size() + 1);
			line.push_back(devices[i]);
		}
		else
		{
			if (line.size() > 1) {
				sort(line.begin(), line.end(), compare_device_by_x);
				vector<int> ll;
				for (auto d : line)
					ll.push_back(d.id);
				res.push_back(ll);
			}
			last = now;
			line = { devices[i] };
		}
	}
	if (line.size() > 1) {
		sort(line.begin(), line.end(), compare_device_by_x);
		vector<int> ll;
		for (auto d : line)
			ll.push_back(d.id);
		res.push_back(ll);
	}

	return res;
}

vector<vector<Device>> CableRouter::getFittingLines(MapInfo* const map, double gap)
{
	vector<vector<Device>> res;

	if (map->devices.size() < 2) return res;

	vector<vector<Device>> devices(map->regions.size() + 1);
	for (auto d : map->devices)
	{
		int rid = d.region_id;
		if (rid == -1) rid = map->regions.size();
		devices[rid].push_back(d);
	}
	for (int rid = 0; rid < map->regions.size(); rid++)
	{
		if (map->regions[rid].align_center)
			continue;
		auto lines = getFittingLines(devices[rid], gap, map->regions[rid].align);
		for (auto l : lines)
		{
			vector<Device> line;
			for (auto id : l)
				line.push_back(map->devices[id]);
			res.push_back(line);
		}
	}
	auto lines = getFittingLines(devices.back(), gap);
	for (auto l : lines)
	{
		vector<Device> line;
		for (auto id : l)
			line.push_back(map->devices[id]);
		res.push_back(line);
	}
	return breakFittingLines(map, res);
}

vector<vector<Device>> CableRouter::breakFittingLine(const vector<Device>& input, MapInfo* const map)
{
	vector<vector<Device>> res;

	if (input.size() < 2) return res;

	vector<Device> l;
	Device last = input[0];
	l = { input[0] };
	for (int i = 1; i < input.size(); i++)
	{
		Device now = input[i];
		Segment s(last.coord, now.coord);
		if (LEN(s) > FARWAY || crossObstacle(map, s))
		{
			if (l.size() > 1)
				res.push_back(l);
			last = now;
			l = { input[i] };
		}
		else
		{
			last = now;
			l.push_back(input[i]);
		}
	}
	if (l.size() > 1)
		res.push_back(l);

	return res;
}

vector<vector<Device>> CableRouter::breakFittingLine(const vector<Device>& input, vector<Segment>& lines, vector<int>& vis)
{
	vector<vector<Device>> res;

	if (input.size() < 2) return res;

	vector<Device> l;
	Device last = input[0];
	l = { input[0] };
	for (int i = 1; i < input.size(); i++)
	{
		Device now = input[i];
		Segment s(last.coord, now.coord);
		if (vis[last.id] || vis[now.id] || cross_lines(lines, last.coord, now.coord))
		{
			if (l.size() > 1)
				res.push_back(l);
			last = now;
			l = { input[i] };
		}
		else
		{
			last = now;
			l.push_back(input[i]);
		}
	}
	if (l.size() > 1)
		res.push_back(l);

	return res;
}

vector<vector<Device>> CableRouter::breakFittingLines(MapInfo* const map, vector<vector<Device>> groups)
{
	vector<vector<Device>> res;
	if (groups.size() < 1) return res;

	for (int i = 0; i < groups.size(); i++)
	{
		auto break_lines = breakFittingLine(groups[i], map);
		res.insert(res.end(), break_lines.begin(), break_lines.end());
	}

	vector<Segment> lines;
	vector<int> vis(map->devices.size(), 0);
	priority_queue<vector<Device>, vector<vector<Device>>, vector_less<vector<Device>>> q;
	for (auto l : res)
		q.push(l);
	reset(res);
	while (!q.empty())
	{
		auto top = q.top();
		q.pop();
		auto size = top.size();
		auto break_lines = breakFittingLine(top, lines, vis);
		if (break_lines.size() == 1 && break_lines[0].size() == size)
		{
			res.push_back(break_lines.front());
			for (auto l : break_lines)
				for (int i = 0; i < l.size(); i++)
				{
					vis[l[i].id] = 1;
					if (i > 0) lines.push_back(Segment(l[i].coord, l[i - 1].coord));
				}
		}
		else
			for (auto l : break_lines) q.push(l);
	}
	return res;
}

vector<Segment> CableRouter::rearrangeCenters(const vector<Segment> centers)
{
	vector<Segment> res;
	vector<vector<Point>> break_pts(centers.size());
	for (int i = 0; i < centers.size(); i++)
	{
		break_pts[i].push_back(centers[i].source());
		break_pts[i].push_back(centers[i].target());
	}
	for (int i = 0; i < centers.size(); i++)
	{
		for (int j = i + 1; j < centers.size(); j++)
		{
			Segment si = centers[i], sj = centers[j];
			if (DIST(si, sj) < 1e-4 &&
				!POINT_EQUAL(si.source(), sj.source()) && 
				!POINT_EQUAL(si.source(), sj.target()) &&
				!POINT_EQUAL(si.target(), sj.source()) && 
				!POINT_EQUAL(si.target(), sj.target()))
			{
				if (!CGAL::do_intersect(si, sj))
				{
					Point pt; Segment seg; int sid;
					if (DIST(si.source(), sj) < 1e-4)
					{
						pt = si.source();
						seg = sj;
						sid = j;
					}
					else if (DIST(si.target(), sj) < 1e-4)
					{
						pt = si.target();
						seg = sj;
						sid = j;
					}
					else if (DIST(sj.source(), si) < 1e-4)
					{
						pt = sj.source();
						seg = si;
						sid = i;
					}
					else
					{
						pt = sj.target();
						seg = si;
						sid = i;
					}
					pt = project_point_to_segment(pt, seg);
					if (!POINT_EQUAL(seg.source(), pt) &&
						!POINT_EQUAL(seg.target(), pt))
						break_pts[sid].push_back(pt);
				}
				else
				{
					CGAL::Object result = CGAL::intersection(si, sj);
					Point pt;
					if (CGAL::assign(pt, result))
					{
						if (!POINT_EQUAL(si.source(), pt) &&
							!POINT_EQUAL(si.target(), pt))
							break_pts[i].push_back(pt);
						if (!POINT_EQUAL(sj.source(), pt) &&
							!POINT_EQUAL(sj.target(), pt))
							break_pts[j].push_back(pt);
					}
					else
					{
						printf("Error::CableRouter::rearrangeCenters:: WTF???\n");
					}
				}
			}
		}
	}
	for (int i = 0; i < break_pts.size(); i++)
	{
		auto pts = break_pts[i];
		if (pts.size() == 2) {
			res.push_back(centers[i]);
			continue;
		}
		sort(pts.begin(), pts.end(), compare_point_by_x_y);
		Point last = pts[0];
		for (int j = 1; j < pts.size(); j++)
		{
			Point now = pts[j];
			if (!POINT_EQUAL(last, now))
			{
				res.push_back(Segment(last, now));
				last = now;
			}
		}
	}

	return res;
}

void CableRouter::expandRooms(vector<Polygon>& rooms, const Polygon& area)
{
	vector<int> room_id;
	vector<Polygon> holes;
	for (int i = 0; i < rooms.size(); i++)
	{
		Polygon room1 = rooms[i];
		vector<Segment> segs;
		for (int j = i + 1; j < rooms.size(); j++)
		{
			Polygon room2 = rooms[j];
			for (int k = 0; k < room2.size(); k++)
				segs.push_back(room2.edge(k));
		}
		for (int j = 0; j < area.size(); j++)
		{
			segs.push_back(area.edge(j));
		}
		for (int j = 0; j < room1.size(); j++)
		{
			Segment s1 = room1.edge(j);
			Vector v1(s1);
			for (auto s2 : segs)
			{
				Vector v2(s2);
				if (abs(VEC_COS(v1, v2)) > 0.8 && DIST(s1, s2) < 300)
				{
					vector<Point> pts, res;
					pts.push_back(project_point_to_segment(s1.source(), s2));
					pts.push_back(project_point_to_segment(s1.target(), s2));
					if (POINT_EQUAL(pts[0], pts[1])) continue;
					pts.push_back(project_point_to_segment(s2.source(), s1));
					if (POINT_EQUAL(pts[0], pts[2])) continue;
					if (POINT_EQUAL(pts[1], pts[2])) continue;
					pts.push_back(project_point_to_segment(s2.target(), s1));
					if (POINT_EQUAL(pts[0], pts[3])) continue;
					if (POINT_EQUAL(pts[1], pts[3])) continue;
					if (POINT_EQUAL(pts[2], pts[3])) continue;
					pts[0] = expand_segment(Segment(s1.source(), pts[0])).target();
					pts[1] = expand_segment(Segment(s1.target(), pts[1])).target();
					pts[2] = expand_segment(Segment(s2.source(), pts[2])).target();
					pts[3] = expand_segment(Segment(s2.target(), pts[3])).target();
					CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(res));
					holes.push_back(construct_polygon(&res));
					room_id.push_back(i);
				}
			}
		}
	}
	for (int i = 0; i < holes.size(); i++)
	{
		EPolygon room = polygon_to_epolygon(rooms[room_id[i]]);
		EPolygon hole = polygon_to_epolygon(holes[i]);
		EPolygon_with_holes res;
		if (CGAL::join(room, hole, res))
			rooms[room_id[i]] = epolygon_to_polygon(res.outer_boundary());
		else
			printf("can't join!\n");
	}
}


double CableRouter::priority_between(LayoutType a, LayoutType b)
{
	if (a == b)
		return 1.0;
	if (a == Ground || b == Ground)
		return 0.0;

	return 0.5;
}
