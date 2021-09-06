#include "MapUtils.h"

using namespace CableRouter;

bool CableRouter::touchObstacle(MapInfo* const data, const Point p, const Point q)
{
	Segment s(p, q);

	// area
	auto edges = segment_search(data->area.area_edge_tree, s.source(), s.target());
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		Segment seg = *(*eit)->data;
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

CDT CableRouter::buildTriangulation(MapInfo* const data)
{
	printf("Triangulation begin\n");

	CDT dt;

	vector<std::pair<Point, VertexInfo>> vec;
	vector<Point> vec_con;

	// insert device, id from 0 to dev_n - 1
	for (auto v = data->devices.begin(); v != data->devices.end(); v++)
	{
		vec.push_back(std::make_pair(v->coord, VertexInfo(v->id, true)));
	}
	dt.insert(vec.begin(), vec.end());

	// insert area point
	reset(vec);
	for (auto v = data->area.info.boundary.vertices_begin(); v != data->area.info.boundary.vertices_end(); v++)
	{
		vec.push_back(std::make_pair(*v, VertexInfo()));
		vec_con.push_back(*v);
	}
	dt.insert(vec.begin(), vec.end());
	dt.insert_constraint(vec_con.begin(), vec_con.end(), true);

	// insert holes point
	std::vector<rbush::TreeNode<PElement>* >& holes = (*data->hole_tree->all());
	for (auto h = holes.begin(); h != holes.end(); h++)
	{
		reset(vec);
		reset(vec_con);
		for (auto v = (*h)->data->boundary.vertices_begin(); v != (*h)->data->boundary.vertices_end(); v++)
		{
			vec.push_back(std::make_pair(*v, VertexInfo()));
			vec_con.push_back(*v);
		}
		dt.insert(vec.begin(), vec.end());
		dt.insert_constraint(vec_con.begin(), vec_con.end(), true);
	}

	int id = data->devices.size();
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
			if (G[pi][qi] == CR_INF)
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
		data->powers[i].id = dt.number_of_vertices() + i;
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
	int dn = data->devices.size();
	int pn = data->powers.size();
	int on = n - dn - pn;
	for (int start = 0; start < dn + pn; start++)
	{
		double* dis = new double[n];
		bool* vis = new bool[n];
		fill(dis, dis + n, CR_INF);
		fill(vis, vis + n, false);
		int sid = start < dn ? start : start + on;
		dis[sid] = 0;
		for (int i = 0; i < n; i++)
		{
			int MIN = CR_INF, u = -1;
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

		delete[] dis;
		delete[] vis;
	}
	printf("Shifting obstacle points end\n");
}


void CableRouter::addWeightCenters(MapInfo* const data, const Point p, const Point q, double& w)
{
	vector<rbush::TreeNode<Segment>* > centers = segment_search(data->cen_line_tree, p, q);

	for (auto eit = centers.begin(); eit != centers.end(); eit++)
	{
		if (CGAL::do_intersect(Segment(p, q), *(*eit)->data))
		{
			w += 5000;
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
				//w += (*oit)->data->weight;
				w += 10000;
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
		Segment seg = *(*eit)->data;
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