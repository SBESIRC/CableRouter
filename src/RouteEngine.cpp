#include "RouteEngine.h"

using namespace CableRouter;

#define OVERLAP_ACCEPT 1600
#define LINE_GAP 500

bool open_list_greater(ASNode* a, ASNode* b)
{
	return a->f() < b->f();
}
bool open_list_less(ASNode* a, ASNode* b)
{
	return a->f() > b->f();
}

ASPath CableRouter::a_star_connect_p2p(MapInfo* const data, Point s, Point t, vector<Segment>& lines)
{
	CDT dt;
	set<Constraint> constraints;

	// start and end
	vector<pair<Point, VertexInfo>> vec;
	vec.push_back(make_pair(s, VertexInfo(0, true)));
	vec.push_back(make_pair(t, VertexInfo(1, true)));

	// area
	Polygon area_boundary = data->area.info.boundary;
	for (int i = 0; i < area_boundary.size(); i++)
	{
		constraints.insert(Constraint(area_boundary.vertex(i), area_boundary.vertex((i + 1) % area_boundary.size())));
	}

	// holes
	for (auto h = data->holes.begin(); h != data->holes.end(); h++)
	{
		for (int i = 0; i < h->size(); i++)
		{
			constraints.insert(Constraint(h->vertex(i), h->vertex((i + 1) % h->size())));
		}
	}

	// rooms
	for (auto r = data->rooms.begin(); r != data->rooms.end(); r++)
	{
		for (int i = 0; i < r->size(); i++)
		{
			Point source = r->vertex(i);
			Point target = r->vertex((i + 1) % r->size());
			int num = 1 + ((int)DIST(source, target)) / 2000;
			Point start = source;
			for (int i = 1; i <= num; i++)
			{
				Point end = Point(
					1.0 * i / num * DOUBLE(target.hx() - source.hx()) + DOUBLE(source.hx()),
					1.0 * i / num * DOUBLE(target.hy() - source.hy()) + DOUBLE(source.hy()));
				constraints.insert(Constraint(start, end));
				start = end;
			}
			Point mid = CGAL::midpoint(source, target);
			if (isValidPoint(*data, mid))
				vec.push_back(make_pair(mid, VertexInfo(-1, true)));
		}
	}

	// centers
	//for (auto c = data->centers.begin(); c != data->centers.end(); c++)
	//{
	//	Point start = c->source();
	//	Point mid = CGAL::midpoint(c->source(), c->target());
	//	if (mid != s && mid != t)
	//		vec.push_back(make_pair(mid, VertexInfo(2, true)));
	//	int num = 1 + ((int)LEN(*c)) / 2000;
	//	for (int i = 1; i <= num; i++)
	//	{
	//		Point end = Point(
	//			1.0 * i / num * DOUBLE(c->target().hx() - c->source().hx()) + DOUBLE(c->source().hx()),
	//			1.0 * i / num * DOUBLE(c->target().hy() - c->source().hy()) + DOUBLE(c->source().hy()));
	//		constraints.insert(Constraint(start, end));
	//		start = end;
	//	}
	//}

	// line
	for (auto l = lines.begin(); l != lines.end(); l++)
	{
		Point start = l->source();
		Point mid = CGAL::midpoint(l->source(), l->target());
		//if (mid != s && mid != t)
		//	vec.push_back(make_pair(mid, VertexInfo(2, true)));
		int num = 1 + ((int)LEN(*l)) / 2000;
		for (int i = 1; i <= num; i++)
		{
			Point end = Point(
				1.0 * i / num * DOUBLE(l->target().hx() - l->source().hx()) + DOUBLE(l->source().hx()),
				1.0 * i / num * DOUBLE(l->target().hy() - l->source().hy()) + DOUBLE(l->source().hy()));
			constraints.insert(Constraint(start, end));
			start = end;
		}
	}

	//for (auto v = data->devices.begin(); v != data->devices.end(); v++)
	//{
	//	if (v->coord == s || v->coord == t) continue;
	//	vec.push_back(make_pair(v->coord, VertexInfo(2, true)));
	//}
	dt.insert(vec.begin(), vec.end());

	for (auto c = constraints.begin(); c != constraints.end(); c++)
	{
		dt.insert_constraint(c->source, c->target);
	}

	mark_domains(dt);

	//printf("Begin a star\n");

	map<Face_handle, int> fid;
	ASNode* nodes = new ASNode[dt.number_of_faces()];
	vector<ASNode*> openlist;
	int n = 0;

	for (auto feit = dt.finite_edges_begin(); feit != dt.finite_edges_end(); feit++)
	{
		if (feit->first->info().not_reach() ||
			feit->first->neighbor(feit->second)->info().not_reach())
		{
			continue;
		}
		//if (feit->first->is_constrained(feit->second))
		//	continue;

		Vertex_handle v1 = feit->first->vertex(dt.cw(feit->second));
		Vertex_handle v2 = feit->first->vertex(dt.ccw(feit->second));

		Face_handle f1 = feit->first;
		Face_handle f2 = feit->first->neighbor(feit->second);

		if (fid.find(f1) == fid.end())
		{
			fid.insert(make_pair(f1, n));
			nodes[n].id = n;
			nodes[n].face = f1;
			nodes[n].is_end = (v1->info().id == 1 || v2->info().id == 1);
			nodes[n].mid = focus_point(f1);
			nodes[n].h = DIST(nodes[n].mid, t);
			if (v1->info().id == 0 || v2->info().id == 0)
			{
				nodes[n].g = DIST(s, nodes[n].mid);
				nodes[n].cross_num = 0;
				nodes[n].open = true;
				nodes[n].is_start = true;
				openlist.push_back(&nodes[n]);
			}
			n++;
		}
		else
		{
			int i = fid[f1];
			if (!nodes[i].open && (v1->info().id == 0 || v2->info().id == 0))
			{
				nodes[i].g = DIST(s, nodes[i].mid);
				nodes[i].cross_num = 0;
				nodes[i].open = true;
				nodes[i].is_start = true;
				openlist.push_back(&nodes[i]);
			}
			if (!nodes[i].is_end && (v1->info().id == 1 || v2->info().id == 1))
			{
				nodes[i].is_end = true;
			}
		}
		if (fid.find(f2) == fid.end())
		{
			fid.insert(make_pair(f2, n));
			nodes[n].id = n;
			nodes[n].face = f2;
			nodes[n].is_end = (v1->info().id == 1 || v2->info().id == 1);
			nodes[n].mid = focus_point(f2);
			nodes[n].h = DIST(nodes[n].mid, t);
			if (v1->info().id == 0 || v2->info().id == 0)
			{
				nodes[n].g = DIST(s, nodes[n].mid);
				nodes[n].cross_num = 0;
				nodes[n].open = true;
				nodes[n].is_start = true;
				openlist.push_back(&nodes[n]);
			}
			n++;
		}
		else
		{
			int i = fid[f2];
			if (!nodes[i].open && (v1->info().id == 0 || v2->info().id == 0))
			{
				nodes[i].g = DIST(s, nodes[i].mid);
				nodes[i].cross_num = 0;
				nodes[i].open = true;
				nodes[i].is_start = true;
				openlist.push_back(&nodes[i]);
			}
			if (!nodes[i].is_end && (v1->info().id == 1 || v2->info().id == 1))
			{
				nodes[i].is_end = true;
			}
		}

		int n1 = fid[f1];
		int n2 = fid[f2];
		nodes[n1].neighbor.insert(n2);
		nodes[n2].neighbor.insert(n1);

		//Point p = feit->first->vertex(dt.cw(feit->second))->point();
		//Point q = feit->first->vertex(dt.ccw(feit->second))->point();
		//res.push_back(Segment(p, q));

	}

	//for (int i = 0; i < openlist.size(); i++)
	//{
	//	ASNode* no = openlist[i];
	//	printf("openlist[%d] id = %d\n", i, openlist[i]->id);
	//}

	//for (int i = 0; i < nodes.size(); i++)
	//{
	//	printf("nodes[%d]: id = %d, end:%d, open:%d, neighbor size:%d\n", i, nodes[i].id, nodes[i].is_end, nodes[i].open, nodes[i].neighbor.size());
	//}
	//return res;

	//printf("Node size: %d, map size: %d, openlist size: %d\n", n, fid.size(), openlist.size());

	ASNode* end_node = NULL;

	sort(openlist.begin(), openlist.end(), open_list_less);

	while (!openlist.empty())
	{
		ASNode* no = openlist.back();
		openlist.pop_back();

		if (no->is_end)
		{
			end_node = no;
			break;
		}

		no->close = true;
		for (auto i = no->neighbor.begin(); i != no->neighbor.end(); i++)
		{
			ASNode* next = &nodes[*i];
			if (next->close) continue;

			double w = DIST(no->mid, next->mid);
			bool cross_cons = false;
			int k = 0;
			while (no->face->neighbor(k) != next->face) k++;
			if (dt.is_constrained(make_pair(no->face, k))) { 
				w += 100000; 
				cross_cons = true;
			}

			if (!next->open)
			{
				next->g = no->g + w;
				next->parent = no->id;
				next->open = true;
				next->cross_num = no->cross_num + (cross_cons ? 1 : 0);
				openlist.push_back(next);
				sort(openlist.begin(), openlist.end(), open_list_less);
			}
			else if (next->g > no->g + w)
			{
				next->g = no->g + w;
				next->cross_num = no->cross_num + (cross_cons ? 1 : 0);
				if (!next->is_start) next->parent = no->id;
				sort(openlist.begin(), openlist.end(), open_list_less);
			}
		}
	}
	if (end_node == NULL)
	{
		printf("NO PATH!\n");
		delete[] nodes;
		dt.clear();
		return ASPath();
	}

	Polyline path = funnel_smooth(dt, nodes, s, t, end_node->id);
	reverse(path.begin(), path.end());

	ASPath ap;
	ap.path = path;
	ap.cross_num = vector<int>(path.size(), 0);
	for (int i = 1; i < path.size(); i++)
	{
		Segment seg(path[i - 1], path[i]);
		ap.cross_num[i] = ap.cross_num[i - 1] + crossRoom(data, seg);
	}

	//printf("Path size: %d\n", path.size());
	delete[] nodes;
	dt.clear();
	return ap;
}

Polyline CableRouter::a_star_connect_p2s(MapInfo* const data, Point s, Segment t, vector<Segment>& lines)
{

	CDT dt;

	// start
	vector<pair<Point, VertexInfo>> vec;
	vec.push_back(make_pair(s, VertexInfo(0, true)));

	// end
	vector<Point> end_points = { t.source(), t.target() };
	end_points = line_break(end_points, 1000);
	set<Point> end_set;
	for (int i = 0; i < end_points.size(); i++)
	{
		vec.push_back(make_pair(end_points[i], VertexInfo(1, true)));
		end_set.insert(end_points[i]);
	}
	dt.insert_constraint(t.source(), t.target());

	// area
	dt.insert_constraint(data->area.info.boundary.vertices_begin(), data->area.info.boundary.vertices_end(), true);

	// holes
	for (auto h = data->holes.begin(); h != data->holes.end(); h++)
	{
		dt.insert_constraint(h->vertices_begin(), h->vertices_end(), true);
	}

	// line
	for (auto l = lines.begin(); l != lines.end(); l++)
	{
		Point start = l->source();
		Point mid = CGAL::midpoint(l->source(), l->target());
		if (mid != s && end_set.find(mid) == end_set.end())
			vec.push_back(make_pair(mid, VertexInfo(2, true)));
		int num = 1 + ((int)LEN(*l)) / 1000;
		for (int i = 1; i <= num; i++)
		{
			Point end = Point(
				1.0 * i / num * DOUBLE(l->target().hx() - l->source().hx()) + DOUBLE(l->source().hx()),
				1.0 * i / num * DOUBLE(l->target().hy() - l->source().hy()) + DOUBLE(l->source().hy()));
			dt.insert_constraint(start, end);
			start = end;
		}
	}

	dt.insert(vec.begin(), vec.end());

	mark_domains(dt);

	map<Face_handle, int> fid;
	ASNode* nodes = new ASNode[dt.number_of_faces()];
	vector<ASNode*> openlist;
	int n = 0;

	for (auto feit = dt.finite_edges_begin(); feit != dt.finite_edges_end(); feit++)
	{
		if (feit->first->info().not_reach() ||
			feit->first->neighbor(feit->second)->info().not_reach())
		{
			continue;
		}
		//if (feit->first->is_constrained(feit->second))
		//	continue;

		Vertex_handle v1 = feit->first->vertex(dt.cw(feit->second));
		Vertex_handle v2 = feit->first->vertex(dt.ccw(feit->second));

		Face_handle f1 = feit->first;
		Face_handle f2 = feit->first->neighbor(feit->second);

		if (fid.find(f1) == fid.end())
		{
			fid.insert(make_pair(f1, n));
			nodes[n].id = n;
			nodes[n].face = f1;
			nodes[n].is_end = (v1->info().id == 1 || v2->info().id == 1);
			nodes[n].mid = focus_point(f1);
			nodes[n].h = DIST(nodes[n].mid, t);
			if (v1->info().id == 0 || v2->info().id == 0)
			{
				nodes[n].g = DIST(s, nodes[n].mid);
				nodes[n].open = true;
				nodes[n].is_start = true;
				openlist.push_back(&nodes[n]);
			}
			n++;
		}
		else
		{
			int i = fid[f1];
			if (!nodes[i].open && (v1->info().id == 0 || v2->info().id == 0))
			{
				nodes[i].g = DIST(s, nodes[i].mid);
				nodes[i].open = true;
				nodes[i].is_start = true;
				openlist.push_back(&nodes[i]);
			}
			if (!nodes[i].is_end && (v1->info().id == 1 || v2->info().id == 1))
			{
				nodes[i].is_end = true;
			}
		}
		if (fid.find(f2) == fid.end())
		{
			fid.insert(make_pair(f2, n));
			nodes[n].id = n;
			nodes[n].face = f2;
			nodes[n].is_end = (v1->info().id == 1 || v2->info().id == 1);
			nodes[n].mid = focus_point(f2);
			nodes[n].h = DIST(nodes[n].mid, t);
			if (v1->info().id == 0 || v2->info().id == 0)
			{
				nodes[n].g = DIST(s, nodes[n].mid);
				nodes[n].open = true;
				nodes[n].is_start = true;
				openlist.push_back(&nodes[n]);
			}
			n++;
		}
		else
		{
			int i = fid[f2];
			if (!nodes[i].open && (v1->info().id == 0 || v2->info().id == 0))
			{
				nodes[i].g = DIST(s, nodes[i].mid);
				nodes[i].open = true;
				nodes[i].is_start = true;
				openlist.push_back(&nodes[i]);
			}
			if (!nodes[i].is_end && (v1->info().id == 1 || v2->info().id == 1))
			{
				nodes[i].is_end = true;
			}
		}

		int n1 = fid[f1];
		int n2 = fid[f2];
		nodes[n1].neighbor.insert(n2);
		nodes[n2].neighbor.insert(n1);

	}

	ASNode* end_node = NULL;

	sort(openlist.begin(), openlist.end(), open_list_less);

	while (!openlist.empty())
	{
		ASNode* no = openlist.back();
		openlist.pop_back();

		if (no->is_end)
		{
			end_node = no;
			break;
		}

		no->close = true;
		for (auto i = no->neighbor.begin(); i != no->neighbor.end(); i++)
		{
			ASNode* next = &nodes[*i];
			if (next->close) continue;

			double w = DIST(no->mid, next->mid);
			int k = 0;
			while (no->face->neighbor(k) != next->face) k++;
			if (dt.is_constrained(make_pair(no->face, k))) w *= 5;

			if (!next->open)
			{
				next->g = no->g + w;
				next->parent = no->id;
				next->open = true;
				openlist.push_back(next);
				sort(openlist.begin(), openlist.end(), open_list_less);
			}
			else if (next->g > no->g + w)
			{
				next->g = no->g + w;
				if (!next->is_start) next->parent = no->id;
				sort(openlist.begin(), openlist.end(), open_list_less);
			}
		}
	}
	if (end_node == NULL)
	{
		printf("NO PATH!\n");
		delete[] nodes;
		dt.clear();
		return Polyline();
	}

	Point end_p;
	for (int i = 0; i < 3; i++)
	{
		if (end_node->face->vertex(i)->info().id == 1)
		{
			end_p = end_node->face->vertex(i)->point();
		}
	}

	Polyline path = funnel_smooth(dt, nodes, s, end_p, end_node->id);
	reverse(path.begin(), path.end());
	//printf("Path size: %d\n", path.size());
	delete[] nodes;
	dt.clear();
	return path;
}

Polyline CableRouter::funnel_smooth(CDT& dt, ASNode* nodes, Point s, Point t, int end_node_id)
{
	Polyline path;
	path.push_back(t);

	int now = end_node_id;
	Point left, right;
	Point p = t;
	bool new_start = true;
	int lid;
	int rid;;
	while (now != -1)
	{
		Point l, r;
		int next = nodes[now].parent;
		if (next != -1)
		{
			int k = 0;
			while (nodes[now].face->neighbor(k) != nodes[next].face) k++;
			l = nodes[now].face->vertex(dt.ccw(k))->point();
			r = nodes[now].face->vertex(dt.cw(k))->point();
			//res.push_back(Segment(l, r));
			//res.push_back(Segment(nodes[now].face->vertex(k)->point(), r));
			//res.push_back(Segment(l, nodes[now].face->vertex(k)->point()));
			//printf("Get l and r\n");
		}
		else
		{
			l = r = s;
			//printf("Get l and r as the start\n");
		}

		Segment ss = shrink_segment(Segment(l, r), 0.6);
		l = ss.source();
		r = ss.target();

		if (new_start)
		{
			left = l;
			right = r;
			lid = now;
			rid = now;
			now = next;
			new_start = false;
			//printf("New begin point, just save l and r\n");
			continue;
		}
		if (p == right)
		{
			left = l;
			right = r;
			lid = now;
			rid = now;
			now = next;
			//printf("p == right\n");
			continue;
		}
		if (p == left)
		{
			left = l;
			right = r;
			lid = now;
			rid = now;
			now = next;
			//printf("p == left\n");
			continue;
		}
		if (l != left)
		{

			//printf("l != left\n");
			// p-l  p-left p-right
			double a, b;

			double x1 = DOUBLE(left.hx() - p.hx());
			double y1 = DOUBLE(left.hy() - p.hy());
			double x2 = DOUBLE(right.hx() - p.hx());
			double y2 = DOUBLE(right.hy() - p.hy());
			double x = DOUBLE(l.hx() - p.hx());
			double y = DOUBLE(l.hy() - p.hy());
			a = (x * y2 - x2 * y) / (x1 * y2 - x2 * y1);
			b = (x * y1 - x1 * y) / (x2 * y1 - x1 * y2);
			if (a > 0 && b > 0)
			{
				//printf("pl between p left and p right\n");
				left = l;
				lid = now;
			}
			else
			{
				a = (x2 * y - x * y2) / (x1 * y - x * y1);
				b = (x2 * y1 - x1 * y2) / (x * y1 - x1 * y);
				if (a > 0 && b > 0)
				{
					//printf("pl cross p right\n");
					path.push_back(right);
					p = right;
					new_start = true;
					next = rid;
				}

			}
		}
		if (r != right)
		{
			//printf("r != right\n");
			// p-l  p-left p-right left-right
			double a, b;

			double x1 = DOUBLE(left.hx() - p.hx());
			double y1 = DOUBLE(left.hy() - p.hy());
			double x2 = DOUBLE(right.hx() - p.hx());
			double y2 = DOUBLE(right.hy() - p.hy());
			double x = DOUBLE(r.hx() - p.hx());
			double y = DOUBLE(r.hy() - p.hy());
			a = (x * y2 - x2 * y) / (x1 * y2 - x2 * y1);
			b = (x * y1 - x1 * y) / (x2 * y1 - x1 * y2);
			if (a > 0 && b > 0)
			{
				//printf("pr between p left and p right\n");
				right = r;
				rid = now;
			}
			else
			{
				a = (x1 * y2 - x2 * y1) / (x * y2 - x2 * y);
				b = (x1 * y - x * y1) / (x2 * y - x * y2);
				if (a > 0 && b > 0)
				{
					//printf("pr cross p left\n");
					path.push_back(left);
					p = left;
					new_start = true;
					next = lid;
				}

			}
		}


		now = next;

	}
	path.push_back(s);
	return path;
}

Polyline CableRouter::line_break(Polyline line, const double gap)
{
	if (line.size() < 2)
		return line;

	Polyline res;
	res.push_back(line[0]);
	for (int i = 0; i < (int)line.size() - 1; i++)
	{
		Point start = line[i];
		Point end = line[i + 1];
		int num = 1 + (int)(DIST(start, end) / gap);
		for (int i = 1; i <= num; i++)
		{
			Point next = Point(
				1.0 * i / num * DOUBLE(end.hx() - start.hx()) + DOUBLE(start.hx()),
				1.0 * i / num * DOUBLE(end.hy() - start.hy()) + DOUBLE(start.hy()));
			res.push_back(next);
		}
	}
	return res;
}

Polyline CableRouter::manhattan_smooth_p2p(MapInfo* const data, Polyline& path, vector<Segment>& exist_lines)
{
	Polyline line = path;
	if (line.size() <= 1) return line;

	Polyline res;
	int now = 0;
	res.push_back(line[now]);
	//bool dirX = true;
	//bool dirY = true;
	while (now != (int)line.size() - 1)
	{
		Point u = line[now];

		int best = -1;
		double best_value = CR_INF;
		bool best_has_mid = false;
		Point best_mid;

		for (int next = now + 1; next < line.size(); next++)
		{
			int far = ((int)line.size() - next);
			Point v = line[next];
			bool is_end = (next == ((int)line.size() - 1));
			if (EQUAL(u.hx(), v.hx()) || EQUAL(u.hy(), v.hy()))
			{
				//bool cross = crossObstacle(data, u, v);
				bool cross = touchObstacle(data, u, v);
				if (!cross)
				{
					int c = cross_num(exist_lines, u, v);
					double force = tooCloseToSun(data, u, v, exist_lines, is_end);
					double value = (1.0 + c) * (1.0 + force) + 10.0 * far;
					//double value = 10.0 * c + 1.0 * force + 5.0 * far;
					if (value <= best_value)
					{
						best_value = value;
						best = next;
						best_has_mid = false;
					}
				}
				continue;
			}

			Point mid1(v.hx(), u.hy());
			Point mid2(u.hx(), v.hy());
			//bool cross1 = crossObstacle(data, u, mid1) || crossObstacle(data, mid1, v);
			//bool cross2 = crossObstacle(data, u, mid2) || crossObstacle(data, mid2, v);			
			bool cross1 = touchObstacle(data, u, mid1) || touchObstacle(data, mid1, v);
			bool cross2 = touchObstacle(data, u, mid2) || touchObstacle(data, mid2, v);
			//bool best_choose1 = false;
			if (!cross1)
			{
				int c = cross_num(exist_lines, u, mid1) + cross_num(exist_lines, mid1, v);
				double force1 = tooCloseToSun(data, u, mid1, exist_lines) + tooCloseToSun(data, mid1, v, exist_lines, is_end);
				double value = (1.0 + c) * (1.0 + force1) + 10.0 * far;
				//double value = 10.0 * c + 1.0 * force1 + 5.0 * far;
				if (value <= best_value)
				{
					best_value = value;
					best = next;
					best_has_mid = true;
					best_mid = mid1;
					//best_choose1 = true;
				}
			}
			if (!cross2)
			{
				int c = cross_num(exist_lines, u, mid2) + cross_num(exist_lines, mid2, v);
				double force2 = tooCloseToSun(data, u, mid2, exist_lines) + tooCloseToSun(data, mid2, v, exist_lines, is_end);
				double value = (1.0 + c) * (1.0 + force2) + 10.0 * far;
				//double value = 10.0 * c + 1.0 * force2 + 5.0 * far;
				if (value <= best_value)
				{
					best_value = value;
					best = next;
					best_has_mid = true;
					//if (!best_choose1 || dirY)
					//{
						best_mid = mid2;
					//}
				}
			}
		}

		if (best != -1)
		{
			//printf("best value = %lf\n", best_value);
			if (best_has_mid)
			{
				//dirX = u.hy() == best_mid.hy();
				//dirY = u.hx() == best_mid.hx();
				line.insert(line.begin() + best, best_mid);
				now = best;
				res.push_back(line[now]);
			}
			else
			{
				//dirX = u.hy() == line[best].hy();
				//dirY = u.hx() == line[best].hx();
				now = best;
				res.push_back(line[now]);
			}
		}
		else
		{
			//dirX = u.hy() == line[now + 1].hy();
			//dirY = u.hx() == line[now + 1].hx();
			now++;
			res.push_back(line[now]);
		}
	}
	return res;
}

Polyline CableRouter::center_connect_p2p(MapInfo* const data, Polyline center, Point s, Point t, vector<Segment>& lines)
{
	Polyline res;
	int n = center.size();

	if (n < 2) return Polyline{ s,t };

	res.push_back(s);

	Vector source = center[1] - center[0];

	Vector right_s = source.direction().perpendicular(CGAL::Orientation::CLOCKWISE).to_vector();
	Vector dir_s = s - center[0];
	right_s /= LEN(right_s);
	double offset_s = right_s * dir_s;
	
	Vector target = center[n - 1] - center[n - 2];

	Vector right_t = target.direction().perpendicular(CGAL::Orientation::CLOCKWISE).to_vector();
	Vector dir_t = t - center[n - 1];
	right_t /= LEN(right_t);
	double offset_t = right_t * dir_t;

	double offset = 0;
	if (offset_s * offset_t > 0)
		offset = offset_s > 0 ? min(offset_s, offset_t) : max(offset_s, offset_t);

	if (offset != 0)
	{
		for (int i = 0; i < n; i++)
		{
			Vector dir, right;
			if (i == 0)
			{
				dir = dir_s;
				right = right_s;
			}
			else if (i == n - 1)
			{
				dir = dir_t;
				right = right_t;
			}
			else
			{
				Vector next = center[i + 1] - center[i];
				Vector last = center[i] - center[i - 1];
				dir = next / LEN(next) - last / LEN(last);
				right = last.direction().perpendicular(CGAL::Orientation::CLOCKWISE).to_vector();
			}
			dir /= LEN(dir);
			double cos_theta = VEC_COS(dir, right);
			if (cos_theta == 0) printf("RouteEngine::ERROR: center_connect_p2p -\n   cos_theta == 0\n");
			dir *= (offset / cos_theta);
			Transformation translate(CGAL::TRANSLATION, dir);
			res.push_back(center[i].transform(translate));
		}
	}
	else
	{
		res.insert(res.end(), center.begin(), center.end());
	}
	res.push_back(t);
	
	return line_simple(res);
}

Polyline CableRouter::manhattan_smooth_basic(MapInfo* const data, ASPath& path, vector<Segment>& exist_lines, Transformation rotate)
{
	Polyline line = path.path;
	if (line.size() <= 2) return line;

	Polyline res;
	int loop_count = 0;
	int now = 0;
	int next_id = (int)line.size() - 1;
	res.push_back(line[now]);
	while (next_id != now)
	{
		Point u = line[now];
		Point v = line[next_id];
		int passport = path.cross_num[next_id] - path.cross_num[now + 1]
			+ crossRoom(data, u, line[now + 1]);

		auto rotate_inv = rotate.inverse();
		Point rot_u = u.transform(rotate_inv);
		Point rot_v = v.transform(rotate_inv);

		if (APPRO_EQUAL(rot_u.hx(), rot_v.hx()) || APPRO_EQUAL(rot_u.hy(), rot_v.hy()))
		{
			bool cross = crossObstacle(data, u, v);
			int cross_r = crossRoom(data, u, v);
			if (!cross && cross_r > max(passport, 0))
				cross = true;
			//bool colli = tooCloseToSun(data, u, v, obstacle_lines);
			if (!cross)
			{
				passport -= cross_r;
				now = next_id;
				res.push_back(line[now]);
				next_id = (int)line.size() - 1;
			}
			else if (next_id - 1 == now)
			{
				passport -= cross_r;
				now = next_id;
				res.push_back(line[now]);
				next_id = (int)line.size() - 1;
			}
			else
			{
				next_id--;
			}
			continue;
		}

		Point mid1 = Point(rot_v.hx(), rot_u.hy()).transform(rotate);
		Point mid2 = Point(rot_u.hx(), rot_v.hy()).transform(rotate);
		bool cross1 = crossObstacle(data, u, mid1) || crossObstacle(data, mid1, v);
		bool cross2 = crossObstacle(data, u, mid2) || crossObstacle(data, mid2, v);

		int cross_r1 = crossRoom(data, u, mid1);
		int cross_r1_add = crossRoom(data, mid1, v);
		if (!cross1 && cross_r1 + cross_r1_add > max(passport, 0))
			cross1 = true;

		int cross_r2 = crossRoom(data, u, mid2);
		int cross_r2_add = crossRoom(data, mid2, v);
		if (!cross2 && cross_r2 + cross_r2_add > max(passport, 0))
			cross2 = true;

		//bool colli1 = tooCloseToSun(data, u, mid1, obstacle_lines) || tooCloseToSun(data, mid1, v, obstacle_lines);
		//bool colli2 = tooCloseToSun(data, u, mid2, obstacle_lines) || tooCloseToSun(data, mid2, v, obstacle_lines);
		if (!cross2 && !cross1)
		{
			double w1, w2;
			w1 = w2 = DIST_M(u, v);
			//addWeightCenters(data, u, mid1, w1);
			//addWeightCenters(data, mid1, v, w1);
			//addWeightCenters(data, u, mid2, w2);
			//addWeightCenters(data, mid2, v, w2);
			w1 += cross_num(exist_lines, u, mid1);
			w1 += cross_num(exist_lines, mid1, v);
			w2 += cross_num(exist_lines, u, mid2);
			w2 += cross_num(exist_lines, mid2, v);
			Point next;
			if (w1 < w2) {
				next = mid1;
				passport -= cross_r1;
			}
			else {
				next = mid2;
				passport -= cross_r2;
			}
			line.insert(line.begin() + next_id, next);
			now = next_id;
			res.push_back(line[now]);
			next_id = (int)line.size() - 1;
		}
		else if (!cross1)
		{
			passport -= cross_r1;
			line.insert(line.begin() + next_id, mid1);
			now = next_id;
			res.push_back(line[now]);
			next_id = (int)line.size() - 1;
		}
		else if (!cross2)
		{
			passport -= cross_r2;
			line.insert(line.begin() + next_id, mid2);
			now = next_id;
			res.push_back(line[now]);
			next_id = (int)line.size() - 1;
		}
		else if (next_id - 1 == now)
		{
			if (loop_count < 10)
			{
				Point mid = CGAL::midpoint(line[now], line[next_id]);
				line.insert(line.begin() + next_id, mid);
				loop_count++;
			}
			else
			{
				passport -= crossRoom(data, u, v);
				now = next_id;
				res.push_back(line[now]);
				next_id = (int)line.size() - 1;
			}
		}
		else
		{
			next_id--;
		}
	}

	return res;
}

Polyline CableRouter::line_simple(Polyline line)
{
	Polyline res;

	// remove duplicated points
	if (line.size() <= 1) return line;

	Point last = line[0];
	res.push_back(last);
	for (int i = 1; i < line.size(); i++)
	{
		if (!POINT_EQUAL(line[i], last))
		{
			last = line[i];
			res.push_back(last);
		}
	}
	line = res;

	// remove colinear points
	if (line.size() <= 2) return line;

	reset(res);
	res.push_back(line[0]);
	int next = 1;

	Point u = line[0];
	Point v = line[next];

	Vector dir = v - u;
	while (next + 1 != line.size())
	{
		u = v;
		v = line[next + 1];
		Vector dirr = v - u;
		double cos_theta = VEC_COS(dir, dirr);
		if (!APPRO_EQUAL(cos_theta, 1))
		{
			res.push_back(line[next]);
			dir = dirr;
		}
		next++;
	}
	res.push_back(line[next]);

	return res;
}

Polyline CableRouter::manhattan_connect(MapInfo* const data, Point u, Point v, Vector prefer, vector<Segment>& lines, Transformation rotate)
{
	Polyline res;

	auto rotate_inv = rotate.inverse();
	Point rot_u = u.transform(rotate_inv);
	Point rot_v = v.transform(rotate_inv);
	prefer = prefer.transform(rotate_inv);

	double dx = abs(rot_u.hx() - rot_v.hx());
	double dy = abs(rot_u.hy() - rot_v.hy());
	if (APPRO_EQUAL(dx, 0) || APPRO_EQUAL(dy, 0))
	{
		if (!crossObstacle(data, u, v) && 
			!crossRoom(data, u, v) &&
			!cross_lines(lines, u, v, true)) {
			res.push_back(u);
			res.push_back(v);
		}
		else
		{
			ASPath ap = a_star_connect_p2p(data, u, v, lines);
			res = manhattan_smooth_basic(data, ap, lines, rotate);
			res = line_simple(res);
		}
	}
	else
	{
		Point mid1 = Point(rot_v.hx(), rot_u.hy()).transform(rotate);
		Point mid2 = Point(rot_u.hx(), rot_v.hy()).transform(rotate);
		bool cross1 = 
			crossObstacle(data, u, mid1) || crossRoom(data, u, mid1) ||
			crossObstacle(data, mid1, v) || crossRoom(data, mid1, v) ||
			cross_lines(lines, u, mid1, true) || cross_lines(lines, mid1, v, true);
		bool cross2 =
			crossObstacle(data, u, mid2) || crossRoom(data, u, mid2) ||
			crossObstacle(data, mid2, v) || crossRoom(data, mid2, v) ||
			cross_lines(lines, u, mid2, true) || cross_lines(lines, mid2, v, true);
		if (!cross2 && !cross1)
		{
			double w1, w2;
			w1 = w2 = DIST_M(u, v);
			addWeightCenters(data, u, mid1, w1);
			addWeightCenters(data, mid1, v, w1);
			addWeightCenters(data, u, mid2, w2);
			addWeightCenters(data, mid2, v, w2);
			//w1 += cross_num(lines, u, mid1);
			//w1 += cross_num(lines, mid1, v);
			//w2 += cross_num(lines, u, mid2);
			//w2 += cross_num(lines, mid2, v);
			if (w1 > w2)
			{
				res.push_back(u);
				res.push_back(mid2);
				res.push_back(v);
			}
			else if (w1 < w2)
			{
				res.push_back(u);
				res.push_back(mid1);
				res.push_back(v);
			}
			else if (abs(prefer.hx()) > 0.001)
			{
				res.push_back(u);
				res.push_back(mid1);
				res.push_back(v);
			}
			else if (abs(prefer.hy()) > 0.001)
			{
				res.push_back(u);
				res.push_back(mid2);
				res.push_back(v);
			}
			else if (abs(DOUBLE(rot_u.hx() - rot_v.hx())) > abs(DOUBLE(rot_u.hy() - rot_v.hy())))
			{
				res.push_back(u);
				res.push_back(mid1);
				res.push_back(v);
			}
			else
			{
				res.push_back(u);
				res.push_back(mid2);
				res.push_back(v);
			}
		}
		else if (!cross2) {
			res.push_back(u);
			res.push_back(mid2);
			res.push_back(v);
		}
		else if (!cross1) {
			res.push_back(u);
			res.push_back(mid1);
			res.push_back(v);
		}
		else
		{
			ASPath ap = a_star_connect_p2p(data, u, v, lines);
			res = manhattan_smooth_basic(data, ap, lines, rotate);
			res = line_simple(res);
		}
	}

	return res;

}

Polyline CableRouter::obstacle_avoid_connect_p2p(MapInfo* const data, Point p, Point q, vector<Segment>& lines)
{
	vector<Segment> blank;
	ASPath ap = a_star_connect_p2p(data, p, q, blank);
	Polyline line = ap.path;
	line = line_break(line, 1000);
	line = manhattan_smooth_p2p(data, line, lines);
	line = line_simple(line);
	return line;
}

Polyline CableRouter::obstacle_avoid_connect_p2s(MapInfo* const data, Point p, Segment s, vector<Segment>& lines)
{
	vector<Segment> blank;
	Polyline line = a_star_connect_p2s(data, p, s, blank);
	line = line_break(line, 1000);
	line = manhattan_smooth_p2s(data, line, s, lines);
	line = line_simple(line);
	return line;
}

Polyline CableRouter::manhattan_smooth_p2s(MapInfo* const data, Polyline& path, Segment des, vector<Segment>& exist_lines)
{
	Polyline line = path;
	if (line.size() <= 1) return line;

	Polyline res;
	int now = 0;
	res.push_back(line[now]);
	//bool dirX = true;
	//bool dirY = true;
	while (now != (int)line.size() - 1)
	{
		Point u = line[now];
		line[(int)line.size() - 1] = project_point_to_segment(u, des);

		int best = -1;
		double best_value = CR_INF;
		bool best_has_mid = false;
		Point best_mid;

		for (int next = now + 1; next < line.size(); next++)
		{
			int far = ((int)line.size() - next);
			Point v = line[next];
			bool is_end = (next == ((int)line.size() - 1));
			if (EQUAL(u.hx(), v.hx()) || EQUAL(u.hy(), v.hy()))
			{
				//bool cross = crossObstacle(data, u, v);
				bool cross = touchObstacle(data, u, v);
				if (!cross)
				{
					int c = cross_num(exist_lines, u, v);
					double force = tooCloseToSun(data, u, v, exist_lines, is_end);
					double value = (1.0 + c) * (1.0 + force) + 10.0 * far;
					//double value = 10.0 * c + 1.0 * force + 5.0 * far;
					if (value <= best_value)
					{
						best_value = value;
						best = next;
						best_has_mid = false;
					}
				}
				continue;
			}

			Point mid1(v.hx(), u.hy());
			Point mid2(u.hx(), v.hy());
			//bool cross1 = crossObstacle(data, u, mid1) || crossObstacle(data, mid1, v);
			//bool cross2 = crossObstacle(data, u, mid2) || crossObstacle(data, mid2, v);			
			bool cross1 = touchObstacle(data, u, mid1) || touchObstacle(data, mid1, v);
			bool cross2 = touchObstacle(data, u, mid2) || touchObstacle(data, mid2, v);
			//bool best_choose1 = false;
			if (!cross1)
			{
				int c = cross_num(exist_lines, u, mid1) + cross_num(exist_lines, mid1, v);
				double force1 = tooCloseToSun(data, u, mid1, exist_lines) + tooCloseToSun(data, mid1, v, exist_lines, is_end);
				double value = (1.0 + c) * (1.0 + force1) + 10.0 * far;
				//double value = 10.0 * c + 1.0 * force1 + 5.0 * far;
				if (value <= best_value)
				{
					best_value = value;
					best = next;
					best_has_mid = true;
					best_mid = mid1;
					//best_choose1 = true;
				}
			}
			if (!cross2)
			{
				int c = cross_num(exist_lines, u, mid2) + cross_num(exist_lines, mid2, v);
				double force2 = tooCloseToSun(data, u, mid2, exist_lines) + tooCloseToSun(data, mid2, v, exist_lines, is_end);
				double value = (1.0 + c) * (1.0 + force2) + 10.0 * far;
				//double value = 10.0 * c + 1.0 * force2 + 5.0 * far;
				if (value <= best_value)
				{
					best_value = value;
					best = next;
					best_has_mid = true;
					//if (!best_choose1 || dirY)
					//{
					best_mid = mid2;
					//}
				}
			}
		}

		if (best != -1)
		{
			//printf("best value = %lf\n", best_value);
			if (best_has_mid)
			{
				//dirX = u.hy() == best_mid.hy();
				//dirY = u.hx() == best_mid.hx();
				line.insert(line.begin() + best, best_mid);
				now = best;
				res.push_back(line[now]);
			}
			else
			{
				//dirX = u.hy() == line[best].hy();
				//dirY = u.hx() == line[best].hx();
				now = best;
				res.push_back(line[now]);
			}
		}
		else
		{
			//dirX = u.hy() == line[now + 1].hy();
			//dirY = u.hx() == line[now + 1].hx();
			now++;
			res.push_back(line[now]);
		}
	}
	return res;
}

double CableRouter::tooCloseToSun(MapInfo* const data, const Point p, const Point q, vector<Segment>& exist, bool is_end)
{
	vector<Segment> obstacle_lines;

	// area
	auto edges = segment_search(data->area.area_edge_tree, p, q);
	for (auto eit = edges.begin(); eit != edges.end(); eit++)
	{
		obstacle_lines.push_back((*eit)->data->seg);
	}

	// holes
	auto holes = segment_search(data->hole_tree, p, q);
	for (auto oit = holes.begin(); oit != holes.end(); oit++)
	{
		Polygon boundary = (*oit)->data->boundary;
		for (auto eit = boundary.edges_begin(); eit != boundary.edges_end(); eit++)
		{
			obstacle_lines.push_back((*eit));
		}
	}

	double res = 0.0;
	Segment pq(p, q);

	for (int i = 0; i < exist.size(); i++)
	{
		Segment s = exist[i];
		if (DIST(pq, s) > 1000) continue;
		if (is_end && (s.source() == q || s.target() == q)) continue;
		Vector vec_pq(q.hx() - p.hx(), q.hy() - p.hy());
		Vector vec_s(s);
		double pq_s = DOUBLE(vec_pq.hx() * vec_s.hx() + vec_pq.hy() * vec_s.hy());
		double G = abs(pq_s) / (0.01 + LEN(pq));
		double force = 2000.0 * G / (0.00001 + DIST(pq, s) * DIST(pq, s));
		res += force;
	}

	for (int i = 0; i < obstacle_lines.size(); i++)
	{
		Segment s = obstacle_lines[i];
		if (DIST(pq, s) > 1000) continue;
		if (is_end && (s.source() == q || s.target() == q)) continue;
		Vector vec_pq(q.hx() - p.hx(), q.hy() - p.hy());
		Vector vec_s(s);
		double pq_s = DOUBLE(vec_pq.hx() * vec_s.hx() + vec_pq.hy() * vec_s.hy());
		double G = abs(pq_s) / (0.01 + LEN(pq));
		double force = 1500.0 * G / (0.00001 + DIST(pq, s) * DIST(pq, s));
		res += force;
	}
	return res;
}