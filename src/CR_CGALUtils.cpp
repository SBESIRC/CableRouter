#include "CR_CGALUtils.h"

using namespace CableRouter;

rbush::TreeNode<Point>* CableRouter::get_point_rtree_node(const Point* p)
{
	rbush::TreeNode<Point>* node = new rbush::TreeNode<Point>();
	node->data = new Point(*p);
	node->bbox.minX = DOUBLE(p->hx());
	node->bbox.minY = DOUBLE(p->hy());
	node->bbox.maxX = DOUBLE(p->hx());
	node->bbox.maxY = DOUBLE(p->hy());
	return node;
}

Polygon CableRouter::epolygon_to_polygon(const EPolygon& ep)
{
	vector<Point> pts;
	for (auto pit = ep.vertices_begin(); pit != ep.vertices_end(); pit++)
		pts.push_back(Point(CGAL::to_double(pit->hx().exact()), CGAL::to_double(pit->hy().exact())));
	return Polygon(pts.begin(), pts.end());
}

EPolygon CableRouter::polygon_to_epolygon(const Polygon& p)
{
	vector<EPoint> pts;
	for (auto pit = p.vertices_begin(); pit != p.vertices_end(); pit++)
		pts.push_back(EPoint(pit->hx(), pit->hy()));
	return EPolygon(pts.begin(), pts.end());
}

bool CableRouter::compare_point_by_x_y(Point a, Point b)
{
	if (a.hx() != b.hx()) return a.hx() < b.hx();
	return a.hy() < b.hy();
}

Polygon CableRouter::construct_polygon(const vector<Point>* coords) {
	Polygon pgn;
	if (coords->front() == coords->back())
		pgn = Polygon(coords->begin(), coords->end() - 1);
	else
		pgn = Polygon(coords->begin(), coords->end());
	if (pgn.is_clockwise_oriented())
		pgn.reverse_orientation();
	return pgn;
}

bool CableRouter::polygons_intersect(const Polygon& p, const Polygon& q)
{
	for (auto peit = p.edges_begin(); peit != p.edges_end(); peit++)
	{
		for (auto qeit = q.edges_begin(); qeit != q.edges_end(); qeit++)
		{
			if (CGAL::do_intersect(*peit, *qeit)) return true;
		}
	}
	return false;
}

bool CableRouter::is_tiny_face_between_obstacles(CDT& ct, CDT::Face_handle fh)
{
	bool is_constrained = false;
	bool is_device = false;
	for (int i = 0; i < 3; i++)
	{
		if (fh->is_constrained(i))
			is_constrained = true;
		if (fh->vertex(i)->info().is_device)
			is_device = true;
	}
	bool tiny = false;
	for (int i = 0; i < 3; i++)
	{
		Point p = fh->vertex(i)->point();
		Point q = fh->vertex(ct.ccw(i))->point();
		if (fh->is_constrained(ct.cw(i)))
		{
			Point op = fh->vertex(ct.cw(i))->point();
			if (is_constrained && DIST(op, Line(p, q)) < 10)
			{
				tiny = true;
				break;
			}
		}
		else if (DIST(p, q) < 0.1)
		{
			auto nei = fh->neighbor(ct.cw(i));
			bool nei_is_con = false;
			for (int i = 0; i < 3; i++)
				if (fh->is_constrained(i))
					nei_is_con = true;
			if (is_constrained && nei_is_con)
			{
				tiny = true;
				break;
			}
		}
	}
	return tiny && !is_device;
}

void CableRouter::mark_domains(CDT& ct, CDT::Face_handle start, int index)
{
	if (start->info().nesting_level != -1) {
		return;
	}
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);
	while (!queue.empty()) {
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if (fh->info().nesting_level == -1) {

			if (is_tiny_face_between_obstacles(ct, fh)) continue;
			fh->info().nesting_level = index;
			for (int i = 0; i < 3; i++) {
				CDT::Edge e(fh, i);
				CDT::Face_handle n = fh->neighbor(i);
				if (n->info().nesting_level == -1) {
					if (!ct.is_constrained(make_pair(fh, i))) queue.push_back(n);
				}
			}
		}
	}
}
void CableRouter::mark_domains(CDT& cdt)
{
	for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
		it->info().nesting_level = -1;
	}

	int domain_id = 0;

	for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
		if (it->info().nesting_level != -1) {
			continue;
		}
		bool flag = false;
		for (int i = 0; i < 3; i++)
		{
			if (it->vertex(i)->info().is_device)
			{
				flag = true;
				break;
			}
		}
		if (flag) mark_domains(cdt, it, domain_id++);
	}
}

double** CableRouter::newDoubleGraph(int n, double value)
{
	double** G = new double* [n];
	for (int i = 0; i < n; i++)
	{
		G[i] = new double[n];
		fill(G[i], G[i] + n, value);
	}
	return G;
}

void CableRouter::deleteGraph(double** G, int n)
{
	for (int i = 0; i < n; i++)
	{
		delete[] G[i];
	}
	delete[] G;
}

Point CableRouter::project_point_to_segment(Point p, Segment s)
{
	if (s.source() == s.target())
		return s.source();

	double apab =
		DOUBLE((p.hx() - s.source().hx()) * (s.target().hx() - s.source().hx())) +
		DOUBLE((p.hy() - s.source().hy()) * (s.target().hy() - s.source().hy()));
	double abab =
		DOUBLE((s.target().hx() - s.source().hx()) * (s.target().hx() - s.source().hx())) +
		DOUBLE((s.target().hy() - s.source().hy()) * (s.target().hy() - s.source().hy()));
	double r = apab / abab;
	if (EQUAL(r, 0) || r < 0)
		return s.source();
	if (EQUAL(r, 1) || r > 1)
		return s.target();
	else
		return Point(
			r * DOUBLE(s.target().hx() - s.source().hx()) + DOUBLE(s.source().hx()),
			r * DOUBLE(s.target().hy() - s.source().hy()) + DOUBLE(s.source().hy()));
}

Segment CableRouter::shrink_segment(Segment seg, double rate)
{
	Point p(
		rate * DOUBLE(seg.source().hx() - seg.target().hx()) + DOUBLE(seg.target().hx()),
		rate * DOUBLE(seg.source().hy() - seg.target().hy()) + DOUBLE(seg.target().hy()));
	Point q(
		rate * DOUBLE(seg.target().hx() - seg.source().hx()) + DOUBLE(seg.source().hx()),
		rate * DOUBLE(seg.target().hy() - seg.source().hy()) + DOUBLE(seg.source().hy()));
	return Segment(p, q);
}

Segment CableRouter::shrink_segment(Segment seg)
{
	double r = 1.0 - 1.0 / LEN(seg);
	return shrink_segment(seg, r);
}

Segment CableRouter::expand_segment(Segment seg)
{
	double r = 1.0 + 1.0 / LEN(seg);
	return shrink_segment(seg, r);
}

Point CableRouter::focus_point(Face_handle f)
{
	double px = 0, py = 0;
	for (int i = 0; i < 3; i++)
	{
		px += DOUBLE(f->vertex(i)->point().hx());
		py += DOUBLE(f->vertex(i)->point().hy());
	}
	return Point(px / 3.0, py / 3.0);
}

vector<Point> CableRouter::point_box(Point p, const double gap)
{
	vector<Point> res;
	double minX = DOUBLE(p.hx() - gap);
	double minY = DOUBLE(p.hy() - gap);
	double maxX = DOUBLE(p.hx() + gap);
	double maxY = DOUBLE(p.hy() + gap);
	res.push_back(Point(minX, maxY));
	res.push_back(Point(maxX, maxY));
	res.push_back(Point(maxX, minY));
	res.push_back(Point(minX, minY));
	return res;
}

int CableRouter::self_cross_num(vector<Segment>& segs)
{
	int res = 0;
	for (int i = 0; i < segs.size(); i++)
	{
		for (int j = i + 1; j < segs.size(); j++)
		{
			Segment si = shrink_segment(segs[i]);
			Segment sj = shrink_segment(segs[j]);
			if (!CGAL::do_intersect(si, sj)) continue;
			CGAL::Object result = CGAL::intersection(si, sj);
			Point pt;
			if (CGAL::assign(pt, result))
			{
				res++;
			}
		}
	}
	return res;
}

int CableRouter::cross_num(vector<Segment>& segs, const Point p, const Point q)
{
	int res = 0;
	for (int i = 0; i < segs.size(); i++)
	{
		Segment si = segs[i];
		Segment sj = Segment(p, q);
		if (!CGAL::do_intersect(si, sj)) continue;
		CGAL::Object result = CGAL::intersection(si, sj);
		Point pt;
		if (CGAL::assign(pt, result))
		{
			res++;
		}
	}
	return res;
}

bool CableRouter::cross_lines(vector<Segment>& segs, const Point p, const Point q)
{
	for (int i = 0; i < segs.size(); i++)
	{
		Segment si = segs[i];
		Segment sj = Segment(p, q);
		if (!CGAL::do_intersect(si, sj)) continue;
		CGAL::Object result = CGAL::intersection(si, sj);
		Point pt;
		if (CGAL::assign(pt, result))
		{
			return true;
		}
	}
	return false;
}

bool CableRouter::cross_lines_shrink(vector<Segment>& segs, const Point p, const Point q)
{
	Segment sj = shrink_segment(Segment(p, q));
	for (int i = 0; i < segs.size(); i++)
	{
		Segment si = segs[i];
		if (!CGAL::do_intersect(si, sj)) continue;
		CGAL::Object result = CGAL::intersection(si, sj);
		Point pt;
		if (CGAL::assign(pt, result))
		{
			return true;
		}
	}
	return false;
}

vector<Segment> CableRouter::get_segments_from_polyline(Polyline& polyline)
{
	vector<Segment> res;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		res.push_back(Segment(polyline[i], polyline[i + 1]));
	}
	return res;
}

vector<Segment> CableRouter::get_segments_from_polylines(vector<Polyline>& polylines)
{
	vector<Segment> res;
	for (int i = 0; i < polylines.size(); i++)
	{
		for (int j = 0; j < polylines[i].size() - 1; j++)
		{
			res.push_back(Segment(polylines[i][j], polylines[i][j + 1]));
		}
	}
	return res;
}

Transformation CableRouter::get_tf_from_dir(Direction dir)
{
	Transformation rotate(CGAL::ROTATION, dir, 1, 100);
	return rotate;
}

Direction CableRouter::to_first_quadrant(Direction dir)
{
	if (EQUAL(dir.dx(), 0) || EQUAL(dir.dy(), 0))
		return Direction(1, 0);

	if (dir.dx() > 0 && dir.dy() > 0)
		return dir;

	if (dir.dx() > 0 && dir.dy() < 0)
		return dir.perpendicular(CGAL::Orientation::COUNTERCLOCKWISE);

	if (dir.dx() < 0 && dir.dy() > 0)
		return dir.perpendicular(CGAL::Orientation::CLOCKWISE);

	if (dir.dx() < 0 && dir.dy() < 0)
		return -dir;

	return dir;
}

Vector CableRouter::to_first_quadrant(Vector dir)
{
	return to_first_quadrant(dir.direction()).to_vector();
}

vector<Point> CableRouter::polyline_intersect(Polyline polyline, const vector<Segment> segs)
{
	vector<Point> res;

	auto lines = get_segments_from_polyline(polyline);
	for (auto si : lines)
	{
		vector<Point> pts;
		for (auto sj : segs)
		{
			if (!CGAL::do_intersect(si, sj)) continue;
			CGAL::Object result = CGAL::intersection(si, sj);
			Point pt;
			if (CGAL::assign(pt, result))
			{
				pts.push_back(pt);
			}
		}
		sort(pts.begin(), pts.end(), compare_point_by_x_y);
		if (si.source().hx() > si.target().hx())
			reverse(pts.begin(), pts.end());
		else if (si.source().hx() == si.target().hx() && si.source().hy() > si.target().hy())
			reverse(pts.begin(), pts.end());
		res.insert(res.end(), pts.begin(), pts.end());
	}
	return res;
}

vector<Point> CableRouter::points_simple(const vector<Point> pts)
{
	vector<Point> res;

	if (pts.size() < 2) return pts;

	Point last = pts[0];
	res.push_back(last);

	for (int i = 1; i < pts.size(); i++)
	{
		Point now = pts[i];
		if (!POINT_EQUAL(last, now))
		{
			last = now;
			res.push_back(last);
		}
	}

	return res;
}
