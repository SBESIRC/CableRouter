#include "CR_CGALUtils.h"

using namespace CableRouter;

rbush::TreeNode<Segment>* CableRouter::get_seg_rtree_node(const Segment* seg)
{
	rbush::TreeNode<Segment>* node = new rbush::TreeNode<Segment>();
	node->data = new Segment(*seg);
	node->bbox.minX = min(DOUBLE(seg->source().hx()), DOUBLE(seg->target().hx()));
	node->bbox.minY = min(DOUBLE(seg->source().hy()), DOUBLE(seg->target().hy()));
	node->bbox.maxX = max(DOUBLE(seg->source().hx()), DOUBLE(seg->target().hx()));
	node->bbox.maxY = max(DOUBLE(seg->source().hy()), DOUBLE(seg->target().hy()));
	return node;
}

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

Polygon CableRouter::construct_polygon(const vector<Point>* coords) {
	CGAL_assertion(coords->front() == coords->back());
	Polygon pgn(coords->begin(), coords->end() - 1);
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
	double apab =
		DOUBLE((p.hx() - s.source().hx()) * (s.target().hx() - s.source().hx())) +
		DOUBLE((p.hy() - s.source().hy()) * (s.target().hy() - s.source().hy()));
	double abab =
		DOUBLE((s.target().hx() - s.source().hx()) * (s.target().hx() - s.source().hx())) +
		DOUBLE((s.target().hy() - s.source().hy()) * (s.target().hy() - s.source().hy()));
	double r = apab / abab;
	if (r <= 0)
		return s.source();
	if (r >= 1)
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