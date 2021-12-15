#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Timer.h>
#include <CGAL/Random.h>

#include "RBush.hpp"

#ifndef _CABLEROUTER_CGALUTILS_H_
#define _CABLEROUTER_CGALUTILS_H_

#define CR_INF  1e10
#define CR_EPS  1e-14

namespace CableRouter
{
    using namespace std;

    const double PI = acos(-1);

    struct VertexInfo
    {
        VertexInfo()
            :id(-1), is_device(false) {}
        VertexInfo(int idx)
            :id(idx), is_device(false) {}
        VertexInfo(int idx, bool isd)
            :id(idx), is_device(isd) {}
        int id;
        bool is_device;
    };

    struct FaceInfo
    {
        FaceInfo() {}
        int nesting_level = -1;
        bool not_reach() {
            return nesting_level == -1;
        }
    };

    typedef CGAL::Exact_predicates_inexact_constructions_kernel             Kernel;
    typedef CGAL::Delaunay_triangulation_2<Kernel>				            Delaunay;
    typedef Kernel::Point_2                                                 Point;
    typedef Kernel::Segment_2                                               Segment;
    typedef Kernel::Vector_2                                                Vector;
    typedef Kernel::Line_2                                                  Line;
    typedef Kernel::Iso_rectangle_2                                         Rectangle;
    typedef Kernel::Direction_2                                             Direction;
    typedef vector<Point>                                                   Polyline;
    typedef CGAL::Polygon_2<Kernel>                                         Polygon;
    typedef CGAL::Polygon_with_holes_2<Kernel>                              Polygon_with_holes;
    typedef CGAL::Aff_transformation_2<Kernel>                              Transformation;

    typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, Kernel> Vb;
    typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, Kernel>     Fbb;
    typedef CGAL::Constrained_triangulation_face_base_2<Kernel, Fbb>        Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                    Tds;
    typedef CGAL::Exact_predicates_tag                                      Itag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, Itag>   CDT;
    typedef CDT::Face_handle                                                Face_handle;
    typedef CDT::Vertex_handle                                              Vertex_handle;

    struct Constraint
    {
        Point source;
        Point target;
        Constraint(Point p, Point q)
            :source(p), target(q) {}
        bool operator < (const Constraint& c) const
        {
            if (source.hx() != c.source.hx()) return source.hx() < c.source.hx();
            if (source.hy() != c.source.hy()) return source.hy() < c.source.hy();
            if (target.hx() != c.target.hx()) return target.hx() < c.target.hx();
            return target.hy() < c.target.hy();
        }
    };

    //#define DOUBLE(x)			                                CGAL::to_double(x.exact())
    #define DOUBLE(x)			                                (x)
    #define DIST_2(x, y)		                                DOUBLE(CGAL::squared_distance((x), (y)))
    #define DIST(x, y)			                                sqrt(DIST_2((x), (y)))
    #define DIST_M(x, y)			                            (abs(DOUBLE((x).hx() - (y).hx())) + abs(DOUBLE((x).hy() - (y).hy())))
    #define LEN_2(x)                                            DOUBLE((x).squared_length())
    #define LEN(x)                                              sqrt(LEN_2((x)))
    #define EQUAL(x, y)                                         (abs(DOUBLE((x) - (y))) < CR_EPS)
    #define APPRO_EQUAL(x, y)                                   (abs(DOUBLE((x) - (y))) < 1e-4)
    #define POINT_EQUAL(x, y)                                   (APPRO_EQUAL((x).hx(), (y).hx()) && APPRO_EQUAL((x).hy(), (y).hy()))
    #define VEC_COS(x, y)                                       ((x) * (y) / LEN((x)) / LEN((y)))

    Polygon construct_polygon(const vector<Point>* coords);
    bool polygons_intersect(const Polygon& p, const Polygon& q);

    bool is_tiny_face_between_obstacles(CDT& cdt, CDT::Face_handle face);
    void mark_domains(CDT& cdt, CDT::Face_handle start, int index);
    void mark_domains(CDT& cdt);

    double** newDoubleGraph(int n, double value);
    void deleteGraph(double** G, int n);

    Point project_point_to_segment(Point p, Segment s);
    Segment shrink_segment(Segment seg, double rate);
    Segment shrink_segment(Segment seg);

    Point focus_point(Face_handle f);
    vector<Point> point_box(Point p, const double gap);

    int self_cross_num(vector<Segment>& segs);
    int cross_num(vector<Segment>& segs, const Point p, const Point q);
    bool cross_lines(vector<Segment>& segs, const Point p, const Point q);

    vector<Segment> get_segments_from_polyline(Polyline& polyline);
    vector<Segment> get_segments_from_polylines(vector<Polyline>& polylines);

    Transformation get_tf_from_dir(Direction dir);
    Direction to_first_quadrant(Direction dir);
    Vector to_first_quadrant(Vector dir);

    rbush::TreeNode<Point>* get_point_rtree_node(const Point* pt);

    // T: has function swap()
    template <class T>
    void reset(T& ele)
    {
        ele.swap(T());
    }

    // T: has function size()
    template <class T>
    struct vector_less
    {
        bool operator() (T a, T b)
        {
            return a.size() < b.size();
        }
    };

    // T: has function size()
    template <class T>
    struct vector_greater
    {
        bool operator() (T a, T b)
        {
            return a.size() > b.size();
        }
    };

    // T: PElement
    template <class T>
    rbush::TreeNode<T>* get_rtree_node(const T* ele)
    {
        Polygon pgn = ele->boundary;
        rbush::TreeNode<T>* node = new rbush::TreeNode<T>();
        node->data = new T(*ele);
        node->bbox.minX = pgn.bbox().xmin();
        node->bbox.minY = pgn.bbox().ymin();
        node->bbox.maxX = pgn.bbox().xmax();
        node->bbox.maxY = pgn.bbox().ymax();
        return node;
    }

    // T: SElement
    template <class T>
    rbush::TreeNode<T>* get_seg_rtree_node(const T* ele)
    {
        Segment seg = ele->seg;
        rbush::TreeNode<T>* node = new rbush::TreeNode<T>();
        node->data = new T(*ele);
        node->bbox.minX = min(DOUBLE(seg.source().hx()), DOUBLE(seg.target().hx()));
        node->bbox.minY = min(DOUBLE(seg.source().hy()), DOUBLE(seg.target().hy()));
        node->bbox.maxX = max(DOUBLE(seg.source().hx()), DOUBLE(seg.target().hx()));
        node->bbox.maxY = max(DOUBLE(seg.source().hy()), DOUBLE(seg.target().hy()));
        return node;
    }

    // T: PElement
    template <class T>
    vector<rbush::TreeNode<T>* > point_search(rbush::RBush<T>* const tree, const Point& p)
    {
        vector<rbush::TreeNode<T>* > ret;
        if (tree) {
            rbush::Bbox bbox;
            bbox.minX = DOUBLE(p.x());
            bbox.minY = DOUBLE(p.y());
            bbox.maxX = DOUBLE(p.x());
            bbox.maxY = DOUBLE(p.y());
            auto feedback = tree->search(bbox);
            if (feedback) {
                for (auto fit = feedback->begin(); fit != feedback->end(); ++fit) {
                    Polygon q = (*fit)->data->boundary;
                    if (q.has_on_bounded_side(p)) {
                        ret.push_back(*fit);
                    }
                }
            }
            delete feedback;
        }
        return ret;
    }

    // T: PElement SElement
    template <class T>
    vector<rbush::TreeNode<T>* > segment_search(rbush::RBush<T>* const tree, const Point& p, const Point& q)
    {
        vector<rbush::TreeNode<T>* > ret;
        if (tree) {
            rbush::Bbox bbox;
            bbox.minX = DOUBLE(min(p.x(), q.x()));
            bbox.minY = DOUBLE(min(p.y(), q.y()));
            bbox.maxX = DOUBLE(max(p.x(), q.x()));
            bbox.maxY = DOUBLE(max(p.y(), q.y()));
            auto feedback = tree->search(bbox);
            if (feedback) {
                for (auto fit = feedback->begin(); fit != feedback->end(); ++fit) {
                    ret.push_back(*fit);
                }
            }
            delete feedback;
        }
        return ret;
    }
}

#endif