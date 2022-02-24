#include "imagewidget.h"
#include <qpen>
#include <algorithm>
#include <set>
#include <queue>
#include <CR_IO.h>
#include <crtdbg.h>

#define INF 1000000
#define MAX_DEV_IN_GROUP 25

// FOR CONNECT
#define DIJ0 0.5

// FOR PARTITION
#define ALPHA	30		// pos relation
#define BETA	15		// size even
#define OMEGA	50		// inner cohesion
#define DELTA	30		// size big
#define GAMMA	10		// cut line len

#define DIJKSTRA_ONLY 0

int SCALE_X = 900;
int SCALE_Y = 900;

// test/  7 8 9 10
// test2/ 9 10 11 12
// test3/ 0 1 2 3 4 5
const std::string FOLD_NAME = "err_json/";
const std::string FILE_NAME = "商业建筑1#-连线测试_Data1";
//const std::string FOLD_NAME = "geojson/";
//const std::string FILE_NAME = "test2";
ImageWidget::ImageWidget()

{
	//-50717.911789, 14000.912710, -43196.242972, 14000.912710, -43196.242972, 15383.207163
	//-50717.911789, 14000.912710, -50717.911789, 15383.207163, -43196.242972, 15383.207163

	//vector<Point> pts1;
	//pts1.push_back(Point(0, 0));
	//pts1.push_back(Point(1, 0));
	//pts1.push_back(Point(2, 1));
	//pts1.push_back(Point(1, 2));
	//pts1.push_back(Point(0, 2));
	//Polygon po1(pts1.begin(), pts1.end());
	//vector<Point> pts2;
	//pts2.push_back(Point(0, 2));
	//pts2.push_back(Point(1, 2));
	//pts2.push_back(Point(2, 1));
	//pts2.push_back(Point(1, 0));
	//pts2.push_back(Point(0, 0));
	//Polygon po2(pts2.begin(), pts2.end());

	//vector<Polyline> res = getBoundaryOf(po1, po2);
	//for (auto pl : res)
	//{
	//	for (auto p : pl)
	//	{
	//		printf("%lf, %lf\n", p.hx(), p.hy());
	//	}
	//}
	//return;
	//Transformation rotate(CGAL::ROTATION, Direction(1, 1.21321412541), 1, 100);
	////Transformation rotate = Transformation();
	//Transformation rotate_inv = rotate.inverse();
	//Point x(1, 0);
	//x = x.transform(rotate_inv);
	//printf("%.15f, %.15f\n", x.hx(), x.hy());
	//x = x.transform(rotate);
	//printf("%.15f, %.15f\n", x.hx(), x.hy());
	//return;

	range.maxX = range.maxY = 500;
	range.minX = range.minY = 0;
	//CableRouteEngine cre;
	//ifstream f(FOLD_NAME + FILE_NAME + ".geojson");
	//stringstream ss;
	//ss << f.rdbuf();
	//f.close();
	//string datastr = ss.str();
	//string res = cre.routing(datastr);
	//printf("%s", res.c_str());
	//return;
	//readFiles("2");
	data = read_from_geojson_file(FOLD_NAME + FILE_NAME + ".geojson");
	preprocess(data);
	printf("read over\n");

	//EPolygon pgn1 = polygon_to_epolygon(data.regions[0].boundary);
	//EPolygon pgn2 = polygon_to_epolygon(data.regions[1].boundary);
	//if (CGAL::do_intersect(pgn1, pgn2))
	//	printf("TRUE\n");
	//std::list<Polygon_with_holes> pwhs;
	//CGAL::intersection(data.regions[0].boundary, data.area.info.boundary, std::back_inserter(pwhs));
	//printf("pwhs.size() = %d\n", pwhs.size());
	//Polygon_with_holes pwh = pwhs.front();


	range.maxX = range.maxY = -1000000000000000;
	range.minX = range.minY = 1000000000000000;

	for (int i = 0; i < data.devices.size(); ++i)
	{
		points_.push_back(data.devices[i].coord);
	}
	printf("device size: %d\n", points_.size());
	for (int i = 0; i < data.powers.size(); ++i)
	{
		power_sources_.push_back(data.powers[i].points[0]);
	}
	printf("power size: %d\n", power_sources_.size());
	for (int i = 0; i < data.centers.size(); ++i)
	{
		centers_.push_back(data.centers[i]);
		//auto bbox = centers_[i].bbox();
		//range.maxX = std::max(range.maxX, bbox.xmax());
		//range.maxY = std::max(range.maxY, bbox.ymax());
		//range.minX = std::min(range.minX, bbox.xmin());
		//range.minY = std::min(range.minY, bbox.ymin());
	}
	printf("center line size: %d\n", centers_.size());
	//for (auto i = data.area.info.boundary.vertices_begin(); i != data.area.info.boundary.vertices_end(); ++i)
	for (auto i = data.area.info.boundary.vertices_begin(); i != data.area.info.boundary.vertices_end(); ++i)
	{
		area_.push_back(*i);
		range.maxX = std::max(range.maxX, i->hx());
		range.maxY = std::max(range.maxY, i->hy());
		range.minX = std::min(range.minX, i->hx());
		range.minY = std::min(range.minY, i->hy());
	}
	printf("area point size: %d\n", area_.size());
	for (int i = 0; i < data.holes.size(); ++i)
	{
		vector<Point> hole;
		for (auto j = data.holes[i].vertices_begin(); j != data.holes[i].vertices_end(); ++j)
		{
			hole.push_back(*j);
		}
		//holes_.push_back(hole);
	}
	printf("hole size: %d\n", holes_.size());
	for (int i = 0; i < data.rooms.size(); ++i)
	{
		vector<Point> room;
		for (auto j = data.rooms[i].vertices_begin(); j != data.rooms[i].vertices_end(); ++j)
		{
			room.push_back(*j);
		}
		rooms_.push_back(room);
	}
	printf("room size: %d\n", rooms_.size());
	for (int i = 0; i < data.regions.size(); ++i)
	{
		vector<Point> region;
		for (auto j = data.regions[i].boundary.vertices_begin(); j != data.regions[i].boundary.vertices_end(); ++j)
		{
			region.push_back(*j);
		}
		regions_.push_back(region);
		for (auto p : data.regions[i].holes)
		{
			reset(region);
			for (auto j = p.vertices_begin(); j != p.vertices_end(); ++j)
			{
				region.push_back(*j);
			}
			regions_.push_back(region);
		}
	}
	printf("region size: %d\n", regions_.size());

	printf("range.minX = %lf\n", range.minX);
	printf("range.maxX = %lf\n", range.maxX);
	printf("range.maxY = %lf\n", range.maxY);
	printf("range.minY = %lf\n", range.minY);

	auto rangeX = range.maxX - range.minX;
	auto rangeY = range.maxY - range.minY;
	if (rangeX > rangeY)
		SCALE_Y = rangeY / rangeX * SCALE_X;
	else
		SCALE_X = rangeX / rangeY * SCALE_Y;
}

void ImageWidget::readFiles(const std::string name)
{
	//Clean();
	//range.maxX = range.maxY = 0;
	//range.minX = range.minY = 10000000;
	//FILE* infile;
	//double x, y;
	//char c;
	//int leftbra;
	//PElement e;

	//// read dev points
	//infile = fopen((FOLD_NAME + "点位" + name).c_str(), "rt");
	//if (infile)
	//{
	//	while (fscanf(infile, "(%lf,%lf)", &x, &y) != EOF) {
	//		points_.push_back(Point(x, y));
	//		data.devices.push_back(Device(Point(x, y), data.devices.size()));
	//	}
	//	fclose(infile);
	//}
	//cout << "点位数： " << points_.size() << endl;





	//std::vector<rbush::TreeNode<PElement>*> hole_nodes;
	//std::vector<rbush::TreeNode<PElement>*> room_nodes;
	//std::vector<rbush::TreeNode<Segment>*> area_nodes;
	//std::vector<rbush::TreeNode<Segment>*> center_nodes;

	//// read center lines
	//infile = fopen((FOLD_NAME + "中心线" + name).c_str(), "rt");
	//if (infile)
	//{
	//	double a, b;
	//	while (fscanf(infile, "((%lf,%lf),(%lf,%lf))", &x, &y, &a, &b) != EOF) {
	//		centers_.push_back(Segment(Point(x, y), Point(a, b)));
	//		center_nodes.push_back(get_seg_rtree_node(&centers_.back()));
	//	}
	//	fclose(infile);
	//}
	//cout << "中心线数： " << centers_.size() << endl;

	//// read area
	//infile = fopen((FOLD_NAME + "防火分区" + name).c_str(), "rt");
	//if (!infile)
	//	leftbra = 0;
	//else if (fscanf(infile, "(((") != EOF)
	//	leftbra = 3;
	//else
	//	leftbra = 0;
	//while (leftbra) {
	//	switch (leftbra)
	//	{
	//	case 3:
	//		fscanf(infile, "%lf,%lf)", &x, &y);
	//		area_.push_back(Point(x, y));
	//		range.maxX = std::max(range.maxX, x);
	//		range.maxY = std::max(range.maxY, y);
	//		range.minX = std::min(range.minX, x);
	//		range.minY = std::min(range.minY, y);
	//		leftbra--;
	//		break;
	//	case 2:
	//		fscanf(infile, "%c", &c);
	//		if (c == ',')
	//		{
	//			fscanf(infile, "(");
	//			leftbra++;
	//			break;
	//		}
	//		else
	//		{
	//			e.boundary = construct_polygon(&area_);
	//			area_.pop_back();
	//			leftbra--;
	//			break;
	//		}
	//	case 1:
	//		fscanf(infile, ",%lf)", &x);
	//		e.weight = x;
	//		data.area.info = e;
	//		leftbra--;
	//		break;
	//	case 0:
	//	default:
	//		break;
	//	}
	//}
	//for (size_t i = 0; i < area_.size(); i++)
	//{
	//	int next = (i + 1) % area_.size();
	//	area_nodes.push_back(get_seg_rtree_node(&Segment(area_[i], area_[next])));
	//}
	//if (infile) fclose(infile);
	//cout << "防火分区点数： " << area_.size() << endl;

	//// read holes
	//infile = fopen((FOLD_NAME + "洞口" + name).c_str(), "rt");
	//if (!infile)
	//	leftbra = 0;
	//else if (fscanf(infile, "(((") != EOF)
	//	leftbra = 3;
	//else
	//	leftbra = 0;
	//vector<Point> hole;
	//while (leftbra >= 0) {
	//	switch (leftbra)
	//	{
	//	case 3:
	//		fscanf(infile, "%lf,%lf)", &x, &y);
	//		hole.push_back(Point(x, y));
	//		leftbra--;
	//		break;
	//	case 2:
	//		fscanf(infile, "%c", &c);
	//		if (c == ',')
	//		{
	//			fscanf(infile, "(");
	//			leftbra++;
	//			break;
	//		}
	//		else
	//		{
	//			e.boundary = construct_polygon(&hole);
	//			hole.pop_back();
	//			holes_.push_back(hole);
	//			reset(hole);
	//			leftbra--;
	//			break;
	//		}
	//	case 1:
	//		fscanf(infile, ",%lf)", &x);
	//		e.weight = x;
	//		hole_nodes.push_back(get_rtree_node(&e));
	//		leftbra--;
	//		break;
	//	case 0:
	//		if (fscanf(infile, "(((") != EOF)
	//			leftbra = 3;
	//		else
	//			leftbra = -1;
	//	default:
	//		break;
	//	}
	//}
	//if (infile) fclose(infile);
	//cout << "洞口个数： " << holes_.size() << endl;

	//// read rooms
	//infile = fopen((FOLD_NAME + "房间框线" + name).c_str(), "rt");
	//if (!infile)
	//	leftbra = 0;
	//else if (fscanf(infile, "(((") != EOF)
	//	leftbra = 3;
	//else
	//	leftbra = 0;
	//vector<Point> room;
	//while (leftbra >= 0) {
	//	switch (leftbra)
	//	{
	//	case 3:
	//		fscanf(infile, "%lf,%lf)", &x, &y);
	//		room.push_back(Point(x, y));
	//		leftbra--;
	//		break;
	//	case 2:
	//		fscanf(infile, "%c", &c);
	//		if (c == ',')
	//		{
	//			fscanf(infile, "(");
	//			leftbra++;
	//			break;
	//		}
	//		else
	//		{
	//			e.boundary = construct_polygon(&room);
	//			room.pop_back();
	//			rooms_.push_back(room);
	//			reset(room);
	//			leftbra--;
	//			break;
	//		}
	//	case 1:
	//		fscanf(infile, ",%lf)", &x);
	//		e.weight = x;
	//		room_nodes.push_back(get_rtree_node(&e));
	//		leftbra--;
	//		break;
	//	case 0:
	//		if (fscanf(infile, "(((") != EOF)
	//			leftbra = 3;
	//		else
	//			leftbra = -1;
	//	default:
	//		break;
	//	}
	//}
	//if (infile) fclose(infile);
	//cout << "房间个数： " << rooms_.size() << endl;

	//data.cen_line_tree = new SegBush(center_nodes);
	//data.area.area_edge_tree = new SegBush(area_nodes);
	//data.hole_tree = new PEBush(hole_nodes);
	//data.room_tree = new PEBush(room_nodes);

}

ImageWidget::~ImageWidget()
{
}

void ImageWidget::paintEvent(QPaintEvent* paintevent)
{
	QPainter painter(this);

	// draw power
	painter.setPen(QPen(Qt::magenta, 5));
	for (size_t i = 0; i < power_sources_.size(); i++)
	{
		painter.drawPoint(XtoCanvas(DOUBLE(power_sources_[i].hx())), YtoCanvas(DOUBLE(power_sources_[i].hy())));
	}

	// draw points
	painter.setPen(QPen(Qt::green, 4));
	for (size_t i = 0; i < points_.size(); i++)
	{
		painter.drawPoint(XtoCanvas(DOUBLE(points_[i].hx())), YtoCanvas(DOUBLE(points_[i].hy())));
	}

	// draw centers
	//painter.setPen(QPen(Qt::magenta, 1, Qt::DashLine));
	painter.setPen(QPen(Qt::magenta, 1));
	if (GlobalCount % 2 < 0)
	for (size_t i = 0; i < centers_.size(); i++)
	{
		painter.drawLine(
			XtoCanvas(DOUBLE(centers_[i].source().hx())), YtoCanvas(DOUBLE(centers_[i].source().hy())),
			XtoCanvas(DOUBLE(centers_[i].target().hx())), YtoCanvas(DOUBLE(centers_[i].target().hy())));
	}
	//painter.setPen(QPen(Qt::red, 1));
	//painter.drawLine(
	//	XtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].source().hx())), YtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].source().hy())),
	//	XtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].target().hx())), YtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].target().hy())));
	//painter.setPen(QPen(Qt::blue, 2));
	//painter.drawPoint(XtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].target().hx())), YtoCanvas(DOUBLE(centers_[GlobalCount % centers_.size()].target().hy())));


	// draw area
	painter.setPen(QPen(Qt::red, 3));
	//Polygon area_boundary = data.area.info.boundary;
	//for (int i = 0; i < area_boundary.size(); i++)
	//{
	//	painter.drawLine(
	//		XtoCanvas(DOUBLE(area_boundary.vertex(i).hx())), YtoCanvas(DOUBLE(area_boundary.vertex(i).hy())),
	//		XtoCanvas(DOUBLE(area_boundary.vertex((i + 1) % area_boundary.size()).hx())), YtoCanvas(area_boundary.vertex((i + 1) % area_boundary.size()).hy()));
	//}
	if (GlobalCount % 2 < 0)
	for (size_t i = 0; i < area_.size(); i++)
	{
		int next = (i + 1) % area_.size();
		painter.drawLine(
			XtoCanvas(DOUBLE(area_[i].hx())), YtoCanvas(DOUBLE(area_[i].hy())),
			XtoCanvas(DOUBLE(area_[next].hx())), YtoCanvas(DOUBLE(area_[next].hy())));
	}

	// draw holes
	painter.setPen(QPen(Qt::yellow, 1));
	for (size_t i = 0; i < holes_.size(); i++)
	{
		for (size_t j = 0; j < holes_[i].size(); j++) {
			int next = (j + 1) % holes_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(holes_[i][j].hx())), YtoCanvas(DOUBLE(holes_[i][j].hy())),
				XtoCanvas(DOUBLE(holes_[i][next].hx())), YtoCanvas(DOUBLE(holes_[i][next].hy())));
		}
	}
	painter.setPen(QPen(Qt::red, 1.1));
	if (holes_.size() > 0)
	for (size_t i = GlobalCount % holes_.size(); i <= GlobalCount % holes_.size(); i++)
	{
		for (size_t j = 0; j < holes_[i].size(); j++) {
			int next = (j + 1) % holes_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(holes_[i][j].hx())), YtoCanvas(DOUBLE(holes_[i][j].hy())),
				XtoCanvas(DOUBLE(holes_[i][next].hx())), YtoCanvas(DOUBLE(holes_[i][next].hy())));
		}
	}

	// draw rooms
	painter.setPen(QPen(Qt::white, 0.5, Qt::DashLine));
	for (size_t i = 0; i < rooms_.size(); i++)
	{
		for (size_t j = 0; j < rooms_[i].size(); j++) {
			int next = (j + 1) % rooms_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(rooms_[i][j].hx())), YtoCanvas(DOUBLE(rooms_[i][j].hy())),
				XtoCanvas(DOUBLE(rooms_[i][next].hx())), YtoCanvas(DOUBLE(rooms_[i][next].hy())));
		}
	}	
	// draw rooms one by one
	//painter.setPen(QPen(Qt::red, 2));
	//if (rooms_.size() > 0)
	//for (size_t i = GlobalCount % rooms_.size(); i <= GlobalCount % rooms_.size(); i++)
	//{
	//	for (size_t j = 0; j < rooms_[i].size(); j++) {
	//		int next = (j + 1) % rooms_[i].size();
	//		painter.drawLine(
	//			XtoCanvas(DOUBLE(rooms_[i][j].hx())), YtoCanvas(DOUBLE(rooms_[i][j].hy())),
	//			XtoCanvas(DOUBLE(rooms_[i][next].hx())), YtoCanvas(DOUBLE(rooms_[i][next].hy())));
	//	}
	//}
	
	// draw ucs
	painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
	if (GlobalCount % 2 == 0)
	for (size_t i = 0; i < regions_.size(); i++)
	//for (size_t i = GlobalCount % regions_.size(); i <= GlobalCount % regions_.size(); i++)
	{
		for (size_t j = 0; j < regions_[i].size(); j++) {
			int next = (j + 1) % regions_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(regions_[i][j].hx())), YtoCanvas(DOUBLE(regions_[i][j].hy())),
				XtoCanvas(DOUBLE(regions_[i][next].hx())), YtoCanvas(DOUBLE(regions_[i][next].hy())));
		}
	}

	// draw triangulation result
	painter.setPen(QPen(Qt::magenta, 1));
	for (size_t i = 0; i < result_triangulation_.size(); i++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		painter.drawLine(
			XtoCanvas(DOUBLE(result_triangulation_[i].source().hx())), YtoCanvas(DOUBLE(result_triangulation_[i].source().hy())),
			XtoCanvas(DOUBLE(result_triangulation_[i].target().hx())), YtoCanvas(DOUBLE(result_triangulation_[i].target().hy())));
	}

	// draw result
	painter.setPen(QPen(Qt::cyan, 1));
	if (result_tree_.size() > 0)
	//for (size_t p = GlobalCount % result_tree_.size(); p <= GlobalCount % result_tree_.size(); p++)
	for (size_t p = 0; p < result_tree_.size(); p++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		vector<Segment> sss = get_segments_from_polyline(result_tree_[p]);

		for (size_t i = 0; i < sss.size(); i++)
		{
			painter.drawLine(
				XtoCanvas(DOUBLE(sss[i].source().hx())), YtoCanvas(DOUBLE(sss[i].source().hy())),
				XtoCanvas(DOUBLE(sss[i].target().hx())), YtoCanvas(DOUBLE(sss[i].target().hy())));
		}
	}
	// draw result one by one
	painter.setPen(QPen(Qt::red, 2));
	if (result_tree_.size() > 0)
	for (size_t p = GlobalCount % result_tree_.size(); p <= GlobalCount % result_tree_.size(); p++)
	//for (size_t p = 0; p < result_tree_.size(); p++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		vector<Segment> sss = get_segments_from_polyline(result_tree_[p]);

		for (size_t i = 0; i < sss.size(); i++)
		//int i = GlobalCount % sss.size();
		{
			painter.drawLine(
				XtoCanvas(DOUBLE(sss[i].source().hx())), YtoCanvas(DOUBLE(sss[i].source().hy())),
				XtoCanvas(DOUBLE(sss[i].target().hx())), YtoCanvas(DOUBLE(sss[i].target().hy())));
		}
	}

	// draw grouping result
	if (result_paths_.size() < 0)
	{
		for (size_t i = 0; i <= GlobalCount % result_paths_.size(); i++)
		{
			painter.setPen(QPen(QBrush(QColor(128 + (17 + i * 13) % 128, 128 + (67 + i * 91) % 128, 128 + (175 + i * 51) % 128)), 2));
			for (auto p = result_paths_[i].begin(); p != result_paths_[i].end(); p++)
			{
				if ((p + 1) != result_paths_[i].end())
					painter.drawLine(
						XtoCanvas(DOUBLE(p->hx())), YtoCanvas(DOUBLE(p->hy())),
						XtoCanvas(DOUBLE((p + 1)->hx())), YtoCanvas(DOUBLE((p + 1)->hy())));
			}
		}
	}


	update();
}

void ImageWidget::mousePressEvent(QMouseEvent* mouseevent)
{
	if (mouseevent->button() == Qt::LeftButton)
	{
		power_sources_.push_back(Point(CanvastoX(mouseevent->pos().rx()), CanvastoY(mouseevent->pos().ry())));
		data.powers.push_back(Power(power_sources_.back()));
		if (power_sources_.size() >= 2 && power_sources_.size() % 2 == 0)
		{
			Polyline x;
			x.push_back(power_sources_[power_sources_.size() - 1]);
			x.push_back(power_sources_[power_sources_.size() - 2]);
			result_tree_.push_back(x);
		}
		update();
	}
	if (mouseevent->button() == Qt::RightButton)
	{
		Point d(CanvastoX(mouseevent->pos().rx()), CanvastoY(mouseevent->pos().ry()));
		points_.push_back(d);
		data.devices.push_back(Device(d));
		preprocess(data);
		update();
	}
}

void ImageWidget::mouseMoveEvent(QMouseEvent* mouseevent)
{
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* mouseevent)
{
}

void ImageWidget::Clean()
{
	//GlobalCount--;
	//return;


	clean_lines();
	//reset(points_);
	//reset(area_);
	//reset(holes_);
	//reset(rooms_);
}

void ImageWidget::ConvexHull()
{
	//reset(points_);
	//int k = 0;
	//for (int i = 0; i <= GlobalCount % engines[k].data.devices.size(); i++)
	//{
	//	points_.push_back(engines[k].data.devices[i].coord);
	//}
	//printf("dist = %lf\n", engines[k].data.G[engines[k].data.devices.size()][GlobalCount % engines[k].data.devices.size()]);
	GlobalCount++;
	//printf("(%lf, %lf)->", centers_[GlobalCount % centers_.size()].source().hx(), centers_[GlobalCount % centers_.size()].source().hy());
	//printf("(%lf, %lf)\n", centers_[GlobalCount % centers_.size()].target().hx(), centers_[GlobalCount % centers_.size()].target().hy());
	//if (result_tree_.size() > 0) {
	//	Polyline poly = result_tree_[GlobalCount % result_tree_.size()];
	//	printf("line size = %d\n", result_tree_[GlobalCount % result_tree_.size()].size());
	//	for (int i = 0; i < poly.size(); i++)
	//	{
	//		printf("(%.15f, %.15f)\n", poly[i].hx(), poly[i].hy());
	//	}
	//}
}

double ImageWidget::XtoCanvas(double x)
{
	return 1.0 * (x - range.minX) / (range.maxX - range.minX) * SCALE_X + 40;
}

double ImageWidget::YtoCanvas(double y)
{
	return 900 - 1.0 * (y - range.minY) / (range.maxY - range.minY) * SCALE_Y + 40;
}

double ImageWidget::CanvastoX(double x)
{
	return 1.0 * (x - 40) / SCALE_X * (range.maxX - range.minX) + range.minX;
}

double ImageWidget::CanvastoY(double y)
{
	return 1.0 * (900 + 40 - y) / SCALE_Y * (range.maxY - range.minY) + range.minY;
}

void ImageWidget::Partition()
{
	clean_lines();
	point_n_ = points_.size();
	if (point_n_ == 0) return;

	// map points to id

	GroupEngine pe;
	GroupParam paramm;
	paramm.max_dev_size = MAX_DEV_IN_GROUP;
	paramm.min_dev_size = max(1, MAX_DEV_IN_GROUP / 2);
	paramm.weight_pos = ALPHA;
	paramm.weight_even = BETA;
	paramm.weight_cohesion = OMEGA;
	paramm.weight_big = DELTA;
	paramm.weight_cut_len = GAMMA;

	CGAL::Timer timer;
	timer.start();

	printf("begin parse\n");
	//MapInfo map = read_from_geojson_file("test" + FILE_NAME + ".geojson");
	//for (int i = 0; i < power_sources_.size(); i++)
	//{
	//map.powers.push_back(Power(power_sources_));
	//}
	printf("begin group\n");
	vector<vector<int>> group_info = pe.grouping(&data, &paramm);

	double sec = timer.time();
	printf("用时：%lf s\n", sec);

	vector<ISData> groups = parse_groups(&data, group_info);

	reset(engines);
	for (int i = 0; i < groups.size(); i++)
	{
		ImmuneSystem re;
		if (!re.init(&groups[i]))
		{
			continue;
		}
		engines.push_back(re);
	}


	sec = timer.time();
	printf("用时：%lf s\n", sec);
}

bool path_compare_0(std::pair<Point, Point> a, std::pair<Point, Point> b)
{
	//int a_up = (a.second.hy() - a.first.hy()) > 0 ? 1 : -1;
	//int b_up = (b.second.hy() - b.first.hy()) > 0 ? 1 : -1;
	//int a_area = -1 * (a_right ? 1 : -2) * (a_up ? 1 : 2);
	//int b_area = -1 * (b_right ? 1 : -2) * (b_up ? 1 : 2);

	//if (a_area != b_area) return a_area > b_area;

	//if (a_right) return abs(DOUBLE(a.second.hy() - a.first.hy())) > abs(DOUBLE(b.second.hy() - b.first.hy()));

	//return abs(DOUBLE(a.second.hy() - a.first.hy())) < abs(DOUBLE(b.second.hy() - b.first.hy()));
	//return a.second.hx() < b.second.hx();
	int a_right = (a.second.hx() - a.first.hx()) > 0 ? 1 : -1;
	int b_right = (b.second.hx() - b.first.hx()) > 0 ? 1 : -1;
	if (a_right != b_right) return a_right < b_right;
	return a_right * abs(DOUBLE(a.second.hy() - a.first.hy())) >
		b_right * abs(DOUBLE(b.second.hy() - b.first.hy()));
}

void ImageWidget::Connect()
{
	//GlobalCount++; return;
	//reset(points_);
	//clean_lines();
	//for (int i = 0; i < GlobalCount; i++)
	//{
	//	points_.push_back(engines[4].data.devices[i].coord);
	//}
	//printf("dev to pwr = %lf\n", engines[4].data.G[GlobalCount - 1][engines[4].data.devices.size()]);
	//printf("pwr to dev = %lf\n", engines[4].data.G[engines[4].data.devices.size()][GlobalCount - 1]);
	//return;
	printf("Routing begin\n");
	CGAL::Timer timer;
	timer.start();
	double sec;
	reset(result_tree_);
	reset(result_triangulation_);
	reset(result_paths_);

	vector<Polyline> cables;

	//ASPath ap = a_star_connect_p2p(&data, data.devices[0].coord, data.devices[1].coord, get_segments_from_polylines(cables));
	//Polyline path = line_simple(ap.path);
	//result_tree_.push_back(path);
	//return;

	vector<Polyline> power_paths(data.powers.size());
	for (int e = 0; e < engines.size(); e++)
	{
		for (int k = 0; k < 100; k++)
			engines[e].run();

		if (engines[e].globlMem.size() < 1)
		{
			printf("Group %d No result!\n", e);
			continue;
		}

		inner_connect(&data, &engines[e], cables, power_paths);
	}
	result_tree_ = cables;
	printf("Routing end\n");
	sec = timer.time();
	printf("用时：%lf s\n", sec); 
	return;

	vector<Polyline> result_paths;
	vector<Segment> exist_lines = get_segments_from_polylines(cables);
	//for (int i = 0; i < power_paths.size(); i++)
	for (int i = 0; i < power_paths.size(); i++)
	{
		Power pwr = data.powers[i];
		for (int j = 0; j < power_paths[i].size(); j++)
		{
			Point dev = power_paths[i][j];
			printf("Power %d: Looking for path %d\n", i, j);
			Polyline pp;
			if (pwr.is_point())
				pp = obstacle_avoid_connect_p2p(&data, pwr.points[0], dev, exist_lines);
			else if (pwr.is_segment())
				pp = obstacle_avoid_connect_p2s(&data, dev, Segment(pwr.points[0], pwr.points[1]), exist_lines);
			else
				printf("invalid power\n");

			printf("Path size: %d\n", pp.size());
			if (pp.size() > 1) result_paths.push_back(pp);
			for (int k = 0; k < (int) pp.size() - 1; k++)
			{
				result_triangulation_.push_back(Segment(pp[k], pp[k + 1]));
				//exist_lines.push_back(Segment(pp[k], pp[k + 1]));
			}
			//vector<Segment> pp_segs = get_segments_from_polyline(pp);
			//exist_lines.insert(exist_lines.end(), pp_segs.begin(), pp_segs.end());
		}
	}


	printf("Routing end\n");
	sec = timer.time();
	printf("用时：%lf s\n", sec);

	//return;

	printf("Begin merge\n");
	DreamTree dream_tree = merge_to_a_tree(result_paths);
	printf("Begin show tree\n");
	result_triangulation_ = get_dream_tree_lines(dream_tree, true);
	//result_triangulation_ = get_dream_tree_lines(dream_tree);
	//reset(centers_);
	//centers_ = get_dream_tree_lines(dream_tree);



	//queue<DreamNodePtr> q;
	//for (int i = 0; i < dream_tree->children.size(); i++)
	//{
	//	q.push(dream_tree->children[i]);
	//}
	//while (!q.empty())
	//{
	//	DreamNodePtr now = q.front();
	//	q.pop();

	//	printf("line num to parent = %d\n", now->line_num_to_parent);

	//	for (int i = 0; i < now->children.size(); i++)
	//	{
	//		q.push(now->children[i]);
	//	}
	//}

}

//vector<Segment> get_outline(vector<Segment>& const centers)
//{
//	vector<Segment> res;
//
//	ARR arr;
//	vector<ASegment> segments;
//	for (auto c : centers) {
//		segments.push_back(ASegment(APoint(c.source().hx(), c.source().hy()), APoint(c.target().hx(), c.target().hy())));
//	}
//	insert(arr, segments.begin(), segments.end());
//
//	for (auto f = arr.faces_begin(); f != arr.faces_end(); f++)
//	{
//		if (f->is_unbounded())
//		{
//			for (auto ccb = f->inner_ccbs_begin(); ccb != f->inner_ccbs_end(); ccb++)
//			{
//				vector<Point> pts;
//				auto e = *ccb;
//				do
//				{
//					APoint p = e->curve().left();
//					APoint q = e->curve().right();
//					if (e->direction() != CGAL::Arr_halfedge_direction::ARR_LEFT_TO_RIGHT)
//						swap(p, q);
//					pts.push_back(Point(p.hx().exact().to_double(), p.hy().exact().to_double()));
//					e = e->next();
//				} while (e != *ccb);
//				pts = line_simple(pts);
//				for (int i = 0; i < pts.size(); i++)
//				{
//					res.push_back(Segment(pts[i], pts[(i + 1) % pts.size()]));
//				}
//			}
//			break;
//		}
//	}
//	return res;
//}
//
//vector<Segment> expand(vector<Segment>& const centers, double gap)
//{
//	vector<Segment> res;
//
//	ARR arr;
//	vector<ASegment> segments;
//	for (auto c : centers) {
//		segments.push_back(ASegment(APoint(c.source().hx(), c.source().hy()), APoint(c.target().hx(), c.target().hy())));
//	}
//	insert(arr, segments.begin(), segments.end());
//
//	for (auto f = arr.faces_begin(); f != arr.faces_end(); f++)
//	{
//		if (f->is_fictitious())
//			continue;
//
//		if (f->is_unbounded())
//		{
//			for (auto ccb = f->inner_ccbs_begin(); ccb != f->inner_ccbs_end(); ccb++) {
//				vector<Point> pts;
//				auto e = *ccb;
//				do
//				{
//					APoint p = e->curve().left();
//					APoint q = e->curve().right();
//					if (e->direction() != CGAL::Arr_halfedge_direction::ARR_LEFT_TO_RIGHT)
//						swap(p, q);
//					pts.push_back(Point(p.hx().exact().to_double(), p.hy().exact().to_double()));
//					//pts.push_back(Point(q.hx().exact().to_double(), q.hy().exact().to_double()));
//					e = e->next();
//				} while (e != *ccb);
//				pts = line_simple(pts);
//				vector<Point> new_pts;
//				for (int i = 0; i < pts.size(); i++)
//				{
//					Segment s(pts[i], pts[(i + 1) % pts.size()]);
//					Direction x = (s.target() - s.source()).direction();
//					x = x.perpendicular(CGAL::Orientation::COUNTERCLOCKWISE);
//					Vector v = x.to_vector();
//					v /= LEN(v);
//					v *= gap;
//					Transformation translate(CGAL::TRANSLATION, v);
//					new_pts.push_back(pts[i].transform(translate));
//					new_pts.push_back(pts[(i + 1) % pts.size()].transform(translate));
//				}
//				for (int i = 0; i < new_pts.size(); i++)
//				{
//					Segment s(new_pts[i], new_pts[(i + 1) % new_pts.size()]);
//					res.push_back(s);
//				}
//			}
//
//			res = get_outline(res);
//		}
//	}
//	return res;
//}


void ImageWidget::Triangulation()
{
	//reset(result_tree_);
	//auto groups = getFittingLines(&data, 100);
	//for (auto l : groups)
	//{
	//	Polyline line;
	//	for (auto d : l)
	//		line.push_back(d.coord);
	//	result_tree_.push_back(line);
	//}
	//return;
	//reset(centers_);
	//for (int i = 0; i < data.regions.size(); i++)
	//{
	//	for (int j = i + 1; j < data.regions.size(); j++)
	//	{
	//		auto res = getBoundaryOf(data.regions[i], data.regions[j]);
	//		for (auto pl : res)
	//			for (int k = 0; k < pl.size() - 1; k++)
	//				centers_.push_back(Segment(pl[k], pl[k + 1]));
	//	}
	//}
	//return;
	//data.centers = expand(data.centers, 600);
	//centers_.insert(centers_.end(), data.centers.begin(), data.centers.end());
	//centers_ = data.centers;

			//for (auto ccb = f->inner_ccbs_begin(); ccb != f->inner_ccbs_end(); ccb++) {
			//	vector<Point> pts;
			//	auto e = *ccb;
			//	do
			//	{
			//		APoint p = e->curve().left();
			//		APoint q = e->curve().right();
			//		if (e->direction() != CGAL::Arr_halfedge_direction::ARR_LEFT_TO_RIGHT)
			//			swap(p, q);
			//		pts.push_back(Point(p.hx().exact().to_double(), p.hy().exact().to_double()));
			//		//pts.push_back(Point(q.hx().exact().to_double(), q.hy().exact().to_double()));
			//		e = e->next();
			//	} while (e != *ccb);
			//	pts = line_simple(pts);
			//	vector<Point> new_pts;
			//	for (int i = 0; i < pts.size(); i++)
			//	{
			//		Segment s(pts[i], pts[(i + 1) % pts.size()]);
			//		Direction x = (s.target() - s.source()).direction();
			//		x = x.perpendicular(CGAL::Orientation::CLOCKWISE);
			//		Vector v = x.to_vector();
			//		v /= LEN(v);
			//		v *= 300;
			//		Transformation translate(CGAL::TRANSLATION, v);
			//		new_pts.push_back(pts[i].transform(translate));
			//		new_pts.push_back(pts[(i + 1) % pts.size()].transform(translate));
			//	}
			//	for (int i = 0; i < new_pts.size(); i++)
			//	{
			//		Segment s(new_pts[i], new_pts[(i + 1) % new_pts.size()]);
			//		data.centers.push_back(s);
			//	}
			//}
		
		//else if (f->has_outer_ccb())
		//{
			//for (auto ccb = f->outer_ccbs_begin(); ccb != f->outer_ccbs_end(); ccb++) {
			//	vector<Point> pts;
			//	auto e = *ccb;
			//	do
			//	{
			//		APoint p = e->curve().left();
			//		APoint q = e->curve().right();
			//		if (e->direction() != CGAL::Arr_halfedge_direction::ARR_LEFT_TO_RIGHT)
			//			swap(p, q);
			//		pts.push_back(Point(p.hx().exact().to_double(), p.hy().exact().to_double()));
			//		e = e->next();
			//	} while (e != *ccb);
			//	pts = line_simple(pts);
			//	for (int i = 0; i < pts.size(); i++)
			//	{
			//		centers_.push_back(Segment(pts[i], pts[(i + 1) % pts.size()]));
			//		data.centers.push_back(Segment(pts[i], pts[(i + 1) % pts.size()]));
			//	}
			//}
		//}
	//typedef CGAL::Gps_segment_traits_2<Kernel> Traits;
	//typedef Traits::Polygon_2 CPolygon;

	//std::vector<CPolygon> pygs;
	//CPolygon carea(area_.begin(), area_.end());
	//CGAL::approximated_inset_2(carea, 100, 1, std::back_inserter(pygs));

	//return;
	reset(result_triangulation_);
	//CGAL::Delaunay_triangulation_2<Kernel> ddt;
	//for (auto p : power_sources_)
	//{
	//	ddt.insert(p);
	//}
	//for (auto e = ddt.finite_edges_begin(); e != ddt.finite_edges_end(); e++)
	//{
	//	auto vi = e->first->vertex(ddt.cw(e->second));
	//	auto vj = e->first->vertex(ddt.ccw(e->second));
	//	Point p = vi->point();
	//	Point q = vj->point();
	//	result_triangulation_.push_back(Segment(p, q));
	//}
	//return;

	CDT dt;
	set<Constraint> constraints;

	// start and end
	vector<pair<Point, VertexInfo>> vec;
	vec.push_back(make_pair(data.devices[0].coord, VertexInfo(0, true)));
	vec.push_back(make_pair(data.devices[6].coord, VertexInfo(1, true)));

	// area
	Polygon area_boundary = data.area.info.boundary;
	for (int i = 0; i < area_boundary.size(); i++)
	{
		constraints.insert(Constraint(area_boundary.vertex(i), area_boundary.vertex((i + 1) % area_boundary.size())));
	}

	// holes
	for (auto h = data.holes.begin(); h != data.holes.end(); h++)
	{
		for (int i = 0; i < h->size(); i++)
		{
			constraints.insert(Constraint(h->vertex(i), h->vertex((i + 1) % h->size())));
		}
	}

	// rooms
	for (auto r = data.rooms.begin(); r != data.rooms.end(); r++)
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
			//vec.push_back(make_pair(source, VertexInfo(-1, true)));
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

	for (auto feit = dt.finite_edges_begin(); feit != dt.finite_edges_end(); feit++)
	{
		auto vi = feit->first->vertex(dt.cw(feit->second));
		auto vj = feit->first->vertex(dt.ccw(feit->second));
		Point p = vi->point();
		Point q = vj->point();
		if (feit->first->info().not_reach() ||
			feit->first->neighbor(feit->second)->info().not_reach())
		{
			//result_tree_.push_back(Polyline{ p, q });
			continue;
		}
		result_triangulation_.push_back(Segment(p, q));
	}
}

void ImageWidget::Open()
{
	// 房间缝隙填充demo
	for (int i = 0; i < data.rooms.size(); i++)
	{
		Polygon room1 = data.rooms[i];
		vector<Segment> segs;
		for (int j = i + 1; j < data.rooms.size(); j++)
		{
			Polygon room2 = data.rooms[j];
			for (int k = 0; k < room2.size(); k++)
				segs.push_back(room2.edge(k));
		}
		for (int j = 0; j < data.area.info.boundary.size(); j++)
		{
			segs.push_back(data.area.info.boundary.edge(j));
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
					printf("cos_theta = %.lf, dist = %.lf\n", VEC_COS(v1, v2), DIST(s1, s2));
					printf("#FFFFFFFFFFFFFF\n");
					vector<Point> pts, res;
					pts.push_back(project_point_to_segment(s1.source(), s2));
					pts.push_back(project_point_to_segment(s1.target(), s2));
					if (POINT_EQUAL(pts[0], pts[1])) continue;
					pts.push_back(project_point_to_segment(s2.source(), s1));
					pts.push_back(project_point_to_segment(s2.target(), s1));
					if (POINT_EQUAL(pts[2], pts[3])) continue;
					CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(res));
					holes_.push_back(res);
				}
			}
		}
	}
	return;
	vector<Segment> lines;
	ASPath ap = a_star_connect_p2p(&data, points_[0], points_[1], lines);
	Polyline path = line_simple(ap.path);
	result_tree_.push_back(path);
	lines = data.borders;
	vector<Point> intersection = polyline_intersect(path, lines);
	printf("######## intersection size        = %d\n", intersection.size());
	intersection = points_simple(intersection);
	printf("######## intersection size simple = %d\n", intersection.size());
	return;
	//CableRouteEngine cre;
	//ifstream f(FOLD_NAME + FILE_NAME + ".geojson");
	//stringstream ss;
	//ss << f.rdbuf();
	//f.close();
	//string datastr = ss.str();
	//for (int i = 0; i < 100; i++)
	//	string res = cre.routing(datastr);
	//printf("%s", res.c_str());
	//return;
	int rid = -1;
	for (int i = 0; i < data.regions.size(); i++)
	{
		if (data.regions[i].align_center)
		{
			rid = i;
			break;
		}
	}
	if (rid == -1)
	{
		printf("No center region\n");
		return;
	}
	vector<Segment> centers = data.regions[rid].centers;

	vector<Point> all = points_;
	vector<Point> pts;

	for (int i = 0; i < all.size(); i++)
	{
		// find nearest point on center-lines
		int nearest_cen;
		Point nearest_pt;
		double MIN = CR_INF;
		for (int cid = 0; cid < centers.size(); cid++)
		{
			Point proj = project_point_to_segment(all[i], centers[cid]);
			double dis = DIST(proj, all[i]);
			if (dis < MIN)
			{
				MIN = dis;
				nearest_cen = cid;
				nearest_pt = proj;
			}
		}
		// break the center-line in centers
		Segment cen = centers[nearest_cen];
		if (POINT_CLOSE(nearest_pt, cen.source()))
			nearest_pt = cen.source();
		else if (POINT_CLOSE(nearest_pt, cen.target()))
			nearest_pt = cen.target();
		else
		{
			centers[nearest_cen] = Segment(cen.source(), nearest_pt);
			centers.push_back(Segment(nearest_pt, cen.target()));
		}
		// get the point id in pts
		// if new point, add it to pts
		bool p_exist = false;
		for (int id = 0; id < pts.size(); id++)
		{
			if (!p_exist && POINT_CLOSE(pts[id], nearest_pt))
			{
				p_exist = true;
				result_triangulation_.push_back(Segment(all[i], nearest_pt));
			}
			if (p_exist) break;
		}
		if (!p_exist)
		{
			result_triangulation_.push_back(Segment(all[i], nearest_pt));
			pts.push_back(nearest_pt);
		}
	}

	reset(centers_);
	vector<int> idx(centers.size() * 2, -1);
	vector<set<int>> adj(pts.size());

	for (int i = 0; i < centers.size(); i++)
	{
		Point p = centers[i].source();
		Point q = centers[i].target();
		bool p_exist = false, q_exist = false;
		for (int id = 0; id < pts.size(); id++)
		{
			if (!p_exist && POINT_CLOSE(pts[id], p))
			{
				p_exist = true;
				idx[i * 2] = id;
			}
			if (!q_exist && POINT_CLOSE(pts[id], q))
			{
				q_exist = true;
				idx[i * 2 + 1] = id;
			}
			if (p_exist && q_exist) break;
		}
		if (!p_exist)
		{
			idx[i * 2] = pts.size();
			pts.push_back(p);
			adj.push_back(set<int>());
		}
		if (!q_exist)
		{
			idx[i * 2 + 1] = pts.size();
			pts.push_back(q);
			adj.push_back(set<int>());
		}
		int pid = idx[i * 2];
		int qid = idx[i * 2 + 1];
		printf("pid = %d, qid = %d\n", pid, qid);
		adj[pid].insert(qid);
		adj[qid].insert(pid);
		centers_.push_back(Segment(pts[pid], pts[qid]));
	}
}

void ImageWidget::clean_lines()
{
	reset(result_triangulation_);
	reset(result_tree_);
	reset(result_paths_);
}
