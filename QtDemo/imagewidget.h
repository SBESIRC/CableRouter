#pragma once

#include <QWidget>
#include <QFileDialog>
#include <map>
#include <qevent.h>
#include <qpainter.h>
#include "GroupEngine.h"
#include "ImmuneSystem.h"
#include "RouteEngine.h"
#include "OptimizeEngine.h"
#include "CableRouteEngine.h"

using namespace CableRouter;
using namespace std;

static int GlobalCount;

class ChildWindow;
QT_BEGIN_NAMESPACE
class QImage;
class QPainter;
QT_END_NAMESPACE

class ImageWidget :
	public QWidget
{
	Q_OBJECT

public:
	ImageWidget();
	~ImageWidget();

protected:
	void paintEvent(QPaintEvent* paintevent);
	void mousePressEvent(QMouseEvent* mouseevent);
	void mouseMoveEvent(QMouseEvent* mouseevent);
	void mouseReleaseEvent(QMouseEvent* mouseevent);

public slots:
	void Open();
	void Clean();
	void ConvexHull();
	void Triangulation();
	void Partition();
	void Connect();

private:
	std::vector<Point> area_;
	std::vector<Point> points_;
	std::vector<Segment> centers_;
	std::vector<std::vector<Point>> holes_;
	std::vector<std::vector<Point>> rooms_;
	std::vector<std::vector<Point>> regions_;
	vector<Point> power_sources_;

	MapInfo data;
	Bbox range;

	CableRouteEngine global_cre;

	vector<ImmuneSystem> engines;

	std::vector<Segment> result_triangulation_;
	std::vector<Polyline> result_tree_;

	std::vector<std::vector<Point>> result_paths_;

	int point_n_;

	void readFiles(const std::string name);

	void clean_lines();

	double XtoCanvas(double x);
	double YtoCanvas(double y);
	double CanvastoX(double x);
	double CanvastoY(double y);
};

