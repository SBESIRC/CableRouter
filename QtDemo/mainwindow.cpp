#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    viewWidget = new ImageWidget();
    Creat_Action();
    Creat_ToolBar();
    Creat_Menu();
    setCentralWidget(viewWidget);
    QPalette pa(this->palette());
    pa.setColor(QPalette::Background, Qt::black);
    pa.setColor(QPalette::ButtonText, Qt::white);
    this->setPalette(pa);
    this->setMinimumSize(1800, 1800);
}

MainWindow::~MainWindow()
{
}

void MainWindow::Creat_Menu()
{
}

void MainWindow::Creat_ToolBar()
{
    toolbar_ = addToolBar(tr("&Main"));

    toolbar_->addAction(action_open_);
    toolbar_->addAction(action_convex_hull_);
    toolbar_->addAction(action_triangulation_);
    toolbar_->addAction(action_partition_);
    toolbar_->addAction(action_connect_);
    toolbar_->addAction(action_clean_);
}

void MainWindow::Creat_Action()
{
    action_open_ = new QAction(tr("&Open"), this);
    connect(action_open_, &QAction::triggered, viewWidget, &ImageWidget::Open);

    action_convex_hull_ = new QAction(tr("&ConvenHull"), this);
    connect(action_convex_hull_, &QAction::triggered, viewWidget, &ImageWidget::ConvexHull);

    action_triangulation_ = new QAction(tr("&Triangulation"), this);
    connect(action_triangulation_, &QAction::triggered, viewWidget, &ImageWidget::Triangulation);

    action_partition_ = new QAction(tr("&Partition"), this);
    connect(action_partition_, &QAction::triggered, viewWidget, &ImageWidget::Partition);

    action_connect_ = new QAction(tr("&Connect"), this);
    connect(action_connect_, &QAction::triggered, viewWidget, &ImageWidget::Connect);

    action_clean_ = new QAction(tr("&Clean"), this);
    connect(action_clean_, &QAction::triggered, viewWidget, &ImageWidget::Clean);
}
