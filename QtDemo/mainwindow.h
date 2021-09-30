#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"
#include "imagewidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = Q_NULLPTR);
    ~MainWindow();

	QMenu* menu_;
	QToolBar* toolbar_;
	QAction* action_open_;
	QAction* action_convex_hull_;
	QAction* action_triangulation_;
	QAction* action_partition_;
	QAction* action_connect_;
	QAction* action_clean_;

	void Creat_Menu();
	void Creat_ToolBar();
	void Creat_Action();

private:
    Ui::MainWindowClass ui;
    ImageWidget* viewWidget;
};
