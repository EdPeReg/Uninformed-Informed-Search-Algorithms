#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QGraphicsScene>
#include <QFileDialog>
#include <QImage>
#include <QMessageBox>

#include <deque>
#include <chrono>
#include <set>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "Algorithm.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void action_bfs();
    void action_dfs();
    void action_ids();
    void action_best_fs();
    void action_a_star_search();
    void process_algorithm();
    void on_set_source_clicked();
    void on_set_dest_clicked();

private:
    Ui::MainWindow *ui;
    cv::Mat img, gray_img, bin_img, skel;

    QImage image;
    bool alg_turn[5]; // Set which algorithm is running.
    bool mazes_pick;
    bool figures_pick;

    static void click_event(int event, int x, int y, int, void*);
    void config_image();
    void print_alg_info(size_t level, size_t nodes_expanded, const std::chrono::nanoseconds& diff);
};
#endif // MAINWINDOW_H
