#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <opencv2/ximgproc.hpp>

/* @brief: Aux structure to get (x,y) from opencv image window. */
struct AuxPoint
{
    cv::Mat img;
    int x = 0;
    int y = 0;
} auxPoint;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , alg_turn {false}
{
    ui->setupUi(this);

    ui->source_x->setReadOnly(true);
    ui->source_y->setReadOnly(true);
    ui->dest_x->setReadOnly(true);
    ui->dest_y->setReadOnly(true);
    ui->algorithm_output->setReadOnly(true);
    ui->algorithm_output->setPlaceholderText(tr("Algorithm output"));

    connect(ui->actionBreadth_first_search, &QAction::triggered, this, &MainWindow::action_bfs);
    connect(ui->actionDepth_first_searc, &QAction::triggered, this, &MainWindow::action_dfs);
    connect(ui->actionIterative_deepening_search, &QAction::triggered, this, &MainWindow::action_ids);
    connect(ui->btn_process, &QPushButton::clicked, this, &MainWindow::process_algorithm);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::action_bfs()
{
    config_image();
    alg_turn[0] = true;
}

void MainWindow::action_dfs()
{
    config_image();
    alg_turn[1] = true;
}

void MainWindow::action_ids()
{
    config_image();
    alg_turn[2] = true;
}

void MainWindow::process_algorithm()
{
    QString aux_x = ui->source_x->text();
    QString aux_y = ui->source_y->text();

    // Get source and destination.
    cv::Point initial_state(aux_x.toInt(), aux_y.toInt());
    aux_x = ui->dest_x->text();
    aux_y = ui->dest_y->text();
    cv::Point final_state(aux_x.toInt(), aux_y.toInt());

    Algorithm algorithm(img, bin_img);

    // bfs.
    if(alg_turn[0] == true)
    {
        auto start = std::chrono::steady_clock::now();
        algorithm.breadth_first_search(initial_state, final_state);
        auto end = std::chrono::steady_clock::now();
        auto diff = end - start;

        if(algorithm.path_exist())
        {
            ui->algorithm_output->append("Max search depth: " + QString::number(algorithm.get_level()));
            ui->algorithm_output->append("Nodes expanded: " + QString::number(algorithm.get_nodes_expanded()));
            ui->algorithm_output->append("Time: " +
                             QString::number(std::chrono::duration<double, std::milli>(diff).count()) +
                             " ms");
            algorithm.draw_path();
        } else {
            QMessageBox::critical(this, tr("NO SOLUTION"), tr("THERE IS NO SOLUTION"));
        }
    }
    // dfs.
    else if(alg_turn[1] == true) {
        auto start = std::chrono::steady_clock::now();
        algorithm.depth_first_search(initial_state, final_state);
        auto end = std::chrono::steady_clock::now();
        auto diff = end - start;

        if(algorithm.path_exist())
        {
            ui->algorithm_output->append("Max search depth: " + QString::number(algorithm.get_level()));
            ui->algorithm_output->append("Nodes expanded: " + QString::number(algorithm.get_nodes_expanded()));
            ui->algorithm_output->append("Time: " +
                             QString::number(std::chrono::duration<double, std::milli>(diff).count()) +
                             " ms");
            algorithm.draw_path();
        } else {
            QMessageBox::critical(this, tr("NO SOLUTION"), tr("THERE IS NO SOLUTION"));
        }
    }
    // ids
    else if(alg_turn[2] == true) {
        auto start = std::chrono::steady_clock::now();
        algorithm.iterative_deepening_search(initial_state, final_state, INT_MAX);
        auto end = std::chrono::steady_clock::now();
        auto diff = end - start;

        if(algorithm.path_exist())
        {
            ui->algorithm_output->append("Max search depth: " + QString::number(algorithm.get_level()));
            ui->algorithm_output->append("Nodes expanded: " + QString::number(algorithm.get_nodes_expanded()));
            ui->algorithm_output->append("Time: " +
                             QString::number(std::chrono::duration<double, std::milli>(diff).count()) +
                             " ms");
            algorithm.draw_path();
        } else {
            QMessageBox::critical(this, tr("NO SOLUTION"), tr("THERE IS NO SOLUTION"));
        }

    }

    ui->dest_x->clear();
    ui->dest_y->clear();
    ui->source_x->clear();
    ui->source_y->clear();

    // Reset turns.
    alg_turn[0] = false;
    alg_turn[1] = false;
    alg_turn[2] = false;
}

/* @brief: Mouse click event from the mouse, given by opencv to get the event and (x,y).
 * @param event: The event from the mouse.
 * @param x: x coordinate.
 * @param y: x coordinate.
 * @param auxStruct: Auxiliar struct used to get the (x,y) coordinates. */
void MainWindow::click_event(int event, int x, int y, int, void * auxStruct)
{
    AuxPoint *point_data = ((AuxPoint*) auxStruct);
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        // Set (x,y) and draw a circle.
        point_data->x = x;
        point_data->y = y;
        cv::circle(point_data->img, cv::Point(x,y), 2, cv::Scalar(200, 255, 100), cv::FILLED);
        cv::imshow("Original", point_data->img);
    }
}

/* @brief: Will config the image to its proper use. */
void MainWindow::config_image()
{
    QDir dir("../images");
    QString file_name = QFileDialog::getOpenFileName(this,
                                    tr("Open Image"),
                                    dir.path(),
                                    tr("Image Files (*.png *.jpg *.bmp *.jpeg)"));

    img = cv::imread(file_name.toStdString(), cv::IMREAD_COLOR);
    if(img.empty())
        QMessageBox::critical(this, tr("ERROR"), tr("ERROR OPENING IMAGE, TRY AGAIN"));

    // Convert to gray and binary.
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img, bin_img, 127, 255, cv::THRESH_BINARY);

    cv::imshow("Original", img);
    ui->algorithm_output->clear();

    auxPoint.img = img;
    cv::setMouseCallback("Original", click_event, &auxPoint);
    cv::waitKey(1);
}

void MainWindow::on_set_source_clicked()
{
    ui->source_x->setText(QString::number(auxPoint.x));
    ui->source_y->setText(QString::number(auxPoint.y));
}

void MainWindow::on_set_dest_clicked()
{
    ui->dest_x->setText(QString::number(auxPoint.x));
    ui->dest_y->setText(QString::number(auxPoint.y));
}
