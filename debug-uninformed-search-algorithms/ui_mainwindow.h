/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionBreadth_first_search;
    QAction *actionDepth_first_searc;
    QAction *actionIterative_deepening_search;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLineEdit *dest_x;
    QLabel *label;
    QLineEdit *source_x;
    QLineEdit *source_y;
    QLineEdit *dest_y;
    QPushButton *set_source;
    QPushButton *set_dest;
    QTextEdit *algorithm_output;
    QPushButton *btn_process;
    QMenuBar *menubar;
    QMenu *menuFiles;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(360, 237);
        actionBreadth_first_search = new QAction(MainWindow);
        actionBreadth_first_search->setObjectName(QString::fromUtf8("actionBreadth_first_search"));
        actionDepth_first_searc = new QAction(MainWindow);
        actionDepth_first_searc->setObjectName(QString::fromUtf8("actionDepth_first_searc"));
        actionIterative_deepening_search = new QAction(MainWindow);
        actionIterative_deepening_search->setObjectName(QString::fromUtf8("actionIterative_deepening_search"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        dest_x = new QLineEdit(centralwidget);
        dest_x->setObjectName(QString::fromUtf8("dest_x"));

        gridLayout->addWidget(dest_x, 1, 2, 1, 1);

        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        source_x = new QLineEdit(centralwidget);
        source_x->setObjectName(QString::fromUtf8("source_x"));

        gridLayout->addWidget(source_x, 0, 2, 1, 1);

        source_y = new QLineEdit(centralwidget);
        source_y->setObjectName(QString::fromUtf8("source_y"));

        gridLayout->addWidget(source_y, 0, 3, 1, 1);

        dest_y = new QLineEdit(centralwidget);
        dest_y->setObjectName(QString::fromUtf8("dest_y"));

        gridLayout->addWidget(dest_y, 1, 3, 1, 1);

        set_source = new QPushButton(centralwidget);
        set_source->setObjectName(QString::fromUtf8("set_source"));

        gridLayout->addWidget(set_source, 0, 1, 1, 1);

        set_dest = new QPushButton(centralwidget);
        set_dest->setObjectName(QString::fromUtf8("set_dest"));

        gridLayout->addWidget(set_dest, 1, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);

        algorithm_output = new QTextEdit(centralwidget);
        algorithm_output->setObjectName(QString::fromUtf8("algorithm_output"));

        verticalLayout->addWidget(algorithm_output);

        btn_process = new QPushButton(centralwidget);
        btn_process->setObjectName(QString::fromUtf8("btn_process"));

        verticalLayout->addWidget(btn_process);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 360, 24));
        menuFiles = new QMenu(menubar);
        menuFiles->setObjectName(QString::fromUtf8("menuFiles"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFiles->menuAction());
        menuFiles->addAction(actionBreadth_first_search);
        menuFiles->addAction(actionDepth_first_searc);
        menuFiles->addAction(actionIterative_deepening_search);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Uninformed search algorithms", nullptr));
        actionBreadth_first_search->setText(QCoreApplication::translate("MainWindow", "Breadth first search", nullptr));
        actionDepth_first_searc->setText(QCoreApplication::translate("MainWindow", "Depth first search", nullptr));
        actionIterative_deepening_search->setText(QCoreApplication::translate("MainWindow", "Iterative Deepening Search", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "End (x,y)", nullptr));
        dest_x->setText(QString());
        label->setText(QCoreApplication::translate("MainWindow", "Start (x,y)", nullptr));
        source_x->setText(QString());
        source_y->setText(QString());
        dest_y->setText(QString());
        set_source->setText(QCoreApplication::translate("MainWindow", "Set (x,y)", nullptr));
        set_dest->setText(QCoreApplication::translate("MainWindow", "Set (x,y)", nullptr));
        btn_process->setText(QCoreApplication::translate("MainWindow", "Process", nullptr));
        menuFiles->setTitle(QCoreApplication::translate("MainWindow", "Algorithms", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
