/********************************************************************************
** Form generated from reading UI file 'mrmpapplication.ui'
**
** Created: Thu 12. Jul 14:19:11 2012
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MRMPAPPLICATION_H
#define UI_MRMPAPPLICATION_H

#include <MRMPDrawingBox.h>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MRMPApplicationClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    MRMPDrawingBox *DrawingBox;
    QGroupBox *groupBox;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QPushButton *buttonConnect;
    QLabel *label_2;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_3;
    QPushButton *buttonLoadWorkspace;
    QPushButton *buttonDrawObstacles_3;
    QPushButton *buttonLoadRobot;
    QPushButton *buttonDrawRobots_3;
    QPushButton *buttonLoadQuery;
    QPushButton *buttonAddConfiguration;
    QPushButton *buttonLoadPath;
    QPushButton *buttonAnimate;
    QLabel *label;
    QPushButton *buttonSaveWorkspace;
    QPushButton *buttonSaveRobot;
    QPushButton *buttonSaveQuery;
    QSpacerItem *verticalSpacer;
    QSpacerItem *verticalSpacer_2;
    QSpacerItem *verticalSpacer_3;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MRMPApplicationClass)
    {
        if (MRMPApplicationClass->objectName().isEmpty())
            MRMPApplicationClass->setObjectName(QString::fromUtf8("MRMPApplicationClass"));
        MRMPApplicationClass->resize(817, 592);
        QIcon icon;
        icon.addFile(QString::fromUtf8("Resources/icons/robot4_32.png"), QSize(), QIcon::Normal, QIcon::Off);
        MRMPApplicationClass->setWindowIcon(icon);
        MRMPApplicationClass->setWindowOpacity(1);
        MRMPApplicationClass->setToolButtonStyle(Qt::ToolButtonIconOnly);
        centralWidget = new QWidget(MRMPApplicationClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(false);
        centralWidget->setAutoFillBackground(false);
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        DrawingBox = new MRMPDrawingBox(centralWidget);
        DrawingBox->setObjectName(QString::fromUtf8("DrawingBox"));
        DrawingBox->setEnabled(true);
        DrawingBox->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        DrawingBox->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        DrawingBox->setDragMode(QGraphicsView::RubberBandDrag);

        gridLayout_2->addWidget(DrawingBox, 0, 0, 1, 1);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        groupBox->setMinimumSize(QSize(251, 521));
        groupBox->setMaximumSize(QSize(251, 16777215));
        groupBox->setFlat(false);
        groupBox->setCheckable(false);
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 280, 231, 44));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setHorizontalSpacing(12);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        buttonConnect = new QPushButton(layoutWidget);
        buttonConnect->setObjectName(QString::fromUtf8("buttonConnect"));

        gridLayout->addWidget(buttonConnect, 11, 1, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 0, 1, 1, 1);

        gridLayoutWidget = new QWidget(groupBox);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(0, 30, 252, 201));
        gridLayout_3 = new QGridLayout(gridLayoutWidget);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        buttonLoadWorkspace = new QPushButton(gridLayoutWidget);
        buttonLoadWorkspace->setObjectName(QString::fromUtf8("buttonLoadWorkspace"));

        gridLayout_3->addWidget(buttonLoadWorkspace, 6, 2, 1, 1);

        buttonDrawObstacles_3 = new QPushButton(gridLayoutWidget);
        buttonDrawObstacles_3->setObjectName(QString::fromUtf8("buttonDrawObstacles_3"));

        gridLayout_3->addWidget(buttonDrawObstacles_3, 2, 2, 1, 1);

        buttonLoadRobot = new QPushButton(gridLayoutWidget);
        buttonLoadRobot->setObjectName(QString::fromUtf8("buttonLoadRobot"));

        gridLayout_3->addWidget(buttonLoadRobot, 6, 1, 1, 1);

        buttonDrawRobots_3 = new QPushButton(gridLayoutWidget);
        buttonDrawRobots_3->setObjectName(QString::fromUtf8("buttonDrawRobots_3"));

        gridLayout_3->addWidget(buttonDrawRobots_3, 2, 1, 1, 1);

        buttonLoadQuery = new QPushButton(gridLayoutWidget);
        buttonLoadQuery->setObjectName(QString::fromUtf8("buttonLoadQuery"));

        gridLayout_3->addWidget(buttonLoadQuery, 6, 0, 1, 1);

        buttonAddConfiguration = new QPushButton(gridLayoutWidget);
        buttonAddConfiguration->setObjectName(QString::fromUtf8("buttonAddConfiguration"));

        gridLayout_3->addWidget(buttonAddConfiguration, 2, 0, 1, 1);

        buttonLoadPath = new QPushButton(gridLayoutWidget);
        buttonLoadPath->setObjectName(QString::fromUtf8("buttonLoadPath"));

        gridLayout_3->addWidget(buttonLoadPath, 8, 1, 1, 1);

        buttonAnimate = new QPushButton(gridLayoutWidget);
        buttonAnimate->setObjectName(QString::fromUtf8("buttonAnimate"));
        buttonAnimate->setEnabled(true);

        gridLayout_3->addWidget(buttonAnimate, 10, 1, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_3->addWidget(label, 1, 1, 1, 1);

        buttonSaveWorkspace = new QPushButton(gridLayoutWidget);
        buttonSaveWorkspace->setObjectName(QString::fromUtf8("buttonSaveWorkspace"));

        gridLayout_3->addWidget(buttonSaveWorkspace, 4, 2, 1, 1);

        buttonSaveRobot = new QPushButton(gridLayoutWidget);
        buttonSaveRobot->setObjectName(QString::fromUtf8("buttonSaveRobot"));

        gridLayout_3->addWidget(buttonSaveRobot, 4, 1, 1, 1);

        buttonSaveQuery = new QPushButton(gridLayoutWidget);
        buttonSaveQuery->setObjectName(QString::fromUtf8("buttonSaveQuery"));

        gridLayout_3->addWidget(buttonSaveQuery, 4, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer, 3, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer_2, 5, 1, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer_3, 9, 1, 1, 1);


        gridLayout_2->addWidget(groupBox, 0, 1, 1, 1);

        MRMPApplicationClass->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MRMPApplicationClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        mainToolBar->setEnabled(true);
        MRMPApplicationClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MRMPApplicationClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MRMPApplicationClass->setStatusBar(statusBar);

        mainToolBar->addSeparator();

        retranslateUi(MRMPApplicationClass);
        QObject::connect(buttonDrawObstacles_3, SIGNAL(clicked()), DrawingBox, SLOT(drawObstaclesButtonPushed()));
        QObject::connect(DrawingBox, SIGNAL(plannerFinished()), MRMPApplicationClass, SLOT(executeComplete()));
        QObject::connect(buttonDrawRobots_3, SIGNAL(clicked()), DrawingBox, SLOT(drawRobotsButtonPushed()));
        QObject::connect(buttonAddConfiguration, SIGNAL(clicked()), DrawingBox, SLOT(addConfigurationButtonPressed()));
        QObject::connect(buttonAnimate, SIGNAL(clicked()), DrawingBox, SLOT(animateButtonPressed()));
        QObject::connect(buttonConnect, SIGNAL(clicked()), DrawingBox, SLOT(connect_server()));
        QObject::connect(buttonLoadWorkspace, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_workspace()));
        QObject::connect(buttonSaveWorkspace, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_workspace()));
        QObject::connect(buttonLoadRobot, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_robot()));
        QObject::connect(buttonSaveRobot, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_robot()));
        QObject::connect(buttonSaveQuery, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_query()));
        QObject::connect(buttonLoadQuery, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_query()));
        QObject::connect(buttonLoadPath, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_path()));

        QMetaObject::connectSlotsByName(MRMPApplicationClass);
    } // setupUi

    void retranslateUi(QMainWindow *MRMPApplicationClass)
    {
        MRMPApplicationClass->setWindowTitle(QApplication::translate("MRMPApplicationClass", "Multiple Robot Motion Planner ", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MRMPApplicationClass", "Controls", 0, QApplication::UnicodeUTF8));
        buttonConnect->setText(QApplication::translate("MRMPApplicationClass", "Connect to Server", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MRMPApplicationClass", "Online Mode", 0, QApplication::UnicodeUTF8));
        buttonLoadWorkspace->setText(QApplication::translate("MRMPApplicationClass", "Load Workspace", 0, QApplication::UnicodeUTF8));
        buttonDrawObstacles_3->setText(QApplication::translate("MRMPApplicationClass", "Draw Obstacle", 0, QApplication::UnicodeUTF8));
        buttonLoadRobot->setText(QApplication::translate("MRMPApplicationClass", "Load Robot", 0, QApplication::UnicodeUTF8));
        buttonDrawRobots_3->setText(QApplication::translate("MRMPApplicationClass", "Draw Robot", 0, QApplication::UnicodeUTF8));
        buttonLoadQuery->setText(QApplication::translate("MRMPApplicationClass", "Load Query", 0, QApplication::UnicodeUTF8));
        buttonAddConfiguration->setText(QApplication::translate("MRMPApplicationClass", "Add Query", 0, QApplication::UnicodeUTF8));
        buttonLoadPath->setText(QApplication::translate("MRMPApplicationClass", "Load Path", 0, QApplication::UnicodeUTF8));
        buttonAnimate->setText(QApplication::translate("MRMPApplicationClass", "Animate", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MRMPApplicationClass", "Offline Mode", 0, QApplication::UnicodeUTF8));
        buttonSaveWorkspace->setText(QApplication::translate("MRMPApplicationClass", "Save Workspace", 0, QApplication::UnicodeUTF8));
        buttonSaveRobot->setText(QApplication::translate("MRMPApplicationClass", "Save Robot", 0, QApplication::UnicodeUTF8));
        buttonSaveQuery->setText(QApplication::translate("MRMPApplicationClass", "Save Query", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MRMPApplicationClass: public Ui_MRMPApplicationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MRMPAPPLICATION_H
