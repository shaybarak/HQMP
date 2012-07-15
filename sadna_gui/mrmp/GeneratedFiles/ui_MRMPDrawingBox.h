/********************************************************************************
** Form generated from reading UI file 'MRMPDrawingBox.ui'
**
** Created: Thu 12. Jul 14:19:11 2012
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MRMPDRAWINGBOX_H
#define UI_MRMPDRAWINGBOX_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MRMPDrawingBoxClass
{
public:

    void setupUi(QWidget *MRMPDrawingBoxClass)
    {
        if (MRMPDrawingBoxClass->objectName().isEmpty())
            MRMPDrawingBoxClass->setObjectName(QString::fromUtf8("MRMPDrawingBoxClass"));
        MRMPDrawingBoxClass->resize(400, 300);

        retranslateUi(MRMPDrawingBoxClass);

        QMetaObject::connectSlotsByName(MRMPDrawingBoxClass);
    } // setupUi

    void retranslateUi(QWidget *MRMPDrawingBoxClass)
    {
        MRMPDrawingBoxClass->setWindowTitle(QApplication::translate("MRMPDrawingBoxClass", "MRMPDrawingBox", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MRMPDrawingBoxClass: public Ui_MRMPDrawingBoxClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MRMPDRAWINGBOX_H
