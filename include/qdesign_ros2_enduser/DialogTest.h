/********************************************************************************
** Form generated from reading UI file 'DialogTest.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DIALOGTEST_H
#define DIALOGTEST_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QPushButton *CancelBTN;
    QPushButton *ConfirmBTN;
    QLabel *label;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(1024, 768);  // Increased window size
        CancelBTN = new QPushButton(Dialog);
        CancelBTN->setObjectName(QString::fromUtf8("CancelBTN"));
        CancelBTN->setGeometry(QRect(300, 680, 89, 25));  // Moved buttons down
        ConfirmBTN = new QPushButton(Dialog);
        ConfirmBTN->setObjectName(QString::fromUtf8("ConfirmBTN"));
        ConfirmBTN->setGeometry(QRect(600, 680, 89, 25));  // Moved buttons down
        label = new QLabel(Dialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(12, 12, 1000, 650));  // Much larger label size
        label->setAlignment(Qt::AlignCenter);  // Center the content
        label->setStyleSheet("QLabel { background-color: black; }");  // Black background

        retranslateUi(Dialog);
        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QCoreApplication::translate("Dialog", "RealSense Camera Viewer", nullptr));
        CancelBTN->setText(QCoreApplication::translate("Dialog", "Cancel", nullptr));
        ConfirmBTN->setText(QCoreApplication::translate("Dialog", "Confirm", nullptr));
        label->setText(QCoreApplication::translate("Dialog", "No Camera Feed", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // DIALOGTEST_H
