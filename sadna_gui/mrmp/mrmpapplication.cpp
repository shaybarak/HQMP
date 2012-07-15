/*******************************************************************
*	File name: 		MRMPApplication.cpp
*	Description:	Implementation of the MRMPApplication class.
*					This class is the Main Window of the GUI application.
*					It handles the GUI display and its subcomponents.
*
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "MRMPApplication.h"
#include <QString>
#include <QFileDialog>
#include <iostream>
#include <fstream>



MRMPApplication::MRMPApplication(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags), constOneMillion(1000000)
{
	ui.setupUi(this);	
}

MRMPApplication::~MRMPApplication()
{
		
}

