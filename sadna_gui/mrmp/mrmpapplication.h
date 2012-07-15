/*******************************************************************
*	File name: 		MRMPApplication.h
*	Description:	Declaration of the MRMPApplication class.
*					This class is the Main Window of the GUI application.
*					It handles the GUI display and its subcomponents.
*
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef QTTESTAPPL_H
#define QTTESTAPPL_H

#include <QtGui/QMainWindow>
#include "ui_MRMPApplication.h"
#include <QFileDialog>

class MRMPApplication : public QMainWindow
{
	Q_OBJECT

////////////////////////
// C'tors & D'tors
////////////////////////

public:
	MRMPApplication(QWidget *parent = 0, Qt::WFlags flags = 0);
	~MRMPApplication();

	virtual bool notify(QObject *rec, QEvent *ev)
    {
        //qDebug() << "MRMPApplication::nofity";
        try {
            return MRMPApplication::notify(rec, ev);
        }
        catch( ... ) {
            //qDebug() << "Unknown Exception: Terminating!";
            exit(0);
        }
        return false;
    }

	
	
////////////////////////
// Modifiers
////////////////////////
	public slots:
	void save_workspace(){
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Workspace"),
													 "/workspace.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->save_workspace(fileName);
		}
	
	}

	void load_workspace(){
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Workspace"),
													 "/workspace.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->load_workspace(fileName);
			this->update();
		}
	}

	void save_robot(){
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Robot"),
													 "/robot.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->save_robot(fileName);
		}
	
	}

	void load_robot(){
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Robot"),
													 "/robot.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->load_robot(fileName);
			this->update();
		}
	}

	void save_query(){
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Query"),
													 "/query.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->save_query(fileName);
		}
	
	}

	void load_query(){
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Query"),
													 "/query.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->load_query(fileName);
			this->update();
		}
	}

	void load_path(){
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Path"),
													 "/path.txt",
													 tr("Text files (*.txt)"));
		//if a dialog is opened and cancelled, "" is returned;
		if(fileName.compare(QString(""))!=0){
			ui.DrawingBox->load_path(fileName);
			this->update();
		}
	}

	/////////////////////////////
// Other methods -- signals.
/////////////////////////////
	signals:
	
////////////////////////
// Data Members
////////////////////////

public slots:
	void executeComplete()
	{
		
		qApp->setQuitOnLastWindowClosed(true);
		qApp->quit();
		//this->close();
		exit(0);

	}

public:
	Ui::MRMPApplicationClass ui;

private :
	enum buttonState{drawState,doneState};

	buttonState drawObstaclesButton;
	buttonState drawRobotsButton;
	buttonState enterTargetButton;
	const double constOneMillion;
};

#endif // QTTESTAPPL_H
