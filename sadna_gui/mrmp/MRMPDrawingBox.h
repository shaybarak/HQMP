/*******************************************************************
*	File name: 		MRMPDrawingBox.h
*	Description:	Declaration of the MRMPDrawingBox class.
*					This class is a component of the GUI main window.
*					It inherits QGraphicsView and it is used as the drawing area of the application.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef OURDRAWINGBOX_H
#define OURDRAWINGBOX_H

#include <QWidget>
#include <QPointF>
#include <QPushButton>
#include <QVector>
#include <QString>
#include <QGraphicsEllipseItem>
#include <QDropEvent>
#include <QGraphicsView>
#include "ui_MRMPDrawingBox.h"
#include "Scene.h"
#include "Socket.h"
#include <QSocketNotifier>

using namespace std;

class MRMPDrawingBox : public QGraphicsView
{
	Q_OBJECT

public:

	enum State {
		IDLE,
		PRESENTATION,
		DRAW_OBSTACLE,
		DRAW_ROBOT,
		DRAW_TARGET,
		ANIMATION
	};

	
	////////////////////////
	// C'tors & D'tors
	////////////////////////
	MRMPDrawingBox(QWidget *parent = 0);
	~MRMPDrawingBox();
	
	//Setter
	void setFileLocation(string fileLocation);
	//Getter
	int  getAnimationMultiplier() { /*return m_AnimationSpeedMultiplier; */}
	////////////////////////
	// Modifiers
	////////////////////////
	template<typename OutputIterator>
	void string_split(std::string str,
					 std::string delim,
					 OutputIterator& oi)
	{
	 int cut_at;

	 while( (cut_at = str.find_first_of(delim)) != str.npos )
	 {
	   if(cut_at > 0)
	   {
		 std::string curr(str.substr(0, cut_at));
		 *oi++ = curr;
	   }
	   str = str.substr(cut_at+1);
	 }

	 if(str.length() > 0)
	   *oi++ = str;

	 return;

	}

	void load_workspace(QString qfile){
		
		setRenderHints(QPainter::Antialiasing);
		m_mpScene->load_workspace(qfile.toStdString().c_str());
		refresh();
	}

	void save_workspace(QString qfile){
		//setSceneRoom();
		m_mpScene->save_workspace(qfile.toStdString().c_str());
	}

	void load_robot(QString qfile){
		
		setRenderHints(QPainter::Antialiasing);
		m_mpScene->load_robot(qfile.toStdString().c_str());
		refresh();
	}

	void save_robot(QString qfile){
		//setSceneRoom();
		m_mpScene->save_robot(qfile.toStdString().c_str());
	}

	void load_query(QString qfile){
		setRenderHints(QPainter::Antialiasing);
		m_mpScene->load_query(qfile.toStdString().c_str());
		refresh();
	}

	void save_query(QString qfile){
		//setSceneRoom();
		m_mpScene->save_query(qfile.toStdString().c_str());
	}
	
	void load_path(QString qfile){
		//setRenderHints(QPainter::Antialiasing);
		m_mpScene->load_path(qfile.toStdString().c_str());
		//refresh();
	}

public slots:
	/* slots, connected to signals from other classes */
	
	//	orders the animation of paths
	void animateButtonPressed();

	void connect_server();

	void read_line();

	void addConfigurationButtonPressed()
	{

		m_mpScene->addConfiguration();
		refresh();
	}

	//	calls refresh
	void refresh()
	{
		this->show();
	}

	//  Gets a signal from the animation objects and deletes proper content
	void animationComplete();

	//	tracks mouse press evens of user.
	virtual void mousePressEvent( QMouseEvent *e );

	//	tracks mouse wheel events
	virtual void wheelEvent ( QWheelEvent *e );

	//	indicates that the obstacles button is pushed.
	void drawObstaclesButtonPushed(); 
	//	indicates that the robots button is pushed.
	void drawRobotsButtonPushed();
	//	clear scene.
	void clear();
	//	clear scene results.
	void clearResults();

	//	setter of the animation multiplier
	void setAnimationMultiplier(int animationFactor) 
	{
		//m_curAnimationSpeed = DEFAULT_ANIMATION_SPEED *(double)(100 - animationFactor)/50; 
		//m_mpScene->setAnimationMultiplier((double)(100 - animationFactor)/50);
	}
	
	// Invokes the motion planning algorithm
	void execute();

	void selected_robot_changed(QString s)
	{
		//m_mpScene->set_selected_robot(s.toInt());
	}
	
signals:
	/* signals to emit to other classes about our changes */
	void plannerFinished();
	
private:	

	// helper method
	QPolygonF converToSceneCords(QPolygonF& poly);

	// sets the scene configuration room
	void setSceneRoom();
	
	///////////////////
	//  DATA MEMBERS
	///////////////////
	
	Scene*							m_mpScene;

	Ui::MRMPDrawingBoxClass			ui;

	enum State						m_currentState;

	Socket_client* m_socket;
	int		m_socket_counter;

	QSocketNotifier* m_notifier;
};




#endif // OURDRAWINGBOX_H
