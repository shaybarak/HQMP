/*******************************************************************
*	File name: 		MRMPDrawingBox.cpp
*	Description:	Implementation of the MRMPDrawingBox class.
*					This class is a component of the GUI main window.
*					It inherits QGraphicsView and it is used as the drawing area of the application.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "MRMPDrawingBox.h"
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QRect>
#include <QRectF>
#include <QGraphicsView>
#include <QPolygonF>
#include <QPropertyAnimation>
#include <QGraphicsPolygonItem>
#include <string>
#include <QPolygonF>
#include <QMessageBox>
#include <QMetaType>


Q_DECLARE_METATYPE(QPolygonF)

#pragma warning(disable: 4291 4503)
using namespace std;

const int DEFAULT_ANIMATION_SPEED = 500;

MRMPDrawingBox::MRMPDrawingBox(QWidget *parent)
	: QGraphicsView(parent)
{
	ui.setupUi(this);
	m_currentState = IDLE;
	m_mpScene=new Scene(this);
	this->setScene( m_mpScene );
	m_mpScene->setSceneRect(this->viewport()->geometry());
	QRectF rect = this->viewport()->geometry();
	QObject::connect(m_mpScene,SIGNAL(displayChanged()),this,SLOT(refresh()));
	//QObject::connect(m_mpScene,SIGNAL(animationDone()),this,SLOT(animationComplete()));

	//m_mpScene->animate();
	refresh();
	m_socket_counter = 0;

	setRenderHints(QPainter::Antialiasing);
}



MRMPDrawingBox::~MRMPDrawingBox()
{
	clear();
}

void MRMPDrawingBox::animationComplete()
{

	m_currentState = IDLE;

	//m_mpScene ->clear_animation();

	refresh();

}

//when our widget is pressed, and we are in a drawing modes we record point.
//and draw it by updating the widget.
void MRMPDrawingBox::mousePressEvent( QMouseEvent *e )
{
	
	QPointF sPos = mapToScene(e->posF().toPoint());
	setRenderHints(QPainter::Antialiasing);
	if (m_currentState == DRAW_OBSTACLE)
	{
		bool bres = m_mpScene->addPointToObstacle(sPos);
		if (bres)
			m_currentState = IDLE;	
	} 
	else if (m_currentState == DRAW_ROBOT)
	{
		bool bres = m_mpScene->addPointToRobot(sPos);
		if (bres)
			m_currentState = IDLE;	
		/*m_mpScene->addRobot(sPos);
		m_currentState = DRAW_TARGET;*/
	} else if (m_currentState == DRAW_TARGET)
	{
		/*m_mpScene->addTarget(sPos);
		m_currentState = IDLE;*/
	}

	this->show();	
}

//	tracks mouse wheel events
void MRMPDrawingBox::wheelEvent ( QWheelEvent *e )
{
	int diff = e->delta();
	m_mpScene->rotateRobot(diff);
	this->show();
}



//	orders the animation of paths
void MRMPDrawingBox::animateButtonPressed()
{
	m_mpScene->animate();
	this->show();
}

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

void MRMPDrawingBox::read_line()
	{
		m_notifier->setEnabled(false);
		std::string r = m_socket->receive_line();
		std::vector<std::string> decomposed_string;
		string_split(r, " ", std::back_inserter(decomposed_string));

		if (m_socket_counter == 0)
		{
			// retrieve filenames of input
			const char* filename_workspace = decomposed_string[0].c_str();
			const char* filename_robot = decomposed_string[1].c_str();
			const char* filename_query = decomposed_string[2].c_str();

			m_mpScene->load_workspace(filename_workspace);
			m_mpScene->load_robot(filename_robot);
			m_mpScene->load_query(filename_query);
		}
		else 
		{
			const char* filename_path = decomposed_string[0].c_str();
			const char* filename_state = decomposed_string[1].c_str();
			m_mpScene->load_path(filename_path);
			m_mpScene->load_state(filename_state);

			animateButtonPressed();
		}

		refresh();

		m_socket_counter++;

		m_notifier->setEnabled(true);
	}

void MRMPDrawingBox::connect_server()
	{
		/*SocketThread thread(this);
		thread.run();*/

		char* host_name ="127.0.0.1";
		int host_port   = 1103;
		m_socket = new Socket_client(host_name, host_port);
		SOCKET socket_id = m_socket->id();

		m_notifier = new QSocketNotifier(socket_id, QSocketNotifier::Read, this);
		connect(m_notifier, SIGNAL(activated(int)), this, SLOT(read_line()));
		m_notifier->setEnabled(true);
	}

QPolygonF MRMPDrawingBox::converToSceneCords(QPolygonF& poly)
{
	QPolygonF f;
	return f;
}

//start recording points for a new obstacle.
void MRMPDrawingBox::drawObstaclesButtonPushed(){
	if (m_currentState == IDLE)
		m_currentState = DRAW_OBSTACLE;
}

//start recording points for a new robot.
void MRMPDrawingBox::drawRobotsButtonPushed(){
	m_currentState = DRAW_ROBOT;
}

void MRMPDrawingBox::clear(){
	this->update();
	this->show();
}
void MRMPDrawingBox::clearResults(){

	this->update();
	this->show();
}

void MRMPDrawingBox::setSceneRoom(){
		QVector<QPointF> pointVector;
		QRect windGeo=this->viewport()->geometry();

		m_mpScene->setRoom(this->mapToScene(windGeo.topLeft()),
			this->mapToScene(windGeo.bottomRight()));
}

// Invokes the motion planning algorithm
void MRMPDrawingBox::execute()
{

}