/*******************************************************************
*	File name: 		Scene.h
*	Description:	Declaration of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef MOTION_PLAN_SCENE_H
#define MOTION_PLAN_SCENE_H

#pragma once

#include <math.h>
#include <vector>
#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <time.h>
#include "GraphicsPolygon.h"
#include <QPropertyAnimation>
#include "basic_typedef.h"
#include <QPen>
#include "colors.h"
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <iostream>
#include <fstream>
#include "get_bounding_circle_center.h"

const double POINT_RADIUS = 1.5;
const double PRECISION = 25;
const int ANIMATION_STAGE_DURATION = 500;
const int ANIMATION_STEP_LENGTH = 50;

class Scene : public QGraphicsScene
{
	Q_OBJECT

public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	Scene(QObject * parent = 0);

	~Scene(void);


	////////////////////////
	// Access methods
	////////////////////////

	////////////////////////
	// Modifiers
	////////////////////////
	/*  Adds the point qp to the obstacle currently drawn */
	bool Scene::addPointToObstacle(QPointF qp);

	bool addPoint(QPointF qp, bool obs)
	{
		QPointF qplast,qpfirst;
		bool noPoints = m_currentPoints.empty();

		//extract last/first points if needed
		if (!noPoints){
			qplast = m_currentPoints.last();
			qpfirst = m_currentPoints.first();
		}

		// check if the new point is the first point, i.e the obstacle is closed
		qreal distance = (qp.rx()-qpfirst.rx())*(qp.rx()-qpfirst.rx()) 
			+ (qp.ry()-qpfirst.ry())*(qp.ry()-qpfirst.ry());

		if ((distance < PRECISION) && !noPoints)
		{
			if (obs)
				createObstacle();
			else 
				createRobot();
			clearLeftover();
			return true;
		}
		else
		{
			// Save current point
			m_currentPoints.push_back(QPointF(qp));
		
			// Add ellipse to the output
			QRectF rectangle = rectForPoint(qp,POINT_RADIUS);
			QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
			ellipse->setRect(rectangle);
			m_currentEllipses.push_back(ellipse);
			addItem(ellipse);

			if (!noPoints)
			{
				QLineF line(qplast,qp);
				QGraphicsLineItem*	lineItem = new QGraphicsLineItem(line,0,this);
				m_currentLines.push_back(lineItem);
				addItem(lineItem);
			}
			return false;
		}
	}

	bool addPointToRobot(QPointF qp)
	{
		return addPoint(qp,false);
	}

	/*	If the current obstacle has at least one point, it turns it into polygon, 
	otherwise it deletes it	*/
	void Scene::createRobot()
	{
		if (m_currentPoints.size() > 1)
		{
			QPolygonF polygon(m_currentPoints);

			QPointF ref_point = reference_point(polygon);

			m_robot_polygon = QPolygonF(m_currentPoints);
			m_robot_polygon.translate(-ref_point.x(), -ref_point.y());
			addConfiguration();
		} 
	}

	static QPointF		reference_point(QPolygonF poly)
	{
		vector<pair<double,double>> pair_points;
		for (int i = 0; i < poly.size(); i++)
		{
			pair<double, double> pair_p(poly[i].x(), poly[i].y());
			pair_points.push_back(pair_p);
		}

		pair<double,double> ref_point_double = get_bounding_circle_center(pair_points.begin(), pair_points.end());
		QPointF ref(ref_point_double.first, ref_point_double.second);

		return ref;
	}

	void addConfiguration()
	{
		QPolygonF polygon = m_robot_polygon;
		// move polygon to center
		QPointF ref_point = polygon.first();
		QMatrix matrix_normalize;
		matrix_normalize.translate(-ref_point.rx(), -ref_point.ry());
		QPolygonF normalized_polygon = matrix_normalize.map(polygon);

		QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
		item->setPolygon(normalized_polygon);
		item->setZValue(0);
		m_displayed_robots.push_back(item);

		item->setBrush(QBrush(CONFIGURATION_COLOR));
		item->setPolygon(polygon);
		item->setActive(true);
		item->setZValue(0);
		item->setFlag( QGraphicsItem::ItemIsMovable );
		item->setFlag( QGraphicsItem::ItemIsSelectable );
		m_conf_rotation.push_back(0);		
		m_selected_configuration = m_conf_rotation.size() - 1;

	}

	/*	If the current obstacle has at least one point, it turns it into polygon, 
	otherwise it deletes it	*/
	void Scene::createObstacle()
	{
		if (m_currentPoints.size() > 1)
		{
			QPolygonF polygon(m_currentPoints);
			QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
			m_vecObstacles.push_back(item);
			item->setBrush(QBrush(OBSTACLE_COLOR));
			item->setPolygon(polygon);
			item->setActive(true);
			item->setZValue(0);
			item->setFlag( QGraphicsItem::ItemIsMovable );
		} 
	}

	/*	Sets the room	*/
	void Scene::setRoom(QPointF topleft, QPointF bottomright);

	void animate();

	//QVariant interpolateQPolygonF(const QPolygonF &start, const QPolygonF &end, qreal progress);

	void rotateRobot(int diff)
	{
		int trueDegrees = diff / 8;
		int degree = trueDegrees / 2;

		if (!m_conf_rotation.empty())
			rotateRobotDegrees(degree);
	}

	void rotateRobotDegrees(double deg)
	{
		double degree = deg;

		QPolygonF robot_poly = m_displayed_robots[m_selected_configuration]->polygon();
		QPointF ref_point = robot_poly[0];

		QMatrix matrix_rotation;
		matrix_rotation.rotate(degree);

		m_conf_rotation[m_selected_configuration] = m_conf_rotation[m_selected_configuration] + degree;
		QPolygonF rotated_polygon = matrix_rotation.map(robot_poly);

		m_displayed_robots[m_selected_configuration] ->setPolygon(rotated_polygon);
		m_displayed_robots[m_selected_configuration]->setZValue(0);
	}

	void set_configuration_state(vector<int> states)
	{
		for (int i = 0; i < states.size(); i++)
		{
			int s = states[i];

			if (s == 4)
			{
				m_displayed_robots[i]->setVisible(false);
				continue;
			}

			m_displayed_robots[i]->setVisible(true);

			QRectF  rect = m_displayed_robots[i]->boundingRect();
			/*QPointF top_left = m_displayed_robots[i]->mapToScene(rect.topLeft());
			QPointF bottom_right = m_displayed_robots[i]->mapToScene(rect.bottomRight());*/
			QPointF top_left = rect.topLeft();
			QPointF bottom_right = rect.bottomRight();

			QLinearGradient gradient(top_left, bottom_right);
			gradient.setColorAt(0,CONFIGURATION_COLOR_TOP[s]);
			gradient.setColorAt(1,CONFIGURATION_COLOR_BOTTOM[s]);
			gradient.setSpread(QGradient::PadSpread);
			QBrush brush(gradient);

			m_displayed_robots[i]->setBrush(brush);
		}

		
		emit displayChanged();
	}

	////////////////////////
	// Other methods
	////////////////////////
	
	///////////////////////
	// Helper methods
	///////////////////////

	/* Returns a rectangle with center point qp and height/width = radius*2 */
	QRectF rectForPoint(QPointF qp, double radius);

	void clearLeftover()
	{
		m_currentPoints.clear();
	
		while (!m_currentEllipses.empty())
		{
			QGraphicsEllipseItem* ellipse = m_currentEllipses.last();
			removeItem(ellipse);
			delete ellipse;
			m_currentEllipses.pop_back();
		}

		while (!m_currentLines.empty())
		{
			QGraphicsLineItem* line = m_currentLines.last();
			removeItem(line);
			delete line;
			m_currentLines.pop_back();
		}
	}

	// Save scene to file
	void	saveToFile(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}

		// write workspace
		int obstacle_num = m_vecObstacles.size();
		fprintf(fOutput, "%d\n", obstacle_num);

		//

	}

	QPointF point_normalize(QPointF p)
	{
		return QPointF(double_normalize_x(p.x()), double_normalize_y(p.y()));
	}

	/*double double_normalize_x(double x)
	{
		return 2 * (x + 218)/542 - 1.0;
	}
	
	double double_normalize_y(double y)
	{
		return 2 * (y + 253)/542 - 1.0;
	}

	QPointF point_unnormalize(QPointF p)
	{
		return QPointF(double_unnormalize_x(p.x()), double_unnormalize_y(p.y()));
	}

	double double_unnormalize_x(double x)
	{
		return (x + 1.0)*542/2 - 218;
	}
	
	double double_unnormalize_y(double y)
	{
		return (y + 1.0)*542/2  - 253;
	}*/

	double double_normalize_x(double x)
	{
		return 1 * (x + 218)/542 - 0.5;
	}
	
	double double_normalize_y(double y)
	{
		return 1 * (y + 253)/542 - 0.5;
	}

	QPointF point_unnormalize(QPointF p)
	{
		return QPointF(double_unnormalize_x(p.x()), double_unnormalize_y(p.y()));
	}

	double double_unnormalize_x(double x)
	{
		return (x + 0.5)*542 - 218;
	}
	
	double double_unnormalize_y(double y)
	{
		return (y + 0.5)*542  - 253;
	}

	static QPolygonF rotate_polygon(QPolygonF	p, double angle)
	{
		// TODO : fix this function to the new reference point
		QPolygonF polygon;
		QPointF ref_point = reference_point(p);

		QMatrix matrix_normalize;
		matrix_normalize.translate(-ref_point.rx(), -ref_point.ry());
		polygon = matrix_normalize.map(p);

		QMatrix matrix_rotation;
		matrix_rotation.rotate(angle);
		polygon = matrix_rotation.map(polygon);

		QMatrix matrix_unnormalize;
		matrix_unnormalize.translate(ref_point.rx(), ref_point.ry());
		polygon = matrix_unnormalize.map(polygon);

		return polygon;
	}

	void	loadFromFile(const char* pi_szFileName);

public slots:
	virtual void mousePressEvent( QMouseEvent *e ){};

	void selectedUpdate()
	{
		for (int i = 0; i < m_displayed_robots.size(); i++)
		{
			if (m_displayed_robots[i]->isSelected())
			{
				m_selected_configuration = i;
				return;
			}
		}
	
	}

	void	save_robot(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}

		// move robot's reference point to (0,0);
		QPolygonF polygon  = m_robot_polygon;
		QPointF ref_point = polygon.first();
		QMatrix matrix;
		matrix.translate(-ref_point.rx(), -ref_point.ry());
		QPolygonF normalized_polygon = matrix.map(polygon);

		write_polygon(fOutput, normalized_polygon);

		fclose(fOutput);
	}

	void	load_robot(const char* pi_szFileName)
	{
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		int num_vertices;
		*in >> num_vertices;

		m_currentPoints =  read_polygon(num_vertices, in);
		createRobot();
		
		ifile.close();
	}

	void	save_workspace(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}
		int size = m_vecObstacles.size();
		fprintf(fOutput, "\n%d\n", size);

		for (int i = 0; i < size; i++)
		{
			QPolygonF fake_polygon = m_vecObstacles[i]->polygon();
			QPolygonF real_polygon = m_vecObstacles[i]->mapToScene(fake_polygon);
			write_polygon(fOutput, real_polygon);
		}

		fclose(fOutput);
	}

	void	load_workspace(const char* pi_szFileName)
	{
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		int num_obstacles;
		*in >> num_obstacles;

		if (num_obstacles != 0)
		{
			for (int i = 0; i < num_obstacles; i++)
			{
				int num_vertices;
				*in >> num_vertices;

				m_currentPoints =  read_polygon(num_vertices, in);
				createObstacle();
			}
		}
		
		ifile.close();
	}

	QVector<QPointF>	read_polygon(int num_vertices, std::istream* in)
	{
		QVector<QPointF> result;
		for (int i = 0; i < num_vertices; i++)
		{
			double x,y;
			*in >> x >> y;
			result.push_back(point_unnormalize(QPointF(x,y)));
		}

		return result;
	}

	void	save_query(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}

		
		int size = m_conf_rotation.size();
		fprintf(fOutput, "\n%d\n", size);

		for (int i = 0; i < size; i++)
		{
			QPointF fake_point = reference_point(m_displayed_robots[i]->polygon());
			QPointF real_point = point_normalize(m_displayed_robots[i]->mapToScene(fake_point));

			fprintf(fOutput, "%.5f %.5f %.5f ", real_point.x(), real_point.y(), m_conf_rotation[i]);
		}
		

		fclose(fOutput);
	}

	void	load_query(const char* pi_szFileName)
	{
		// TODO: fix 
		//const char* pi_szFileName2 = "D:\\Projects\\Software\\MRPoly\\MRMPApplication\\query.txt";
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		for (int i = 0; i < m_displayed_robots.size(); i++)
		{
			removeItem(m_displayed_robots[i]);
		}
		
		m_displayed_robots.clear();

		m_conf_rotation.clear();

		int num_confs;
		*in >> num_confs;

		for (int i = 0; i < num_confs; i++)
		{
			addConfiguration();
			int conf = m_selected_configuration;

			double x,y;
			double theta;
			*in >> x >> y >> theta;
			QPointF point(x,y);
			QPointF scene_point = point_unnormalize(point);

			rotateRobotDegrees(theta);
			m_displayed_robots[conf]->translate(scene_point.x(), scene_point.y());
		}
		ifile.close();
	}

	void	load_path(const char* pi_szFileName)
	{
		//const char* pi_szFileName2 = "D:\\Projects\\Software\\MRPoly\\MRMPApplication\\path.txt";

		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		m_path.clear();

		int num_sections;
		*in >> num_sections;

		for (int i = 0; i < num_sections; i++)
		{
			vector<QConf> current_section;
			double duration; 

			for (int r = 0; r < 2; r++)
			{
				double x,y;
				double theta;
				*in >> x >> y >> theta;
				QPointF point(x,y);
				QPointF scene_point;
				if (i == 0)
					scene_point = point_unnormalize(point);
				else
					scene_point =QPointF(x * 542, y * 542);
				
				QConf conf(scene_point, theta);
				current_section.push_back(conf);
			}

			if (i > 0)
			{
				*in >> duration; 
				m_path_length.push_back(duration);
			}

			m_path.push_back(current_section);
		}

		ifile.close();
	}

	void	load_state(const char* pi_szFileName)
	{
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		m_conf_state.clear();
		m_conf_change.clear();
		m_current_state = 0;

		int num_sections;
		*in >> num_sections;

		for (int i = 0; i < num_sections; i++)
		{
			int change;
			*in >> change;

			m_conf_change.push_back(change);
			
			vector<int> current_state;

			for (int i = 0; i < m_displayed_robots.size(); i++)
			{
				int s;
				*in >> s;
				current_state.push_back(s);	
			}
			
			m_conf_state.push_back(current_state);
		}

		ifile.close();
	}
	

	void animationComplete()
	{
		change_conf_state();
		if (m_displayed_robots.size()==1)
		 {
			 addItem(m_displayed_robots[0]);
		 }
	}

	void write_polygon(FILE* file, QPolygonF poly)
	{
		int num_vertices = poly.size();
		fprintf(file, "\n %d ", num_vertices);

		for (int i = 0; i < num_vertices; i++)
		{
			QPointF norm_point = point_normalize(poly[i]);
			double dx = norm_point.rx();
			double dy = norm_point.ry();

			fprintf(file, "%.2f %.2f ", dx, dy);
		}
	}

	void	clear_animation()
{
	for (int i = 0; i < m_displayed_animation_objects.size(); i++)
	{
		removeItem(m_displayed_animation_objects[i]);
	}

	for (int i = 0; i < m_displayed_animation_objects.size(); i++)
	{
		delete m_displayed_animation_objects[i];
	}

	m_displayed_animation_objects.clear();
}

	void	change_conf_state()
	{
		if (m_current_state < m_conf_state.size())
			set_configuration_state(m_conf_state[m_current_state]);

		m_current_state++;
	}
signals: 
	void displayChanged();

	void animationDone();

protected:
	
	virtual void keyPressEvent (QKeyEvent* evt);

	////////////////////////
	// Data Members
	////////////////////////
public:
	
	//	obstacles information
	QVector<QPointF>				m_currentPoints;
	QVector<QGraphicsEllipseItem*>	m_currentEllipses;
	QVector<QGraphicsLineItem*>		m_currentLines;
	QVector<QGraphicsPolygonItem*>	m_vecObstacles;

	//	robots information
	QPolygonF									m_robot_polygon;
	QVector<QGraphicsPolygonItem*>				m_displayed_robots;
	QVector<QGraphicsPolygonItem*>				m_displayed_animation_objects;
	QVector<double>								m_conf_rotation;
	int											m_selected_configuration;

	QPointF							m_roomTL;
	QPointF							m_roomBR;

	vector<vector<QConf>>			m_path;
	vector<double>					m_path_length;

	vector<int>						m_conf_change;
	vector<vector<int>>				m_conf_state;
	int								m_current_state;
};

#endif
