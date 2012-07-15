/*******************************************************************
*	File name: 		Scene.h
*	Description:	Implementation of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "Scene.h"
#include <QPropertyAnimation>
#include <iostream>
#include <fstream>
#include <QDataStream>
#include <qcolor.h>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
 #include <QVariantAnimation>
#include <QMetaType>

////////////////////////
// C'tors & D'tors
////////////////////////

Q_DECLARE_METATYPE(QPolygonF)

Scene::Scene(QObject * parent ) : QGraphicsScene(parent)
{
	connect(this, SIGNAL(selectionChanged()), this, SLOT(selectedUpdate()));
}

Scene::~Scene(void)
{
}

	////////////////////////
	// Access methods
	////////////////////////

	////////////////////////
	// Modifiers
	////////////////////////
	
/* Adds the point qp to the obstacle currently drawn */
bool Scene::addPointToObstacle(QPointF qp)
{	
	return addPoint(qp,true);
}

/*	Sets the room	*/
void Scene::setRoom(QPointF topleft, QPointF bottomright)
{
	m_roomTL = topleft;
	m_roomBR = bottomright;
}


///////////////////////
// Helper methods
///////////////////////

/* Returns a rectangle with center point qp and height/width = radius*2 */
QRectF Scene::rectForPoint(QPointF qp, double radius){
	qreal  xCord = qp.rx() - radius;
	qreal  yCord = qp.ry() - radius;
	QRectF rectangle( xCord,  yCord,  radius*2, radius*2);
	return rectangle;
}

	////////////////////////
	// Other methods
	////////////////////////


void Scene::keyPressEvent (QKeyEvent* evt){
	
	bool isDeleteKey (evt->key() == Qt::Key_Delete);

}

QVariant interpolateQPolygonF(const QPolygonF &start, const QPolygonF &end, qreal progress)
{
    QPolygonF result(start);
    /*if (start.size() == end.size()) {
        for (int i = 0; i < start.size(); ++i) {
            // progress 0 .. 1
            result[i] = start.at(i) * (1.0 - progress) + end.at(i) * progress;
        }
    }*/
	
	QPointF start_ref = Scene::reference_point(start);
	QPointF end_ref = Scene::reference_point(end);
	
	QPointF translation = (end_ref - start_ref) * progress;
	result.translate(translation);

	QLineF line_start(start[1],start[0]);
	QLineF line_end(end[1],end[0]);

	double angle_start = line_start.angle();
	double angle_end = line_end.angle();

	double angle_diff = angle_start - angle_end;

	if (abs(angle_diff) > 180)
	{
		if (angle_diff > 0)
			angle_diff = - (360 - angle_diff);
		else 
			angle_diff = - (- 360 - angle_diff);
	}

	result = Scene::rotate_polygon(result, angle_diff * progress);

    QVariant res;
    res.setValue(result);
    return res;
}

void Scene::animate()
	{
		 clear_animation();

		 if (m_displayed_robots.size()==1)
		 {
			 removeItem(m_displayed_robots[0]);
		 }

		//Fill Vector
		qRegisterAnimationInterpolator<QPolygonF>(interpolateQPolygonF);
		vector<GraphicsPolygon*> graphics_polygon(2);
		for (int i = 0; i< 2; i++)
			graphics_polygon[i] = new GraphicsPolygon;

		bool path_exists = true;
		bool state_exists = true;
		
		// if path empty create "artificial path"
		if (m_path.empty())
		{
			vector<vector<QConf>>  path(2);
			for (int r = 0; r < 1; r++)
			{
				QPolygonF first_poly = m_displayed_robots[0]->polygon();
				QPointF first_ref = reference_point(first_poly);

				QPointF map_first_ref = m_displayed_robots[0]->mapToScene(first_ref);
				QConf start(map_first_ref, m_conf_rotation[0]);
				path[r].push_back(start);

				for (int i = 1; i < m_displayed_robots.size(); i++)
				{
					QPolygonF first_poly = m_displayed_robots[i-1]->polygon();
					QPointF first_ref = reference_point(first_poly);
					QPointF map_first_ref=m_displayed_robots[i-1]->mapToScene(first_ref);
				
					QPolygonF second_poly = m_displayed_robots[i]->polygon();
					QPointF second_ref = reference_point(second_poly);
					QPointF map_second_ref=m_displayed_robots[i]->mapToScene(second_ref);

					QConf target((map_second_ref - map_first_ref), m_conf_rotation[i] - m_conf_rotation[i-1]);
					path[r].push_back(target);
				}
			}

			for (int r = 1; r < 2; r++)
			{
				QPolygonF first_poly = m_displayed_robots.last()->polygon();
				QPointF first_ref = reference_point(first_poly);

				QPointF map_first_ref = m_displayed_robots.last()->mapToScene(first_ref);
				QConf start(map_first_ref, m_conf_rotation.last());
				path[r].push_back(start);

				for (int i = m_displayed_robots.size() - 2; i >= 0; i--)
				{
					QPolygonF first_poly = m_displayed_robots[i+1]->polygon();
					QPointF first_ref = reference_point(first_poly);
					QPointF map_first_ref=m_displayed_robots[i+1]->mapToScene(first_ref);
				
					QPolygonF second_poly = m_displayed_robots[i]->polygon();
					QPointF second_ref = reference_point(second_poly);
					QPointF map_second_ref=m_displayed_robots[i]->mapToScene(second_ref);

					QConf target((map_second_ref - map_first_ref), m_conf_rotation[i] - m_conf_rotation[i+1]);
					path[r].push_back(target);
				}
			}
			
			for (int  i = 0; i < min<int>(path[0].size(), path[1].size()); i++)
			{
				vector<QConf> conf;
				conf.push_back(path[0][i]);
				conf.push_back(path[1][i]);
				m_path.push_back(conf);
			}
			
			path_exists = false;
		}

		/*if (m_conf_state.empty())
		{
			m_conf_change.push_back(0);
			m_conf_change.push_back(1);
			m_conf_change.push_back(2);
			
			m_conf_state = vector<vector<int>>(3);

			m_conf_state[0].push_back(1);
			m_conf_state[0].push_back(0);
			m_conf_state[0].push_back(2);

			m_conf_state[1].push_back(1);
			m_conf_state[1].push_back(3);
			m_conf_state[1].push_back(2);

			m_conf_state[2].push_back(3);
			m_conf_state[2].push_back(3);
			m_conf_state[2].push_back(3);
		}*/


		m_current_state = 0;
		if (!m_conf_state.empty())
			change_conf_state();

		// reconstruct polygon
		QVector<QPointF> points;
		for (int i = 0; i < m_robot_polygon.size(); i++)
		{
			points.push_back(m_robot_polygon[i]);
		}

		//QPolygonF robot_polygon(points);

		// prepare (move robot to start configuration)
		vector<QPolygonF> polygon(2);
		for (int r = 0; r < 2; r++)
		{
			polygon[r] = m_robot_polygon;

			QMatrix tranlate_to_start;
			QPointF translate_start_point = m_path[0][r].first - reference_point(polygon[r]);
			tranlate_to_start.translate(translate_start_point.x(), translate_start_point.y());
			polygon[r] = tranlate_to_start.map(polygon[r]);
			polygon[r] = rotate_polygon(polygon[r], m_path[0][r].second);
			graphics_polygon[r]->setPolygon(polygon[r]);
			graphics_polygon[r]->setBrush(QBrush(ANIMATED_ROBOT_COLOR[r]));
			graphics_polygon[r]->setPen(QPen(ROBOT_COLOR[r]));
			graphics_polygon[r]->setZValue(1);
			
			//Add To Scene the Graphic Polygon
			addItem(graphics_polygon[r]);
		}

		QSequentialAnimationGroup* seq_animation = new QSequentialAnimationGroup;

		for (int i = 1; i < m_path.size(); i++)
		{
			QParallelAnimationGroup* par_animation = new QParallelAnimationGroup;
			vector<QPropertyAnimation*> prop_animation(2);

			vector<int> robot_min;

			for (int r = 0; r < 2; r++)
			{
				prop_animation[r] = new QPropertyAnimation(graphics_polygon[r],"Polygon");
				prop_animation[r]->setDirection(QAbstractAnimation::Forward);

				int xy_steps = m_path[i][r].first.manhattanLength() / 10 + 1;
				int theta_steps = abs(m_path[i][r].second) / 2;

				robot_min.push_back(max<int>(xy_steps, theta_steps));
			}

			int num_steps = max<int>(max<int>(robot_min[0], robot_min[1]), 20);

			for (int r = 0; r < 2; r++)
			{
				prop_animation[r]->setDuration(m_path_length[i-1] * 1000);
				QVariant start_variant;
				start_variant.setValue(polygon[r]);
				prop_animation[r]->setStartValue(start_variant);

				QPointF translation_point = m_path[i][r].first;
				double  rotation_angle = m_path[i][r].second;

				QMatrix matrix_tranlate;
				matrix_tranlate.translate(translation_point.x(), translation_point.y());
				polygon[r] = matrix_tranlate.map(polygon[r]);

				polygon[r] = rotate_polygon(polygon[r], rotation_angle);

				QVariant end_variant;
				end_variant.setValue(polygon[r]);
				prop_animation[r]->setEndValue(end_variant);
				par_animation->addAnimation(prop_animation[r]);
			}		
			
			if (!m_conf_state.empty() && m_conf_change[m_current_state] == i)
			{
				QObject::connect(par_animation,SIGNAL( finished() ),this,SLOT( change_conf_state() ));
			}

			seq_animation->addAnimation(par_animation);
		}
		
		QObject::connect(seq_animation,SIGNAL( finished() ),this,SLOT( animationComplete() ));

		m_displayed_animation_objects.push_back(graphics_polygon[0]);
		m_displayed_animation_objects.push_back(graphics_polygon[1]);

		seq_animation -> start();
		if (!path_exists)
			m_path.clear();
	}


void	Scene::loadFromFile(const char* pi_szFileName)
{

}

