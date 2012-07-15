/*******************************************************************
*	File name: 		QGraphicsPolygon.h
*	Description:	Declaration of the QGraphicsPolygon class.
*					This class is used to animate a polygon of the scene.
*					It does so by declaring a private property of the inherited QGraphicsPolygonItem.
*	Author:			Rotem Shaul
*
*	Date:			08/10
*******************************************************************/

#include <QObject>
#include <QGraphicsPolygonItem>
#include <QPolygonF>
#include <QDragMoveEvent>
#include <QDropEvent>

class GraphicsPolygon : public QObject, public QGraphicsPolygonItem
{
    Q_OBJECT
    Q_PROPERTY(QPolygonF Polygon WRITE setPolygonProperty READ polygonProperty)

public:
    explicit GraphicsPolygon(QObject *parent = 0);
	
    void setPolygonProperty(const QPolygonF &polygon) {setPolygon(polygon);}
    QPolygonF polygonProperty(){return polygon();}
signals:

protected slots:
	virtual void dragMoveEvent ( QDragMoveEvent * event ){
		event->accept();
	}
	virtual void dropEvent ( QDropEvent*  event ){
		
	}

};



















/*#ifndef GRAPHICSPOLYGON_H
#define GRAPHICSPOLYGON_H

#include <QObject>
#include <QGraphicsPolygonItem>
#include <QPolygonF>

class GraphicsPolygon : public QObject, public QGraphicsPolygonItem
{
    Q_OBJECT
    Q_PROPERTY(QPolygonF Polygon WRITE setPolygonProperty READ polygonProperty)
public:
    explicit GraphicsPolygon(QObject *parent = 0);
    void setPolygonProperty(const QPolygonF &polygon) {setPolygon(polygon);}
    QPolygonF polygonProperty(){return polygon();}
signals:

public slots:

};

#endif // GRAPHICSPOLYGON_H*/