#include <qcolor.h> 
#include <QBrush>
#include <QLinearGradient>
#include <vector>

const int NUM_COLORS = 10;

const Qt::GlobalColor GROUP_COLORS[] = {Qt ::blue,
	Qt ::red, Qt ::magenta, Qt ::green, 
	Qt ::yellow, Qt ::cyan, Qt ::darkBlue, 
	Qt ::darkGreen, Qt ::darkRed,  Qt ::darkYellow};

const Qt::BrushStyle	ROBOT_STYLE		= Qt::SolidPattern;

const Qt::BrushStyle	TARGET_STYLE	= Qt::DiagCrossPattern;

const Qt::GlobalColor	OBSTACLE_COLOR	= Qt::lightGray;

const Qt::GlobalColor 	ROBOT_COLOR[] = {Qt::blue, Qt::red};

const Qt::GlobalColor	CONFIGURATION_COLOR = Qt::white;

const Qt::GlobalColor	ANIMATED_ROBOT_COLOR[] = {Qt::cyan, Qt::green};

const Qt::GlobalColor	CONFIGURATION_COLOR_TOP[] = {Qt::white, Qt::blue, Qt::white, Qt::blue};

const Qt::GlobalColor	CONFIGURATION_COLOR_BOTTOM[] = {Qt::white, Qt::white, Qt::red, Qt::red};

const Qt::BrushStyle	OBSTACLE_STYLE	= Qt::SolidPattern;
