// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <iostream>

//boost
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

//Program Related
#include "..\Project\Includes.h"
#include "..\Project\CompilationFlags.h"
#include "..\Project\CgalTypedefs.h"
#include "..\Project\Globals.h"
#include "..\Project\Configuration.h"

//Utils - GeometryUtils
#include "..\Utils\Geometry_utils\BoundingUtils.h"
#include "..\Utils\Geometry_utils\GeometryIO.h"
#include "..\Utils\Geometry_utils\GetClosestUtils.h"
#include "..\Utils\Geometry_utils\Less_than_point.h"

//Utils - IntervalUtils
#include "..\Utils\Interval_utils\Interval.h"
#include "..\Utils\Interval_utils\IntervalSet.h"

//Utils - Number_utils
#include "..\Utils\Number_utils\Sqrt_approximation.h"
#include "..\Utils\Number_utils\AK_conversions_1.h"

//Utils - Polygon_utils
#include "..\Utils\Polygon_utils\do_intersect_predicates.h"
#include "..\Utils\Polygon_utils\ExtendedPolygon.h"
#include "..\Utils\Polygon_utils\Is_in_polygon_predicates.h"
#include "..\Utils\Polygon_utils\MotionPlanningUtils.h"
#include "..\Utils\Polygon_utils\Polygon_rotations.h"
#include "..\Utils\Polygon_utils\Polygon_translations.h"
#include "..\Utils\Polygon_utils\PolygonIO.h"
#include "..\Utils\Polygon_utils\PolygonUtils.h"
#include "..\Utils\Polygon_utils\Smart_polygon_with_holes.h"

//Utils - Random_utils
#include "..\Utils\Random_utils\RandomUtils.h"

//Utils - Rotation_utils
#include "..\Utils\Rotation_utils\Rotation.h"
#include "..\Utils\Rotation_utils\RotationRange.h"
#include "..\Utils\Rotation_utils\RotationUtils.h"

//Utils - UI_utils
#include "..\Utils\UI_utils\Environment.h"
#include "..\Utils\UI_utils\Graphics.h"
#include "..\Utils\UI_utils\InputReader.h"
#include "..\Utils\UI_utils\TimeManager.h"

//Custom logger
#include "..\Utils\logging.h"
