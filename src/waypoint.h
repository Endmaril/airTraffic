#ifndef __AIRTRAFFIC_WAYPOINT__
#define __AIRTRAFFIC_WAYPOINT__

#include <osg/ref_ptr>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/IntersectVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osg/ShapeDrawable>
#include <osg/Material>

class Waypoint
{
private:
    static osg::ref_ptr<osg::Node> geo;
    osg::Vec3 position;
    std::vector<Waypoint*> links;
    osg::ref_ptr<osg::MatrixTransform> nodeWaypointTransform;
public:
    Waypoint(osg::Vec3 position);
    ~Waypoint();

    osg::ref_ptr<osg::MatrixTransform> getNode();
    void linkTo(Waypoint* point);
    int getNumLinks();
    Waypoint* getRandomLink();
    osg::Vec3 getPosition();
};


void buildPathGraph(osg::ref_ptr<osg::Node> city, std::vector<Waypoint*>& waypoints);

#endif
