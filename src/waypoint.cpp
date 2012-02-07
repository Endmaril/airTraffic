#include "waypoint.h"

osg::ref_ptr<osg::Node> Waypoint::geo = NULL;

Waypoint::Waypoint(osg::Vec3 position) : position(position) {
    if(!geo)
        geo = osgDB::readNodeFile("data/city/waypoint.3ds");

    nodeWaypointTransform = new osg::MatrixTransform();
    nodeWaypointTransform -> setMatrix(osg::Matrix::translate(position));
    nodeWaypointTransform -> addChild(geo);
}

Waypoint::~Waypoint() {
}

osg::ref_ptr<osg::Node> Waypoint::getNode() {
    return nodeWaypointTransform;
}

void Waypoint::linkTo(Waypoint* point) {
    links.push_back(point);
}

Waypoint* Waypoint::getRandomLink() {
    return links[rand() % links.size()];
}

osg::Vec3 Waypoint::getPosition() {
    return position;
}