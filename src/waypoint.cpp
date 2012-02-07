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

void buildPathGraph(osg::ref_ptr<osg::Node> city, std::vector<Waypoint*>& waypoints) {
    osgUtil::IntersectVisitor intersectVisitor;

    for(unsigned i = 0; i < waypoints.size(); i++) {
        int c = 0;
        for(unsigned j = 0; j < waypoints.size(); j++) {
            if(i != j) {

                //TODO: test left and right lanes
                osg::ref_ptr<osgUtil::LineSegmentIntersector> lineSegmentIntersector = new osgUtil::LineSegmentIntersector(waypoints[i]->getPosition(), waypoints[j]->getPosition());
                osgUtil::IntersectionVisitor intersectionVisitor(lineSegmentIntersector);
                intersectionVisitor.setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
                city->accept(intersectionVisitor);

                // nothing between both waypoints
                if(!lineSegmentIntersector->containsIntersections()) {
                    waypoints[i]->linkTo(waypoints[j]);
                    c++;
                }
            }
        }
        std::cout << "Waypoint#" << i << " got " << c << " neighbours." << std::endl;
    }
}