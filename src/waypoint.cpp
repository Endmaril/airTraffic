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

osg::ref_ptr<osg::MatrixTransform> Waypoint::getNode() {
    return nodeWaypointTransform;
}

void Waypoint::linkTo(Waypoint* point) {
    links.push_back(point);
}

Waypoint* Waypoint::getRandomLink() {
    if(links.size() == NULL)
        return NULL;
    return links[rand() % links.size()];
}

osg::Vec3 Waypoint::getPosition() {
    return position;
}

int Waypoint::getNumLinks()
{
    return links.size();
}

void buildPathGraph(osg::ref_ptr<osg::Node> city, std::vector<Waypoint*>& waypoints) {
    osgUtil::IntersectVisitor intersectVisitor;

    //define distance between two points alog the edges
    double distanceBetweenPoints = 5.0;

    //Define the sphere for points along the edges
    osg::ref_ptr<osg::Sphere> sphereShape = new osg::Sphere(osg::Vec3d(0.0, 0.0, 0.0), .3);
    osg::ref_ptr<osg::ShapeDrawable> sphereDrawable = new osg::ShapeDrawable(sphereShape);
    
    osg::ref_ptr<osg::Geode> sphere = new osg::Geode;
    sphere->addDrawable(sphereDrawable);
    
    osg::ref_ptr<osg::Material> matSphere = new osg::Material();
    matSphere->setDiffuse(osg::Material::FRONT, osg::Vec4d(0.0, 0.0, 1.0, 0.0));
    sphere->getOrCreateStateSet()->setAttribute(matSphere.get());

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
                    
                    //Adding blue spheres along the way
                    if (i < j)
                    {
                        osg::ref_ptr<osg::MatrixTransform> wpi = waypoints[i]->getNode();
                        osg::Vec3d unitVector = waypoints[j]->getPosition() - waypoints[i]->getPosition();
                        double maxDistance = unitVector.normalize(), distanceDone = 0;
                        while ( distanceDone + distanceBetweenPoints < maxDistance )
                        {
                            distanceDone += distanceBetweenPoints;
                            osg::ref_ptr<osg::MatrixTransform> pointMT = new osg::MatrixTransform();
                            pointMT -> setMatrix(osg::Matrixd::translate(unitVector * distanceDone - osg::Vec3d(0.0,0.0,0.6)));
                            pointMT -> addChild(sphere);
                            wpi -> addChild(pointMT);
                        }
                    }
                }
            }
        }
        std::cout << "Waypoint#" << i << " got " << c << " neighbors." << std::endl;
    }
}
