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

#include "waypoint.h"

using namespace std;

// list of each waypoint
std::vector<Waypoint*> waypoints;

// group of each different car
osg::ref_ptr<osg::Node> nodeCar = osgDB::readNodeFile("data/Vehicles-LowRes/Car.ive");

// return direction from a to b, according to c as the direction of the up vector
osg::Quat eulerQuat(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c = osg::Z_AXIS)
{
    osg::Quat quat; 
    osg::Matrix matrix; 
    matrix.makeLookAt(a, b, c);
    matrix.get(quat);
    return quat.inverse();
}

// fill points with a list of random waypoint positions, making a looped path
bool createRandomPath(std::vector<osg::Vec3>& points, Waypoint* waypoint = NULL, Waypoint* first = NULL)
{
    // initialization
    if(waypoint == NULL)
    {
        // find a first point that got at least one neighbour
        do
        {
            waypoint = waypoints[rand() % waypoints.size()];
        } while(waypoint->getNumLinks() == 0);
    }

    // keep first point, in order to detect loops
    if(!first)
    {
        first = waypoint;
    }

    // add point
    points.push_back(waypoint->getPosition());

    // get a neighbour
    waypoint = waypoint->getRandomLink();
    if(!waypoint)
        return false;

    // make a looping path, 5 points long minimum
    if(first != waypoint || points.size() < 5)
    {
        // add another point
        createRandomPath(points, waypoint, first);
    }
    
    return true;
}

// create a random looped path
osg::ref_ptr<osg::MatrixTransform> createRandomPath() {
    osg::ref_ptr<osg::MatrixTransform> pathTransform = new osg::MatrixTransform();
    osg::ref_ptr<osg::AnimationPath> pathAnimation = new osg::AnimationPath();
    osg::ref_ptr<osg::AnimationPathCallback> pathAnimationCallback = new osg::AnimationPathCallback(pathAnimation);
    std::vector<osg::Vec3> points;

    pathTransform->setUpdateCallback(pathAnimationCallback);

    // determine the path : loop through random points
    createRandomPath(points);
    std::cout << "Path is " << points.size() << " points long." << std::endl;

    // make an animationpath correspond to the points we selected
    // first point
    osg::Quat direction(eulerQuat(points[0], points[1]));
    pathAnimation->insert(0, osg::AnimationPath::ControlPoint(points[0], direction));
    float time = 0.0f;
    for(int i = 1; i < points.size(); i++)
    {
        time += 0.1 * (points[i] - points[i - 1]).length();

        // get to next point
        pathAnimation->insert(time - 0.5, osg::AnimationPath::ControlPoint(points[i], direction));

        // get in the right direction towards following point
        direction = eulerQuat(points[i], points[(i + 1) % points.size()]);
        pathAnimation->insert(time + 0.5, osg::AnimationPath::ControlPoint(points[i], direction));
    }
    // manage looping path smoothly
    direction = eulerQuat(points[points.size() - 1], points[0]);
    time += 0.1 * (points[points.size() - 1] - points[0]).length();
    pathAnimation->insert(time - 0.5, osg::AnimationPath::ControlPoint(points[0], direction));
    direction = eulerQuat(points[0], points[1]);
    pathAnimation->insert(time + 0.5, osg::AnimationPath::ControlPoint(points[0], direction));

    return pathTransform;
}

// gives a random car into the whole set
osg::ref_ptr<osg::Node> createCar() {
    osg::Group* groupCar = nodeCar->asGroup();

    osg::ref_ptr<osg::MatrixTransform> nodeCarTransform = new osg::MatrixTransform(); 
    nodeCarTransform -> setMatrix(osg::Matrix::rotate(M_PI / 2.0, 0, 1, 0) * osg::Matrix::rotate(M_PI / 2.0, 0, 0, 1)/* * osg::Matrix::translate(2, 0, 0)*/);
    nodeCarTransform -> addChild(groupCar->getChild(rand() % groupCar->getNumChildren()));

    return nodeCarTransform;
}

// gives the whole city
osg::ref_ptr<osg::Node> createTerrain() {
    osg::ref_ptr<osg::Node> terrainTile = osgDB::readNodeFile("data/city/buildings.3ds");
    // generate every possible paths
    buildPathGraph(terrainTile, waypoints);

    // gives dirt brownish dirt color to the floor
    osg::ref_ptr<osg::Material> cityMaterial = new osg::Material();
    cityMaterial->setDiffuse(osg::Material::FRONT, osg::Vec4d(1.0, 135.0/255.0, 0.0, 0.0));
    terrainTile->getOrCreateStateSet()->setAttribute(cityMaterial.get());

    return terrainTile;
}

// create the whole scene
osg::ref_ptr<osg::Group> createSceneGraph() {

    osg::ref_ptr<osg::Group> root = new osg::Group();

    osg::ref_ptr<osg::MatrixTransform> skyDomeTransform = new osg::MatrixTransform(); 

    skyDomeTransform -> setMatrix(
            osg::Matrix::scale(0.04, 0.04, 0.1)
            );

    // add every waypoint
    for(unsigned i = 0; i < waypoints.size(); i++)
        root->addChild(waypoints[i]->getNode());

    // add the skydome
    root -> addChild(skyDomeTransform.get());
    osg::ref_ptr<osg::Node> skyDomeModel = osgDB::readNodeFile("data/skydome.osg");
    skyDomeTransform -> addChild(skyDomeModel);
    skyDomeTransform -> setMatrix( skyDomeTransform -> getMatrix() * 
    osg::Matrixd::translate(-skyDomeTransform -> getBound().center()));

    // 
    root -> addChild(createTerrain());
    

    // let light be
    osg::ref_ptr<osg::Light> light = new osg::Light();
    light -> setLightNum(0);
    light -> setAmbient( osg::Vec4d(.4f, .4f, .4f, 1.0) );
    light -> setDiffuse( osg::Vec4(.8f, .8f, .8f, 1.0f) );
    light -> setSpecular( osg::Vec4(.8f, .8f, .8f, 1.0f) );

    light -> setPosition( osg::Vec4(300.0f, 300.0f, 300.0f, 0.0f) );

    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
    lightSource -> setLight(light);
    root -> addChild(lightSource);

    // add cars
    for(int i = 0; i < 30; i++)
    {
        osg::ref_ptr<osg::MatrixTransform> car = createRandomPath();
        car->addChild(createCar());
        root->addChild(car);
    }

    return root;
}

int main() {
    srand(time(NULL));

    // add waypoints
    for(float z = 5.0f; z <= 75.0f; z += 35.0f)
    {
        waypoints.push_back(new Waypoint(osg::Vec3(11, 11, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(-11, -11, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(-19, 20, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(-6, -34, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(-36, -10, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(47, 54, z)));
        waypoints.push_back(new Waypoint(osg::Vec3(48, -37, z)));
    }

    // keyswitchmanipulator between:
    // 1: trackball
    // 2: flight
    // 3: drive
    // 4: terrain
    // 5: animationpath placing you in a fictive car flying randomly
    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keySwitchManipulator = new osgGA::KeySwitchMatrixManipulator();

    osg::ref_ptr<osgGA::TrackballManipulator> trackBallManip = new osgGA::TrackballManipulator();
    keySwitchManipulator -> addMatrixManipulator('1', "trackball", trackBallManip);
    
    osg::ref_ptr<osgGA::FlightManipulator> flightManip = new osgGA::FlightManipulator();
    keySwitchManipulator -> addMatrixManipulator('2', "flight", flightManip);
    
    osg::ref_ptr<osgGA::DriveManipulator> driveManip = new osgGA::DriveManipulator();
    keySwitchManipulator -> addMatrixManipulator('3', "drive", driveManip);
    
    osg::ref_ptr<osgGA::TerrainManipulator> terrainManip = new osgGA::TerrainManipulator();
    keySwitchManipulator -> addMatrixManipulator('4', "terrain", terrainManip);

    osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();

    viewer -> setCameraManipulator(keySwitchManipulator);

    osg::ref_ptr<osgViewer::StatsHandler> stats = new osgViewer::StatsHandler();
    viewer -> addEventHandler(stats);

    osg::ref_ptr<osgViewer::RecordCameraPathHandler> pathRecorder = new osgViewer::RecordCameraPathHandler();
    viewer -> addEventHandler(pathRecorder);

    osg::ref_ptr<osg::Group> scene = createSceneGraph();
    viewer -> setSceneData(scene);
    
    keySwitchManipulator -> setHomePosition(osg::Vec3d(230 * cos(M_PI/6), 0, 230 * sin(M_PI/6)), 
                              osg::Vec3d(0.0, 0.0, 0.0),
                              osg::Vec3d(-sin(M_PI/6), 0.0, cos(M_PI/6)));
    
    osg::ref_ptr<osg::AnimationPath> camPath = ((osg::AnimationPathCallback*)(createRandomPath() -> getUpdateCallback())) -> getAnimationPath();
    osg::ref_ptr<osgGA::AnimationPathManipulator> animManip = new osgGA::AnimationPathManipulator(camPath.get());
    keySwitchManipulator -> addMatrixManipulator('5', "anim", animManip);
    
    viewer->run();
    return 0;
}
