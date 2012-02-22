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

std::vector<Waypoint*> waypoints;

osg::Quat eulerQuat(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c = osg::Z_AXIS)
{
    osg::Quat quat; 
    osg::Matrix matrix; 
    matrix.makeLookAt(a, b, c);
    matrix.get(quat);
    return quat.inverse();
}

bool createRandomPath(std::vector<osg::Vec3>& points, Waypoint* waypoint = NULL, Waypoint* first = NULL)
{
    if(waypoint == NULL)
    {
        // find a first point that got at least one neighbour
        do
        {
            waypoint = waypoints[rand() % waypoints.size()];
        } while(waypoint->getNumLinks() == 0);
    }

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

    // make a looping path, 20 points long minimum
    if(first != waypoint/* || points.size() < 20*/)
    {

        createRandomPath(points, waypoint, first);
    }
    
    //points.push_back(first->getPosition());
    
    return true;
}

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
    osg::Quat direction(eulerQuat(points[0], points[1]));
    pathAnimation->insert(0, osg::AnimationPath::ControlPoint(points[0], direction));
    float time = 0.0f;
    for(int i = 1; i < points.size(); i++)
    {
        time += 0.1 * (points[i] - points[i - 1]).length();

        pathAnimation->insert(time - 0.5, osg::AnimationPath::ControlPoint(points[i], direction));

        direction = eulerQuat(points[i], points[(i + 1) % points.size()]);
        pathAnimation->insert(time + 0.5, osg::AnimationPath::ControlPoint(points[i], direction));
    }
    direction = eulerQuat(points[points.size() - 1], points[0]);
    time += 0.1 * (points[points.size() - 1] - points[0]).length();
    pathAnimation->insert(time - 0.5, osg::AnimationPath::ControlPoint(points[0], direction));
    direction = eulerQuat(points[0], points[1]);
    pathAnimation->insert(time + 0.5, osg::AnimationPath::ControlPoint(points[0], direction));

    return pathTransform;
}

osg::ref_ptr<osg::Node> createCar() {
    osg::ref_ptr<osg::Node> nodeCar = osgDB::readNodeFile("data/Vehicles-LowRes/Car.ive");
    osg::Group* groupCar = nodeCar->asGroup();

    osg::ref_ptr<osg::MatrixTransform> nodeCarTransform = new osg::MatrixTransform(); 
    nodeCarTransform -> setMatrix(osg::Matrix::rotate(M_PI / 2.0, 0, 1, 0) * osg::Matrix::rotate(M_PI / 2.0, 0, 0, 1)/* * osg::Matrix::translate(2, 0, 0)*/);
    nodeCarTransform -> addChild(groupCar->getChild(rand() % groupCar->getNumChildren()));

    return nodeCarTransform;
}

osg::ref_ptr<osg::Node> createTerrain() {

    //osg::ref_ptr<osgTerrain::TerrainTile> terrainTile = new osgTerrain::TerrainTile();
    
    //osg::ref_ptr<osgTerrain::ImageLayer> elevationLayer = new osgTerrain::ImageLayer(),
                           //colorLayer = new osgTerrain::ImageLayer();
    
    //elevationLayer -> setImage(osgDB::readImageFile("terrain.bmp"));
    
    //osg::Texture::FilterMode filter = osg::Texture::LINEAR;
    //elevationLayer -> setMagFilter(filter);
    //elevationLayer -> setMinFilter(filter);
    
    //osg::ref_ptr<osgTerrain::Locator> locator = new osgTerrain::Locator();
    //locator->setCoordinateSystemType(osgTerrain::Locator::PROJECTED);
    //locator->setTransformAsExtents(-1024, -1024, 1024, 1024);

    //elevationLayer -> setLocator(locator);
    
    //elevationLayer -> transform(1.0f, 1.0f);
    
    //terrainTile -> setElevationLayer(elevationLayer);
    
    //colorLayer->setImage(osgDB::readImageFile("rocks.bmp"));

    //colorLayer -> setMagFilter(filter);
    //colorLayer -> setMinFilter(filter);

    //terrainTile -> setColorLayer(0, colorLayer);
    
    osg::ref_ptr<osg::Node> terrainTile = osgDB::readNodeFile("data/city/buildings.3ds");
    buildPathGraph(terrainTile, waypoints);

    osg::ref_ptr<osg::MatrixTransform> terrainScale = new osg::MatrixTransform();
    terrainScale -> setMatrix(
                        osg::Matrix::scale(1.0, 1.0, 1.0)
                    );
    terrainTile->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    terrainScale -> addChild(terrainTile);
    
    osg::ref_ptr<osg::Material> cityMaterial = new osg::Material();
    cityMaterial->setDiffuse(osg::Material::FRONT, osg::Vec4d(1.0, 135.0/255.0, 0.0, 0.0));
    terrainTile->getOrCreateStateSet()->setAttribute(cityMaterial.get());
    
    return terrainScale;
}

osg::ref_ptr<osg::Group> createSceneGraph() {

    osg::ref_ptr<osg::Group> root = new osg::Group();

    osg::ref_ptr<osg::MatrixTransform> skyDomeTransform = new osg::MatrixTransform(); 

    skyDomeTransform -> setMatrix(
            osg::Matrix::scale(0.2, 0.2, 0.4)
            );

    for(unsigned i = 0; i < waypoints.size(); i++)
        root->addChild(waypoints[i]->getNode());

    root -> addChild(skyDomeTransform.get());

    root -> addChild(createTerrain());
    
    osg::ref_ptr<osg::Node> skyDomeModel = osgDB::readNodeFile("skydome.osg");
    
    skyDomeTransform -> addChild(skyDomeModel);
    
    skyDomeTransform -> setMatrix( skyDomeTransform -> getMatrix() * 
    osg::Matrixd::translate(-skyDomeTransform -> getBound().center()));

    osg::ref_ptr<osg::Light> light = new osg::Light();
    light -> setLightNum(0);
    light -> setAmbient( osg::Vec4d(.4f, .4f, .4f, 1.0) );
    light -> setDiffuse( osg::Vec4(.8f, .8f, .8f, 1.0f) );
    light -> setSpecular( osg::Vec4(.8f, .8f, .8f, 1.0f) );

    light -> setPosition( osg::Vec4(300.0f, 300.0f, 300.0f, 0.0f) );

    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
    lightSource -> setLight(light);
    root -> addChild(lightSource);

    for(int i = 0; i < 10; i++)
    {
        osg::ref_ptr<osg::MatrixTransform> car = createRandomPath();
        car->addChild(createCar());
        root->addChild(car);
     }


    return root;
}

void drawRoads() {
    
}

int main() {
    srand(time(NULL));

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

    viewer -> setSceneData(createSceneGraph());
    
    osg::ref_ptr<osg::AnimationPath> camPath = ((osg::AnimationPathCallback*)(createRandomPath() -> getUpdateCallback())) -> getAnimationPath();
    osg::ref_ptr<osgGA::AnimationPathManipulator> animManip = new osgGA::AnimationPathManipulator(camPath.get());
    keySwitchManipulator -> addMatrixManipulator('5', "anim", animManip);
    
    viewer->run();
    return 0;
}
