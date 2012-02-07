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

using namespace std;

osg::Quat eulerQuat(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c = osg::Z_AXIS)
{
    osg::Quat quat; 
    osg::Matrix matrix; 
    matrix.makeLookAt(a, b, c);
    matrix.get(quat);
    return quat.inverse();
}

osg::ref_ptr<osg::Node> createCar() {
    float speed = 10.0f;

    osg::ref_ptr<osg::Node> nodeCar = osgDB::readNodeFile("data/Vehicles-LowRes/Car.ive");
    osg::ref_ptr<osg::MatrixTransform> nodeCarTransform = new osg::MatrixTransform(); 
    osg::ref_ptr<osg::MatrixTransform> nodeCarTransform2 = new osg::MatrixTransform();
    nodeCarTransform2 -> setMatrix(osg::Matrix::rotate(3.14159 / 2.0, 0, 1, 0) * osg::Matrix::rotate(3.14159 / 2.0, 0, 0, 1));

    osg::Group* groupCar = nodeCar->asGroup();
    int i = rand() % groupCar->getNumChildren();
    nodeCar = groupCar->getChild(i);

    osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath();
    std::vector<osg::Vec3> points;
    points.push_back(osg::Vec3(-46, 53, 1.5));
    points.push_back(osg::Vec3(47, 52, 1.5));
    points.push_back(osg::Vec3(47, -12, 17));
    points.push_back(osg::Vec3(-47, -12, 170));

    osg::Quat direction(eulerQuat(points[0], points[1]));
    animationPath->insert(0, osg::AnimationPath::ControlPoint(points[0], direction));
    for(int i = 1; i < points.size() - 1; i++)
    {
        animationPath->insert(speed * (float)i - 1, osg::AnimationPath::ControlPoint(points[i], direction));

        direction = eulerQuat(points[i], points[i + 1]);
        animationPath->insert(speed * (float)i + 1, osg::AnimationPath::ControlPoint(points[i], direction));
    }
    animationPath->insert(speed * (float)(points.size() - 1), osg::AnimationPath::ControlPoint(points[points.size() - 1], direction));

    osg::ref_ptr<osg::AnimationPathCallback> animationPathCallback = new osg::AnimationPathCallback(animationPath);
    nodeCarTransform->setUpdateCallback(animationPathCallback);
    nodeCarTransform->addChild(nodeCarTransform2);
    nodeCarTransform2->addChild(nodeCar);

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

    root -> addChild(skyDomeTransform.get());
     
    root -> addChild(createTerrain());
//~ 
    //~ osg::ref_ptr<osg::Node> skyDomeModel = osgDB::readNodeFile("skydome.osg");
//~ 
    //~ skyDomeTransform -> addChild(skyDomeModel);
    //~ 
    //~ skyDomeTransform -> setMatrix( skyDomeTransform -> getMatrix() * 
            //~ osg::Matrixd::translate(-skyDomeTransform -> getBound().center()));
    
     osg::ref_ptr<osg::Light> light = new osg::Light();
     light -> setLightNum(0);
     light -> setAmbient( osg::Vec4d(.1, .1, .1, 1.0) );
     light -> setDiffuse( osg::Vec4(.8f, .8f, .8f, 1.0f) );
     light -> setSpecular( osg::Vec4(.8f, .8f, .8f, 1.0f) );
     
     light -> setPosition( osg::Vec4(0.0f, 0.0f, 300.0f, 0.0f) );
     
     osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
     lightSource -> setLight(light);
     root -> addChild(lightSource);
    
    root->addChild(createCar());


    return root;
}

int main() {

    srand(time(NULL));
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

    osg::ref_ptr<osgGA::AnimationPathManipulator> animManip = new osgGA::AnimationPathManipulator("saved_animation.path");
    keySwitchManipulator -> addMatrixManipulator('5', "anim", animManip);

    viewer -> setSceneData(createSceneGraph());
    
    viewer->run();
    return 0;
}

