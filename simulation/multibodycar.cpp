#include "multibodycar.h"

MultibodyCar::MultibodyCar(QObject *parent) : Multibody(parent)
{

}

MultibodyCar::~MultibodyCar()
{

}

void MultibodyCar::initPhysics()
{

}

void MultibodyCar::exitPhysics()
{

}

void MultibodyCar::updateGraphics(){


}

void MultibodyCar::renderScene()
{

}

void MultibodyCar::resetCamera()
{

}

bool MultibodyCar::mouseMoveCallback(float x, float y)
{
    return true;
}

bool MultibodyCar::mouseButtonCallback(int button, int state, float x, float y)
{
return true;
}

bool MultibodyCar::keyboardCallback(int key, int state)
{
return true;
}

void MultibodyCar::vrControllerMoveCallback(int controllerId, float pos[], float orientation[], float analogAxis) {}

void MultibodyCar::vrControllerButtonCallback(int controllerId, int button, int state, float pos[], float orientation[]){}

void MultibodyCar::vrHMDMoveCallback(int controllerId, float pos[], float orientation[]){}

void MultibodyCar::vrGenericTrackerMoveCallback(int controllerId, float pos[], float orientation[]){}

void MultibodyCar::processCommandLineArgs(int argc, char *argv[]){}

void MultibodyCar::physicsDebugDraw(int debugFlags)
{

}

void MultibodyCar::stepSimulation(float deltaTime)
{

}
