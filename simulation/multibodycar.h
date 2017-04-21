#ifndef MULTIBODYCAR_H
#define MULTIBODYCAR_H

#include <QObject>
#include "multibody.h"

class MultibodyCar : public Multibody
{
    Q_OBJECT

public:

    explicit MultibodyCar(QObject *parent=0);
    ~MultibodyCar();

    virtual void    initPhysics();
    virtual void    exitPhysics();
    virtual void	updateGraphics();
    virtual void	stepSimulation(float deltaTime)=0;
    virtual void	renderScene();
    virtual void	physicsDebugDraw(int debugFlags)=0;//for now we reuse the flags in Bullet/src/LinearMath/btIDebugDraw.h
    //reset camera is only called when switching demo. this way you can restart (initPhysics) and watch in a specific location easier
    virtual void	resetCamera();
    virtual bool	mouseMoveCallback(float x,float y);
    virtual bool	mouseButtonCallback(int button, int state, float x, float y);
    virtual bool	keyboardCallback(int key, int state);

    virtual void	vrControllerMoveCallback(int controllerId, float pos[4], float orientation[4], float analogAxis);
    virtual void	vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orientation[4]);
    virtual void	vrHMDMoveCallback(int controllerId, float pos[4], float orientation[4]);
    virtual void	vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orientation[4]);

    virtual void	processCommandLineArgs(int argc, char* argv[]);

private:

    btMultiBody* m_multiBody;
    btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
    bool m_once;
    int m_frameCount;

};

#endif // MULTIBODYCAR_H
