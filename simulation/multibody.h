#ifndef MULTIBODY_H
#define MULTIBODY_H

#include <QObject>
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

enum MyFilterModes
{
    FILTER_GROUPAMASKB_AND_GROUPBMASKA2=0,
    FILTER_GROUPAMASKB_OR_GROUPBMASKA2
};

struct MyOverlapFilterCallback2 : public btOverlapFilterCallback
{
    int m_filterMode;

    MyOverlapFilterCallback2()
    :m_filterMode(FILTER_GROUPAMASKB_AND_GROUPBMASKA2)
    {
    }

    virtual ~MyOverlapFilterCallback2()
    {}
    // return true when pairs need collision
    virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
    {
        if (m_filterMode==FILTER_GROUPAMASKB_AND_GROUPBMASKA2)
        {
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            return collides;
        }

        if (m_filterMode==FILTER_GROUPAMASKB_OR_GROUPBMASKA2)
        {
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            return collides;
        }
        return false;
    }
};

class Multibody : public QObject
{
    Q_OBJECT
public:

    static const float gravity;

    explicit Multibody(QObject *parent = 0);
    virtual ~Multibody();

    virtual void initPhysics()=0;

    virtual void exitPhysics();
    virtual void updateGraphics(){}
    virtual void stepSimulation(float deltaTime);
    virtual void renderScene();
    virtual void physicsDebugDraw(int debugFlags);//for now we reuse the flags in Bullet/src/LinearMath/btIDebugDraw.h
    //reset camera is only called when switching demo. this way you can restart (initPhysics) and watch in a specific location easier
    virtual void resetCamera(){}
    virtual bool mouseMoveCallback(float x,float y)=0;
    virtual bool mouseButtonCallback(int button, int state, float x, float y);
    virtual bool keyboardCallback(int key, int state);

    virtual void vrControllerMoveCallback(int controllerId, float pos[4], float orientation[4], float analogAxis) {}
    virtual void vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orientation[4]){}
    virtual void vrHMDMoveCallback(int controllerId, float pos[4], float orientation[4]){}
    virtual void vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orientation[4]){}

    virtual void processCommandLineArgs(int argc, char* argv[]){}

    virtual void removePickingConstraint();
    virtual btVector3 getRayTo(int x, int y);
    virtual void createEmptyDynamicsWorld();
    virtual bool pickBody(const btVector3 &rayFromWorld, const btVector3 &rayToWorld);
    virtual bool movePickedBody(const btVector3 &rayFromWorld, const btVector3 &rayToWorld);
    virtual btBoxShape *createBoxShape(const btVector3 &halfExtents);
    virtual btRigidBody *createRigidBody(float mass, const btTransform &startTransform,
                                         btCollisionShape *shape, const btVector4 &color = btVector4(1, 0, 0, 1));
    virtual void syncPhysicsToGraphics();

protected:


protected:

    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
    MyOverlapFilterCallback2* m_filterCallback;
    btOverlappingPairCache* m_pairCache;
    btBroadphaseInterface*	m_broadphase;
    btCollisionDispatcher*	m_dispatcher;
    btMultiBodyConstraintSolver*	m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btMultiBodyDynamicsWorld* m_dynamicsWorld;

    //data for picking objects
    class btRigidBody* m_pickedBody;
    class btTypedConstraint* m_pickedConstraint;
    class btMultiBodyPoint2Point* m_pickingMultiBodyPoint2Point;

    btVector3 m_oldPickingPos;
    btVector3 m_hitPos;
    btScalar m_oldPickingDist;
    bool m_prevCanSleep;


};

#endif // MULTIBODY_H
