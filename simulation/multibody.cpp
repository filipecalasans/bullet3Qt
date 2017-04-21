#include "multibody.h"

const float Multibody::gravity = 9.81;

Multibody::Multibody(QObject *parent) :
    QObject(parent),
    m_filterCallback(0),
    m_broadphase(0),
    m_dispatcher(0),
    m_solver(0),
    m_collisionConfiguration(0),
    m_dynamicsWorld(0),
    m_pickedBody(0),
    m_pickedConstraint(0),
    m_pickingMultiBodyPoint2Point(0),
    m_prevCanSleep(false)
{

}

Multibody::~Multibody()
{

}

void Multibody::createEmptyDynamicsWorld()
{
    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();
    m_filterCallback = new MyOverlapFilterCallback2();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

    m_pairCache = new btHashedOverlappingPairCache();

    m_pairCache->setOverlapFilterCallback(m_filterCallback);

    m_broadphase = new btDbvtBroadphase(m_pairCache);//btSimpleBroadphase();

    m_solver = new btMultiBodyConstraintSolver;

    m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

    m_dynamicsWorld->setGravity(btVector3(0, -Multibody::gravity, 0));
}

void Multibody::stepSimulation(float deltaTime)
{
    if (m_dynamicsWorld)
    {
        m_dynamicsWorld->stepSimulation(deltaTime);
    }
}

void Multibody::exitPhysics()
{
    removePickingConstraint();
    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them

    if (m_dynamicsWorld)
    {

        int i;
        for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
        {
            m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
        }

        for (i = m_dynamicsWorld->getNumMultiBodyConstraints() - 1; i >= 0; i--)
        {
            btMultiBodyConstraint* mbc = m_dynamicsWorld->getMultiBodyConstraint(i);
            m_dynamicsWorld->removeMultiBodyConstraint(mbc);
            delete mbc;
        }

        for (i = m_dynamicsWorld->getNumMultibodies() - 1; i >= 0; i--)
        {
            btMultiBody* mb = m_dynamicsWorld->getMultiBody(i);
            m_dynamicsWorld->removeMultiBody(mb);
            delete mb;
        }
        for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
        {
            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            m_dynamicsWorld->removeCollisionObject(obj);
            delete obj;
        }
    }
    //delete collision shapes
    for (int j = 0; j<m_collisionShapes.size(); j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();

    delete m_dynamicsWorld;
    m_dynamicsWorld = 0;

    delete m_solver;
    m_solver=0;

    delete m_broadphase;
    m_broadphase=0;

    delete m_dispatcher;
    m_dispatcher=0;

    delete m_pairCache;
    m_pairCache = 0;

    delete m_filterCallback;
    m_filterCallback = 0;

    delete m_collisionConfiguration;
    m_collisionConfiguration=0;
}

void Multibody::removePickingConstraint()
{
    if (m_pickedConstraint)
    {
        m_dynamicsWorld->removeConstraint(m_pickedConstraint);
        delete m_pickedConstraint;
        m_pickedConstraint = 0;
        m_pickedBody = 0;
    }
    if (m_pickingMultiBodyPoint2Point)
    {
        m_pickingMultiBodyPoint2Point->getMultiBodyA()->setCanSleep(m_prevCanSleep);
        btMultiBodyDynamicsWorld* world = (btMultiBodyDynamicsWorld*) m_dynamicsWorld;
        world->removeMultiBodyConstraint(m_pickingMultiBodyPoint2Point);
        delete m_pickingMultiBodyPoint2Point;
        m_pickingMultiBodyPoint2Point = 0;
    }
}

void Multibody::syncPhysicsToGraphics()
{
    if (m_dynamicsWorld)
    {
        m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
    }
}

void Multibody::renderScene()
{
    if (m_dynamicsWorld)
    {
        m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

        m_guiHelper->render(m_dynamicsWorld);
    }

}

void Multibody::physicsDebugDraw(int debugDrawFlags)
{
    if (m_dynamicsWorld)
    {
        if (m_dynamicsWorld->getDebugDrawer())
        {
            m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
        }
        m_dynamicsWorld->debugDrawWorld();
    }

}

bool Multibody::keyboardCallback(int key, int state)
{
    return false;//don't handle this key
}

