#include "multibodycar.h"

const btScalar MultibodyCar::radius(0.2);
const btScalar MultibodyCar::kp = 100;
const btScalar MultibodyCar::kd = 20;
const btScalar MultibodyCar::maxForce = 100;

MultibodyCar::MultibodyCar(QObject *parent) : Multibody(parent)
{

}

MultibodyCar::~MultibodyCar()
{

}

void MultibodyCar::initPhysics()
{

    /* TODO: ADD MULTIBODY DECLARATION HERE */

//    {
//        SliderParams slider("Kp",&kp);
//        slider.m_minVal=-200;
//        slider.m_maxVal=200;
//        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//    }
//    {
//        SliderParams slider("Kd",&kd);
//        slider.m_minVal=-50;
//        slider.m_maxVal=50;
//        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//    }
//    {
//        SliderParams slider("max force",&maxForce);
//        slider.m_minVal=0;
//        slider.m_maxVal=100;
//        m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
//    }




//    int upAxis = 1;
//    gJointFeedbackInWorldSpace = true;
//    gJointFeedbackInJointFrame = true;

//    m_guiHelper->setUpAxis(upAxis);


//    this->createEmptyDynamicsWorld();
//    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
//    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
//                //btIDebugDraw::DBG_DrawConstraints
//                +btIDebugDraw::DBG_DrawWireframe
//                +btIDebugDraw::DBG_DrawContactPoints
//                +btIDebugDraw::DBG_DrawAabb
//                );//+btIDebugDraw::DBG_DrawConstraintLimits);

//    m_dynamicsWorld->setGravity(btVector3(0,-10,0));
//    btTransform baseWorldTrans;
//    baseWorldTrans.setIdentity();
//    baseWorldTrans.setOrigin(btVector3(1,2,3));
//    m_multiBody = createInvertedPendulumMultiBody(m_dynamicsWorld, m_guiHelper, baseWorldTrans, true);

//    //for (int i=pMultiBody->getNumLinks()-1;i>=0;i--)//
//    for (int i=0;i<m_multiBody->getNumLinks();i++)
//    {
//        btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
//        m_multiBody->getLink(i).m_jointFeedback = fb;
//        m_jointFeedbacks.push_back(fb);
//        //break;
//    }

}

void MultibodyCar::stepSimulation(float deltaTime)
{

    static btScalar offset = -0.1*SIMD_PI;

    m_frameCount++;
    if ((m_frameCount&0xff)==0 )
    {
        offset = -offset;
    }
    btScalar target= SIMD_PI+offset;
    qDesiredArray.resize(0);
    qDesiredArray.resize(m_multiBody->getNumLinks(),target);

    for (int joint = 0; joint<m_multiBody->getNumLinks();joint++)
    {
        int dof1 = 0;
        btScalar qActual = m_multiBody->getJointPosMultiDof(joint)[dof1];
        btScalar qdActual = m_multiBody->getJointVelMultiDof(joint)[dof1];
        btScalar positionError = (qDesiredArray[joint]-qActual);
        double desiredVelocity = 0;
        btScalar velocityError = (desiredVelocity-qdActual);
        btScalar force = kp * positionError + kd*velocityError;
        btClamp(force,-maxForce,maxForce);
        m_multiBody->addJointTorque(joint, force);
    }




    if (m_frameCount==100)
    {
        const char* gPngFileName = "pendulum";


        if (gPngFileName)
        {


            //printf("gPngFileName=%s\n",gPngFileName);

            //sprintf(resultFileName.toUtf8().data(),"%s%d.png",gPngFileName,m_frameCount);
            //b3Printf("Made screenshot %s",resultFileName.toUtf8().data());
            //this->m_guiHelper->getAppInterface()->dumpNextFrameToPng(fileName);
        }
    }
    m_dynamicsWorld->stepSimulation(1./60.,0);//240,0);

    static int count = 0;
    if ((count& 0x0f)==0)
    {
#if 0
        for (int i=0;i<m_jointFeedbacks.size();i++)
        {

            b3Printf("F_reaction[%i] linear:%f,%f,%f, angular:%f,%f,%f",
                     i,
                     m_jointFeedbacks[i]->m_reactionForces.m_topVec[0],
                    m_jointFeedbacks[i]->m_reactionForces.m_topVec[1],
                    m_jointFeedbacks[i]->m_reactionForces.m_topVec[2],

                    m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[0],
                    m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[1],
                    m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[2]

                    );

        }
#endif
    }
    count++;


    /*
    b3Printf("base angvel = %f,%f,%f",m_multiBody->getBaseOmega()[0],
             m_multiBody->getBaseOmega()[1],
             m_multiBody->getBaseOmega()[2]
             );
    */
    //   btScalar jointVel =m_multiBody->getJointVel(0);

    //    b3Printf("child angvel = %f",jointVel);



}
