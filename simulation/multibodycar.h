#ifndef MULTIBODYCAR_H
#define MULTIBODYCAR_H

#include <QObject>
#include "multibody.h"

#include "LinearMath/btScalar.h"

class MultibodyCar : public Multibody
{
    Q_OBJECT

public:

    explicit MultibodyCar(QObject *parent=0);
    ~MultibodyCar();

    virtual void    initPhysics();
    virtual void	stepSimulation(float deltaTime);

private:

    btMultiBody* m_multiBody;
    btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
    bool m_once;
    int m_frameCount;

    btAlignedObjectArray<btScalar> qDesiredArray;
    QString resultFileName;

    static const btScalar radius;
    static const btScalar kp;
    static const btScalar kd;
    static const btScalar maxForce;

};

#endif // MULTIBODYCAR_H
