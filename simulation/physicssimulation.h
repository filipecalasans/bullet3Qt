#ifndef PHYSICSSIMULATION_H
#define PHYSICSSIMULATION_H

#include <QObject>

class PhysicsSImulation : public QObject
{
    Q_OBJECT

public:

    explicit PhysicsSImulation(const QString &simName, QObject *parent = 0);

signals:

public slots:

private:

    QString simName;

};

#endif // PHYSICSSIMULATION_H
