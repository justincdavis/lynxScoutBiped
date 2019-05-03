#include "mainwindow.h"
#include <QApplication>
#include "chain.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "frames_io.hpp"
#include <stdio.h>
#include <iostream>

using namespace KDL;

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;
    w.show();


    //BUILDING THE LEGS
    Chain leftLeg = Chain();
    Chain rightLeg = Chain();

    //Create the left leg chain
    Joint torsoToHipJoint(Joint::None);
    Frame torsoToHipFrame = Frame(Vector(4.454, 0, 0));
    leftLeg.addSegment(Segment(torsoToHipJoint, torsoToHipFrame));

    Joint hipToHipJoint(Joint::RotZ);
    Frame hipToHipFrame = Frame(Rotation::EulerXYZ( M_PI, 0, 0)) * Frame(Vector(0, 0, 5.458));
    leftLeg.addSegment(Segment(hipToHipJoin, hipToHipFrame));

    Joint hipToThighJoint(Joint::RotZ);
    Frame hipToThighFrame = Frame(Rotation::EulerXYZ(-M_PI / 2, 0, 0)) * Frame(Vector(0, 2.908, 0));
    leftLeg.addSegment(Segment(hipToThighJoint, hipToThighFrame));

    Joint thighToKneeJoint(Joint::RotZ);
    Frame thighToKneeFrame = Frame(Rotation::EulerXYZ(0, M_PI / 2, 0)) * Frame(Vector(0, 7.46, 0));
    leftLeg.addSegment(Segment(thighToKneeJoint, thighToKneeFrame));

    Joint kneeToAnkleJoint(Joint::RotZ);
    Frame kneeToAnkleFrame = Frame(Vector(0, 5.729, 0));
    leftLeg.addSegment(Segment(kneeToAnkleJoint, kneeToAnkleFrame));

    Joint ankleToFootJoint(Joint::RotZ);
    Frame ankleToFootFrame = Frame(Vector(0, 5.497, 0));
    leftLeg.addSegment(Segment(ankleToFootJoint, ankleToFootFrame));

    Joint footToFootJoint(Joint::RotZ);
    Frame footToFootFrame = Frame(Rotation::EulerXYZ(0, M_PI / 2, 0)) * Frame(Vector(0, 2.306, 0));
    leftLeg.addSegment(Segment(footToFootJoint, footToFootFrame));

    //Create the right leg chain



    return a.exec();
}
