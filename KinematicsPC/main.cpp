#include "mainwindow.h"
#include <QApplication>
#include "chain.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolverpos_lma.hpp"
#include "chainiksolvervel_wdls.hpp"
#include "chainiksolverpos_nr_jl.hpp"
#include "frames_io.hpp"
#include <stdio.h>
#include <iostream>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QCursor>
#include <QPoint>
#include <QSize>
#include <QDebug>

//signal and slot method
//checking for mouse move events
//Qtimer to space them out

using namespace KDL;

//need to seperate the solving stuff out of these two methods
//the solvers should be in their own function but only created once
Chain makeLeftLeg(){
    Chain leftLeg = Chain();

    //CREATING THE LEFT LEG CHAIN
    Joint ltorsoToHipJoint(Joint::None);
    Frame ltorsoToHipFrame = Frame(Vector(.04454, 0, 0));
    leftLeg.addSegment(Segment(ltorsoToHipJoint, ltorsoToHipFrame));

    Joint lhipToHipJoint(Joint::RotZ);
    Frame lhipToHipFrame = Frame(Rotation::EulerZYX(0, 0, M_PI)) * Frame(Vector(0, 0, .05458));
    leftLeg.addSegment(Segment(lhipToHipJoint, lhipToHipFrame));

    Joint lhipToThighJoint(Joint::RotZ);
    Frame lhipToThighFrame = Frame(Rotation::EulerZYX(0, 0, -M_PI / 2)) * Frame(Vector(0, .02908, 0));
    leftLeg.addSegment(Segment(lhipToThighJoint, lhipToThighFrame));

    Joint lthighToKneeJoint(Joint::RotZ);
    Frame lthighToKneeFrame = Frame(Rotation::EulerZYX(0, M_PI / 2, 0)) * Frame(Vector(0, .0746, 0));
    leftLeg.addSegment(Segment(lthighToKneeJoint, lthighToKneeFrame));

    Joint lkneeToAnkleJoint(Joint::RotZ);
    Frame lkneeToAnkleFrame = Frame(Vector(0, .05729, 0));
    leftLeg.addSegment(Segment(lkneeToAnkleJoint, lkneeToAnkleFrame));

    Joint lankleToFootJoint(Joint::RotZ);
    Frame lankleToFootFrame = Frame(Vector(0, .05497, 0));
    leftLeg.addSegment(Segment(lankleToFootJoint, lankleToFootFrame));

    Joint lfootToFootJoint(Joint::RotZ);
    Frame lfootToFootFrame = Frame(Rotation::EulerZYX(0, M_PI / 2, 0)) * Frame(Vector(0, .02306, 0));
    leftLeg.addSegment(Segment(lfootToFootJoint, lfootToFootFrame));

    //DEFINING MAX AND MINS FOR THE SERVO JOINTS
    JntArray leftJointArrayMin = JntArray(7);
    JntArray leftJointArrayMax = JntArray(7);

    leftJointArrayMin(0) = 0;
    leftJointArrayMin(1) = -0.7853981634;
    leftJointArrayMin(2) = -0.7853981634;
    leftJointArrayMin(3) = -0.7853981634;
    leftJointArrayMin(4) = -0.7853981634;
    leftJointArrayMin(5) = -0.7853981634;
    leftJointArrayMin(6) = -0.7853981634;

    leftJointArrayMax(0) = 0;
    leftJointArrayMax(1) = 0.7853981634;
    leftJointArrayMax(2) = 0.7853981634;
    leftJointArrayMax(3) = 0.7853981634;
    leftJointArrayMax(4) = 0.7853981634;
    leftJointArrayMax(5) = 0.7853981634;
    leftJointArrayMax(6) = 0.7853981634;

    //FORWARD KINEMATIC SOLVERS
    ChainFkSolverPos_recursive leftFkSolver = ChainFkSolverPos_recursive(leftLeg);

    //INVERSE KINEMATIC SOLVER
    ChainIkSolverVel_wdls leftIkSolverVel = ChainIkSolverVel_wdls(leftLeg);
    ChainIkSolverPos_NR_JL leftIkSolverPos = ChainIkSolverPos_NR_JL(leftLeg, leftJointArrayMin, leftJointArrayMax, leftFkSolver, leftIkSolverVel);

    //RESULTS JOINT ARRAY
    JntArray leftResults[7];

    return leftLeg;
}

Chain makeRightLeg(){
    Chain rightLeg = Chain();

    //CREATING THE RIGHT LEG CHAIN
    Joint rtorsoToHipJoint(Joint::None);
    Frame rtorsoToHipFrame = Frame(Vector(.04454, 0, 0));
    rightLeg.addSegment(Segment(rtorsoToHipJoint, rtorsoToHipFrame));

    Joint rhipToHipJoint(Joint::RotZ);
    Frame rhipToHipFrame = Frame(Rotation::EulerZYX(0, 0, -M_PI)) * Frame(Vector(0, 0, .05458));
    rightLeg.addSegment(Segment(rhipToHipJoint, rhipToHipFrame));

    Joint rhipToThighJoint(Joint::RotZ);
    Frame rhipToThighFrame = Frame(Rotation::EulerZYX(0, 0, M_PI / 2)) * Frame(Vector(0, .02908, 0));
    rightLeg.addSegment(Segment(rhipToThighJoint, rhipToThighFrame));

    Joint rthighToKneeJoint(Joint::RotZ);
    Frame rthighToKneeFrame = Frame(Rotation::EulerZYX(0, -M_PI / 2, 0)) * Frame(Vector(0, .0746, 0));
    rightLeg.addSegment(Segment(rthighToKneeJoint, rthighToKneeFrame));

    Joint rkneeToAnkleJoint(Joint::RotZ);
    Frame rkneeToAnkleFrame = Frame(Vector(0, .05729, 0));
    rightLeg.addSegment(Segment(rkneeToAnkleJoint, rkneeToAnkleFrame));

    Joint rankleToFootJoint(Joint::RotZ);
    Frame rankleToFootFrame = Frame(Vector(0, .05497, 0));
    rightLeg.addSegment(Segment(rankleToFootJoint, rankleToFootFrame));

    Joint rfootToFootJoint(Joint::RotZ);
    Frame rfootToFootFrame = Frame(Rotation::EulerZYX(0, -M_PI / 2, 0)) * Frame(Vector(0, .02306, 0));
    rightLeg.addSegment(Segment(rfootToFootJoint, rfootToFootFrame));

    //DEFINING MAX AND MINS FOR THE SERVO JOINTS
    JntArray rightJointArrayMin = JntArray(7);
    JntArray rightJointArrayMax = JntArray(7);

    rightJointArrayMin(0) = 0;
    rightJointArrayMin(1) = -0.7853981634;
    rightJointArrayMin(2) = -0.7853981634;
    rightJointArrayMin(3) = -0.7853981634;
    rightJointArrayMin(4) = -0.7853981634;
    rightJointArrayMin(5) = -0.7853981634;
    rightJointArrayMin(6) = -0.7853981634;

    rightJointArrayMax(0) = 0;
    rightJointArrayMax(1) = 0.7853981634;
    rightJointArrayMax(2) = 0.7853981634;
    rightJointArrayMax(3) = 0.7853981634;
    rightJointArrayMax(4) = 0.7853981634;
    rightJointArrayMax(5) = 0.7853981634;
    rightJointArrayMax(6) = 0.7853981634;

    //FORWARD KINEMATIC SOLVERS
    ChainFkSolverPos_recursive rightFkSolver = ChainFkSolverPos_recursive(rightLeg);

    //INVERSE KINEMATIC SOLVER
    ChainIkSolverVel_wdls rightIkSolverVel = ChainIkSolverVel_wdls(rightLeg);
    ChainIkSolverPos_NR_JL rightIkSolverPos = ChainIkSolverPos_NR_JL(rightLeg, rightJointArrayMin, rightJointArrayMax, rightFkSolver, rightIkSolverVel);

    //RESULTS JOINT ARRAY
    JntArray rightResults[7];

    return rightLeg;
}

int main(int argc, char *argv[])
{

    if(serial.open(QIODevice::ReadWrite)){
        serial.write("M114 \n");
        serial.close();
    }

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
