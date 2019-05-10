#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>

//implement the QserialPort and connect the readyRead
//send some simple data from the arduino for the computer to read

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateRobot);
    timer->start(100);

    QSerialPort serial;
    serial.setPortName("COM3");
    serial.setBaudRate(QSerialPort::Baud9600);
    connect(serial, &QSerialPort::readyRead(), this, &MainWindow::readData);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateRobot(){
    std::cout << "hi" << std::endl;
}

void MainWindow::readData(){

}
