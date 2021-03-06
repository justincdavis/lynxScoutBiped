#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "iostream"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void updateRobot();
    void readData();

signals:


private:
    Ui::MainWindow *ui;
    QSerialPort serial;
};

#endif // MAINWINDOW_H
