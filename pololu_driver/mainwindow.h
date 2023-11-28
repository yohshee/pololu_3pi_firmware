#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <vector>

#include "pololurobot.h"
#include "robotaction.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void EnableButtons(bool bEnable);
    void ExecuteAction(RobotAction act);
    void ExecuteSequence();

    double GetSpeed()                 { return dDefSpeed; }
    double GetTurnRate()              { return dDefTurnRate; }
    void SetSpeed(double dNewSpeed)   { dDefSpeed = dNewSpeed; }
    void SetTurnRate(double dNewRate) { dDefTurnRate = dNewRate; }

private slots:
    void on_actionConnect_to_Robot_triggered();

    void on_actionExit_triggered();

    void on_btnForward_clicked();

    void on_btnBack_clicked();

    void on_btnTurnLeft_clicked();

    void on_btnTurnRight_clicked();

    void on_btnClear_clicked();

    void on_timerEvent();

    void on_btnRun_clicked();

private:
    Ui::MainWindow *ui;

private:
    // UI helpers
    QTimer* pTimer;

    // The robot
    PololuRobot robot;

    // Data members
    std::vector<RobotAction> vActionSet;
    double dDefSpeed;
    double dDefTurnRate;
};

#endif // MAINWINDOW_H
