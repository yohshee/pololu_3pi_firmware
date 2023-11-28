/*
 * Main window for the Pololu driver application.
 *
 * Author: Rick Coogle
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <math.h>
#include <QMessageBox>
#include <QInputDialog>

#include "pololurobot.h"
#include "robotaction.h"


using namespace std;

////////////////////////////////////////////////////////////////
// Construction/Destruction

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    dDefSpeed = 0.1;
    dDefTurnRate = M_PI;

    ui->setupUi(this);
    this->setWindowTitle("Pololu Driver!");

    // Default ui settings
    ui->optGoNow->setChecked(true);
    ui->txtBattery->setText("0 V");
    ui->sldSensor->setMaximum(POLOLU_MAX_SENSOR);

    EnableButtons(false);

    // Kick off timer
    pTimer = new QTimer(this);
    connect(pTimer, SIGNAL(timeout()), this, SLOT(on_timerEvent()));
    pTimer->start(1000);
}

MainWindow::~MainWindow()
{
    pTimer->stop();
    delete pTimer;

    robot.CloseRobot();

    delete ui;
}

//////////////////////////////////////////////////////////////
// Utility methods


// Sets the enable state on all buttons in the window
void MainWindow::EnableButtons(bool bEnable)
{
    ui->btnBack->setEnabled(bEnable);
    ui->btnForward->setEnabled(bEnable);
    ui->btnTurnLeft->setEnabled(bEnable);
    ui->btnTurnRight->setEnabled(bEnable);
}


// Executes a single robot action
void MainWindow::ExecuteAction(RobotAction act)
{
    switch(act.GetType()) {
    case RobotAction::ACT_NULL:
        // do nothing
        break;
    case RobotAction::ACT_BACK:
        // move backwards
        robot.SetVel(-dDefSpeed, 0);
        usleep(static_cast<useconds_t>(act.GetDuration()*1000));
        robot.Stop();
        break;
    case RobotAction::ACT_FORWARD:
        // move forward
        robot.SetVel(dDefSpeed, 0);
        usleep(static_cast<useconds_t>(act.GetDuration()*1000));
        robot.Stop();
        break;
    case RobotAction::ACT_TURN_LEFT:
        // turn left
        robot.SetVel(0, dDefTurnRate);
        usleep(static_cast<useconds_t>(act.GetDuration()*1000));
        robot.Stop();
        break;
    case RobotAction::ACT_TURN_RIGHT:
        // turn right
        robot.SetVel(0, -dDefTurnRate);
        usleep(static_cast<useconds_t>(act.GetDuration()*1000));
        robot.Stop();
        break;
    default:
        // XXX: For now, ignore anomalous actions.
        break;
    }
}

// Executes the currently stored sequence of robot actions.
// If the sequence is empty, nothing happens.
void MainWindow::ExecuteSequence()
{
    RobotAction curAct;
    vector<RobotAction>::iterator iterAct;

    if(vActionSet.empty())
        return;

    for(iterAct = vActionSet.begin(); iterAct != vActionSet.end(); iterAct++) {
        curAct = *iterAct;
        ExecuteAction(curAct);
    }
}

/////////////////////////////////////////////////////////////
// User Defined Slots

void MainWindow::on_timerEvent()
{
    int iSensValue = 2000;
    int iVoltage = 0;
    QString strVoltText;

    if(robot.IsActive()) {
        // Update the slider with a sensor reading
        robot.GetLinePos(&iSensValue);
        ui->sldSensor->setValue(iSensValue);

        // Get the current battery charge
        robot.GetCharge(&iVoltage);

        strVoltText = QString("%1 V").arg(static_cast<double>(iVoltage) / 1000.0);
        ui->txtBattery->setText(strVoltText);
    }
}

/////////////////////////////////////////////////////////////
// System-Generated Slots

void MainWindow::on_actionConnect_to_Robot_triggered()
{
    bool bOk;
    int iRet;
    QString strDevice;

    // Get a device name from the user
    strDevice = QInputDialog::getText(this,
                                      "Robot Serial Device",
                                      "Robot Device Name:",
                                      QLineEdit::Normal,
                                      "/dev/ttyXbee1", &bOk);

    if(bOk) {
        iRet = robot.OpenRobot(strDevice);

        if(iRet == 1) {
            QMessageBox::critical(this,
                                 "Serial Error",
                                 "Could not open serial device!");
        }
        else if(iRet == 2) {
            QMessageBox::critical(this,
                                  "Serial Connection",
                                  "Connected to robot successfully, but could not ping. Connection closed.");
        }
        else {
            EnableButtons(true);

            QMessageBox::information(this,
                                     "Serial Connection",
                                     "Connected to robot.");
        }
    }
}

void MainWindow::on_actionExit_triggered()
{
    // Clean up and close the window
    this->close();
}

void MainWindow::on_btnForward_clicked()
{
    RobotAction act;

    // Depending on how it's set up, either issue a command or queue up an action

    act.SetType(RobotAction::ACT_FORWARD);
    act.SetDuration(RobotAction::QUARTER_SEC);

    if(ui->optGoNow->isChecked()) {
        ExecuteAction(act);
    }
    else if(ui->optSave->isChecked()) {
        vActionSet.push_back(act);
        ui->lstSequence->addItem("Move Forward");
    }
}

void MainWindow::on_btnBack_clicked()
{
    RobotAction act;

    act.SetType(RobotAction::ACT_BACK);
    act.SetDuration(RobotAction::QUARTER_SEC);

    if(ui->optGoNow->isChecked()) {
        ExecuteAction(act);
    }
    else if(ui->optSave->isChecked()) {
        vActionSet.push_back(act);
        ui->lstSequence->addItem("Move Back");
    }
}

void MainWindow::on_btnTurnLeft_clicked()
{
    RobotAction act;

    act.SetType(RobotAction::ACT_TURN_LEFT);
    act.SetDuration(RobotAction::QUARTER_SEC);

    if(ui->optGoNow->isChecked()) {
        ExecuteAction(act);
    }
    else if(ui->optSave->isChecked()) {
        vActionSet.push_back(act);
        ui->lstSequence->addItem("Turn Left");
    }
}

void MainWindow::on_btnTurnRight_clicked()
{
    RobotAction act;

    act.SetType(RobotAction::ACT_TURN_RIGHT);
    act.SetDuration(RobotAction::QUARTER_SEC);

    if(ui->optGoNow->isChecked()) {
        ExecuteAction(act);
    }
    else if(ui->optSave->isChecked()) {
        vActionSet.push_back(act);
        ui->lstSequence->addItem("Turn Right");
    }
}

void MainWindow::on_btnClear_clicked()
{
    // Clear it all out
    ui->lstSequence->clear();
    vActionSet.clear();
}


void MainWindow::on_btnRun_clicked()
{
    // Simple enough.
    if(ui->lstSequence->count() > 0) {
        EnableButtons(false);
        ExecuteSequence();
        EnableButtons(true);
    }
    else {
        QMessageBox::warning(this,
                             "No Sequence",
                             "You haven't added any robot actions!");
    }
}
