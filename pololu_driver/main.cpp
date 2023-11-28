/*
 * Main function for the Pololu driver application.
 *
 * Author: Rick Coogle
 */

#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setOrganizationName("Georgia Tech");
    app.setApplicationName("Pololu Driver");

    MainWindow win;
    win.show();

    return app.exec();
}
