#include <QApplication>
#include <QFileInfo>

#include "ColorGUI.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "colorpicker");

  ros::NodeHandle nh;

  QApplication a(argc, argv);

  ColorGUI w(nh);

  a.connect(&a, SIGNAL(lastWindowClosed()),
	    &a, SLOT(quit()));

  if (argc == 2 && QFileInfo(argv[1]).exists()) {
    w.load(argv[1]);
  }

  w.show();

  return a.exec();

}
