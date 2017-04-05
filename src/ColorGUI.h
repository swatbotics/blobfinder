// -*- mode: c++ -*-
#include <QMainWindow>
#include <QLabel>
#include <QTreeWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QAction>
#include <QStatusBar>

#include <deque>

#include "ColorLUT.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#endif


class ColorGUI: public QMainWindow {

  Q_OBJECT

public:
  
  ColorGUI(ros::NodeHandle&);
  virtual ~ColorGUI();

public slots:

  void load();
  void load(const QString& filename);
  
  void save();
  void saveAs();

  void fileNew();

protected:

  void mousePressEvent(QMouseEvent * event);
  void mouseMoveEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void closeEvent(QCloseEvent* event);

private slots:
  
  void handleROS();
  void onAddClicked();
  void onRemoveClicked();
  void onRenameClicked();
  void onCurrentChanged();

  void undo();
  void redo();
  void freezeUndo();
  void repopulateList();
  void updateActions();

  void updateFocus();
  void updateView();

private:

  struct UndoState {
    UndoState();
    QStringList clist;
    ColorLUT lut;
    size_t cidx;
    bool modified;
  };

  void imageReceived(const sensor_msgs::Image::ConstPtr& msg);

  bool askSave(const QString& question="");

  UndoState& pushUndo();
  UndoState& current();
  const UndoState& current() const;
  

  ros::NodeHandle& nh;
  ros::Subscriber  imgSub;

  typedef std::deque<UndoState> UndoStack;

  UndoStack undoStack;
  size_t undoIndex;

  double scale;
  cv::Mat image_yuv;
  int throttle;
  bool in_dialog;
  
  QLabel* inputView;
  QLabel* outputView;

  QTreeWidget* colorList;
  
  QCheckBox* liveCheck;
  
  QSpinBox* yRangeSpin;
  QSpinBox* cRangeSpin;
  QSpinBox* openSizeSpin;



  QPushButton* addColorButton;
  QPushButton* removeColorButton;
  QPushButton* renameColorButton;

  QAction* fileNewAction;
  QAction* fileOpenAction;
  QAction* fileSaveAction;
  QAction* fileSaveAsAction;
  QAction* fileQuitAction;

  QAction* editUndoAction;
  QAction* editRedoAction;

  QStatusBar* statusBar;

  enum MouseAction {
    MOUSE_NONE,
    MOUSE_ADD,
    MOUSE_REMOVE
  };

  MouseAction mouseAction;
  bool firstMouse;

  QString fullPath;
  QString filename;

};
