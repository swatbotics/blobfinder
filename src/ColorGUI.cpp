#include "ColorGUI.h"

#include <QTimer>
#include <QGridLayout>
#include <QToolBar>
#include <QMouseEvent>
#include <QCursor>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QCloseEvent>

#ifndef Q_MOC_RUN
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#endif



enum { 
  UNDO_SIZE = 10
};

const QString titleStr = "Blob Color Picker";

ColorGUI::UndoState::UndoState(): cidx(ColorLUT::npos), modified(false) {}

ColorGUI::ColorGUI(ros::NodeHandle& n): nh(n) {

  setWindowTitle(titleStr);


  QWidget* w = new QWidget(this);
  setCentralWidget(w);

  QGridLayout* gl = new QGridLayout(w);
  gl->setSpacing(4);
  gl->setMargin(6);

  inputView = new QLabel(w);
  outputView = new QLabel(w);

  inputView->setBackgroundRole(QPalette::Dark);
  inputView->setForegroundRole(QPalette::Dark);
  inputView->setAutoFillBackground(true);
  inputView->setMinimumSize(320, 240);
  inputView->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  
  outputView->setBackgroundRole(QPalette::Dark);
  outputView->setForegroundRole(QPalette::Dark);
  outputView->setAutoFillBackground(true);
  outputView->setMinimumSize(320, 240);
  outputView->setAlignment(Qt::AlignLeft | Qt::AlignTop);

  QWidget* w2 = new QWidget(w);
  QVBoxLayout* vl = new QVBoxLayout(w2);

  colorList = new QTreeWidget(w2);
  colorList->setSelectionMode( QAbstractItemView::SingleSelection );

  addColorButton = new QPushButton("Add", w2);
  renameColorButton = new QPushButton("Rename", w2);
  removeColorButton = new QPushButton("Remove", w2);

  vl->addWidget(colorList);
  vl->addWidget(addColorButton);
  vl->addWidget(renameColorButton);
  vl->addWidget(removeColorButton);

  QTreeWidgetItem* header = new QTreeWidgetItem(QStringList("Color"));

  colorList->setHeaderItem(header);

  gl->addWidget(inputView, 0, 0);
  gl->addWidget(outputView, 1, 0);

  gl->addWidget(w2, 0, 1, 2, 1);

  gl->setColumnStretch(0, 1);

  QToolBar* toolbar = addToolBar("Tools");
  
  QLabel* l1 = new QLabel("Y radius: ");
  yRangeSpin = new QSpinBox();
  yRangeSpin->setRange(0, (1 << ColorLUT::ybits));
  yRangeSpin->setValue(1);

  QLabel* l2 = new QLabel("Cr/Cb radius: ");
  cRangeSpin = new QSpinBox();
  cRangeSpin->setRange(0, (1 << ColorLUT::cbits));
  cRangeSpin->setValue(1);

  QLabel* l3 = new QLabel("Opening: ");
  openSizeSpin = new QSpinBox();
  openSizeSpin->setRange(0, 5);
  openSizeSpin->setValue(1);

  liveCheck = new QCheckBox("Live feed");
  liveCheck->setChecked(true);

  toolbar->addWidget(l1);
  toolbar->addWidget(yRangeSpin);

  toolbar->addSeparator();

  toolbar->addWidget(l2);
  toolbar->addWidget(cRangeSpin);

  toolbar->addSeparator();

  toolbar->addWidget(liveCheck);

  toolbar->addSeparator();

  toolbar->addWidget(l3);
  toolbar->addWidget(openSizeSpin);

  imgSub = nh.subscribe("image", 4, 
			&ColorGUI::imageReceived, this);

  QTimer* t = new QTimer(this);

  connect(t, SIGNAL(timeout()), this, SLOT(handleROS()));

  t->start(10);


  mouseAction = MOUSE_NONE;
  
  connect(colorList, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
	  this, SLOT(onCurrentChanged()));

  connect(addColorButton, SIGNAL(clicked()),
	  this, SLOT(onAddClicked()));

  connect(removeColorButton, SIGNAL(clicked()),
	  this, SLOT(onRemoveClicked()));

  connect(renameColorButton, SIGNAL(clicked()),
	  this, SLOT(onRenameClicked()));


  fileNewAction = new QAction("&New", this);
  fileNewAction->setShortcut(QKeySequence("Ctrl+N"));
  connect(fileNewAction, SIGNAL(triggered()),
	  this, SLOT(fileNew()));

  fileOpenAction = new QAction("&Open...", this);
  fileOpenAction->setShortcut(QKeySequence("Ctrl+O"));
  connect(fileOpenAction, SIGNAL(triggered()),
	  this, SLOT(load()));

  fileSaveAction = new QAction("&Save", this);
  fileSaveAction->setShortcut(QKeySequence("Ctrl+S"));
  connect(fileSaveAction, SIGNAL(triggered()),
	  this, SLOT(save()));

  fileSaveAsAction = new QAction("&Save as...", this);
  fileSaveAsAction->setShortcut(QKeySequence("Ctrl+Shift+S"));
  connect(fileSaveAsAction, SIGNAL(triggered()),
	  this, SLOT(saveAs()));

  fileQuitAction = new QAction("&Quit", this);
  fileQuitAction->setShortcut(QKeySequence("Ctrl+Q"));
  connect(fileQuitAction, SIGNAL(triggered()),
	  this, SLOT(close()));

  editUndoAction = new QAction("&Undo", this);
  editUndoAction->setShortcut(QKeySequence("Ctrl+Z"));
  connect(editUndoAction, SIGNAL(triggered()),
	  this, SLOT(undo()));

  editRedoAction = new QAction("&Redo", this);
  editRedoAction->setShortcut(QKeySequence("Ctrl+Y"));
  connect(editRedoAction, SIGNAL(triggered()),
	  this, SLOT(redo()));

  QMenuBar* mb = menuBar();

  QMenu* file = new QMenu("&File", mb);
  file->addAction(fileNewAction);
  file->addAction(fileOpenAction);
  file->addAction(fileSaveAction);
  file->addAction(fileSaveAsAction);
  file->addAction(fileQuitAction);

  QMenu* edit = new QMenu("&Edit", mb);
  edit->addAction(editUndoAction);
  edit->addAction(editRedoAction);
  
  mb->addMenu(file);
  mb->addMenu(edit);
  
  statusBar = QMainWindow::statusBar();

  fileNew();

  throttle = 0;
  in_dialog = false;
  
}

ColorGUI::~ColorGUI() {
}

bool ColorGUI::askSave(const QString& question) {

  if (undoStack.empty() || !current().modified) {
    return true;
  }
      
  int answer = 
    QMessageBox::question(this, 
			  titleStr + " - Are you sure?", 
			  question + 
			  "If you have unsaved changes, they "
			  "will be lost",
			  QMessageBox::Yes | QMessageBox::No);
  return (answer == QMessageBox::Yes);
}

void ColorGUI::fileNew() {

  if (!askSave("Are you sure you want "
	       "to create a new file? ")) { 
    return; 
  }

  undoStack.clear();

  undoStack.push_back(UndoState());

  filename = "";
  fullPath = "";

  undoIndex = 0;
  repopulateList();
  updateActions();


}


void ColorGUI::imageReceived(const sensor_msgs::Image::ConstPtr& msg) {

  if (!liveCheck->isChecked()) { return; }

  if (in_dialog) {
    throttle = (throttle + 1) % 5;
    if (throttle) { return; }
  }
    

  const std::string& encoding = sensor_msgs::image_encodings::RGB8;
  
  cv::Mat image_rgb = cv_bridge::toCvCopy(msg, encoding)->image;

  // handle size
  int w = (inputView->width() + outputView->width()) / 2;
  int h = (inputView->height() + outputView->height()) / 2;
  

  scale = 1.0;

  if (image_rgb.cols > w) {
    double fx = double(w) / image_rgb.cols;
    scale = fx;
  } 
  if (image_rgb.rows > h) {
    double fy = double(h) / image_rgb.rows;
    scale = std::min(scale, fy);
  }

  if (scale != 1) {
    cv::Mat tmp;
    cv::resize(image_rgb, tmp, cv::Size(0,0), scale, scale, cv::INTER_CUBIC);
    image_rgb = tmp;
  }
    
  cv::cvtColor(image_rgb, image_yuv, CV_RGB2YCrCb);

  QImage tmpImage(image_rgb.data, 
		  image_rgb.cols, 
		  image_rgb.rows,
		  image_rgb.step,
		  QImage::Format_RGB888);
    
  inputView->setPixmap(QPixmap::fromImage(tmpImage));

  updateView();

}

void ColorGUI::updateView() {

  if (image_yuv.empty() || undoStack.empty()) { return; }

  const UndoState& cur = current();

  cv::Mat mask(image_yuv.rows, image_yuv.cols, CV_8U);

  cv::Mat mask_rgb;
  
  cur.lut.getImageColor(image_yuv, cur.cidx, mask);

  int o = openSizeSpin->value();

  if (o) {

    o = 2*o + 1;

    cv::Mat m = cv::getStructuringElement(cv::MORPH_ELLIPSE,
					  cv::Size(o,o));
				    
				    
  
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, 
		     m, cv::Point(-1,-1), 
		     1, 
		     cv::BORDER_REPLICATE);

  }

  cv::cvtColor(mask, mask_rgb, CV_GRAY2RGB);

  ColorLUT::RegionInfoVec infos;

  cur.lut.getRegionInfo(mask, infos);

  if (infos.empty()) {

    statusBar->showMessage("No blobs.");

  } else {

    const ColorLUT::RegionInfo& info = infos[0];

    statusBar->showMessage(QString("Largest blob at (%1, %2) with area %3").
			   arg(info.mean.x/scale, 0, 'f', 2).
			   arg(info.mean.y/scale, 0, 'f', 2).
			   arg(info.area/scale/scale, 0, 'f', 0));

    cv::circle(mask_rgb, info.mean, 3, CV_RGB(255,0,255), 1, CV_AA);

    cv::Size esize(2*sqrt(info.b1.dot(info.b1)),
		   2*sqrt(info.b2.dot(info.b2)));

    double eangle = atan2(info.b1.y, info.b1.x)*180/M_PI;

    cv::ellipse(mask_rgb, info.mean, esize, eangle, 0, 360, CV_RGB(255,0,255), 1, CV_AA);


  }
  

  QImage tmpImage(mask_rgb.data, 
		  mask_rgb.cols, 
		  mask_rgb.rows,
		  mask_rgb.step,
		  QImage::Format_RGB888);
  
  
  outputView->setPixmap(QPixmap::fromImage(tmpImage));


}

void ColorGUI::closeEvent(QCloseEvent* event) {
  
  if (!askSave("Are you sure you want to quit? ")) { 
    event->ignore();
  } else {
    QMainWindow::closeEvent(event);
  }

}

void ColorGUI::handleROS() {
  
  if (ros::ok()) { 
    ros::spinOnce();
  } else {
    if (undoStack.size()) {
      current().modified = false;
    }
    this->close(); 
  }
  
}

void ColorGUI::mousePressEvent(QMouseEvent* event) {

  if (mouseAction != MOUSE_NONE) {
    mouseReleaseEvent(event);
  }

  bool left = event->button() == Qt::LeftButton;
  bool right = event->button() == Qt::RightButton;

  if ( (left || right) && 
       colorList->currentItem() && 
       !image_yuv.empty() && inputView->underMouse()) {

    if (left) {
      mouseAction = MOUSE_ADD;
    } else {
      mouseAction = MOUSE_REMOVE;
    }
    firstMouse = true;

    mouseMoveEvent(event);

  } else {
    
    mouseAction = MOUSE_NONE;
    QMainWindow::mousePressEvent(event);


  }


}

void ColorGUI::mouseMoveEvent(QMouseEvent* event) {
  if (mouseAction != MOUSE_NONE) {
    if (inputView->underMouse()) {
      QPoint p = inputView->mapFromGlobal(event->globalPos());
      if (p.x() >= 0 && p.x() <= image_yuv.cols &&
	  p.y() >= 0 && p.y() <= image_yuv.rows) {


	if (firstMouse) {
	  pushUndo();
	  firstMouse = false;
	}

	cv::Vec3b pixel = image_yuv.at<cv::Vec3b>(p.y(), p.x());


	int yr = yRangeSpin->value();
	int cr = cRangeSpin->value();

	if (mouseAction == MOUSE_ADD) {
	  current().lut.addToColor(pixel, current().cidx, yr, cr);
	} else {
	  current().lut.removeFromColor(pixel, current().cidx, yr, cr);
	}

	updateView();

      }
    }
    event->accept();
  } else {
    QMainWindow::mouseMoveEvent(event);
  }
}

void ColorGUI::mouseReleaseEvent(QMouseEvent* event) {
  if (mouseAction != MOUSE_NONE) {
    mouseAction = MOUSE_NONE;
  } 
}

void ColorGUI::onAddClicked() {
  
  in_dialog = true;

  QString name = 
    QInputDialog::getText(this, 
			  titleStr + "- Add Color",
			  "Enter the name for the new color");

  in_dialog = false;

  if (name.isEmpty()) { return; }
  
  UndoState& cur = pushUndo();

  size_t cidx = cur.lut.addColor(name.toUtf8().constData());

  if (cidx == ColorLUT::npos) {
    undo();
    freezeUndo();
    return;
  }

  cur.clist.push_back(name);

  QTreeWidgetItem* i = new QTreeWidgetItem(colorList, QStringList(name));

  colorList->setCurrentItem(i);
  
}

void ColorGUI::onRemoveClicked() {

  QTreeWidgetItem* i = colorList->currentItem();
  
  UndoState& cur = pushUndo();

  cur.lut.removeColor(cur.cidx);
  cur.clist.takeAt( cur.clist.indexOf(i->text(0)) );

  delete i;


}

void ColorGUI::onRenameClicked() {

  QTreeWidgetItem* i = colorList->currentItem();
  QString prev = i->text(0);

  in_dialog = true;

  QString name = 
    QInputDialog::getText(this, 
			  titleStr + "- Rename Color",
			  "Enter the new name for the color",
			  QLineEdit::Normal, prev);

  in_dialog = false;

  if (name.isEmpty() || name == prev) { return; }

  i->setText(0, name);

  UndoState& cur = pushUndo();

  cur.lut.colornames[cur.cidx] = name.toUtf8().constData();
  cur.clist[cur.clist.indexOf(prev)] = name;
  

}

void ColorGUI::onCurrentChanged() {

  QTreeWidgetItem* i = colorList->currentItem();

  UndoState& cur = current();

  if (!i) {
    cur.cidx = ColorLUT::npos;
    return;
  }

  const ColorLUT& lut = cur.lut;

  QString s = i->text(0);
  
  cur.cidx = lut.lookupColor(s.toUtf8().constData());

  updateFocus();
  
}

void ColorGUI::updateFocus() {

  const UndoState& cur = current();
  
  size_t nc = 0;
  for (size_t i=0; i<ColorLUT::numcolors; ++i) {
    if (cur.lut.colornames[i] != "") { ++nc; }
  }

  addColorButton->setEnabled(nc < ColorLUT::numcolors);
  removeColorButton->setEnabled(cur.cidx != ColorLUT::npos);
  renameColorButton->setEnabled(cur.cidx != ColorLUT::npos);

}

void ColorGUI::load() {

  if (!askSave("Are you sure you want to "
	       "open another file? ")) {
    return;
  }

  in_dialog = true;

  QString fp = QFileDialog::getOpenFileName(this, titleStr + " - choose file to open", QString(), "*.data");

  in_dialog = false;


  if (fp.isNull() || fp.isEmpty()) { return; }

  load(fp);

}

void ColorGUI::load(const QString& fn) {

  undoStack.clear();
  undoStack.push_back(UndoState());
  undoIndex = 0;

  UndoState& cur = undoStack.back();

  cur.lut.load(fn.toUtf8().constData());

  for (size_t i=0; i<ColorLUT::numcolors; ++i) {
    if (!cur.lut.colornames[i].empty()) {
      cur.clist.push_back(cur.lut.colornames[i].c_str());
    }
  }
  
  fullPath = fn;
  filename = QFileInfo(fullPath).fileName();

  repopulateList();
  updateActions();

  
}

void ColorGUI::save() {

  if (fullPath.isEmpty()) { 
    saveAs();
    if (fullPath.isEmpty()) { return; }
  }

  while (undoIndex > 0) {
    undoStack.pop_front();
    --undoIndex;
  }

  current().lut.save(fullPath.toUtf8().constData());
  current().modified = false;

  updateActions();

}

void ColorGUI::saveAs() {

  in_dialog = true;

  QString fp = QFileDialog::getSaveFileName(this, titleStr + " - save file", QString(), "*.data");

  in_dialog = false;

  if (fp.isNull() || fp.isEmpty()) { return; }

  fullPath = fp;
  filename = QFileInfo(fp).fileName();
  
  save();

}

void ColorGUI::repopulateList() {
  colorList->blockSignals(true);
  QTreeWidgetItem* i = 0;
  do { 
    i = colorList->topLevelItem(0);
    delete i;
  } while (i);
  const UndoState& cur = current();
  QTreeWidgetItem* selected = 0;
  for (int i=0; i<cur.clist.size(); ++i) {
    QTreeWidgetItem* item = new QTreeWidgetItem(colorList,
						QStringList(cur.clist[i]));
    if (cur.cidx != ColorLUT::npos && 
	cur.lut.lookupColor(cur.clist[i].toUtf8().constData()) ==
	cur.cidx) {
      selected = item;
    }
  }
  if (selected) {
    colorList->setCurrentItem(selected);
  }
  onCurrentChanged();
  updateFocus();
  colorList->blockSignals(false);
}

void ColorGUI::undo() {
  if (undoIndex == 0) { return; } 
  --undoIndex;
  repopulateList();
  updateActions();
  updateView();
}

void ColorGUI::updateActions() {

  editUndoAction->setEnabled(undoIndex > 0);
  editRedoAction->setEnabled(undoIndex + 1 < undoStack.size());

  QString title = titleStr;
  if (filename.isEmpty()) {
    title += " - untitled";
  } else {
    title += " - " + filename;
  } 
  if (current().modified) {
    title += "*";
  }

  setWindowTitle(title);


  //fileSaveAction->setEnabled(current().modified);
}

void ColorGUI::redo() {
  if (undoIndex+1 >= undoStack.size()) { return; }
  ++undoIndex;
  repopulateList();
  updateActions();
  updateView();
}

void ColorGUI::freezeUndo() {

  while (undoStack.size() > undoIndex + 1) {
    undoStack.pop_back();
  }
  updateActions();

}

ColorGUI::UndoState& ColorGUI::pushUndo() {

  freezeUndo();
  assert(undoIndex == undoStack.size() - 1);

  undoStack.push_back(undoStack.back());
  ++undoIndex;

  while (undoStack.size() > UNDO_SIZE) {
    undoStack.pop_front();
    --undoIndex;
  }

  assert(undoIndex == undoStack.size() - 1);

  undoStack.back().modified = true;

  updateActions();

  return undoStack.back();

}

ColorGUI::UndoState& ColorGUI::current() {
  return undoStack[undoIndex];
}

const ColorGUI::UndoState& ColorGUI::current() const {
  return undoStack[undoIndex];
}


