#ifndef CC_COMPASS_INFO_HEADER
#define CC_COMPASS_INFO_HEADER

#include <qdialog.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qdialogbuttonbox.h>
#include <qboxlayout.h>
#include <qtextedit.h>
#include <qfile.h>
#include <QTextStream>

class ccCompassInfo : public QDialog
{

public:
	explicit ccCompassInfo(QWidget *parent = 0);
};

#endif