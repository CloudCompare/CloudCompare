#ifndef CC_COMPASS_INFO_HEADER
#define CC_COMPASS_INFO_HEADER

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QTextEdit>
#include <QFile>
#include <QTextStream>

class ccCompassInfo : public QDialog
{

public:
	explicit ccCompassInfo(QWidget *parent = 0);
};

#endif