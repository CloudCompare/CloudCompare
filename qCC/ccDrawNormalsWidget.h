#ifndef CCDRAWNORMALSWIDGET_H
#define CCDRAWNORMALSWIDGET_H

#include <QWidget>
#include <QButtonGroup>

#include "ccPointCloud.h"

namespace Ui {
class ccDrawNormalsWidget;
}

class ccDrawNormalsWidget : public QWidget
{
	Q_OBJECT

public:
	explicit ccDrawNormalsWidget(ccPointCloud * cloud, QWidget *parent = nullptr);
	~ccDrawNormalsWidget();
	void readSettings();
	void writeSettings();

	enum normalColor {YELLOW, RED, GREEN, BLUE, BLACK};

	void normalLengthValueChanged(double value);
	void setNormalColor();

private:
	Ui::ccDrawNormalsWidget *ui;
	ccPointCloud * m_cloud;
	QButtonGroup *buttonGroup;
};

#endif // CCDRAWNORMALSWIDGET_H
