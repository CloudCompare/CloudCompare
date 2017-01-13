#include "ccCompassInfo.h"

ccCompassInfo::ccCompassInfo(QWidget *parent)
	: QDialog(parent)
{
	setFixedWidth(800);
	setFixedHeight(600);

	//setup GUI components
	QTextEdit* l = new QTextEdit(this);
	l->acceptRichText();
	l->setReadOnly(true);

	QPushButton *okButton = new QPushButton(tr("Ok"));
	QDialogButtonBox *buttonBox = new QDialogButtonBox(Qt::Horizontal);
	buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept())); //done
	QVBoxLayout *lt = new QVBoxLayout;
	lt->addWidget(l);
	lt->addWidget(buttonBox);
	setLayout(lt);

	//load text
	QFile file(":/CC/plugin/qCompass/info.html");
	if (file.open(QIODevice::ReadOnly))
	{
		QTextStream in(&file);
		QString html = in.readAll();
		l->setText(html);
		file.close();
	}
	else
	{
		l->setText("Error loading documentation file....");
	}
}