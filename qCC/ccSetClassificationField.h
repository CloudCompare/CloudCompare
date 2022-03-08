#ifndef CCSETCLASSIFICATIONFIELD_H
#define CCSETCLASSIFICATIONFIELD_H

#include <QDialog>


namespace Ui {
class ccSetClassificationField;
}

#include <ccHObject.h>

class ccSetClassificationFieldDlg : public QDialog
{
    Q_OBJECT

public:
    explicit ccSetClassificationFieldDlg(QWidget *parent = nullptr);
    ~ccSetClassificationFieldDlg() override;
    void readSettings(void);
    void writeSettings(void);

    bool setClassificationField(const ccHObject::Container &selectedEntities);
    int getClass() const;

private:
    Ui::ccSetClassificationField *m_ui;
    int m_class;
};

#endif // CCSETCLASSIFICATIONFIELD_H
