#ifndef CCSETCLASSIFICATIONFIELD_H
#define CCSETCLASSIFICATIONFIELD_H

#include <QDialog>


namespace Ui {
class ccSetClassificationField;
}

//namespace ccHObject{
//class Container;
//}

#include <ccHObject.h>

class ccSetClassificationField : public QDialog
{
    Q_OBJECT

public:
    explicit ccSetClassificationField(QWidget *parent = nullptr);
    ~ccSetClassificationField();
    void readSettings(void);
    void writeSettings(void);

    bool setClassificationField(const ccHObject::Container &selectedEntities);
    int getClass();

private:
    Ui::ccSetClassificationField *ui;
    int class_;
};

#endif // CCSETCLASSIFICATIONFIELD_H
