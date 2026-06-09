#ifndef SEGMENTER_DLG_HEADER
#define SEGMENTER_DLG_HEADER

#include <QDialog>

namespace Ui {
    class SegmenterDialog;
}

class SegmenterDlg : public QDialog
{
    Q_OBJECT

public:
    explicit SegmenterDlg(QWidget* parent = nullptr);
    virtual ~SegmenterDlg();

    double getSpatialWeight() const;
    double getChromaticWeight() const;
    double getThreshold() const;
    bool isAddingPositiveSeeds() const;

    void setStatusMessage(const QString& msg);
    void setPointCount(int count);

signals:
    void stateChanged();
    void applyRequested();
    void undoRequested();
    void redoRequested();
    void clearRequested();

private slots:
    void onSliderMoved();

private:
    Ui::SegmenterDialog* ui;
};

#endif