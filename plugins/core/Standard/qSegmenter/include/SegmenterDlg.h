#ifndef SEGMENTER_DLG_H
#define SEGMENTER_DLG_H

#include <QDialog>

// Forward declaration of the UI namespace created by Qt's UIC compiler
namespace Ui {
class SegmenterDialog;
}

class SegmenterDlg : public QDialog
{
    Q_OBJECT

public:
    explicit SegmenterDlg(QWidget *parent = nullptr);
    ~SegmenterDlg() override;

    // Getters for the algorithm math
    double getSpatialWeight() const;
    double getChromaticWeight() const;
    double getThreshold() const;

    // Check which radio button is active
    bool isAddingPositiveSeeds() const;

    void setStatusMessage(const QString& msg);
    void setPointCount(int count);

signals:
    // We emit these when the user clicks the buttons so the main logic can react
    void undoRequested();
    void redoRequested();
    void clearRequested();
    
    // Emitted when the user changes from positive to negative, or tweaks a slider
    void stateChanged(); 

private:
    Ui::SegmenterDialog *ui;
};

#endif // SEGMENTER_DLG_H