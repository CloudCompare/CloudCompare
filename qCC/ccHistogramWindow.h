//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2127                                                              $
//$LastChangedDate:: 2012-05-11 11:35:03 +0200 (ven., 11 mai 2012)         $
//**************************************************************************
//

#ifndef CC_HISTOGRAM_WINDOW_HEADER
#define CC_HISTOGRAM_WINDOW_HEADER

//Qt
#include <QGLWidget>
#include <QDialog>
#include <QHBoxLayout>
#include <QFont>

//qCC_db
#include <ccScalarField.h>

//! String with associated 2D position
struct geoString
{
    int x,y;
    char str[256];
};
//! Vector of geoStrings
typedef std::vector<geoString> stringVector;

//! Histogram widget
class ccHistogramWindow : public QGLWidget
{
    Q_OBJECT

public:

	//! Default constructor
    ccHistogramWindow(QWidget *parent = 0);

    void setInfoStr(const char* str);
    void addStr(int x, int y, const char* str);
    void clearStr();

    //histogram variables setters and getters
    void setValues(ccScalarField* values);
    void setNumberOfClasses(unsigned  n); //n should be a mutliple of 4
    void setMinVal(double val) {minVal = val;};
    void setMaxVal(double val) {maxVal = val;};

    void setHistoValues(unsigned* _histoValues, unsigned _numberOfClasses); //by default, the object will eventually delete histoValues !
    void histoValuesShouldBeDestroyed(bool value);

    //overlay curve
    void setCurveValues(double* _curveValues, unsigned _numberOfCurvePoints); //by default, the object will eventually delete curveValues !
    void curveValuesShouldBeDestroyed(bool value);

protected:

    //mouse events handling
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent* event);

    void closeEvent(QCloseEvent *event);

    //inherited from QGLWidget
    //void initializeGL();
    //void resizeGL(int w, int h);
    void paintGL();


    unsigned getMaxHistoVal();

    //libération de la mémoire
    void clear();
    //recalcul des valeurs de l'histogrammes
    void computeHistoValues();

    char infoStr[256];
    bool viewInitialized;
    bool numberOfClassesCanBeChanged;
    stringVector toDraw;

    //table and type
    ccScalarField* theValues;

    //histogram variables
    unsigned numberOfClasses;
    double minVal,maxVal;
    unsigned maxHistoVal;

    //overlay curve
    double* curveValues;
    double maxCurveValue;
    unsigned numberOfCurvePoints;
    bool destroyCurveValues;

    //parametres d'affichage
    unsigned* histoValues;
    bool histoValuesShouldBeRecomputed;
    bool destroyHistoValues;

    //histogram display area
    int roi[4];
    //les boutons "+" et "-"
    int xMinusButton,yMinusButton,xPlusButton,yPlusButton;
    int buttonSize;

    //vertical indicator
    bool drawVerticalIndicator;
    double verticalIndicatorPositionPercent;

	//rendering font
	QFont m_renderingFont;
};

//! Encapsulating dialog for ccHistogramWindow
class ccHistogramWindowDlg : public QDialog
{
public:
    //! Default constructor
    ccHistogramWindowDlg(QWidget* parent = 0) : QDialog(parent)
    {
        win = new ccHistogramWindow(this);

        QHBoxLayout* hboxLayout = new QHBoxLayout(this);
        hboxLayout->addWidget(win);
        hboxLayout->setContentsMargins(0,0,0,0);
    }

    //! Returns encapsulated ccHistogramWindow
    ccHistogramWindow* window() { return win; };

protected:
    ccHistogramWindow* win;
};

#endif
