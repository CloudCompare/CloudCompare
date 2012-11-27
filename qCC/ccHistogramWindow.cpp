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
//$Rev:: 2174                                                              $
//$LastChangedDate:: 2012-06-25 19:03:20 +0200 (lun., 25 juin 2012)        $
//**************************************************************************
//

#include "ccHistogramWindow.h"
#include "ccGuiParameters.h"

//qCC_db
#include <ccColorTablesManager.h>

//QT
#include <QCloseEvent>
#include <QIcon>

//CCLib
#include <CCMiscTools.h>

//System
#include <assert.h>

ccHistogramWindow::ccHistogramWindow(QWidget* parent/*=0*/) : QGLWidget(parent)
{
	setWindowTitle("CloudCompare Histogram");
	setWindowIcon(QIcon(QString::fromUtf8(":/CC/Old/images/old_cc/cc_histogramIcon.gif")));

    setFocusPolicy(Qt::StrongFocus);

	viewInitialized = false;
	strcpy(infoStr,"NO INFO");

	roi[0]=10; roi[1]=10; roi[2]=width()-10; roi[3]=height()-40;

	numberOfClasses = std::max(roi[2]-roi[0],1);
	minVal = maxVal = 0;
	maxHistoVal = 0;
	xMinusButton = yMinusButton = xPlusButton = yPlusButton = 0;
	theValues   = NULL;
	histoValues = NULL;
	numberOfCurvePoints = 0;
	maxCurveValue = 0.0;
	curveValues = NULL;
	buttonSize  = 13;

	drawVerticalIndicator = false;
	numberOfClassesCanBeChanged = false;
	histoValuesShouldBeRecomputed = false;
	destroyHistoValues = false;
	destroyCurveValues = false;
	verticalIndicatorPositionPercent = 0;

    setMinimumSize(400,300);
    resize(400,375);

	//font for text rendering
	m_renderingFont.setFamily(QString::fromUtf8("Arial"));
	m_renderingFont.setBold(false);
	//m_renderingFont.setWeight(75);
}

void ccHistogramWindow::clear()
{
	if (destroyHistoValues && histoValues)
        delete[] histoValues;

	if (destroyCurveValues && curveValues)
        delete[] curveValues;
}

void ccHistogramWindow::closeEvent(QCloseEvent *event)
{
    clear();
    event->accept();
}

void ccHistogramWindow::setNumberOfClasses(unsigned n)  //n should be a mutliple of 4
{
	if (n==0)
		return;

	numberOfClasses=n;

	if (histoValuesShouldBeRecomputed)
	{
		if (histoValues)
            delete[] histoValues;
		histoValues = new unsigned[numberOfClasses];
		memset(histoValues,0,sizeof(unsigned)*numberOfClasses);
		computeHistoValues();
	}

	//update max histogram value
	maxHistoVal = getMaxHistoVal();

	//update vertical indicator position
	if (drawVerticalIndicator && roi[2]>roi[0])
		verticalIndicatorPositionPercent = (int)(verticalIndicatorPositionPercent*(double)n)/(double)n;
}

void drawBox(int xButton,int yButton,int buttonSize)
{
	glBegin(GL_LINE_LOOP);
	glVertex2i(xButton,yButton);
	glVertex2i(xButton+buttonSize-1,yButton);
	glVertex2i(xButton+buttonSize-1,yButton-buttonSize+1);
	glVertex2i(xButton,yButton-buttonSize+1);
	glEnd();
}

//structure for recursive display of labels
struct hlabel
{
    int leftXpos; 			/**< left label center pos **/
	int leftXmax; 			/**< left label 'ROI' max **/
	double leftVal; 		/**< left label value **/

	int rightXpos;			/**< right label center pos **/
	int rightXmin;			/**< right label 'ROI' min **/
	double rightVal;		/**< right label value **/
};

void ccHistogramWindow::paintGL()
{
	//for proper display of labels!
	m_renderingFont.setPointSize(ccGui::Parameters().defaultFontSize);
	QFontMetrics strMetrics(m_renderingFont);
	//precision (same as color scale)
	unsigned precision = ccGui::Parameters().displayedNumPrecision;

	makeCurrent();

	glClearColor(0.2f,0.0f,0.2f,0.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	int w = width();
	int h = height();
	float wHalf = (float)w*0.5f;
	float hHalf = (float)h*0.5f;

	//we always reinit the OpenGL context (simpler, safer)
	glViewport(0,0,width(),height());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,w-1,0,h-1,-1.0,1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//default color: white
	glColor3ubv(ccColor::white);

	//margins, etc.
	const int c_outerMargin = 10;
	const int c_innerMargin = 2;
	const int c_ticksSize = 5;
	const int c_strHeight = strMetrics.height();
	const int c_strDescent = strMetrics.descent();

	//+/- buttons (top-right)
	if (numberOfClassesCanBeChanged)
	{
		int yMin = h-c_outerMargin;
		int yHalf = yMin-(buttonSize/2); //to cope with odd button sizes
		//"minus"
		xMinusButton = w-(c_outerMargin+2*buttonSize+c_innerMargin*2);
		yMinusButton = yMin;
		drawBox(xMinusButton,yMinusButton,buttonSize);
		glBegin(GL_LINES);
		glVertex2i(xMinusButton+c_innerMargin,yHalf);
		glVertex2i(xMinusButton+buttonSize-1-c_innerMargin-1,yHalf);
		glEnd();

		//"plus"
		xPlusButton = xMinusButton+buttonSize+2*c_innerMargin;
		yPlusButton = yMin;
		drawBox(xPlusButton,yPlusButton,buttonSize);
		glBegin(GL_LINES);
		glVertex2i(xPlusButton+c_innerMargin,yHalf);
		glVertex2i(xPlusButton+buttonSize-1-c_innerMargin-1,yHalf);
		glVertex2i(xPlusButton+buttonSize/2,yMin-buttonSize+c_innerMargin+1);
		glVertex2i(xPlusButton+buttonSize/2,yMin-c_innerMargin-1);
		glEnd();
	}
	else
	{
		xMinusButton = w;
		yMinusButton = h;
	}

	//top-left corner
	//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
	glColor3f(1.0f,1.0f,1.0f);
	renderText(c_outerMargin, c_outerMargin+c_strHeight-c_strDescent, strMetrics.elidedText(QString("%0 [%1 classes]").arg(infoStr).arg(numberOfClasses),Qt::ElideRight,xMinusButton-c_outerMargin), m_renderingFont);

	//custom labels
	stringVector::const_iterator p;
	for (p = toDraw.begin(); p!= toDraw.end(); ++p)
		renderText(p->x, height()-p->y, p->str, m_renderingFont);

	//can't go any further without data!
	if (!histoValues)
        return;

	//update horizontal & vertical axes position so that their labels can be properly displayed
	int maxYLabelWidth = strMetrics.width(QString::number(maxHistoVal));
	QString firstlabel = QString::number(minVal,'f',precision);
	int firstXLabelWidth = strMetrics.width(firstlabel);
	QString lastlabel = QString::number(maxVal,'f',precision);
	int lastXLabelWidth = strMetrics.width(lastlabel);
	roi[0] = c_outerMargin+std::max(maxYLabelWidth+c_innerMargin,firstXLabelWidth/2);	//Xmin
	roi[1] = c_outerMargin+c_strHeight+c_innerMargin+c_ticksSize;						//Ymin
	roi[2] = w-c_outerMargin-lastXLabelWidth/2;											//Xmax
	roi[3] = h-(2*c_outerMargin+c_strHeight+c_strHeight/2);								//Ymax

	//histogram width
	int dx = roi[2]-roi[0];
	//histogram height
	int dy = roi[3]-roi[1];
	if (dx<2 || dy<2)
		return;

	//draw both axes
	glColor3ubv(ccColor::white);
	glBegin(GL_LINES);
	//vertical
	glVertex2f(roi[0],roi[1]);
	glVertex2f(roi[0],roi[3]);
	//horizontal
	glVertex2f(roi[0],roi[1]);
	glVertex2f(roi[2],roi[1]);
	glEnd();

	//horizontal labels
	{
		//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
		glColor3f(1.0f,1.0f,1.0f);

		//draw first value tick & label
			glBegin(GL_LINES);
		glVertex2f(roi[0],roi[1]-c_ticksSize);
		glVertex2f(roi[0],roi[1]);
			glEnd();
		renderText(roi[0]-firstXLabelWidth/2, h-c_outerMargin, firstlabel, m_renderingFont);

		//draw last value tick & label
		glBegin(GL_LINES);
		glVertex2f(roi[0]+dx,roi[1]-c_ticksSize);
		glVertex2f(roi[0]+dx,roi[1]);
		glEnd();
		renderText(roi[0]+dx-lastXLabelWidth/2, h-c_outerMargin, lastlabel, m_renderingFont);

		//we recursively display the remaining horizontal labels (unitl there's no room left)
		hlabel centerLabel;
		centerLabel.leftXmax = firstXLabelWidth/2;
		centerLabel.rightXmin = dx-lastXLabelWidth/2;

		//free space available between first and last label?
		const int c_labelMargin = 2*(strMetrics.width("123456789")/9); //twice the mean (digit) character width
		if (centerLabel.leftXmax+2*c_labelMargin<centerLabel.rightXmin)
		{
			centerLabel.leftVal = minVal;
			centerLabel.leftXpos = 0;
			centerLabel.rightVal = maxVal;
			centerLabel.rightXpos = dx;
			std::vector<hlabel> currentLevellabels;
			currentLevellabels.push_back(centerLabel);

			bool proceedWithNextLevel=true;
			while (proceedWithNextLevel)
			{
				std::vector<hlabel> nextLevelLabels;
				std::vector< std::pair<int,QString> > strToDisplay;
				while (!currentLevellabels.empty())
				{
					hlabel currentLabel = currentLevellabels.back();
					currentLevellabels.pop_back();

					//draw corresponding tick
					int x = (currentLabel.leftXpos+currentLabel.rightXpos)/2;
					glBegin(GL_LINES);
					glVertex2f(roi[0]+x,roi[1]-c_ticksSize);
					glVertex2f(roi[0]+x,roi[1]);
					glEnd();

					//try to draw label as well
					double value = (currentLabel.leftVal+currentLabel.rightVal)/2.0;
					QString valueStr = QString::number(value,'f',precision);
					int valueStrHalfWidth = strMetrics.width(valueStr)/2;
					if (x-valueStrHalfWidth-c_labelMargin > currentLabel.leftXmax
						&& x+valueStrHalfWidth+c_labelMargin < currentLabel.rightXmin)
					{
						//we push the label str (and position) in 'strToDisplay' vector
						strToDisplay.push_back(std::pair<int,QString>(roi[0]+x-valueStrHalfWidth,valueStr));
						//renderText(roi[0]+x-valueStrHalfWidth, h-c_outerMargin, valueStr, m_renderingFont);
						//add sub labels to next level queue
						hlabel leftLabel = currentLabel;
						leftLabel.rightVal = value;
						leftLabel.rightXpos = x;
						leftLabel.rightXmin = x-valueStrHalfWidth;

						hlabel rightLabel = currentLabel;
						rightLabel.leftVal = value;
						rightLabel.leftXpos = x;
						rightLabel.leftXmax = x+valueStrHalfWidth;

						nextLevelLabels.push_back(leftLabel);
						nextLevelLabels.push_back(rightLabel);
	}
					else
					{
						//not enough space for current label! we stop here
						proceedWithNextLevel=false;
					}
				}

				if (proceedWithNextLevel)
	{
					//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
					glColor3f(1.0f,1.0f,1.0f);

					//we only display labels if 'proceedWithNextLevel' is true
					while (!strToDisplay.empty())
					{
						renderText(strToDisplay.back().first, h-c_outerMargin, strToDisplay.back().second, m_renderingFont);
						strToDisplay.pop_back();
					}

					if (nextLevelLabels.empty())
						proceedWithNextLevel = false;
					else
						currentLevellabels = nextLevelLabels;
				}
			}
		}
	}

	//vertical labels
	{
		//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
		glColor3f(1.0f,1.0f,1.0f);

		const unsigned n=4;
		for (int i=0;i<=n;++i)
		{
			int y = roi[1] + (dy*i)/n;
			glBegin(GL_LINES);
			glVertex2f(roi[0]-5,y);
			glVertex2f(roi[0],y);
			glEnd();

			int vi = (int)((float)(maxHistoVal*i)/(float)n);
			QString valueStr = QString::number(vi);
			renderText(roi[0]-strMetrics.width(valueStr)-(c_ticksSize+c_innerMargin), h-(y-c_strHeight/2+c_strDescent), valueStr, m_renderingFont);
		}
	}

	//vertical scaling
	double maxDisplayedHistoVal = (double)ccMax(maxHistoVal,1);
	float yScale = (float)dy/(float)ccMax(maxCurveValue,maxDisplayedHistoVal);
	unsigned cumul=0;

	//the histogram itself
	{
		float x=(float)(roi[0]+1);
		float y=(float)(roi[1]+1);
		float barWidth = (float)dx/(float)numberOfClasses;

		for (int i=0;i<(int)numberOfClasses;++i)
		{
			const unsigned& val=histoValues[i];

			if ((double)i/(double)numberOfClasses < verticalIndicatorPositionPercent)
				cumul += val;

			if (theValues)
			{
				double dist = (maxVal-minVal) * (double)i / (double)numberOfClasses;
				float nCol = theValues->normalize(minVal+dist);
				if (nCol < 0)
					glColor3ubv(ccColor::lightGrey);
				else
					glColor3ubv(ccColorTablesManager::GetUniqueInstance()->getColor(nCol,theValues->getColorRamp()));
			}
			else
			{
				unsigned colIndex = (unsigned)((float)i*(float)DEFAULT_COLOR_RAMP_SIZE/(float)numberOfClasses);
				glColor3ubv(ccColorTablesManager::GetUniqueInstance()->getColor(colIndex,BGYR)); //default color ramp
			}

			float barHeight=(float)val*yScale;
			glBegin(GL_QUADS);
			glVertex2f(x,y);
			glVertex2f(x+barWidth,y);
			glVertex2f(x+barWidth,y+barHeight);
			glVertex2f(x,y+barHeight);
			glEnd();

			x+=barWidth;
		}
	}

	//overlay curve?
	if (curveValues && numberOfCurvePoints>1)
	{
		float step = (float)dx/(float)numberOfCurvePoints;
		float x=(float)(roi[0]+1);
		float y=(float)(roi[1]+1);

		//white by default
		glColor3ubv(ccColor::white);

		glBegin(GL_LINE_STRIP);
		for (unsigned i=0;i<numberOfCurvePoints;++i)
		{
			glVertex2f(x,y+curveValues[i]*yScale);
			x+=step;
		}
		glEnd();
	}

	//vertical hint
	if (drawVerticalIndicator)
	{
		//red by default
		glColor3ubv(ccColor::red);

		//horizontal position
		int x = roi[0] + 1 + (int)(verticalIndicatorPositionPercent*(double)dx);
		int y = roi[3] - c_strHeight;

		glBegin(GL_LINES);
		glVertex2f(x,roi[1]);
		glVertex2f(x,roi[3]);
		glEnd();

		bool leftSide = (verticalIndicatorPositionPercent>0.5);
		unsigned bin = (unsigned)(verticalIndicatorPositionPercent * (double)numberOfClasses);
		QString valueStr = QString("bin %0").arg(bin);

		//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
		glColor3f(1.0f,0.0f,0.0f);
		renderText(leftSide ? x-strMetrics.width(valueStr)-c_innerMargin : x+c_innerMargin, h-y, valueStr, m_renderingFont);
		y -= (c_strHeight+c_innerMargin);
		if (theValues)
		{
			valueStr = QString("< %0 %").arg(100.0*(double)cumul/(double)theValues->currentSize(),0,'f',3);
			renderText(leftSide ? x-strMetrics.width(valueStr)-c_innerMargin : x+c_innerMargin, h-y, valueStr, m_renderingFont);
			y -= (c_strHeight+c_innerMargin);
		}
		valueStr = QString("val = %0").arg(minVal+(maxVal-minVal)*verticalIndicatorPositionPercent,0,'f',precision);
		renderText(leftSide ? x-strMetrics.width(valueStr)-c_innerMargin : x+c_innerMargin, h-y, valueStr, m_renderingFont);
	}
}

void ccHistogramWindow::setInfoStr(const char* str)
{
	strcpy(infoStr,str);
}

void ccHistogramWindow::addStr(int x, int y, const char* str)
{
	geoString item;
	item.x = x;
	item.y = y;
	strcpy(item.str,str);

	toDraw.push_back(item);

	//Console::print("string %s ajoutee (%s)\n",item.str,str);
}

void ccHistogramWindow::clearStr()
{
	toDraw.clear();
}

void ccHistogramWindow::setValues(ccScalarField* values)
{
	assert(values);

	theValues = values;
	setMinVal(theValues->getMin());
	setMaxVal(theValues->getMax());

	numberOfClassesCanBeChanged = true;
	histoValuesShouldBeRecomputed = true;
};

void ccHistogramWindow::setHistoValues(unsigned* _histoValues, unsigned _numberOfClasses)
{
    assert(_histoValues);

	histoValues = _histoValues;
	numberOfClassesCanBeChanged = false;
	histoValuesShouldBeRecomputed = false;

	setNumberOfClasses(_numberOfClasses);
}

void ccHistogramWindow::setCurveValues(double* _curveValues, unsigned _numberOfCurvePoints)
{
	assert(_curveValues);

	curveValues = _curveValues;
	numberOfCurvePoints = _numberOfCurvePoints;

	//on recherche la hauteur maximale de la courbe
	maxCurveValue = 0.0;
	for (unsigned i=0;i<_numberOfCurvePoints;++i)
		maxCurveValue = ccMax(maxCurveValue,curveValues[i]);
}

void ccHistogramWindow::computeHistoValues()
{
	if (!histoValues || !theValues)
        return;

	//Console::print("Recalcul des valeurs de l'histogramme (%i classes) !\n",numberOfClasses);

	double val=0.0;
	double coef = (maxVal==minVal ? 1.0 : double(numberOfClasses)/(maxVal-minVal));

	//Console::print("min/maxVal=[%f,%f] --> coef=%f\n",minVal,maxVal,coef);

	//on accumule les points dans l'histogramme
	theValues->placeIteratorAtBegining();
	unsigned i;
	for (i=0;i<theValues->currentSize();++i)
	{
		val = double(theValues->getCurrentValue());
		theValues->forwardIterator();

		//on filtre
		if (val>=minVal && val<maxVal)
		{
			unsigned index = (unsigned)floor((val-minVal)*coef);
			assert(index<numberOfClasses);
			++histoValues[index];
		}
		//cas particulier
		else if (val==maxVal)
		{
            ++histoValues[numberOfClasses-1];
		}
	}
}

unsigned ccHistogramWindow::getMaxHistoVal()
{
	if (!histoValues)
        return 0;

	unsigned i,maxHistoVal = 0;
	for (i=0;i<numberOfClasses;++i)
        maxHistoVal=ccMax(maxHistoVal,histoValues[i]);

	return maxHistoVal;
}

void ccHistogramWindow::histoValuesShouldBeDestroyed(bool value)
{
    destroyHistoValues = value;
}

void ccHistogramWindow::curveValuesShouldBeDestroyed(bool value)
{
    destroyCurveValues = value;
}

void ccHistogramWindow::mousePressEvent(QMouseEvent *event)
{
    mouseMoveEvent(event);
}

void ccHistogramWindow::mouseMoveEvent(QMouseEvent *event)
{
	bool actionDetected = false;
    if (event->buttons() & Qt::LeftButton)
    {
        int x=event->x();
        int y=height()-event->y();

        if (numberOfClassesCanBeChanged)
        {
            //est-ce que l'utilisateur appuie sur le bouton "-"
            if (numberOfClasses>4)
            {
                if ((y>=yMinusButton-buttonSize)&&(y<=yMinusButton))
                {
                    if ((x>=xMinusButton)&&(x<=xMinusButton+buttonSize))
                    {
                        setNumberOfClasses(numberOfClasses-4);
                        actionDetected = true;
                    }
                }
            }

            //est-ce que l'utilisateur appuie sur le bouton "+"
            if (!actionDetected)
                if ((y>=yPlusButton-buttonSize)&&(y<=yPlusButton))
                {
                    if ((x>=xPlusButton)&&(x<=xPlusButton+buttonSize))
                    {
                        setNumberOfClasses(numberOfClasses+4);
                        actionDetected = true;
                    }
                }
        }

        //est-ce que l'utilisateur clique ailleurs sur l'histogramme
        if (!actionDetected)
        {
            if ((x>roi[0])&&(x<=roi[2])&&(y>roi[1])&&(y<=roi[3]))
			{
				drawVerticalIndicator = true;
				if (roi[2]>roi[0])
				{
					int verticalIndicatorPosition = (int)numberOfClasses*(x-roi[0])/(roi[2]-roi[0]);
					verticalIndicatorPositionPercent = (double)verticalIndicatorPosition/(double)numberOfClasses;
				}
				else
				{
					verticalIndicatorPositionPercent = 0;
				}
				actionDetected = true;
			}
        }
    }
    else event->ignore();

    if (actionDetected)
        updateGL();
}

void ccHistogramWindow::wheelEvent(QWheelEvent* event)
{
    if (!numberOfClassesCanBeChanged)
    {
        event->ignore();
        return;
    }

    if (event->delta()<0)
    {
        if (numberOfClasses>4)
        {
            setNumberOfClasses(numberOfClasses-4);
            updateGL();
        }
    }
    else //if (event->delta()>0)
    {
        setNumberOfClasses(numberOfClasses+4);
        updateGL();
    }
}
