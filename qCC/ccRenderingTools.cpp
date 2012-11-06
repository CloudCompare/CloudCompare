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
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//

#include "ccRenderingTools.h"
#include "ccGuiParameters.h"

//qCC
#include "ccConsole.h"
#include "ccGLWindow.h"

//qCC_db
#include <ccIncludeGL.h>
#include <ccGenericPointCloud.h>
#include <ccGBLSensor.h>

//CCLib
#include <ScalarField.h>
#include <CCMiscTools.h>

//Qt
#include <QLabel>
#include <QDialog>
#include <QVBoxLayout>

//system
#include <assert.h>

void ccRenderingTools::ShowDepthBuffer(ccGBLSensor* sensor, QWidget* parent)
{
	if (!sensor)
		return;

	CCLib::GroundBasedLidarSensor::DepthBuffer dB = sensor->getDepthBuffer();
	if (!dB.zBuff)
		return;

	const colorType* colorTable = ccColorTablesManager::GetUniqueInstance()->getColorTable(BGYR);

	DistanceType *_zBuff = dB.zBuff;
	DistanceType maxDist = 0.0;
	DistanceType minDist = 0.0;
	int x,y;
	for (x=0;x<dB.h_buff*dB.l_buff;++x)
	{
		if (x==0)
		{
			maxDist = minDist = *_zBuff;
		}
		else if (*_zBuff > 0.0)
		{
			maxDist = ccMax(maxDist,*_zBuff);
			minDist = ccMin(minDist,*_zBuff);
		}
		++_zBuff;
	}

	DistanceType coef = 0.0;
	if (maxDist-minDist < ZERO_TOLERANCE)
	{
		ccConsole::Warning("[ShowDepthBuffer] Flat buffer! (max depth=0)");
		//return;
	}
	else
	{
		coef = DistanceType(DEFAULT_COLOR_RAMP_SIZE-1)/(maxDist-minDist);
	}

	QImage bufferImage(dB.l_buff,dB.h_buff,QImage::Format_RGB32);

	_zBuff = dB.zBuff;
	for (y=0;y<dB.h_buff;++y)
		for (x=0;x<dB.l_buff;++x)
		{
			const colorType* col = (*_zBuff >= minDist ? colorTable + ((int)((*_zBuff - minDist) * coef))*4 : ccColor::black);
			bufferImage.setPixel(x,dB.h_buff-1-y,qRgb(col[0],col[1],col[2]));
			++_zBuff;
		}

		ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(sensor->getParent());

		QDialog* dlg = new QDialog(parent);
		dlg->setWindowTitle(QString("%0 depth buffer [%1 x %2]").arg(cloud->getName()).arg(dB.l_buff).arg(dB.h_buff));
		dlg->setMinimumSize(dB.l_buff,dB.h_buff);
		dlg->setMaximumSize(dB.l_buff,dB.h_buff);
		QVBoxLayout* vboxLayout = new QVBoxLayout(dlg);
		vboxLayout->setContentsMargins(0,0,0,0);
		QLabel* label = new QLabel(dlg);
		label->setScaledContents(true);
		vboxLayout->addWidget(label);

		label->setPixmap(QPixmap::fromImage(bufferImage));
		//dlg->resize(dB.l_buff,dB.h_buff);
		dlg->show();
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context)
{
	const ccScalarField* sf = context.sfColorScaleToDisplay;

	if (!sf)
		return;

	DistanceType minVal = sf->getMin();
	DistanceType minDisplayed = sf->getMinDisplayed();
	DistanceType minSaturation = sf->getMinSaturation();
	DistanceType maxSaturation = sf->getMaxSaturation();
	DistanceType maxDisplayed = sf->getMaxDisplayed();
	DistanceType maxVal = sf->getMax();

	bool strictlyPositive = sf->isPositive() || minVal>=0.0;
	bool absSaturation = sf->absoluteSaturation();
	bool logScale = sf->logScale();

	int cubeSize = ccGui::Parameters().colorScaleSquareSize;
	int spaceBetweenElements = 4;
	int i;

	//this vector stores the values that will be "represented" by the scale
	//they will be automatically displayed in a regular "pace"
	std::vector<ScaleElement> theScaleElements;
	std::vector<DistanceType> theCubeEquivalentDist; //to deduce its color!

	int maxNumberOfCubes = int(floor(float(context.glH-120)/float(cubeSize+2*spaceBetweenElements)));

	int colorRampSteps = context.sfColorScaleToDisplay->getColorRampSteps();
	CC_COLOR_RAMPS colorRampType = context.sfColorScaleToDisplay->getColorRamp();

	//first we fill the two vectors below with scale "values"
	if (strictlyPositive || !absSaturation) //only positive values
	{
		bool dispZero = ccGui::Parameters().colorScaleAlwaysShowZero && minDisplayed>0.0 && strictlyPositive;
		bool dispMinVal = false;//(minVal<minDisplayed);
		bool dispMinDispVal = true;
		bool dispMinSat = (minSaturation>minDisplayed && minSaturation<maxSaturation);
		bool dispMaxSat = (maxSaturation>=minSaturation && maxSaturation<maxDisplayed);
		bool dispMaxDispVal = (maxDisplayed>minDisplayed && maxDisplayed<maxVal);
		bool dispMaxVal = true;

		int addedCubes = int(dispZero) + int(dispMinVal) + int(dispMinDispVal) + int(dispMinSat) + int(dispMaxSat) + int(dispMaxDispVal) + int(dispMaxVal);

		//not enough room for display!
		if (maxNumberOfCubes < addedCubes)
			return;

		//number of cubes available for ramp display
		int numberOfCubes = ccMin(maxNumberOfCubes-addedCubes,colorRampSteps);

		DistanceType startValue = minVal; //we want it to be the same color as 'minVal' even if we start at '0'
		if (dispZero)
			theScaleElements.push_back(ScaleElement(0.0,true,true));

		if (dispMinVal)
		{
			//precedent cube color
			if (!theScaleElements.empty())
				theCubeEquivalentDist.push_back(startValue);

			theScaleElements.push_back(ScaleElement(minVal,true,dispMinDispVal || dispMinSat));
			startValue = minVal;
		}

		if (dispMinDispVal)
		{
			//precedent cube color
			if (!theScaleElements.empty())
				theCubeEquivalentDist.push_back(startValue);

			theScaleElements.push_back(ScaleElement(minDisplayed,true,dispMinSat));
			startValue = minDisplayed;
		}

		if (dispMinSat)
		{
			//precedent cube color
			if (!theScaleElements.empty())
				theCubeEquivalentDist.push_back(startValue);

			theScaleElements.push_back(ScaleElement(minSaturation));
			startValue = minSaturation;
		}

		//the actual color ramp
		if (numberOfCubes>0 && minSaturation<maxSaturation && minDisplayed<maxDisplayed)
		{
			DistanceType endValue = (dispMaxSat ? maxSaturation : maxDisplayed);
			DistanceType intervale = (endValue-startValue)/(DistanceType)numberOfCubes;
			DistanceType firstValue = startValue;

			if (logScale)
			{
				DistanceType endValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(endValue)));
				DistanceType startValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(startValue)));
				intervale = (endValueLog-startValueLog)/(DistanceType)numberOfCubes;
				firstValue = startValueLog;
			}

			if (intervale < ZERO_TOLERANCE)
			{
				//finally, we won't draw this ramp!
				theScaleElements.back().condensed = true;
			}
			else
			{
				if (logScale)
				{
					for (i=0;i<numberOfCubes;++i)
					{
						DistanceType logVal = firstValue+intervale*0.5;
						theCubeEquivalentDist.push_back(exp(logVal*log(10.0)));
						firstValue += intervale;
						theScaleElements.push_back(ScaleElement(exp(firstValue*log(10.0)),true,false));
					}
				}
				else
				{
					for (i=0;i<numberOfCubes;++i)
					{
						theCubeEquivalentDist.push_back(firstValue+intervale*0.5);
						firstValue += intervale;
						theScaleElements.push_back(ScaleElement(firstValue,true,false));
					}
				}
			}
		}

		if (dispMaxSat && dispMaxDispVal)
		{
			theCubeEquivalentDist.push_back(maxSaturation);
			theScaleElements.back().condensed = true;
			theScaleElements.push_back(ScaleElement(maxDisplayed, true, true));
		}

		if ((dispMaxSat || dispMaxDispVal) && dispMaxVal)
		{
			theCubeEquivalentDist.push_back(maxVal);
			theScaleElements.back().condensed = true;
			theScaleElements.push_back(ScaleElement(maxVal));
		}
	}
	else //both positive and negative values
	{
		//TODO FIXME!!!

		//if the ramp should be symmetrical
		bool symmetry = ccGui::Parameters().colorScaleAlwaysSymmetrical;
		if (symmetry)
		{
			//we display the color ramp between -maxDisp and +maxDisp
			DistanceType maxDisp = ccMax(-minVal,maxVal);

			bool dispZero = true;
			bool dispMinSat = (minSaturation>0.0);
			bool dispMaxSat = (maxSaturation>minSaturation && maxSaturation<maxDisp);
			bool dispMaxVal = true;

			int addedCubes = 2 * (int(dispZero && dispMinSat) + int(dispMaxSat && dispMaxVal));

			//not enough room for display!
			if (maxNumberOfCubes < addedCubes)
				return;

			//number of cubes available for ramp display
			int numberOfCubes = ccMin((maxNumberOfCubes-addedCubes)/2,colorRampSteps);

			//1st section: -maxDisp
			DistanceType startValue = -maxDisp;
			if (dispMaxVal)
				theScaleElements.push_back(ScaleElement(-maxDisp,true,dispMaxSat));

			//2nd section: -maxSaturation
			if (dispMaxSat)
			{
				//precedent cube color
				if (!theScaleElements.empty())
					theCubeEquivalentDist.push_back(startValue);
				theScaleElements.push_back(ScaleElement(-maxSaturation));
				startValue = -maxSaturation;
			}

			//3rd section: the real color ramp (negative part)
			if (numberOfCubes>1)
			{
				DistanceType endValue = (dispMinSat ? -minSaturation : 0.0);
				DistanceType intervale = (endValue-startValue)/(DistanceType)numberOfCubes;
				DistanceType firstValue = startValue;

				if (logScale)
				{
					DistanceType endValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(-endValue)));
					DistanceType startValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(-startValue)));
					intervale = -(endValueLog-startValueLog)/(DistanceType)numberOfCubes;
					firstValue = startValueLog;
				}

				if (intervale < ZERO_TOLERANCE)
				{
					//finally, we won't draw this ramp!
					theScaleElements.back().condensed = true;
				}
				else
				{
					if (logScale)
					{
						for (i=0;i<numberOfCubes-1;++i)
						{
							DistanceType logVal = firstValue-intervale*0.5;
							theCubeEquivalentDist.push_back(-exp(logVal*log(10.0)));
							firstValue -= intervale;
							//if (i==0 && firstValue>maxVal) //specific case: all values in the tail
							//	theScaleElements.push_back(ScaleElement(maxVal,true));
							//else
								theScaleElements.push_back(ScaleElement(-exp(firstValue*log(10.0)),true));
						}
					}
					else
					{
						for (i=0;i<numberOfCubes-1;++i)
						{
							theCubeEquivalentDist.push_back(firstValue + intervale*0.5);
							firstValue += intervale;
							//if (i==0 && firstValue>maxVal) //specific case: all values in the tail
							//	theScaleElements.push_back(ScaleElement(maxVal,true));
							//else
								theScaleElements.push_back(ScaleElement(firstValue,true));
						}
					}
				}
			}

			//4th section: -minSaturation
			if (dispMinSat)
			{
				theCubeEquivalentDist.push_back(-minSaturation);
				theScaleElements.push_back(ScaleElement(-minSaturation, true, true));
			}

			//5th section: zero
			if (dispZero)
			{
				theCubeEquivalentDist.push_back(0.5*theCubeEquivalentDist.back());
				theScaleElements.push_back(ScaleElement(0, true, dispMinSat));
			}

			//6th section: minSaturation
			if (dispMinSat)
			{
				theCubeEquivalentDist.push_back(0.0);
				theScaleElements.push_back(ScaleElement(minSaturation));
			}

			//7th section: the real color ramp (positive part)
			if (numberOfCubes>1)
			{
				DistanceType intervale = (maxSaturation-minSaturation)/(DistanceType)numberOfCubes;
				DistanceType firstValue = minSaturation;

				if (logScale)
				{
					DistanceType endValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(maxSaturation)));
					DistanceType startValueLog = log10(ccMax((DistanceType)ZERO_TOLERANCE,fabs(minSaturation)));
					intervale = (endValueLog-startValueLog)/(DistanceType)numberOfCubes;
					firstValue = startValueLog;
				}

				if (intervale < ZERO_TOLERANCE)
				{
					//finally, we won't draw this ramp!
					theScaleElements.back().condensed = true;
				}
				else
				{
					if (logScale)
					{
						for (i=0;i<numberOfCubes-1;++i)
						{
							DistanceType logVal = firstValue+intervale*0.5;
							theCubeEquivalentDist.push_back(exp(logVal*log(10.0)));
							firstValue += intervale;
							//if (i+2==numberOfCubes && firstValue<minVal) //specific case: all values in the head
							//	theScaleElements.push_back(ScaleElement(minVal,true));
							//else
								theScaleElements.push_back(ScaleElement(exp(firstValue*log(10.0)),true));
						}
					}
					else
					{
						for (i=0;i<numberOfCubes-1;++i)
						{
							theCubeEquivalentDist.push_back(firstValue + intervale*0.5);
							firstValue += intervale;
							//if (i+2==numberOfCubes && firstValue<minVal) //specific case: all values in the head
							//	theScaleElements.push_back(ScaleElement(minVal,true));
							//else
								theScaleElements.push_back(ScaleElement(firstValue,true));
						}
					}
				}
			}

			//8th section: maxSaturation
			if (dispMaxSat)
			{
				theCubeEquivalentDist.push_back(maxSaturation);
				theScaleElements.push_back(ScaleElement(maxSaturation,true,true));
			}

			//9th section: maxVal
			if (dispMaxVal)
			{
				theCubeEquivalentDist.push_back(maxDisp);
				theScaleElements.push_back(ScaleElement(maxDisp));
			}
		}
		else
		{
			//TODO
		}
	}

	if (theScaleElements.empty())
		return;

	//scale height
	unsigned n = theScaleElements.size();
	//assert(theCubeEquivalentDist.size()+(dispZero ? 1 : 0)==n);
	int scaleHeight = (cubeSize+2*spaceBetweenElements)*n;

	const int xShift = cubeSize+20;
	const int yShift = -40;

	//centered orthoprojective view (-halfW,-halfH,halfW,halfH)
	int halfW = (context.glW>>1);
	int halfH = (context.glH>>1);

	/*** now we can render the scale ***/

	//(x,y): current display area coordinates
	int x = halfW-xShift;
	int y = -(scaleHeight/2+yShift);

	//first horizontal delimiter
	glBegin(GL_LINES);
	glVertex2i(x,y);
	glVertex2i(x+cubeSize,y);
	glEnd();

	ccGLWindow* win = (ccGLWindow*)context._win;
	assert(win);

	QFont font = win->font();
	font.setPointSize(ccGui::Parameters().defaultFontSize);

	if (theScaleElements[0].textDisplayed)
		win->displayText(QString::number(theScaleElements[0].value, logScale ? 'E' : 'f', ccGui::Parameters().displayedNumPrecision), halfW+x-5, y+halfH, true, 0, font);

	const colorType* lineColor = ccColor::white;
	//clear background?
	if (ccGui::Parameters().backgroundCol[0] + ccGui::Parameters().backgroundCol[1] + ccGui::Parameters().backgroundCol[2] > 3*128)
		lineColor = ccColor::black;

	const colorType* col;
	for (i=0;i+1<(int)n;++i)
	{
		y += spaceBetweenElements;

		//a colored cube
		//d = 0.5*(theScaleElements[i].value + theScaleElements[i+1].value);
		DistanceType d = theCubeEquivalentDist[i];

		DistanceType normalizedDist = sf->normalize(d);
		if (normalizedDist<0.0)
			col = (context.greyForNanScalarValues ? ccColor::lightGrey : 0);
		else
			col = ccColorTablesManager::GetUniqueInstance()->getColor(normalizedDist,colorRampSteps,colorRampType);

		if (i==0 && theScaleElements[i].condensed)
		{
			//DOWN ARROW

			glBegin(GL_LINE_LOOP);
			glColor3ubv(lineColor);
			glVertex2i(x,y+cubeSize);
			glVertex2i(x+cubeSize,y+cubeSize);
			glVertex2i(x+cubeSize/2,y);
			glEnd();

			if (col)
			{
				glBegin(GL_POLYGON);
				glColor3ubv(col);
				glVertex2i(x,y+cubeSize);
				glVertex2i(x+cubeSize,y+cubeSize);
				glVertex2i(x+cubeSize/2,y);
				glEnd();
			}
		}
		else if (i+2 == (int)n && theScaleElements[i].condensed)
		{
			//UP ARROW

			glBegin(GL_LINE_LOOP);
			glColor3ubv(lineColor);
			glVertex2i(x,y);
			glVertex2i(x+cubeSize,y);
			glVertex2i(x+cubeSize/2,y+cubeSize);
			glEnd();

			if (col)
			{
				glBegin(GL_POLYGON);
				glColor3ubv(col);
				glVertex2i(x,y+1);
				glVertex2i(x+cubeSize,y+1);
				glVertex2i(x+cubeSize/2,y+cubeSize);
				glEnd();
			}
		}
		else //RECTANGLE
		{
			if (!theScaleElements[i].condensed)
			{
				//simple box
				if (col)
				{
					glBegin(GL_POLYGON);
					glColor3ubv(col);
					glVertex2i(x,y);
					glVertex2i(x+cubeSize,y);
					glVertex2i(x+cubeSize,y+cubeSize);
					glVertex2i(x,y+cubeSize);
					glEnd();
				}

				glBegin(GL_LINE_LOOP);
				glColor3ubv(lineColor);
				glVertex2i(x,y);
				glVertex2i(x+cubeSize,y);
				glVertex2i(x+cubeSize,y+cubeSize);
				glVertex2i(x,y+cubeSize);
				glEnd();
			}
			else
			{
				float third = (float)cubeSize *0.8/3.0f;
				//slashed box
				if (col)
				{
					glColor3ubv(col);

					glBegin(GL_POLYGON);
					glVertex2i(x,y);
					glVertex2f(x,(float)y+third);
					glVertex2f(x+cubeSize,(float)y+2.0f*third);
					glVertex2i(x+cubeSize,y);
					glEnd();

					glBegin(GL_POLYGON);
					glVertex2i(x,y+cubeSize);
					glVertex2i(x+cubeSize,y+cubeSize);
					glVertex2f(x+cubeSize,(float)(y+cubeSize)-third);
					glVertex2f(x,(float)(y+cubeSize)-2.0*third);
					glEnd();
				}

				glColor3ubv(lineColor);
				glBegin(GL_LINE_LOOP);
				glVertex2i(x,y);
				glVertex2f(x,(float)y+third);
				glVertex2f(x+cubeSize,(float)y+2.0f*third);
				glVertex2i(x+cubeSize,y);
				glEnd();

				glBegin(GL_LINE_LOOP);
				glVertex2i(x,y+cubeSize);
				glVertex2i(x+cubeSize,y+cubeSize);
				glVertex2f(x+cubeSize,(float)(y+cubeSize)-third);
				glVertex2f(x,(float)(y+cubeSize)-2.0*third);
				glEnd();
			}
		}

		y += cubeSize+spaceBetweenElements;

		//separator
		glColor3ubv(lineColor);
		glBegin(GL_LINES);
		glVertex2i(x,y);
		glVertex2i(x+cubeSize,y);
		glEnd();

		if (theScaleElements[i+1].textDisplayed)
		{
			DistanceType dispValue = theScaleElements[i+1].value;
			win->displayText(QString::number(dispValue,logScale ? 'E' : 'f',ccGui::Parameters().displayedNumPrecision), halfW+x-5, y+halfH, true, 0, font);
		}
	}

	//Scale title
	if (context.colorRampTitle[0]!=0)
	{
		QString sfTitle = QString("[")+QString(context.colorRampTitle)+QString("]");
		win->displayText(sfTitle, context.glW-cubeSize/2, (y+cubeSize)+halfH, true, 0, font);
	}

}
