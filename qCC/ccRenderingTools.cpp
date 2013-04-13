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

	const ccGBLSensor::DepthBuffer& depthBuffer = sensor->getDepthBuffer();
	if (!depthBuffer.zBuff)
		return;

	//determine min and max depths
	ScalarType minDist = 0, maxDist = 0;
	{
		const ScalarType *_zBuff = depthBuffer.zBuff;
		for (int x=0; x<depthBuffer.height*depthBuffer.width; ++x,++_zBuff)
		{
			if (x==0)
			{
				maxDist = minDist = *_zBuff;
			}
			else if (*_zBuff > 0)
			{
				maxDist = std::max(maxDist,*_zBuff);
				minDist = std::min(minDist,*_zBuff);
			}
		}
	}

	QImage bufferImage(depthBuffer.width,depthBuffer.height,QImage::Format_RGB32);
	{
		ccColorScale::Shared colorScale = ccColorScalesManager::GetDefaultScale();
		assert(colorScale);
		ScalarType coef = maxDist-minDist < ZERO_TOLERANCE ? 0.0 : (ScalarType)(ccColorScale::MAX_STEPS-1)/(maxDist-minDist);

		const ScalarType* _zBuff = depthBuffer.zBuff;
		for (int y=0;y<depthBuffer.height;++y)
		{
			for (int x=0;x<depthBuffer.width;++x,++_zBuff)
			{
				const colorType* col = (*_zBuff >= minDist ? colorScale->getColorByIndex(static_cast<unsigned>((*_zBuff-minDist)*coef)) : ccColor::black);
				bufferImage.setPixel(x,depthBuffer.height-1-y,qRgb(col[0],col[1],col[2]));
			}
		}
	}

	QDialog* dlg = new QDialog(parent);
	dlg->setWindowTitle(QString("%0 depth buffer [%1 x %2]").arg(sensor->getParent()->getName()).arg(depthBuffer.width).arg(depthBuffer.height));
	dlg->setFixedSize(bufferImage.size());
	QVBoxLayout* vboxLayout = new QVBoxLayout(dlg);
	vboxLayout->setContentsMargins(0,0,0,0);
	QLabel* label = new QLabel(dlg);
	label->setScaledContents(true);
	vboxLayout->addWidget(label);

	label->setPixmap(QPixmap::fromImage(bufferImage));
	dlg->show();
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context)
{
	const ccScalarField* sf = context.sfColorScaleToDisplay;
	if (!sf)
		return;

	ScalarType minVal = sf->getMin();
	ScalarType minDisplayed = sf->getMinDisplayed();
	ScalarType minSaturation = sf->getMinSaturation();
	ScalarType maxSaturation = sf->getMaxSaturation();
	ScalarType maxDisplayed = sf->getMaxDisplayed();
	ScalarType maxVal = sf->getMax();

	bool strictlyPositive = minVal >= 0;
	bool absSaturation = sf->absoluteSaturation();
	bool logScale = sf->logScale();

	const int c_cubeSize = ccGui::Parameters().colorScaleSquareSize;
	const int c_defaultSpace = 4;

	//this vector stores the values that will be "represented" by the scale
	//they will be automatically displayed in a regular "pace"
	std::vector<ScaleElement> theScaleElements;
	std::vector<ScalarType> theCubeEquivalentDist; //to deduce its color!

	int maxNumberOfCubes = (int)(floor((float)(context.glH-120)/(float)(c_cubeSize+2*c_defaultSpace)));

	ccColorScale::Shared colorScale = context.sfColorScaleToDisplay->getColorScale();
	if (!colorScale)
	{
		assert(false);
		return;
	}
	unsigned colorRampSteps = context.sfColorScaleToDisplay->getColorRampSteps();

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
		int numberOfCubes = std::min<int>(maxNumberOfCubes-addedCubes,colorRampSteps);

		ScalarType startValue = minVal; //we want it to be the same color as 'minVal' even if we start at '0'
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
			ScalarType endValue = (dispMaxSat ? maxSaturation : maxDisplayed);
			ScalarType intervale = (endValue-startValue)/(ScalarType)numberOfCubes;
			ScalarType firstValue = startValue;

			if (logScale)
			{
				ScalarType endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(endValue)));
				ScalarType startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(startValue)));
				intervale = (endValueLog-startValueLog)/(ScalarType)numberOfCubes;
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
					for (int i=0;i<numberOfCubes;++i)
					{
						ScalarType logVal = firstValue+intervale*0.5;
						theCubeEquivalentDist.push_back(exp(logVal*log(10.0)));
						firstValue += intervale;
						theScaleElements.push_back(ScaleElement(exp(firstValue*log(10.0)),true,false));
					}
				}
				else
				{
					for (int i=0;i<numberOfCubes;++i)
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
			ScalarType maxDisp = std::max(-minVal,maxVal);

			bool dispZero = true;
			bool dispMinSat = (minSaturation>0.0);
			bool dispMaxSat = (maxSaturation>minSaturation && maxSaturation<maxDisp);
			bool dispMaxVal = true;

			int addedCubes = 2 * (int(dispZero && dispMinSat) + int(dispMaxSat && dispMaxVal));

			//not enough room for display!
			if (maxNumberOfCubes < addedCubes)
				return;

			//number of cubes available for ramp display
			int numberOfCubes = std::min<int>((maxNumberOfCubes-addedCubes)/2,colorRampSteps);

			//1st section: -maxDisp
			ScalarType startValue = -maxDisp;
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
				ScalarType endValue = (dispMinSat ? -minSaturation : 0.0);
				ScalarType intervale = (endValue-startValue)/(ScalarType)numberOfCubes;
				ScalarType firstValue = startValue;

				if (logScale)
				{
					ScalarType endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(-endValue)));
					ScalarType startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(-startValue)));
					intervale = -(endValueLog-startValueLog)/(ScalarType)numberOfCubes;
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
						for (int i=0;i<numberOfCubes-1;++i)
						{
							ScalarType logVal = firstValue-intervale*0.5;
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
						for (int i=0;i<numberOfCubes-1;++i)
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
				ScalarType intervale = (maxSaturation-minSaturation)/(ScalarType)numberOfCubes;
				ScalarType firstValue = minSaturation;

				if (logScale)
				{
					ScalarType endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(maxSaturation)));
					ScalarType startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(minSaturation)));
					intervale = (endValueLog-startValueLog)/(ScalarType)numberOfCubes;
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
						for (int i=0;i<numberOfCubes-1;++i)
						{
							ScalarType logVal = firstValue+intervale*0.5;
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
						for (int i=0;i<numberOfCubes-1;++i)
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
	unsigned n = (unsigned)theScaleElements.size();
	//assert(theCubeEquivalentDist.size()+(dispZero ? 1 : 0)==n);
	int scaleHeight = (c_cubeSize+2*c_defaultSpace)*n;

	const int xShift = c_cubeSize+20;
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
	glVertex2i(x+c_cubeSize,y);
	glEnd();

	ccGLWindow* win = (ccGLWindow*)context._win;
	assert(win);

	if (theScaleElements[0].textDisplayed)
		win->displayText(QString::number(theScaleElements[0].value, logScale ? 'E' : 'f', ccGui::Parameters().displayedNumPrecision), halfW+x-5, y+halfH, ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VMIDDLE);

	const colorType* lineColor = ccColor::white;
	//clear background?
	if (ccGui::Parameters().backgroundCol[0] + ccGui::Parameters().backgroundCol[1] + ccGui::Parameters().backgroundCol[2] > 3*128)
		lineColor = ccColor::black;

	for (int i=0;i+1<(int)n;++i)
	{
		y += c_defaultSpace;

		//a colored cube
		//d = 0.5*(theScaleElements[i].value + theScaleElements[i+1].value);
		ScalarType d = theCubeEquivalentDist[i];

		ScalarType normalizedDist = sf->normalize(d);
		const colorType* col = 0;
		if (normalizedDist >= 0)
			col = colorScale->getColorByRelativePos(normalizedDist,colorRampSteps);
		else
			col = (sf->areNaNValuesShownInGrey()  ? ccColor::lightGrey : 0);

		if (i==0 && theScaleElements[i].condensed)
		{
			//DOWN ARROW

			glBegin(GL_LINE_LOOP);
			glColor3ubv(lineColor);
			glVertex2i(x,y+c_cubeSize);
			glVertex2i(x+c_cubeSize,y+c_cubeSize);
			glVertex2i(x+c_cubeSize/2,y);
			glEnd();

			if (col)
			{
				glBegin(GL_POLYGON);
				glColor3ubv(col);
				glVertex2i(x,y+c_cubeSize);
				glVertex2i(x+c_cubeSize,y+c_cubeSize);
				glVertex2i(x+c_cubeSize/2,y);
				glEnd();
			}
		}
		else if (i+2 == (int)n && theScaleElements[i].condensed)
		{
			//UP ARROW

			glBegin(GL_LINE_LOOP);
			glColor3ubv(lineColor);
			glVertex2i(x,y);
			glVertex2i(x+c_cubeSize,y);
			glVertex2i(x+c_cubeSize/2,y+c_cubeSize);
			glEnd();

			if (col)
			{
				glBegin(GL_POLYGON);
				glColor3ubv(col);
				glVertex2i(x,y+1);
				glVertex2i(x+c_cubeSize,y+1);
				glVertex2i(x+c_cubeSize/2,y+c_cubeSize);
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
					glVertex2i(x+c_cubeSize,y);
					glVertex2i(x+c_cubeSize,y+c_cubeSize);
					glVertex2i(x,y+c_cubeSize);
					glEnd();
				}

				glBegin(GL_LINE_LOOP);
				glColor3ubv(lineColor);
				glVertex2i(x,y);
				glVertex2i(x+c_cubeSize,y);
				glVertex2i(x+c_cubeSize,y+c_cubeSize);
				glVertex2i(x,y+c_cubeSize);
				glEnd();
			}
			else
			{
				float third = (float)c_cubeSize *0.8/3.0f;
				//slashed box
				if (col)
				{
					glColor3ubv(col);

					glBegin(GL_POLYGON);
					glVertex2i(x,y);
					glVertex2f(x,(float)y+third);
					glVertex2f(x+c_cubeSize,(float)y+2.0f*third);
					glVertex2i(x+c_cubeSize,y);
					glEnd();

					glBegin(GL_POLYGON);
					glVertex2i(x,y+c_cubeSize);
					glVertex2i(x+c_cubeSize,y+c_cubeSize);
					glVertex2f(x+c_cubeSize,(float)(y+c_cubeSize)-third);
					glVertex2f(x,(float)(y+c_cubeSize)-2.0*third);
					glEnd();
				}

				glColor3ubv(lineColor);
				glBegin(GL_LINE_LOOP);
				glVertex2i(x,y);
				glVertex2f(x,(float)y+third);
				glVertex2f(x+c_cubeSize,(float)y+2.0f*third);
				glVertex2i(x+c_cubeSize,y);
				glEnd();

				glBegin(GL_LINE_LOOP);
				glVertex2i(x,y+c_cubeSize);
				glVertex2i(x+c_cubeSize,y+c_cubeSize);
				glVertex2f(x+c_cubeSize,(float)(y+c_cubeSize)-third);
				glVertex2f(x,(float)(y+c_cubeSize)-2.0*third);
				glEnd();
			}
		}

		y += c_cubeSize+c_defaultSpace;

		//separator
		glColor3ubv(lineColor);
		glBegin(GL_LINES);
		glVertex2i(x,y);
		glVertex2i(x+c_cubeSize,y);
		glEnd();

		if (theScaleElements[i+1].textDisplayed)
		{
			ScalarType dispValue = theScaleElements[i+1].value;
			win->displayText(QString::number(dispValue,logScale ? 'E' : 'f',ccGui::Parameters().displayedNumPrecision), halfW+x-5, y+halfH, ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VMIDDLE);
		}
	}

	//Scale title
	const char* sfName = context.sfColorScaleToDisplay->getName();
	if (sfName)
	{
		//QString sfTitle = QString("[%1]").arg(sfName);
		QString sfTitle(sfName);
		win->displayText(sfTitle, context.glW-c_cubeSize/2, (y+c_cubeSize)+halfH, ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VTOP);
	}

}
