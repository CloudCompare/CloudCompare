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

//! Graphical scale atomical element
struct ScaleElement
{
	//! Starting value
	double value;
	//! Specifies whether the value should be displayed
	bool textDisplayed;
	//! Specifies whether the cube is condensed or not
	bool condensed;

	//! Default constructor
	ScaleElement(double val, bool dispText = true, bool isCondensed = false)
		: value(val)
		, textDisplayed(dispText)
		, condensed(isCondensed)
	{
	}
};


//structure for recursive display of labels
struct vlabel
{
	int yPos; 		/**< label center pos **/
	int yMin; 		/**< label 'ROI' min **/
	int yMax; 		/**< label 'ROI' max **/
	double val; 	/**< label value **/

	//default constructor
	vlabel(int y, int y1, int y2, double v) : yPos(y), yMin(y1), yMax(y2),val(v) { assert(y2>=y1); }
};

//! A set of 'vlabel' structures
typedef std::list<vlabel> vlabelSet;

//helper: returns the neighbouring labels at a given position
//(first: above label, second: below label)
//Warning: set must be already sorted!
typedef std::pair<vlabelSet::iterator,vlabelSet::iterator> vlabelPair;
static vlabelPair GetVLabelsAround(int y, vlabelSet& set)
{
	if (set.size()==0)
	{
		return vlabelPair(set.end(),set.end());
	}
	else
	{
		vlabelSet::iterator it1 = set.begin();
		if (y < it1->yPos)
			return vlabelPair(set.end(),it1);
		vlabelSet::iterator it2 = it1; it2++;
		for (; it2 != set.end(); ++it2, ++it1)
		{
			if (y <= it2->yPos) // '<=' to make sure the last label stays at the top!
				return vlabelPair(it1,it2);
		}
		return vlabelPair(it1,set.end());
	}
}

//! For log scale inversion
const double c_log10 = log(10.0);

//! Convert standard range to log scale
void ConvertToLogScale(ScalarType& dispMin, ScalarType& dispMax)
{
	ScalarType absDispMin = ( dispMax < 0 ? std::min(-dispMax,-dispMin) : std::max<ScalarType>(dispMin,0) );
	ScalarType absDispMax = std::max(fabs(dispMin),fabs(dispMax));
	dispMin = log10(std::max(absDispMin,(ScalarType)ZERO_TOLERANCE));
	dispMax = log10(std::max(absDispMax,(ScalarType)ZERO_TOLERANCE));
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context)
{
	const ccScalarField* sf = context.sfColorScaleToDisplay;
	if (!sf || !sf->getColorScale())
		return;

//#define USE_OLD_SCALE_RENDERING
#ifndef USE_OLD_SCALE_RENDERING

	ccGLWindow* win = static_cast<ccGLWindow*>(context._win);
	if (!win)
		return;

	bool logScale = sf->logScale();
	bool symmetricalScale = sf->symmetricalScale();
	bool alwaysShowZero = sf->isZeroAlwaysShown();
	
	//set of particular values
	//DGM: we work with doubles for maximum accuracy
	std::set<double> keyValues;
	if (!logScale)
	{
		keyValues.insert(sf->displayRange().min());
		keyValues.insert(sf->displayRange().start());
		keyValues.insert(sf->displayRange().stop());
		keyValues.insert(sf->displayRange().max());
		keyValues.insert(sf->saturationRange().min());
		keyValues.insert(sf->saturationRange().start());
		keyValues.insert(sf->saturationRange().stop());
		keyValues.insert(sf->saturationRange().max());

		if (symmetricalScale)
			keyValues.insert(-sf->saturationRange().max());

		if (alwaysShowZero)
			keyValues.insert(0.0);
	}
	else
	{
		ScalarType minDisp = sf->displayRange().min();
		ScalarType maxDisp = sf->displayRange().max();
		ConvertToLogScale(minDisp,maxDisp);
		keyValues.insert(minDisp);
		keyValues.insert(maxDisp);

		ScalarType startDisp = sf->displayRange().start();
		ScalarType stopDisp = sf->displayRange().stop();
		ConvertToLogScale(startDisp,stopDisp);
		keyValues.insert(startDisp);
		keyValues.insert(stopDisp);

		keyValues.insert(sf->saturationRange().min());
		keyValues.insert(sf->saturationRange().start());
		keyValues.insert(sf->saturationRange().stop());
		keyValues.insert(sf->saturationRange().max());
	}

	//Internally, the elements in a set are already sorted
	//std::sort(keyValues.begin(),keyValues.end());

	if (!sf->areNaNValuesShownInGrey())
	{
		//remove 'hidden' values
		if (!logScale)
		{
			for (std::set<double>::iterator it = keyValues.begin(); it != keyValues.end(); )
			{
				if (!sf->displayRange().isInRange(static_cast<ScalarType>(*it)) && (!alwaysShowZero || *it != 0)) //we keep zero if the user has explicitely asked for it!
				{
					std::set<double>::iterator toDelete = it;
					++it;
					keyValues.erase(toDelete);
				}
				else
				{
					++it;
				}
		}
      }
		else
		{
			//convert actual display range to log scale
			//(we can't do the opposite, otherwise we get accuracy/round-off issues!)
			ScalarType dispMin = sf->displayRange().start();
			ScalarType dispMax = sf->displayRange().stop();
			ConvertToLogScale(dispMin,dispMax);

			for (std::set<double>::iterator it = keyValues.begin(); it != keyValues.end(); )
			{
				if (*it >= dispMin && *it <= dispMax)
				{
					++it;
				}
				else
				{
					std::set<double>::iterator toDelete = it;
					++it;
					keyValues.erase(toDelete);
				}
			}
		}
	}

	//Font metrics for proper display of labels!
	QFontMetrics strMetrics(win->font());

	//default color: text color
	const unsigned char* textColor = ccGui::Parameters().textDefaultCol;

	//histogram?
	const::ccScalarField::Histogram histogram = sf->getHistogram();
	bool showHistogram = (ccGui::Parameters().colorScaleShowHistogram && !logScale && histogram.maxValue != 0 && histogram.size() > 1);

	//display area
	const int strHeight = strMetrics.height();
	const int scaleWidth = ccGui::Parameters().colorScaleRampWidth;
	const int scaleMaxHeight = (keyValues.size() > 1 ? std::max(context.glH-140,2*strHeight) : scaleWidth); //if 1 value --> we draw a cube

	//centered orthoprojective view (-halfW,-halfH,halfW,halfH)
	int halfW = (context.glW>>1);
	int halfH = (context.glH>>1);

	//top-right corner of the scale ramp
	const int xShift = 20 + (showHistogram ? scaleWidth/2 : 0);
	const int yShift = halfH-scaleMaxHeight/2-10;

	glPushAttrib(GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_LINE_SMOOTH);
	glDisable(GL_DEPTH_TEST);
	
	std::vector<double> sortedKeyValues(keyValues.begin(),keyValues.end());
	double maxRange = sortedKeyValues.back()-sortedKeyValues.front();

	//const colorType* lineColor = ccColor::white;
	////clear background?
	//if (ccGui::Parameters().backgroundCol[0] + ccGui::Parameters().backgroundCol[1] + ccGui::Parameters().backgroundCol[2] > 3*128)
	//	lineColor = ccColor::black;
	const colorType* lineColor = textColor;

	//display color ramp
	{
		//(x,y): current display area coordinates (top-left corner)
		int x = halfW-xShift-scaleWidth;
		int y = halfH-yShift-scaleMaxHeight;

		if (keyValues.size() > 1)
		{
			int histoStart = x+scaleWidth+std::min(std::max(scaleWidth/8,3),15);

			glLineWidth(1.0f);
			glBegin(GL_LINES);
			for (int j=0; j<scaleMaxHeight; ++j)
			{
				double value = sortedKeyValues.front() + ((double)j * maxRange) / (double)scaleMaxHeight;
				if (logScale)
					value = exp(value*c_log10);
				const colorType* col = sf->getColor(static_cast<ScalarType>(value));
				glColor3ubv(col ? col : ccColor::lightGrey);

				glVertex2i(x,y+j);
				glVertex2i(x+scaleWidth,y+j);
				
				if (showHistogram)
				{
					double bind = (value-(double)sf->displayRange().min())*(double)(histogram.size()-1)/(double)sf->displayRange().maxRange();
					int bin = static_cast<int>(floor(bind));
					
					double hVal = 0.0;
					if (bin >= 0 && bin < (int)histogram.size()) //in symmetrical case we can get values outside of the real SF range
					{
						hVal = (double)histogram[bin];
						if (bin+1 < (int)histogram.size())
						{
							//linear interpolation
							double alpha = bind-(double)bin;
							hVal = (1.0-alpha) * hVal + alpha * (double)histogram[bin+1];
						}
					}

					int xSpan = std::max(static_cast<int>(hVal / (double)histogram.maxValue * (double)(scaleWidth/2)),1);
					glVertex2i(histoStart,y+j);
					glVertex2i(histoStart+xSpan,y+j);
				}
			}
			glEnd();
		}
		else
		{
			//if there's a unique (visible) scalar value, we only draw a square!
			double value = sortedKeyValues.front();
			if (logScale)
				value = exp(value*c_log10);
			const colorType* col = sf->getColor(value);
			glColor3ubv(col ? col : ccColor::lightGrey);
			glBegin(GL_POLYGON);
			glVertex2i(x,y);
			glVertex2i(x+scaleWidth,y);
			glVertex2i(x+scaleWidth,y+scaleMaxHeight-1);
			glVertex2i(x,y+scaleMaxHeight-1);
			glEnd();
		}

		//scale border
		glLineWidth(2.0);
		glColor3ubv(lineColor);
		glBegin(GL_LINE_LOOP);
		glVertex2i(x,y);
		glVertex2i(x+scaleWidth,y);
		glVertex2i(x+scaleWidth,y+scaleMaxHeight);
		glVertex2i(x,y+scaleMaxHeight);
		glEnd();
	}

	//display labels
	{
		//list of labels to draw
		vlabelSet drawnLabels;

		//add first label
		drawnLabels.push_back(vlabel(0,0,strHeight,sortedKeyValues.front()));

		if (keyValues.size() > 1)
		{
			//add last label
			drawnLabels.push_back(vlabel(scaleMaxHeight,scaleMaxHeight-strHeight,scaleMaxHeight,sortedKeyValues.back()));
		}

		//we try to display the other keyPoints (if any)
		if (keyValues.size() > 2)
		{
			assert(maxRange > 0.0);
			const int minGap = strHeight;
			for (size_t i=1; i<keyValues.size()-1; ++i)
			{
				int yScale = static_cast<int>((sortedKeyValues[i]-sortedKeyValues[0]) * (double)scaleMaxHeight / maxRange);
				vlabelPair nLabels = GetVLabelsAround(yScale,drawnLabels);

				assert(nLabels.first != drawnLabels.end() && nLabels.second != drawnLabels.end());
				if (	(nLabels.first == drawnLabels.end() || nLabels.first->yMax <= yScale - minGap)
					&&	(nLabels.second == drawnLabels.end() || nLabels.second->yMin >= yScale + minGap))
				{
					//insert it at the right place (so as to keep a sorted list!)
					drawnLabels.insert(nLabels.second,vlabel(yScale,yScale-strHeight/2,yScale+strHeight/2,sortedKeyValues[i]));
				}
			}
		}

		//now we recursively display labels where we have some rool left
		if (drawnLabels.size() > 1)
		{
			const int minGap = strHeight*2;

			size_t drawnLabelsBefore = 0; //just to init the loop
			size_t drawnLabelsAfter = drawnLabels.size(); 

			//proceed until no more label can be inserted
			while (drawnLabelsAfter > drawnLabelsBefore)
			{
				drawnLabelsBefore = drawnLabelsAfter;

				vlabelSet::iterator it1 = drawnLabels.begin();
				vlabelSet::iterator it2 = it1; it2++;
				for (; it2 != drawnLabels.end(); ++it2)
				{
					if (it1->yMax + 2*minGap < it2->yMin)
					{
						//insert label
						double val = (it1->val + it2->val)/2.0;
						int yScale = static_cast<int>((val-sortedKeyValues[0]) * (double)scaleMaxHeight / maxRange);

						//insert it at the right place (so as to keep a sorted list!)
						drawnLabels.insert(it2,vlabel(yScale,yScale-strHeight/2,yScale+strHeight/2,val));
					}
					it1 = it2;
				}

				drawnLabelsAfter = drawnLabels.size();
			}
		}

		//display labels

		//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
		glColor3f((float)textColor[0]/255.0f,(float)textColor[1]/255.0f,(float)textColor[2]/255.0f);

		//Scalar field name
		const char* sfName = sf->getName();
		if (sfName)
		{
			//QString sfTitle = QString("[%1]").arg(sfName);
			QString sfTitle(sfName);
			if (logScale)
				sfTitle += QString("[Log scale]");
			//we leave some (vertical) space for the top-most label!
			win->displayText(sfTitle, context.glW-xShift, context.glH-yShift+strMetrics.height(), ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VTOP);
		}

		//precision (same as color scale)
		const unsigned precision = ccGui::Parameters().displayedNumPrecision;
		//format
		const char format = (sf->logScale() ? 'E' : 'f');
		//tick
		const int tickSize = 4;

		//for labels
		const int x = context.glW-xShift-scaleWidth-2*tickSize-1;
		const int y = context.glH-yShift-scaleMaxHeight;
		//for ticks
		const int xTick = halfW-xShift-scaleWidth-tickSize-1;
		const int yTick = halfH-yShift-scaleMaxHeight;

		for (vlabelSet::iterator it = drawnLabels.begin(); it != drawnLabels.end(); ++it)
		{
			vlabelSet::iterator itNext = it; itNext++;
			//position
			unsigned char align = ccGLWindow::ALIGN_HRIGHT;
			if (it == drawnLabels.begin())
				align |= ccGLWindow::ALIGN_VTOP;
			else if (itNext == drawnLabels.end())
				align |= ccGLWindow::ALIGN_VBOTTOM;
			else
				align |= ccGLWindow::ALIGN_VMIDDLE;

			double value = it->val;
			if (logScale)
				value = exp(value*c_log10);

			win->displayText(QString::number(value,format,precision), x, y+it->yPos, align);
			glBegin(GL_LINES);
			glVertex2f(xTick,yTick+it->yPos);
			glVertex2f(xTick+tickSize,yTick+it->yPos);
			glEnd();
		}
	}

	glPopAttrib();

#else

	double minVal = sf->getMin();
	double minDisplayed = sf->displayRange().start();
	double minSaturation = sf->saturationRange().start();
	double maxSaturation = sf->saturationRange().stop();
	double maxDisplayed = sf->displayRange().stop();
	double maxVal = sf->getMax();

	bool strictlyPositive = (minVal >= 0);
	bool symmetricalScale = sf->symmetricalScale();
	bool logScale = sf->logScale();

	const int c_cubeSize = ccGui::Parameters().colorScaleRampWidth;
	const int c_defaultSpace = 4;

	//this vector stores the values that will be "represented" by the scale
	//they will be automatically displayed in a regular "pace"
	std::vector<ScaleElement> theScaleElements;
	std::vector<double> theCubeEquivalentDist; //to deduce its color!

	int maxNumberOfCubes = (int)(floor((float)(context.glH-120)/(float)(c_cubeSize+2*c_defaultSpace)));

	ccColorScale::Shared colorScale = context.sfColorScaleToDisplay->getColorScale();
	if (!colorScale)
	{
		assert(false);
		return;
	}
	unsigned colorRampSteps = context.sfColorScaleToDisplay->getColorRampSteps();

	//first we fill the two vectors below with scale "values"
	if (strictlyPositive || !symmetricalScale) //only positive values
	{
		bool dispZero = minDisplayed>0.0 && strictlyPositive;
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

		double startValue = minVal; //we want it to be the same color as 'minVal' even if we start at '0'
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
			double endValue = (dispMaxSat ? maxSaturation : maxDisplayed);
			double intervale = (endValue-startValue)/(double)numberOfCubes;
			double firstValue = startValue;

			if (logScale)
			{
				double endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(endValue)));
				double startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(startValue)));
				intervale = (endValueLog-startValueLog)/(double)numberOfCubes;
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
						double val = firstValue+intervale*static_cast<double>(i);
						double logVal = val+intervale*0.5;
						theCubeEquivalentDist.push_back(exp(logVal*log(10.0)));

						theScaleElements.push_back(ScaleElement(exp((val+intervale)*log(10.0)),true,false));
					}
				}
				else
				{
					for (int i=0;i<numberOfCubes;++i)
					{
						double val = firstValue+intervale*static_cast<double>(i);

						theCubeEquivalentDist.push_back(val+intervale*0.5);
						theScaleElements.push_back(ScaleElement(val+intervale,true,false));
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
			double maxDisp = std::max(-minVal,maxVal);

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
			double startValue = -maxDisp;
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
				double endValue = (dispMinSat ? -minSaturation : 0.0);
				double intervale = (endValue-startValue)/(double)numberOfCubes;
				double firstValue = startValue;

				if (logScale)
				{
					double endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(-endValue)));
					double startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(-startValue)));
					intervale = -(endValueLog-startValueLog)/(double)numberOfCubes;
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
							double logVal = firstValue-intervale*0.5;
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
				double intervale = (maxSaturation-minSaturation)/(double)numberOfCubes;
				double firstValue = minSaturation;

				if (logScale)
				{
					double endValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(maxSaturation)));
					double startValueLog = log10(std::max<double>(ZERO_TOLERANCE,fabs(minSaturation)));
					intervale = (endValueLog-startValueLog)/(double)numberOfCubes;
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
							double logVal = firstValue+intervale*0.5;
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
	const int yShift = 40;

	//centered orthoprojective view (-halfW,-halfH,halfW,halfH)
	int halfW = (context.glW>>1);
	int halfH = (context.glH>>1);

	/*** now we can render the scale ***/

	//(x,y): current display area coordinates
	int x = halfW-xShift;
	int y = yShift-scaleHeight/2;

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
		double d = theCubeEquivalentDist[i];

		const colorType* col = sf->getColor(d);

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
			double dispValue = theScaleElements[i+1].value;
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

#endif
}
