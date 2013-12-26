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
		ScalarType coef = maxDist-minDist < ZERO_TOLERANCE ? 0 : static_cast<ScalarType>(ccColorScale::MAX_STEPS-1)/(maxDist-minDist);

		const ScalarType* _zBuff = depthBuffer.zBuff;
		for (int y=0; y<depthBuffer.height; ++y)
		{
			for (int x=0; x<depthBuffer.width; ++x,++_zBuff)
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
	ccGLWindow* win = static_cast<ccGLWindow*>(context._win);

	DrawColorRamp(sf,win,context.glW,context.glH,context.renderZoom);
}

void ccRenderingTools::DrawColorRamp(const ccScalarField* sf, ccGLWindow* win, int glW, int glH, float renderZoom/*=1.0f*/)
{
	if (!sf || !sf->getColorScale())
		return;

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

	const ccGui::ParamStruct& displayParams = win->getDisplayParameters();

	//default color: text color
	const unsigned char* textColor = displayParams.textDefaultCol;

	//histogram?
	const::ccScalarField::Histogram histogram = sf->getHistogram();
	bool showHistogram = (displayParams.colorScaleShowHistogram && !logScale && histogram.maxValue != 0 && histogram.size() > 1);

	//display area
	QFont font = win->getTextDisplayFont(); //takes rendering zoom into account!
	const int strHeight = static_cast<int>(displayParams.defaultFontSize * renderZoom); //QFontMetrics(font).height() --> always returns the same value?!
	const int scaleWidth = static_cast<int>(displayParams.colorScaleRampWidth * renderZoom);
	const int scaleMaxHeight = (keyValues.size() > 1 ? std::max(glH-static_cast<int>(140 * renderZoom), 2*strHeight) : scaleWidth); //if 1 value --> we draw a cube

	//centered orthoprojective view (-halfW,-halfH,halfW,halfH)
	int halfW = (glW>>1);
	int halfH = (glH>>1);

	//top-right corner of the scale ramp
	const int xShift = static_cast<int>(20 * renderZoom) + (showHistogram ? scaleWidth/2 : 0);
	const int yShift = halfH-scaleMaxHeight/2 - static_cast<int>(10 * renderZoom);

	glPushAttrib(GL_LINE_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_LINE_SMOOTH);
	glDisable(GL_DEPTH_TEST);
	
	std::vector<double> sortedKeyValues(keyValues.begin(),keyValues.end());
	double maxRange = sortedKeyValues.back()-sortedKeyValues.front();

	//const colorType* lineColor = ccColor::white;
	////clear background?
	//if (displayParams.backgroundCol[0] + displayParams.backgroundCol[1] + displayParams.backgroundCol[2] > 3*128)
	//	lineColor = ccColor::black;
	const colorType* lineColor = textColor;

	//display color ramp
	{
		//(x,y): current display area coordinates (top-left corner)
		int x = halfW-xShift-scaleWidth;
		int y = halfH-yShift-scaleMaxHeight;

		if (keyValues.size() > 1)
		{
			int histoStart = x+scaleWidth+std::min(std::max(scaleWidth/8,3),static_cast<int>(15 * renderZoom));

			glLineWidth(1.0f * renderZoom);
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
			const colorType* col = sf->getColor(static_cast<ScalarType>(value));
			glColor3ubv(col ? col : ccColor::lightGrey);
			glBegin(GL_POLYGON);
			glVertex2i(x,y);
			glVertex2i(x+scaleWidth,y);
			glVertex2i(x+scaleWidth,y+scaleMaxHeight-1);
			glVertex2i(x,y+scaleMaxHeight-1);
			glEnd();
		}

		//scale border
		glLineWidth(2.0f * renderZoom);
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
			win->displayText(sfTitle, glW-xShift, glH-yShift+strHeight, ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VTOP, 0, 0, &font);
		}

		//precision (same as color scale)
		const unsigned precision = displayParams.displayedNumPrecision;
		//format
		const char format = (sf->logScale() ? 'E' : 'f');
		//tick
		const int tickSize = static_cast<int>(4 * renderZoom);

		//for labels
		const int x = glW-xShift-scaleWidth-2*tickSize-1;
		const int y = glH-yShift-scaleMaxHeight;
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

			win->displayText(QString::number(value,format,precision), x, y+it->yPos, align, 0, 0, &font);
			glBegin(GL_LINES);
			glVertex2i(xTick,yTick+it->yPos);
			glVertex2i(xTick+tickSize,yTick+it->yPos);
			glEnd();
		}
	}

	glPopAttrib();
}
