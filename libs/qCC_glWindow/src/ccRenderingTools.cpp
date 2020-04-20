//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccRenderingTools.h"

//qCC
#include "ccGLWindow.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccGBLSensor.h>
#include <ccGenericPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>

#include <numeric>

void ccRenderingTools::ShowDepthBuffer(ccGBLSensor* sensor, QWidget* parent/*=0*/, unsigned maxDim/*=1024*/)
{
	if (!sensor)
		return;

	const ccDepthBuffer& depthBuffer = sensor->getDepthBuffer();
	if (depthBuffer.zBuff.empty())
	{
		return;
	}

	//determine min and max depths
	PointCoordinateType minDist = 0.0f;
	PointCoordinateType maxDist = 0.0f;
	{
		const PointCoordinateType* _zBuff = depthBuffer.zBuff.data();
		double sumDist = 0.0;
		double sumDist2 = 0.0;
		unsigned count = 0;
		for (unsigned x = 0; x < depthBuffer.height*depthBuffer.width; ++x, ++_zBuff)
		{
			if (x == 0)
			{
				maxDist = minDist = *_zBuff;
			}
			else if (*_zBuff > 0)
			{
				maxDist = std::max(maxDist, *_zBuff);
				minDist = std::min(minDist, *_zBuff);
			}

			if (*_zBuff > 0)
			{
				sumDist += *_zBuff;
				sumDist2 += *_zBuff * *_zBuff;
				++count;
			}
		}

		if (count)
		{
			double avg = sumDist / count;
			double stdDev = sqrt(fabs(sumDist2 / count - avg*avg));
			//for better dynamics
			maxDist = std::min(maxDist, static_cast<PointCoordinateType>(avg + 1.0 * stdDev));
		}
	}

	QImage bufferImage(depthBuffer.width, depthBuffer.height, QImage::Format_RGB32);
	{
		ccColorScale::Shared colorScale = ccColorScalesManager::GetDefaultScale();
		assert(colorScale);
		ScalarType coef = maxDist - minDist < ZERO_TOLERANCE ? 0 : static_cast<ScalarType>(ccColorScale::MAX_STEPS - 1) / (maxDist - minDist);

		const PointCoordinateType* _zBuff = depthBuffer.zBuff.data();
		for (unsigned y = 0; y < depthBuffer.height; ++y)
		{
			for (unsigned x = 0; x < depthBuffer.width; ++x, ++_zBuff)
			{
				const ccColor::Rgb& col = (*_zBuff >= minDist ? colorScale->getColorByIndex(static_cast<unsigned>((std::min(maxDist, *_zBuff) - minDist)*coef)) : ccColor::black);
				bufferImage.setPixel(x, depthBuffer.height - 1 - y, qRgb(col.r, col.g, col.b));
			}
		}
	}

	QDialog* dlg = new QDialog(parent);
	dlg->setWindowTitle(QString("%0 depth buffer [%1 x %2]").arg(sensor->getParent()->getName()).arg(depthBuffer.width).arg(depthBuffer.height));

	unsigned maxDBDim = std::max<unsigned>(depthBuffer.width, depthBuffer.height);
	unsigned scale = 1;
	while (maxDBDim > maxDim)
	{
		maxDBDim >>= 1;
		scale <<= 1;
	}
	dlg->setFixedSize(bufferImage.size()/scale);

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
using vlabelSet = std::list<vlabel>;

//helper: returns the neighbouring labels at a given position
//(first: above label, second: below label)
//Warning: set must be already sorted!
using vlabelPair = std::pair<vlabelSet::iterator,vlabelSet::iterator>;

static vlabelPair GetVLabelsAround(int y, vlabelSet& set)
{
	if (set.empty())
	{
		return vlabelPair(set.end(), set.end());
	}
	else
	{
		vlabelSet::iterator it1 = set.begin();
		if (y < it1->yPos)
		{
			return vlabelPair(set.end(), it1);
		}
		vlabelSet::iterator it2 = it1; ++it2;
		for (; it2 != set.end(); ++it2, ++it1)
		{
			if (y <= it2->yPos) // '<=' to make sure the last label stays at the top!
				return vlabelPair(it1,it2);
		}
		return vlabelPair(it1,set.end());
	}
}

//For log scale inversion
const double c_log10 = log(10.0);

//Convert standard range to log scale
void ConvertToLogScale(ScalarType& dispMin, ScalarType& dispMax)
{
	ScalarType absDispMin = (dispMax < 0 ? std::min(-dispMax, -dispMin) : std::max<ScalarType>(dispMin, 0)); 
	ScalarType absDispMax = std::max(std::abs(dispMin), std::abs(dispMax));
	dispMin = std::log10(std::max(absDispMin, std::numeric_limits<ScalarType>::epsilon()));
	dispMax = std::log10(std::max(absDispMax, std::numeric_limits<ScalarType>::epsilon()));
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context)
{
	const ccScalarField* sf = context.sfColorScaleToDisplay;
	ccGLWindow* display = static_cast<ccGLWindow*>(context.display);

	DrawColorRamp(context, sf, display, context.glW, context.glH, context.renderZoom);
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context, const ccScalarField* sf, ccGLWindow* win, int glW, int glH, float renderZoom/*=1.0f*/)
{
	if (!sf || !sf->getColorScale() || !win)
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	bool logScale = sf->logScale();
	bool symmetricalScale = sf->symmetricalScale();
	bool alwaysShowZero = sf->isZeroAlwaysShown();

	//set of particular values
	//DGM: we work with doubles for maximum accuracy
	ccColorScale::LabelSet keyValues;
	bool customLabels = false;
	try
	{
		ccColorScale::Shared colorScale = sf->getColorScale();
		if (colorScale && colorScale->customLabels().size() >= 2)
		{
			keyValues = colorScale->customLabels();

			if (alwaysShowZero)
			{
				keyValues.insert(0.0);
			}

			customLabels = true;
		}
		else if (!logScale)
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
			ConvertToLogScale(minDisp, maxDisp);
			keyValues.insert(minDisp);
			keyValues.insert(maxDisp);

			ScalarType startDisp = sf->displayRange().start();
			ScalarType stopDisp = sf->displayRange().stop();
			ConvertToLogScale(startDisp, stopDisp);
			keyValues.insert(startDisp);
			keyValues.insert(stopDisp);

			keyValues.insert(sf->saturationRange().min());
			keyValues.insert(sf->saturationRange().start());
			keyValues.insert(sf->saturationRange().stop());
			keyValues.insert(sf->saturationRange().max());
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return;
	}

	//magic fix (for infinite values!)
	{
		for (ccColorScale::LabelSet::iterator it = keyValues.begin(); it != keyValues.end(); ++it)
		{
#if defined(CC_WINDOWS) && defined(_MSC_VER)
			if (!_finite(*it))
#else
			if (!std::isfinite(*it))
#endif
			{
				bool minusInf = (*it < 0);
				keyValues.erase(it);
				if (minusInf)
					keyValues.insert(-std::numeric_limits<ScalarType>::max());
				else
					keyValues.insert(std::numeric_limits<ScalarType>::max());
				it = keyValues.begin(); //restart the process (easier than trying to be intelligent here ;)
			}
		}
	}

	//Internally, the elements in a set are already sorted
	//std::sort(keyValues.begin(), keyValues.end());

	if (!customLabels && !sf->areNaNValuesShownInGrey())
	{
		//remove 'hidden' values
		if (!logScale)
		{
			for (ccColorScale::LabelSet::iterator it = keyValues.begin(); it != keyValues.end(); )
			{
				if (!sf->displayRange().isInRange(static_cast<ScalarType>(*it)) && (!alwaysShowZero || *it != 0)) //we keep zero if the user has explicitely asked for it!
				{
					ccColorScale::LabelSet::iterator toDelete = it;
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
			ConvertToLogScale(dispMin, dispMax);

			for (ccColorScale::LabelSet::iterator it = keyValues.begin(); it != keyValues.end(); )
			{
				if (*it >= dispMin && *it <= dispMax)
				{
					++it;
				}
				else
				{
					ccColorScale::LabelSet::iterator toDelete = it;
					++it;
					keyValues.erase(toDelete);
				}
			}
		}
	}

	const ccGui::ParamStruct& displayParams = win->getDisplayParameters();

	//default color: text color
	const ccColor::Rgbub& textColor = displayParams.textDefaultCol;

	//histogram?
	const ccScalarField::Histogram histogram = sf->getHistogram();
	bool showHistogram = (displayParams.colorScaleShowHistogram && !logScale && histogram.maxValue != 0 && histogram.size() > 1);

	//display area
	QFont font = win->getTextDisplayFont(); //takes rendering zoom into account!
	const int strHeight = static_cast<int>(displayParams.defaultFontSize * renderZoom); //QFontMetrics(font).height() --> always returns the same value?!
	const int scaleWidth = static_cast<int>(displayParams.colorScaleRampWidth * renderZoom);
	const int scaleMaxHeight = (keyValues.size() > 1 ? std::max(glH - static_cast<int>(140 * renderZoom), 2 * strHeight) : scaleWidth); //if 1 value --> we draw a cube

	//centered orthoprojective view (-halfW,-halfH,halfW,halfH)
	int halfW = (glW >> 1);
	int halfH = (glH >> 1);

	//top-right corner of the scale ramp
	const int xShift = static_cast<int>(20 * renderZoom) + (showHistogram ? scaleWidth / 2 : 0);
	const int yShift = halfH - scaleMaxHeight / 2 - static_cast<int>(10 * renderZoom);

	glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
	glFunc->glDisable(GL_DEPTH_TEST);

	std::vector<double> sortedKeyValues(keyValues.begin(), keyValues.end());
	double maxRange = sortedKeyValues.back() - sortedKeyValues.front();

	//display color ramp
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(renderZoom);

		//(x,y): current display area coordinates (top-left corner)
		int x = halfW-xShift-scaleWidth;
		int y = halfH-yShift-scaleMaxHeight;

		if (keyValues.size() > 1)
		{
			int histoStart = x + scaleWidth + std::min(std::max(scaleWidth / 8, 3), static_cast<int>(15 * renderZoom));

			glFunc->glBegin(GL_LINES);
			for (int j=0; j<scaleMaxHeight; ++j)
			{
				double baseValue = sortedKeyValues.front() + (j * maxRange) / scaleMaxHeight;
				double value = baseValue;
				if (logScale)
				{
					value = exp(value*c_log10);
				}
				const ccColor::Rgb* col = sf->getColor(static_cast<ScalarType>(value));
				if (!col)
				{
					//special case: if we have user-defined labels, we want all the labels to be displayed with their associated color
					if (customLabels)
					{
						assert(sf->getColorScale() && !sf->getColorScale()->isRelative());
						col = sf->getColorScale()->getColorByValue(value, &ccColor::lightGreyRGB);
					}
					else
					{
						col = &ccColor::lightGreyRGB;
					}
				}
				assert(col);
				glFunc->glColor3ubv(col->rgb);

				glFunc->glVertex2i(x, y + j);
				glFunc->glVertex2i(x + scaleWidth, y + j);

				if (showHistogram)
				{
					double bind = (value - sf->displayRange().min())*(histogram.size() - 1) / sf->displayRange().maxRange();
					int bin = static_cast<int>(floor(bind));

					double hVal = 0.0;
					if (bin >= 0 && bin < static_cast<int>(histogram.size())) //in symmetrical case we can get values outside of the real SF range
					{
						hVal = histogram[bin];
						if (bin + 1 < static_cast<int>(histogram.size()))
						{
							//linear interpolation
							double alpha = bind - static_cast<double>(bin);
							hVal = (1.0 - alpha) * hVal + alpha * histogram[bin + 1];
						}
					}

					int xSpan = std::max(static_cast<int>(hVal / histogram.maxValue * (scaleWidth / 2)), 1);
					glFunc->glVertex2i(histoStart, y + j);
					glFunc->glVertex2i(histoStart + xSpan, y + j);
				}
			}
			glFunc->glEnd();
		}
		else
		{
			//if there's a unique (visible) scalar value, we only draw a square!
			double value = sortedKeyValues.front();
			if (logScale)
				value = exp(value*c_log10);
			const ccColor::Rgb* col = sf->getColor(static_cast<ScalarType>(value));
			glFunc->glColor3ubv(col ? col->rgb : ccColor::lightGreyRGB.rgb);
			glFunc->glBegin(GL_POLYGON);
			glFunc->glVertex2i(x,y);
			glFunc->glVertex2i(x+scaleWidth,y);
			glFunc->glVertex2i(x+scaleWidth,y+scaleMaxHeight-1);
			glFunc->glVertex2i(x,y+scaleMaxHeight-1);
			glFunc->glEnd();
		}

		//scale border
		const ccColor::Rgbub& lineColor = textColor;
		glFunc->glColor3ubv(lineColor.rgb);
		glFunc->glLineWidth(2.0f * renderZoom);
		glFunc->glEnable(GL_LINE_SMOOTH);
		glFunc->glBegin(GL_LINE_LOOP);
		glFunc->glVertex2i(x,y);
		glFunc->glVertex2i(x+scaleWidth,y);
		glFunc->glVertex2i(x+scaleWidth,y+scaleMaxHeight);
		glFunc->glVertex2i(x,y+scaleMaxHeight);
		glFunc->glEnd();

		glFunc->glPopAttrib();
	}

	//display labels
	{
		//list of labels to draw
		vlabelSet drawnLabels;

		//add first label
		drawnLabels.emplace_back(0, 0, strHeight, sortedKeyValues.front());

		if (keyValues.size() > 1)
		{
			//add last label
			drawnLabels.emplace_back(scaleMaxHeight, scaleMaxHeight - strHeight, scaleMaxHeight, sortedKeyValues.back());
		}

		//we try to display the other keyPoints (if any)
		if (keyValues.size() > 2)
		{
			assert(maxRange > 0.0);
			const int minGap = strHeight;
			for (size_t i=1; i<keyValues.size()-1; ++i)
			{
				int yScale = static_cast<int>((sortedKeyValues[i]-sortedKeyValues[0]) * scaleMaxHeight / maxRange);
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

		//now we recursively display labels for which we have some room left
		if (!customLabels && drawnLabels.size() > 1)
		{
			const int minGap = strHeight*2;

			size_t drawnLabelsBefore = 0; //just to init the loop
			size_t drawnLabelsAfter = drawnLabels.size();

			//proceed until no more label can be inserted
			while (drawnLabelsAfter > drawnLabelsBefore)
			{
				drawnLabelsBefore = drawnLabelsAfter;

				vlabelSet::iterator it1 = drawnLabels.begin();
				vlabelSet::iterator it2 = it1; ++it2;
				for (; it2 != drawnLabels.end(); ++it2)
				{
					if (it1->yMax + 2*minGap < it2->yMin)
					{
						//insert label
						double val = (it1->val + it2->val)/2.0;
						int yScale = static_cast<int>((val-sortedKeyValues[0]) * scaleMaxHeight / maxRange);

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
		glFunc->glColor3f(textColor.r / 255.0f, textColor.g / 255.0f, textColor.b / 255.0f);

		//Scalar field name
		const char* sfName = sf->getName();
		if (sfName)
		{
			//QString sfTitle = QString("[%1]").arg(sfName);
			QString sfTitle(sfName);
			if (sf->getGlobalShift() != 0)
				sfTitle += QString("[Shifted]");
			if (logScale)
				sfTitle += QString("[Log scale]");
			//we leave some (vertical) space for the top-most label!
			win->displayText(sfTitle, glW-xShift, glH-yShift+strHeight, ccGLWindow::ALIGN_HRIGHT | ccGLWindow::ALIGN_VTOP, 0, nullptr, &font);
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
			vlabelSet::iterator itNext = it; ++itNext;
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

			win->displayText(QString::number(value,format,precision), x, y+it->yPos, align, 0, nullptr, &font);
			glFunc->glBegin(GL_LINES);
			glFunc->glVertex2i(xTick,yTick+it->yPos);
			glFunc->glVertex2i(xTick+tickSize,yTick+it->yPos);
			glFunc->glEnd();
		}
	}

	glFunc->glPopAttrib();
}
