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
#include "ccGLWindowInterface.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccGBLSensor.h>
#include <ccGenericPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>
#include <QFontMetrics>

#include <numeric>

void ccRenderingTools::ShowDepthBuffer(ccGBLSensor* sensor, QWidget* parent/*=nullptr*/, unsigned maxDim/*=1024*/)
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
				sumDist2 += static_cast<double>(*_zBuff) * *_zBuff;
				++count;
			}
		}

		if (count)
		{
			double avg = sumDist / count;
			double stdDev = sqrt(std::abs(sumDist2 / count - avg*avg));
			//for better dynamics
			maxDist = std::min(maxDist, static_cast<PointCoordinateType>(avg + 1.0 * stdDev));
		}
	}

	QImage bufferImage(depthBuffer.width, depthBuffer.height, QImage::Format_RGB32);
	{
		ccColorScale::Shared colorScale = ccColorScalesManager::GetDefaultScale();
		assert(colorScale);
		ScalarType coef = CCCoreLib::LessThanEpsilon(maxDist - minDist) ? 0 : static_cast<ScalarType>(ccColorScale::MAX_STEPS - 1) / (maxDist - minDist);

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
struct VLabel
{
	int yPos = 0; 				/**< label center pos **/
	int yMin = 0; 				/**< label 'ROI' min **/
	int yMax = 0; 				/**< label 'ROI' max **/
	ccColorScale::Label label; 	/**< label value **/

	//default constructor
	VLabel(int y, int y1, int y2, const ccColorScale::Label& l)
		: yPos(y)
		, yMin(y1)
		, yMax(y2)
		, label(l)
	{
		assert(y2 >= y1);
	}
};

//! A set of 'VLabel' structures
using VLabelSet = std::list<VLabel>;

//helper: returns the neighbouring labels at a given position
//(first: above label, second: below label)
//Warning: set must be already sorted!
using VLabelPair = std::pair<VLabelSet::iterator, VLabelSet::iterator>;

static VLabelPair GetVLabelsAround(int y, VLabelSet& set)
{
	if (set.empty())
	{
		return VLabelPair(set.end(), set.end());
	}
	else
	{
		VLabelSet::iterator it1 = set.begin();
		if (y < it1->yPos)
		{
			return VLabelPair(set.end(), it1);
		}
		VLabelSet::iterator it2 = it1; ++it2;
		for (; it2 != set.end(); ++it2, ++it1)
		{
			if (y <= it2->yPos) // '<=' to make sure the last label stays at the top!
				return VLabelPair(it1, it2);
		}
		return VLabelPair(it1, set.end());
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
	ccGLWindowInterface* display = static_cast<ccGLWindowInterface*>(context.display);

	DrawColorRamp(context, sf, display, context.glW, context.glH, context.renderZoom);
}

template<class OpenGLFunc> static void DrawRectangleAsQuad(OpenGLFunc& glFunc, int x1, int y1, int x2, int y2)
{
	glFunc.glBegin(GL_QUADS);
	glFunc.glVertex2i(x1, y1);
	glFunc.glVertex2i(x2, y1);
	glFunc.glVertex2i(x2, y2);
	glFunc.glVertex2i(x1, y2);
	glFunc.glEnd();
}

void ccRenderingTools::DrawColorRamp(const CC_DRAW_CONTEXT& context, const ccScalarField* sf, ccGLWindowInterface* win, int glW, int glH, float renderZoom/*=1.0f*/)
{
	if (!sf || !sf->getColorScale() || !win)
	{
		return;
	}

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	//display parameters
	const ccGui::ParamStruct& displayParams = win->getDisplayParameters();

	//request the area available to draw the color ramp
	int yStart = 0, yStop = 0;
	win->computeColorRampAreaLimits(yStart, yStop);

	const int scaleWidth = static_cast<int>(displayParams.colorScaleRampWidth * renderZoom);
	int scaleMaxHeight = yStop - yStart;

	if (scaleMaxHeight < scaleWidth)
	{
		// not enough room to display the color scale
		return;
	}

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
			if (!std::isfinite(it->value))
			{
				bool minusInf = (it->value < 0);
				keyValues.erase(it);
				if (minusInf)
					keyValues.insert({ std::numeric_limits<ScalarType>::lowest(), "-Inf" });
				else
					keyValues.insert({ std::numeric_limits<ScalarType>::max(), "+Inf" });
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
				if (!sf->displayRange().isInRange(static_cast<ScalarType>(it->value)) && (!alwaysShowZero || it->value != 0)) //we keep zero if the user has explicitely asked for it!
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
				if (it->value >= dispMin && it->value <= dispMax)
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

	//default color: text color
	const ccColor::Rgbub& textColor = displayParams.textDefaultCol;

	//histogram?
	const ccScalarField::Histogram histogram = sf->getHistogram();
	bool showHistogram = (displayParams.colorScaleShowHistogram && !logScale && histogram.maxValue != 0 && histogram.size() > 1);

	//Scalar field title (= name)
	const std::string& sfName = sf->getName();
	QString sfTitle = (sfName.empty() ? "Unnamed" : QString::fromStdString(sfName));
	if (logScale)
	{
		sfTitle += QString("[Log scale]");
	}

	//display font
	QFont font = win->getTextDisplayFont(); //takes rendering zoom into account!
	QFontMetrics fm(font);
	const int strHeight = fm.boundingRect("1").height(); //we assume that all chararcter will have the same height

	//histogram is 50% of the scale width by default (if displayed)
	int histogramWidth = (showHistogram ? scaleWidth / 2 : 0);

	//right border of the scale ramp rectangle
	const int rightMargin = static_cast<int>(20 * renderZoom);
	int xStart = glW - 1 - (scaleWidth + histogramWidth + rightMargin);

	//do we have room for the scalar field name
	bool showSFTitle = false;
	const float spaceBelowTitleRatio = 0.75f; // we add a margin below the title, proportional to the font size
	const QRect titleRect = fm.boundingRect(sfTitle);
	int titleMargin = static_cast<int>(titleRect.height() * spaceBelowTitleRatio);
	int titleHeight = titleRect.height() + titleMargin;
	if (scaleMaxHeight >= titleHeight + scaleWidth)
	{
		showSFTitle = true;
		yStart += titleHeight;
		scaleMaxHeight -= titleHeight;
	}

	if (keyValues.size() > 1)  //multiple values
	{
		//we need at least the room to display 2 values/labels
		if (scaleMaxHeight < 2 * strHeight)
		{
			return;
		}
	}
	else //if only 1 value --> we draw a cube
	{
		scaleMaxHeight = scaleWidth;
	}

	//we are in centered orthoprojective view (-halfW, -halfH, halfW, halfH)
	int halfW = glW / 2;
	int halfH = glH / 2;

	glFunc->glPushAttrib(GL_DEPTH_BUFFER_BIT);
	glFunc->glDisable(GL_DEPTH_TEST);

	std::vector<ccColorScale::Label> sortedKeyValues(keyValues.begin(), keyValues.end());
	double maxRange = sortedKeyValues.back().value - sortedKeyValues.front().value;

	const int borderWidth = 2 * renderZoom;

	//draw the color ramp
	{
		//(x,y): current display area coordinates (top-left corner)
		int x = xStart - halfW;
		int y = halfH - yStart - scaleMaxHeight;

		//first draw the 'border' as white quad
		const ccColor::Rgbub& borderColor = textColor;
		ccGL::Color(glFunc, borderColor);
		//GL_LINE_LOOP cuts the corners, and GL_LINES tries to round the borders!
		DrawRectangleAsQuad<QOpenGLFunctions_2_1>(*glFunc, x - borderWidth, y - borderWidth, x + scaleWidth + borderWidth, y + scaleMaxHeight + 1 + borderWidth);

		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(renderZoom);

		if (keyValues.size() > 1)
		{
			int xHisto = x + scaleWidth + std::min(std::max(scaleWidth / 8, 3), static_cast<int>(15 * renderZoom));

			glFunc->glBegin(GL_LINES);
			for (int j = 0; j <= scaleMaxHeight; ++j)
			{
				double baseValue = sortedKeyValues.front().value + (j * maxRange) / scaleMaxHeight;
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
				ccGL::Color(glFunc, *col);

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
					glFunc->glVertex2i(xHisto, y + j);
					glFunc->glVertex2i(xHisto + xSpan, y + j);
				}
			}
			glFunc->glEnd();
		}
		else
		{
			//if there's a unique (visible) scalar value, we only draw a square!
			double value = sortedKeyValues.front().value;
			if (logScale)
				value = exp(value*c_log10);
			
			const ccColor::Rgb* col = sf->getColor(static_cast<ScalarType>(value));
			ccGL::Color(glFunc, col ? *col : ccColor::lightGreyRGB);
			DrawRectangleAsQuad(*glFunc, x, y, x + scaleWidth, y + scaleMaxHeight + 1);
		}

		glFunc->glPopAttrib();
	}

	//Some versions of Qt seem to need glColorf instead of glColorub! (see https://bugreports.qt-project.org/browse/QTBUG-6217)
	glFunc->glColor3f(textColor.r / 255.0f, textColor.g / 255.0f, textColor.b / 255.0f);

	//SF name
	if (showSFTitle)
	{
		//we try to center the title
		int xMiddle = xStart + scaleWidth / 2;
		int xText = xMiddle;
		unsigned char align = ccGLWindowInterface::ALIGN_HMIDDLE;
		int leftMargin = static_cast<int>(5 * renderZoom);
		if (xMiddle + titleRect.width() / 2 + leftMargin >= glW)
		{
			//we will have to align to the right most position
			xText = glW - 1 - leftMargin;
			align = ccGLWindowInterface::ALIGN_HRIGHT;
		}
		align |= ccGLWindowInterface::ALIGN_VTOP;

		win->displayText(sfTitle, xText, glH - 1 - yStart + titleMargin, align, 0.0f, nullptr, &font);
	}

	//display the labels
	{
		//list of labels to draw
		VLabelSet drawnLabels;

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
			for (size_t i = 1; i < keyValues.size() - 1; ++i)
			{
				int yScale = static_cast<int>((sortedKeyValues[i].value - sortedKeyValues[0].value) * scaleMaxHeight / maxRange);
				VLabelPair nLabels = GetVLabelsAround(yScale, drawnLabels);

				assert(nLabels.first != drawnLabels.end() && nLabels.second != drawnLabels.end());
				if (	(nLabels.first == drawnLabels.end() || nLabels.first->yMax <= yScale - minGap)
					&&	(nLabels.second == drawnLabels.end() || nLabels.second->yMin >= yScale + minGap))
				{
					//insert it at the right place (so as to keep a sorted list!)
					drawnLabels.insert(nLabels.second, VLabel(yScale, yScale - strHeight / 2, yScale + strHeight / 2, sortedKeyValues[i]));
				}
			}
		}

		//now we recursively display labels for which we have some room left
		if (!customLabels && drawnLabels.size() > 1)
		{
			const int minGap = strHeight * 2;

			size_t drawnLabelsBefore = 0; //just to init the loop
			size_t drawnLabelsAfter = drawnLabels.size();

			//proceed until no more label can be inserted
			while (drawnLabelsAfter > drawnLabelsBefore)
			{
				drawnLabelsBefore = drawnLabelsAfter;

				VLabelSet::iterator it1 = drawnLabels.begin();
				VLabelSet::iterator it2 = it1; ++it2;
				for (; it2 != drawnLabels.end(); ++it2)
				{
					if (it1->yMax + 2 * minGap < it2->yMin)
					{
						//insert label
						double val = (it1->label.value + it2->label.value) / 2;
						int yScale = static_cast<int>((val - sortedKeyValues[0].value) * scaleMaxHeight / maxRange);

						//insert it at the right place (so as to keep a sorted list!)
						drawnLabels.insert(it2, VLabel(yScale, yScale - strHeight / 2, yScale + strHeight / 2, val));
					}
					it1 = it2;
				}

				drawnLabelsAfter = drawnLabels.size();
			}
		}

		//format
		const char numberFormat = (sf->logScale() ? 'E' : 'f');
		const int tickSize = static_cast<int>(4 * renderZoom);

		//for labels
		const int xText = xStart - 2 * tickSize - borderWidth;
		const int yText = glH - 1 - yStart - scaleMaxHeight;
		//for ticks
		const int xTick = xStart - halfW - tickSize - borderWidth;
		const int yTick = halfH - yStart - scaleMaxHeight;

		for (VLabelSet::iterator it = drawnLabels.begin(); it != drawnLabels.end(); ++it)
		{
			VLabelSet::iterator itNext = it;
			++itNext;
			
			//position
			unsigned char align = ccGLWindowInterface::ALIGN_HRIGHT;
			if (it == drawnLabels.begin())
			{
				//specific case: first label
				align |= ccGLWindowInterface::ALIGN_VTOP;
			}
			else if (itNext == drawnLabels.end())
			{
				//specific case: last label
				align |= ccGLWindowInterface::ALIGN_VBOTTOM;
			}
			else
			{
				align |= ccGLWindowInterface::ALIGN_VMIDDLE;
			}

			QString numberStr;
			if (it->label.text.isEmpty())
			{
				double value = it->label.value;
				if (logScale)
				{
					value = exp(value*c_log10);
				}
				numberStr = QString::number(value, numberFormat, displayParams.displayedNumPrecision);
			}
			else
			{
				numberStr = it->label.text;
			}

			//display label
			win->displayText(	numberStr,
								xText,
								yText + it->yPos,
								align,
								0.0f,
								nullptr,
								&font);

			//draw the tick
			glFunc->glBegin(GL_LINES);
			glFunc->glVertex2i(xTick, yTick + it->yPos);
			glFunc->glVertex2i(xTick + tickSize, yTick + it->yPos);
			glFunc->glEnd();
		}
	}

	glFunc->glPopAttrib();
}
