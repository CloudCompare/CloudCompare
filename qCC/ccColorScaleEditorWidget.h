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

//Inspired from ccColorScaleEditorWidget by Richard Steffen (LGPL 2.1)

#ifndef CC_COLOR_SCALE_EDITOR_WIDGET_HEADER
#define CC_COLOR_SCALE_EDITOR_WIDGET_HEADER

//Qt
#include <QWidget>

//qCC_db
#include <ccColorScale.h>

//! Color scale element as a widget
class ColorScaleElementSlider : public QWidget, public ccColorScaleElement
{
	Q_OBJECT

public:

	//! Default constructor
	ColorScaleElementSlider(double relativePos = 0.0, QColor color = Qt::black, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Sets selection state
	void setSelected(bool state) { m_selected = state; }

	//! Returns selection state
	bool isSelected() const { return m_selected; }

	//! Comparison operator between two (pointers on) color scale elements
	static bool IsSmaller(const ColorScaleElementSlider* e1, const ColorScaleElementSlider* e2)
	{
		return e1->getRelativePos() < e2->getRelativePos();
	}

protected:

	//inherited from QWidget
	void paintEvent(QPaintEvent* e) override;

	//! Selection state
	bool m_selected;

	//! Widget orientation
	Qt::Orientation m_orientation;
};

//! Set of color scale elements (widgets)
class ColorScaleElementSliders : public QList<ColorScaleElementSlider*>
{
public:

	//! Adds a slider element and sort the whole set
	/** Should be used instead of push_back/push_front!
	**/
	void addSlider(ColorScaleElementSlider* slider);

	//! Sorts the set
	void sort();

	//! Remove all sliders
	void clear();

	//! Remove a given slider
	void removeAt(int i);

	//! Returns the currently selected slider index (or -1 if none)
	int selected() const;

	//! Returns the index of a given slider
	int indexOf(ColorScaleElementSlider* slider);
};

//! Shared set of color scale elements (widgets)
using SharedColorScaleElementSliders = QSharedPointer<ColorScaleElementSliders>;

//! Base color scale editor (sub)Widget
/** A widget with a margin (along a preferred orientation)
**/
class ColorScaleEditorBaseWidget : public QWidget
{
	Q_OBJECT

public:

	//! Defautl constructor
	ColorScaleEditorBaseWidget(SharedColorScaleElementSliders sliders, 
		Qt::Orientation orientation,
		int margin,
		QWidget* parent = nullptr)
		: QWidget(parent)
		, m_sliders(sliders)
		, m_orientation(orientation)
		, m_margin(margin)
	{}

	//! Returns useful length
	int length() const { return (m_orientation == Qt::Horizontal ? contentsRect().width() : contentsRect().height())-2*m_margin; }

	//! Sets associated sliders set
	virtual void setSliders(SharedColorScaleElementSliders sliders) { m_sliders = sliders; update(); }

	//! Returns orientation
	Qt::Orientation getOrientation() const { return m_orientation; }
	//! Returns margin
	int getMargin() const { return m_margin; }

protected:

	//! Associated sliders
	SharedColorScaleElementSliders m_sliders;

	//! Orientation
	Qt::Orientation m_orientation;

	//! Margin
	int m_margin;
};

//! Color bar widget
class ColorBarWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	ColorBarWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

signals:

	//! Signal emitted when the mouse (left) button is clicked
	/** \param relativePos relative click position (between 0 and 1)
	**/
	void pointClicked(double relativePos);

protected:

	//inherited from QWidget
	void paintEvent(QPaintEvent* e) override;
	void mousePressEvent(QMouseEvent* e) override;
};

//! All sliders widget
class SlidersWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	SlidersWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Manually selects a slider
	void select(int index, bool silent=false);

	//! Adds a new slider widget
	/** \param relativePos slider position (relatively to scale boundaries [0.0,1.0])
	\param color slider color
	\return created slider (pointer on)
	**/
	ColorScaleElementSlider* addNewSlider(double relativePos, QColor color);

	//! Updates slider position
	void updateSliderPos(int index);

	//! Updates all sliders positions
	void updateAllSlidersPos();

signals:

	//! Signal emitted when a slider is changed (position or color)
	void sliderModified(int index);

	//! Signal emitted when a slider is selected
	void sliderSelected(int index);

protected:

	//inherited from QWidget
	void mousePressEvent(QMouseEvent* e) override;
	void mouseMoveEvent(QMouseEvent* e) override;
	//virtual void mouseReleaseEvent(QMouseEvent* e);
	void mouseDoubleClickEvent(QMouseEvent* e) override;
	void resizeEvent(QResizeEvent* e) override;

};

//! All sliders labels widget
class SliderLabelWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	SliderLabelWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Sets text color
	inline void setTextColor(QColor color) { m_textColor = color; }

	//! Sets displayed numbers precision
	inline void setPrecision(int precision) { m_precision = precision; }

protected:

	//inherited from QWidget
	void paintEvent(QPaintEvent* e) override;

	//! Text color
	QColor m_textColor;

	//! Precision
	int m_precision;
};

//! Color scale editor dialog
class ccColorScaleEditorWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	ccColorScaleEditorWidget(QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Destructor
	~ccColorScaleEditorWidget() override = default;

	//! Returns the current number of color scale steps
	inline int getStepCount() const { return (m_sliders ? m_sliders->size() : 0); }

	//! Returns a given slider (pointer on)
	inline const ColorScaleElementSlider* getStep(int index) { return m_sliders ? m_sliders->at(index) : nullptr; }

	//! Sets a given slider color
	void setStepColor(int index, QColor color);

	//! Sets a given slider relative position
	void setStepRelativePosition(int index, double relativePos);

	//! Returns currently selected step index
	inline int getSelectedStepIndex() const { return m_sliders ? m_sliders->selected() : -1; }

	//! Sets currently selected step index
	void setSelectedStepIndex(int index, bool silent=false);

	//! Deletes a given step
	/** Warning: first and last steps shouldn't be deleted!
	**/
	void deleteStep(int index);

	//! Exports the current color scale
	void exportColorScale(ccColorScale::Shared& destScale) const;

	//! Imports the current color scale
	void importColorScale(ccColorScale::Shared scale);

	//! Sets whether to show the color elements labels or not
	void showLabels(bool state);

	//! Sets the labels color
	void setLabelColor(QColor color);

	//! Sets the labels precision
	void setLabelPrecision(int precision);

	//inherited from ColorScaleEditorBaseWidget
	void setSliders(SharedColorScaleElementSliders sliders) override;

signals:

	//! Signal emitted when a slider is selected
	void stepSelected(int index);

	//! Signal emitted when a slider is modified
	void stepModified(int index);

protected slots:

	//! Slot called when a 'point' is clicked on the color bar 
	void onPointClicked(double relativePos);

	//! Slot called when a slider is moved or its color is changed
	void onSliderModified(int sliderIndex);

	//! Slot called when a slider is selected
	void onSliderSelected(int sliderIndex);

protected:

	//! Associated color bar
	ColorBarWidget* m_colorBarWidget;

	//! Associated sliders widget
	SlidersWidget* m_slidersWidget;

	//! Associated (sliders) labels widget
	SliderLabelWidget* m_labelsWidget;

};

#endif //CC_COLOR_SCALE_EDITOR_DIALOG_HEADER
