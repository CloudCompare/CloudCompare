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

#ifndef CC_COLOR_SCALE_HEADER
#define CC_COLOR_SCALE_HEADER

//Local
#include "ccColorTypes.h"
#include "ccSerializableObject.h"

//Qt
#include <QList>
#include <QSharedPointer>

//System
#include <set>

//! Color scale element: one value + one color
class ccColorScaleElement
{
public:
	
	//! Default constructor
	ccColorScaleElement() : m_relativePos(0.0), m_color(Qt::black) {}

	//! Constructor from a (relative) position and a color
	ccColorScaleElement(double relativePos, const QColor& color) : m_relativePos(relativePos), m_color(color) {}

	//! Sets associated value (relative to scale boundaries)
	/** \param pos relative position (always between 0.0 and 1.0!)
	**/
	inline void setRelativePos(double pos) { m_relativePos = pos; }
	//! Returns step position (relative to scale boundaries)
	/** \return relative position (always between 0.0 and 1.0!)
	**/
	inline double getRelativePos() const { return m_relativePos; }

	//! Sets color
	inline void setColor(const QColor& color) { m_color = color; }
	//! Returns color
	inline const QColor& getColor() const { return m_color; }

	//! Comparison operator between two color scale elements
	inline static bool IsSmaller(const ccColorScaleElement& e1, const ccColorScaleElement& e2)
	{
		return e1.getRelativePos() < e2.getRelativePos();
	}

protected:
	
	//! Step (relative) position
	/** Must always be between 0.0 and 1.0!
	**/
	double m_relativePos;
	//! Color
	QColor m_color;
};

//! Color scale
/** A color scale is defined by several 'steps' corresponding to given colors.
	The color between each step is linearly interpolated. A valid color scale
	must have at least 2 steps, one at relative position 0.0 (scale start) and
	one at relative position 1.0 (scale end). Steps can't be defined outside
	this interval.

	For faster access, an array of interpolated colors is maintained internally.
	Be sure that the 'refresh' method has been called after any modification
	of the scale steps (position or color).
**/
class QCC_DB_LIB_API ccColorScale : public ccSerializableObject
{
public:

	//! Shared pointer type
	using Shared = QSharedPointer<ccColorScale>;

	//! Creates a new color scale (with auto-generated unique id)
	/** Warning: color scale is relative by default.
	**/
	static ccColorScale::Shared Create(const QString& name); 

	//! Default constructor
	/** \param name scale name
		\param uuid UUID (automatically generated if none is provided)
		Scale are 'relative' by default (can be changed afterwards, see setAbsolute).
		On construction they already have the two extreme steps defined (at position
		0.0 and 1.0).
	**/
	ccColorScale(const QString& name, const QString& uuid = QString());

	//! Destructor
	~ccColorScale() override = default;

	//! Minimum number of steps
	static constexpr unsigned MIN_STEPS = 2;

	//! Default number of steps for display
	static constexpr unsigned DEFAULT_STEPS = 256;

	//! Maximum number of steps (internal representation)
	static constexpr unsigned MAX_STEPS = 1024;

	//! Returns name
	inline const QString& getName() const { return m_name; }
	//! Sets name
	void setName(const QString& name) { m_name = name; }

	//! Returns unique ID
	const QString &getUuid() const { return m_uuid; }
	//! Sets unique ID
	void setUuid(const QString& uuid) { m_uuid = uuid; }
	//! Generates a new unique ID
	void generateNewUuid();

	//! Returns whether scale is relative or absoute
	/** Relative means that internal 'values' are percentage.
	**/
	inline bool isRelative() const { return m_relative; }

	//! Sets scale as relative
	inline void setRelative() { m_relative = true; }

	//! Sets scale as absolute
	void setAbsolute(double minVal, double maxVal);

	//! Get absolute scale boundaries
	/** Warning: only valid with absolute scales!
	**/
	void getAbsoluteBoundaries(double& minVal, double& maxVal) const;

	//! Returns whether scale is locked or not
	inline bool isLocked() const { return m_locked; }

	//! Sets whether scale is locked or not
	inline void setLocked(bool state) { m_locked = state; }

	//! Type of a list of custom labels
	using LabelSet = std::set<double>;

	//! Returns the list of custom labels (if any)
	inline LabelSet& customLabels() { return m_customLabels; }
	//! Returns the list of custom labels (if any - const version)
	inline const LabelSet& customLabels() const { return m_customLabels; }

	//! Sets the list of custom labels (only if the scale is absolute)
	/** \warning May throw std::bad_alloc exception)
	**/
	inline void setCustomLabels(const LabelSet& labels) { m_customLabels = labels; }

	//! Returns the current number of steps
	/** A valid scale should always have at least 2 steps!
	**/
	inline int stepCount() const { return m_steps.size(); }

	//! Access to a given step
	inline ccColorScaleElement& step(int index) { return m_steps[index]; }

	//! Access to a given step (const)
	inline const ccColorScaleElement& step(int index) const { return m_steps[index]; }

	//! Adds a step
	/** Scale must not be locked.
	**/
	void insert(const ccColorScaleElement& step, bool autoUpdate = true);

	//! Deletes a given step
	/** The first and last index shouldn't be deleted!
		Scale must not be locked.
	**/
	void remove(int index, bool autoUpdate = true);

	//! Clears all steps
	/** There should be at least 2 steps for the scale to be valid!
		Scale must not be locked.
	**/
	void clear();

	//! Updates internal representation
	/** Must be called at least once after any modification
		(before using this scale).
	**/
	void update();

	//! Returns relative position of a given value (wrt to scale absolute min and max)
	/** Warning: only valid with absolute scales! Use 'getColorByRelativePos' otherwise.
	**/
	inline double getRelativePosition(double value) const
	{
		assert(m_updated && !m_relative);
		return (value - m_absoluteMinValue) / m_absoluteRange;
	}

	//! Returns color by value
	/** Warning: only valid with absolute scales!
		\param value value
		\param outOfRangeColor default color to return if relativePos if out of [0;1]
		\return corresponding color
	**/
	inline const ccColor::Rgb* getColorByValue(double value, const ccColor::Rgb* outOfRangeColor = nullptr) const
	{
		assert(m_updated && !m_relative);
		double relativePos = getRelativePosition(value);
		return (relativePos >= 0.0 && relativePos <= 1.0 ? getColorByRelativePos(relativePos) : outOfRangeColor);
	}

	//! Returns color by relative position in scale
	/** \param relativePos relative position (should be in [0;1])
		\param outOfRangeColor default color to return if relativePos if out of [0;1]
		\return corresponding color
	**/
	inline const ccColor::Rgb* getColorByRelativePos(double relativePos, const ccColor::Rgb* outOfRangeColor = nullptr) const
	{
		assert(m_updated);
		if (relativePos >= 0.0 && relativePos <= 1.0)
			return &getColorByIndex(static_cast<unsigned>(relativePos * (MAX_STEPS - 1)));
		else
			return outOfRangeColor;
	}

	//! Returns color by relative position in scale with a given 'resolution'
	/** \param relativePos relative position (must be between 0 and 1!)
		\param steps desired resolution (must be greater than 1 and smaller than MAX_STEPS)
		\param outOfRangeColor default color to return if relativePos if out of [0;1]
		\return corresponding color
	**/
	inline const ccColor::Rgb* getColorByRelativePos(double relativePos, unsigned steps, const ccColor::Rgb* outOfRangeColor = nullptr) const
	{
		assert(m_updated);
		if (relativePos >= 0.0 && relativePos <= 1.0)
		{
			//quantized (16 bits) version --> much faster than floor!
			unsigned index = (static_cast<unsigned>((relativePos*steps)*65535.0)) >> 16;
			return &getColorByIndex((index*(MAX_STEPS - 1)) / steps);
		}
		else
		{
			return outOfRangeColor;
		}
	}

	//! Returns color by index
	/** \param index color index in m_rgbaScale array (must be below MAX_STEPS)
		\return corresponding color
	**/
	inline const ccColor::Rgb& getColorByIndex(unsigned index) const
	{
		assert(m_updated && index < MAX_STEPS);
		return m_rgbaScale[index];
	}

	//! Saves this color scale as an XML file
	bool saveAsXML(const QString& filename) const;
	//! Loads a color scale from an XML file
	static Shared LoadFromXML(const QString& filename);

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

protected:

	//! Sort elements
	void sort();

	//! Name
	QString m_name;

	//! Unique ID
	QString m_uuid;

	//! Elements
	QList<ccColorScaleElement> m_steps;

	//! Internal representation (RGB)
	ccColor::Rgb m_rgbaScale[MAX_STEPS];

	//! Internal representation validity
	bool m_updated;

	//! Whether scale is relative or not
	bool m_relative;

	//! Whether scale is locked or not
	bool m_locked;

	//! 'Absolute' minimum value
	/** Only used if scale is 'absolute' (i.e. not relative).
		'Absolute' should not be taken in its mathematical meaning!
	**/
	double m_absoluteMinValue;

	//! 'Absolute' range
	/** Only used if scale is 'absolute' (i.e. not relative).
		'Absolute' should not be taken in its mathematical meaning!
	**/
	double m_absoluteRange;

	//! List of custom labels
	LabelSet m_customLabels;
};

#endif //CC_COLOR_SCALE_HEADER
