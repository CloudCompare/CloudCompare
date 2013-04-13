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

#ifndef CC_COLOR_SCALE_HEADER
#define CC_COLOR_SCALE_HEADER

//Local
#include "ccBasicTypes.h"
#include "ccSerializableObject.h"

//Qt
#include <QString> 
#include <QSharedPointer>
#include <QColor>
#include <QList>

//System
#include <assert.h>

//! Color scale element: one value + one color
class ccColorScaleElement
{
public:
	
	//! Default constructor
	ccColorScaleElement() : m_value(0.0), m_color(Qt::black) {}

	//! Constructor from a value and a color
	ccColorScaleElement(double value, QColor color) : m_value(value), m_color(color) {}

    //! Sets associated value
	inline void setValue(double val) { m_value = val; }
    //! Returns associated value
	inline double getValue() const { return m_value; }

    //! Sets color
	inline void setColor(QColor color) { m_color = color; }
    //! Returns color
	inline const QColor& getColor() const { return m_color; }

	//! Comparison operator between two color scale elements
	inline static bool IsSmaller(const ccColorScaleElement& e1, const ccColorScaleElement& e2)
	{
		return e1.getValue() < e2.getValue();
	}

protected:
	
	//! Associated value
	double m_value;
	//! Color
	QColor m_color;
};

//! Color scale
/** Internally, colors are always stored in the RGBA  format (to improve access speed... at least on Windows ;)
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccColorScale : public ccSerializableObject
#else
class ccColorScale : public ccSerializableObject
#endif
{
public:

	//! Shared pointer type
	typedef QSharedPointer<ccColorScale> Shared;

	//! Creates a new color scale (with auto-generated unique id)
	ccColorScale::Shared Create(QString name, bool relative = true); 

	//! Default constructor
	/** \param name scale name
		\param uuid UUID (automatically generated if none is provided)
		\param relative whether scale is relative or not (can be changed afterwards)
	**/
	ccColorScale(QString name, QString uuid = QString(), bool relative = true);

	//! Destructor
	virtual ~ccColorScale();

	//! Minimum number of steps
	static const unsigned MIN_STEPS = 2;

	//! Default number of steps for display
	static const unsigned DEFAULT_STEPS = 256;

	//! Maximum number of steps (internal representation)
	static const unsigned MAX_STEPS = 1024;

	//! Returns name
	QString getName() const { return m_name; }
	//! Sets name
	void setName(const QString& name) { m_name = name; }

	//! Returns unique ID
	QString getUuid() const { return m_uuid; }
	//! Sets unique ID
	void setUuid(QString uuid) { m_uuid = uuid; }
	//! Generates a new unique ID
	void generateNewUuid();

	//! Returns whether scale is relative or absoute
	/** Relative means that internal 'values' are percentage.
	**/
	bool isRelative() const { return m_relative; }

	//! Sets whether scale is relative or absoute
	void setRelative(bool state) { m_relative = state; }

	//! Returns whether scale is locked or not
	bool isLocked() const { return m_locked; }

	//! Sets whether scale is locked or not
	void setLocked(bool state) { m_locked = state; }

	//! Returns the current number of steps
	/** There must be at least 2 steps for the scale to be valid!
	**/
	int stepCount() const { return m_steps.size(); }

	//! Access to a given step
	ccColorScaleElement& step(int index) { return m_steps[index]; }

	//! Access to a given step (const)
	const ccColorScaleElement& step(int index) const { return m_steps[index]; }

	//! Adds a step
	/** Scale must not be locked.
	**/
	void insert(const ccColorScaleElement& step, bool autoUpdate = true);

	//! Deletes a given step
	/** There must be at least 2 steps for the scale to be valid!
		Scale must not be locked.
	**/
	void remove(int index, bool autoUpdate = true);

	//! Removes all steps
	/** There must be at least 2 steps for the scale to be valid!
		Scale must not be locked.
	**/
	void clear();

	//! Updates internal representation
	void update();

	//! Returns relative position of a given value (wrt to scale min and max)
	inline double getRelativePosition(double value) const
	{
		assert(m_updated);
		return (value - m_minValue)/m_range;
	}

	//! Returns color by value
	/** Warning: check first that scale is relative or not! (see ccColorScale::isRelative)
	**/
	inline const colorType* getColorByValue(double value) const
	{
		double relativePos = getRelativePosition(value);
		return (relativePos >= 0.0 && relativePos <= 1.0 ? getColorByRelativePos(relativePos) : 0);
	}

	//! Returns color by relative position in scale
	/** \param relativePos relative position (must be between 0 and 1!)
	**/
	inline const colorType* getColorByRelativePos(double relativePos) const
	{
		assert(m_updated && relativePos >= 0.0 && relativePos <= 1.0);
		return getColorByIndex((unsigned)(relativePos * (double)(MAX_STEPS-1)));
	}

	//! Returns color by relative position in scale with a given 'resolution'
	/** \param relativePos relative position (must be between 0 and 1!)
		\param steps desired resolution (must be greater than 1 and smaller than MAX_STEPS)
	**/
	inline const colorType* getColorByRelativePos(double relativePos, unsigned steps) const
	{
		//quantized (16 bits) version --> much faster than floor!
		unsigned index = ((unsigned)((relativePos*(double)(steps-1)+0.5)*65536.0))>>16;
		return getColorByIndex((index*(MAX_STEPS-1)) / (steps-1));
	}

	//! Returns color by index
	/** \param index color index in m_rgbaScale array (must be below MAX_STEPS)
	**/
	inline const colorType* getColorByIndex(unsigned index) const
	{
		assert(m_updated && index < MAX_STEPS);
		return m_rgbaScale + (index << 2);
	}

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion);

protected:

	//! Sort elements
	void sort();

    //! Name
	QString m_name;

    //! Unique ID
	QString m_uuid;

	//! Elements
	QList<ccColorScaleElement> m_steps;

	//! Internal representation (RGBA)
	colorType m_rgbaScale[MAX_STEPS*4];

	//! Internal representation validity
	bool m_updated;

	//! Whether scale is relative or not
	bool m_relative;

	//! Whether scale is locked or not
	bool m_locked;

	//! Min value (for faster access)
	double m_minValue;

	//! Range (for faster access)
	double m_range;

};

#endif //CC_COLOR_SCALE_HEADER
