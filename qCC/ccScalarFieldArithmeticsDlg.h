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

#ifndef CC_SF_ARITMETHIC_DLG_HEADER
#define CC_SF_ARITMETHIC_DLG_HEADER

#include <QDialog>

class ccPointCloud;

namespace Ui
{
	class SFArithmeticsDlg;
}

//! Dialog to choose 2 scalar fields (SF) and one operation for arithmetics processing
class ccScalarFieldArithmeticsDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccScalarFieldArithmeticsDlg(ccPointCloud* cloud, QWidget* parent = nullptr);
	~ccScalarFieldArithmeticsDlg() override;
	
	//! Arithmetic operations
	enum Operation {	/* Operations requiring two SFs */
						PLUS		= 0,
						MINUS		= 1,
						MULTIPLY	= 2,
						DIVIDE		= 3,
						/* Operations requiring only one SF */
						SQRT		= 4,
						POW2		= 5,
						POW3		= 6,
						EXP			= 7,
						LOG			= 8,
						LOG10		= 9,
						COS			= 10,
						SIN			= 11,
						TAN			= 12,
						ACOS		= 13,
						ASIN		= 14,
						ATAN		= 15,
						INT			= 16,
						INVERSE		= 17,
						/* Invalid enum. (always last) */
						INVALID		= 255
	};

	//! Returns selected operation
	Operation getOperation() const;

	//! Returns the operation enumerator based on its name
	static Operation GetOperationByName(const QString& name);

	//! Returns operation name
	static QString GetOperationName(Operation op, const QString& sf1, const QString& sf2 = QString());

	//! Applies operation on a given cloud
	/** Should be applied on the same cloud as the one input to the constructor
		Otherwise you'd better know what you're doing ;).
		\param cloud cloud on which to apply the SF operation
		\return success
	**/
	bool apply(ccPointCloud* cloud);

	//! Secondary SF descriptor
	struct SF2
	{
		SF2()
			: isConstantValue(true)
			, constantValue(0)
			, sfIndex(-1)
		{}

		bool isConstantValue;
		double constantValue;
		int sfIndex;
	};

	//! Applies operation on a given cloud
	/** \param cloud cloud on which to apply the SF operation
		\param op operation
		\param sf1Idx first (or only) scalar field index
		\param inplace whether the operation should be applied in place (SF1). Otherwise a new SF will be created.
		\param sf2 secondary scalar field / value (only for PLUS, MINUS, MULTIPLY and DIVIDE operations)
		\param parent parent widget (optional)
		\return success
	**/
	static bool Apply(ccPointCloud* cloud, Operation op, int sf1Idx, bool inplace, SF2* sf2 = nullptr, QWidget* parent = nullptr);

protected slots:
	
	//! Called when the operation combo-box is modified
	void onOperationIndexChanged(int index);

	//! Called when the SF2 combo-box is modified
	void onSF2IndexChanged(int index);

protected:
	
	//! Returns first selected SF index
	int getSF1Index();
	//! Returns second selected SF index
	int getSF2Index();
	
private:
	Ui::SFArithmeticsDlg* m_ui;
};

#endif //CC_SF_ARITMETHIC_DLG_HEADER
