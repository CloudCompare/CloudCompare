// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_METADATA_STEPPER_DIALOG_HEADER
#define CC_METADATA_STEPPER_DIALOG_HEADER

#include <ui_metadataStepperDlg.h>

#include <QDialog>
#include <QStringList>
#include <QVector>

//! Dialog for stepping through graph edges and filling metadata (BatGraph F7)
/** Loads an edges CSV, presents each edge in turn with fields appropriate
    to its type, and writes the updated CSV after every Apply.
 **/
class ccMetadataStepperDlg : public QDialog, public Ui::MetadataStepperDlg
{
	Q_OBJECT

  public:
	explicit ccMetadataStepperDlg(QWidget* parent = nullptr);

	//! Load edges from a CSV file
	/** Returns false if the file cannot be parsed. **/
	bool loadEdges(const QString& csvPath);

  private slots:
	void onApplyNext();
	void onNext();
	void onPrev();
	void onSkipCompleteChanged(int state);

  private:
	struct EdgeRow
	{
		QStringList cols; // raw CSV values, one per column

		QString col(int idx) const
		{
			return (idx >= 0 && idx < cols.size()) ? cols[idx].trimmed() : QString();
		}
		void setCol(int idx, const QString& val)
		{
			while (cols.size() <= idx)
				cols.append(QString());
			cols[idx] = val;
		}

		bool isVertical(int edgeTypeIdx) const;
		bool isLift(int edgeTypeIdx) const;
		bool isComplete(int stepsUpIdx, int hoursIdx, int statusIdx) const;
	};

	void applyCurrentToRow(EdgeRow& row) const;
	void populateFromRow(const EdgeRow& row);
	void updateDisplay();
	void updateStairsGroupVisibility(const QString& edgeType);
	bool advance(int dir);
	bool isVisible(int idx) const;
	void saveEdges() const;

	QVector<EdgeRow> m_edges;
	QStringList      m_header;
	int              m_current = -1;
	QString          m_csvPath;

	// Column indices (resolved once on load)
	int m_fromIdx      = -1;
	int m_toIdx        = -1;
	int m_edgeTypeIdx  = -1;
	int m_stepsUpIdx   = -1;
	int m_stepsDownIdx = -1;
	int m_handrailIdx  = -1;
	int m_elevatorIdx  = -1;
	int m_gradientIdx  = -1;
	int m_hoursIdx     = -1;
	int m_statusIdx    = -1;
	int m_fixedTimeIdx = -1;
	int m_weightIdx    = -1;
	int m_sourceIdx    = -1;
};

#endif
