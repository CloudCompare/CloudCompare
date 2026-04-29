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

#ifndef CC_NODE_STEPPER_DIALOG_HEADER
#define CC_NODE_STEPPER_DIALOG_HEADER

#include <ui_nodeStepperDlg.h>

#include <QDialog>
#include <QSet>
#include <QStringList>
#include <QVector>

//! Dialog for stepping through graph nodes and assigning names (BatGraph F5)
/** Loads a nodes CSV (label, x, y, z), presents each unnamed node in turn,
    and allows the operator to assign a name from a pre-loaded list or type one
    manually. The updated CSV is saved on close.
 **/
class ccNodeStepperDlg : public QDialog, public Ui::NodeStepperDlg
{
	Q_OBJECT

  public:
	explicit ccNodeStepperDlg(QWidget* parent = nullptr);

	//! Load graph nodes from a CSV file (label, [source_cloud_id,] x, y, z)
	/** Returns false if the file cannot be parsed. **/
	bool loadNodes(const QString& csvPath);

  private slots:
	void onApplyName();
	void onNext();
	void onPrev();
	void onLoadNameList();
	void onClearNameList();
	void onShowLabelledChanged(int state);

  private:
	struct NodeRow
	{
		QString name;
		QString sourceCloudId;
		double  x = 0, y = 0, z = 0;
		bool    hasName() const { return !name.isEmpty() && name != "(unlabelled)"; }
	};

	void updateDisplay();
	void updateProgress();
	void updateNameCombo();
	void saveNodes() const;

	//! Whether a row should be visited given the "show labelled" toggle
	bool isVisible(int idx) const;
	//! Step to the next/prev row that passes isVisible; returns false if none
	bool advance(int dir);

	QVector<NodeRow> m_nodes;
	int              m_current = -1;
	QStringList      m_nameList;
	QSet<QString>    m_usedNames;
	QString          m_csvPath;
};

#endif
