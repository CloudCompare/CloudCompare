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

#include "ccMetadataStepperDlg.h"

// Qt
#include <QFile>
#include <QMessageBox>
#include <QTextStream>

// Vertical edge types that have stairs/escalator/lift fields
static bool edgeTypeIsVertical(const QString& t)
{
	const QString lc = t.toLower();
	return lc == "elev" || lc == "stairs" || lc == "escalator" || lc == "lift";
}

static bool edgeTypeIsLift(const QString& t)
{
	const QString lc = t.toLower();
	return lc == "lift" || lc == "elev_lift";
}

// An edge is "complete" when it has at least steps + status + hours filled.
// We use this loosely — any edge with a non-blank Status counts as visited.
bool ccMetadataStepperDlg::EdgeRow::isComplete(int stepsUpIdx, int hoursIdx, int statusIdx) const
{
	const bool hasStatus = statusIdx >= 0 && !col(statusIdx).isEmpty() && col(statusIdx) != "(not set)";
	return hasStatus;
}

bool ccMetadataStepperDlg::EdgeRow::isVertical(int edgeTypeIdx) const
{
	return edgeTypeIsVertical(col(edgeTypeIdx));
}

bool ccMetadataStepperDlg::EdgeRow::isLift(int edgeTypeIdx) const
{
	return edgeTypeIsLift(col(edgeTypeIdx));
}

// Helper: find column index case-insensitively
static int findCol(const QStringList& header, const QString& name)
{
	for (int i = 0; i < header.size(); ++i)
	{
		if (header[i].trimmed().toLower() == name.toLower())
			return i;
	}
	return -1;
}

// Helper: ensure a column exists in header and all rows, return its index
static int ensureCol(QStringList& header, QVector<ccMetadataStepperDlg::EdgeRow>& rows, const QString& name)
{
	int idx = findCol(header, name);
	if (idx < 0)
	{
		idx = header.size();
		header.append(name);
		for (auto& row : rows)
			row.cols.append(QString());
	}
	return idx;
}

ccMetadataStepperDlg::ccMetadataStepperDlg(QWidget* parent)
    : QDialog(parent, Qt::Tool)
    , Ui::MetadataStepperDlg()
{
	setupUi(this);
	setWindowTitle(tr("Metadata Stepper — BatGraph F7"));

	connect(applyNextButton, &QPushButton::clicked, this, &ccMetadataStepperDlg::onApplyNext);
	connect(nextButton, &QPushButton::clicked, this, &ccMetadataStepperDlg::onNext);
	connect(prevButton, &QPushButton::clicked, this, &ccMetadataStepperDlg::onPrev);
	connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(skipCompleteCheckBox, &QCheckBox::stateChanged, this, &ccMetadataStepperDlg::onSkipCompleteChanged);
}

bool ccMetadataStepperDlg::loadEdges(const QString& csvPath)
{
	QFile f(csvPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::critical(this, tr("Metadata Stepper"), tr("Cannot open file:\n%1").arg(csvPath));
		return false;
	}

	m_csvPath = csvPath;
	m_edges.clear();
	m_header.clear();

	QTextStream in(&f);
	bool        firstLine = true;

	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;

		if (firstLine)
		{
			firstLine = false;
			m_header  = line.split(',');
			for (auto& h : m_header)
				h = h.trimmed();
			continue;
		}

		EdgeRow row;
		row.cols = line.split(',');
		for (auto& c : row.cols)
			c = c.trimmed();
		m_edges.append(row);
	}

	if (m_edges.isEmpty())
	{
		QMessageBox::warning(this, tr("Metadata Stepper"), tr("No edge rows found in:\n%1").arg(csvPath));
		return false;
	}

	// Resolve / create column indices
	m_fromIdx     = findCol(m_header, "FromNode");
	if (m_fromIdx < 0) m_fromIdx = findCol(m_header, "From");
	m_toIdx       = findCol(m_header, "ToNode");
	if (m_toIdx < 0) m_toIdx = findCol(m_header, "To");
	m_edgeTypeIdx = findCol(m_header, "EdgeType");
	if (m_edgeTypeIdx < 0) m_edgeTypeIdx = findCol(m_header, "Edge Type");

	// Ensure metadata columns exist (adds them if absent)
	m_stepsUpIdx   = ensureCol(m_header, m_edges, "StepsUp");
	m_stepsDownIdx = ensureCol(m_header, m_edges, "StepsDown");
	m_handrailIdx  = ensureCol(m_header, m_edges, "Handrail");
	m_elevatorIdx  = ensureCol(m_header, m_edges, "Elevator ID");
	m_gradientIdx  = ensureCol(m_header, m_edges, "Gradient");
	m_hoursIdx     = ensureCol(m_header, m_edges, "Operational_Hours");
	m_statusIdx    = ensureCol(m_header, m_edges, "Status");
	m_fixedTimeIdx = ensureCol(m_header, m_edges, "FixedTravelTime_s");
	m_weightIdx    = ensureCol(m_header, m_edges, "Weight_s");
	m_sourceIdx    = ensureCol(m_header, m_edges, "Source");

	m_current = -1;
	advance(+1);
	updateDisplay();
	return true;
}

bool ccMetadataStepperDlg::isVisible(int idx) const
{
	if (idx < 0 || idx >= m_edges.size())
		return false;
	if (!skipCompleteCheckBox->isChecked())
		return true;
	return !m_edges[idx].isComplete(m_stepsUpIdx, m_hoursIdx, m_statusIdx);
}

bool ccMetadataStepperDlg::advance(int dir)
{
	const int n     = m_edges.size();
	int       start = (m_current == -1) ? (dir > 0 ? 0 : n - 1) : m_current + dir;

	for (int step = 0; step < n; ++step)
	{
		int idx = start + step * dir;
		if (idx < 0 || idx >= n)
			break;
		if (isVisible(idx))
		{
			m_current = idx;
			return true;
		}
	}
	return false;
}

void ccMetadataStepperDlg::updateDisplay()
{
	const bool hasEdge = (m_current >= 0 && m_current < m_edges.size());

	applyNextButton->setEnabled(hasEdge);
	nextButton->setEnabled(hasEdge);
	prevButton->setEnabled(hasEdge);

	if (!hasEdge)
	{
		fromLineEdit->clear();
		toLineEdit->clear();
		edgeTypeLineEdit->clear();
		progressLabel->setText(tr("Edge 0 of 0"));
		return;
	}

	const EdgeRow& row = m_edges[m_current];
	populateFromRow(row);

	// Progress
	int total = 0, reached = 0;
	for (int i = 0; i < m_edges.size(); ++i)
	{
		if (isVisible(i))
		{
			++total;
			if (i <= m_current)
				++reached;
		}
	}
	progressLabel->setText(tr("Edge %1 of %2").arg(reached).arg(total));

	const QString edgeType = (m_edgeTypeIdx >= 0) ? row.col(m_edgeTypeIdx) : QString();
	updateStairsGroupVisibility(edgeType);
}

void ccMetadataStepperDlg::populateFromRow(const EdgeRow& row)
{
	fromLineEdit->setText((m_fromIdx >= 0) ? row.col(m_fromIdx) : QString());
	toLineEdit->setText((m_toIdx >= 0) ? row.col(m_toIdx) : QString());
	edgeTypeLineEdit->setText((m_edgeTypeIdx >= 0) ? row.col(m_edgeTypeIdx) : QString());

	// Steps / vertical
	const QString su = row.col(m_stepsUpIdx);
	stepsUpSpinBox->setValue(su.isEmpty() ? 0 : su.toInt());

	const QString sd = row.col(m_stepsDownIdx);
	stepsDownSpinBox->setValue(sd.isEmpty() ? 0 : sd.toInt());

	const QString hr = row.col(m_handrailIdx).toLower();
	handrailComboBox->setCurrentIndex(hr == "y" ? 1 : (hr == "n" ? 2 : 0));

	elevatorIdLineEdit->setText(row.col(m_elevatorIdx));

	// General
	gradientLineEdit->setText(row.col(m_gradientIdx));
	hoursLineEdit->setText(row.col(m_hoursIdx));

	const QString status = row.col(m_statusIdx).toLower();
	if (status == "open")
		statusComboBox->setCurrentIndex(1);
	else if (status == "closed")
		statusComboBox->setCurrentIndex(2);
	else if (status == "seasonal")
		statusComboBox->setCurrentIndex(3);
	else if (status == "step-free")
		statusComboBox->setCurrentIndex(4);
	else
		statusComboBox->setCurrentIndex(0);

	const QString ft = row.col(m_fixedTimeIdx);
	fixedTimeSpinBox->setValue(ft.isEmpty() ? 0.0 : ft.toDouble());

	const QString ws = row.col(m_weightIdx);
	weightSpinBox->setValue(ws.isEmpty() ? 0.0 : ws.toDouble());

	sourceLineEdit->setText(row.col(m_sourceIdx));
}

void ccMetadataStepperDlg::applyCurrentToRow(EdgeRow& row) const
{
	const QString edgeType = edgeTypeLineEdit->text();
	const bool    vertical = edgeTypeIsVertical(edgeType);
	const bool    lift     = edgeTypeIsLift(edgeType);

	if (vertical)
	{
		row.setCol(m_stepsUpIdx, stepsUpSpinBox->value() > 0 ? QString::number(stepsUpSpinBox->value()) : QString());
		row.setCol(m_stepsDownIdx, stepsDownSpinBox->value() > 0 ? QString::number(stepsDownSpinBox->value()) : QString());
		const int hi = handrailComboBox->currentIndex();
		row.setCol(m_handrailIdx, hi == 1 ? "Y" : (hi == 2 ? "N" : QString()));
		if (lift)
			row.setCol(m_elevatorIdx, elevatorIdLineEdit->text().trimmed());
	}

	row.setCol(m_gradientIdx, gradientLineEdit->text().trimmed());
	row.setCol(m_hoursIdx, hoursLineEdit->text().trimmed());

	const int si = statusComboBox->currentIndex();
	const QStringList statusVals = {"", "open", "closed", "seasonal", "step-free"};
	row.setCol(m_statusIdx, si > 0 ? statusVals[si] : QString());

	row.setCol(m_fixedTimeIdx, fixedTimeSpinBox->value() > 0.0 ? QString::number(fixedTimeSpinBox->value(), 'f', 2) : QString());
	row.setCol(m_weightIdx, weightSpinBox->value() > 0.0 ? QString::number(weightSpinBox->value(), 'f', 2) : QString());
	row.setCol(m_sourceIdx, sourceLineEdit->text().trimmed());
}

void ccMetadataStepperDlg::updateStairsGroupVisibility(const QString& edgeType)
{
	const bool vertical = edgeTypeIsVertical(edgeType);
	const bool lift     = edgeTypeIsLift(edgeType);
	stairsGroup->setVisible(vertical);
	elevatorIdLabel->setVisible(lift);
	elevatorIdLineEdit->setVisible(lift);
}

void ccMetadataStepperDlg::onApplyNext()
{
	if (m_current < 0 || m_current >= m_edges.size())
		return;

	applyCurrentToRow(m_edges[m_current]);
	saveEdges();
	onNext();
}

void ccMetadataStepperDlg::onNext()
{
	if (!advance(+1))
	{
		QMessageBox::information(this, tr("Metadata Stepper"), tr("No more edges to step through."));
		return;
	}
	updateDisplay();
}

void ccMetadataStepperDlg::onPrev()
{
	if (!advance(-1))
	{
		QMessageBox::information(this, tr("Metadata Stepper"), tr("Already at the first edge."));
		return;
	}
	updateDisplay();
}

void ccMetadataStepperDlg::onSkipCompleteChanged(int /*state*/)
{
	if (m_current >= 0 && !isVisible(m_current))
	{
		m_current = -1;
		advance(+1);
	}
	updateDisplay();
}

void ccMetadataStepperDlg::saveEdges() const
{
	if (m_csvPath.isEmpty() || m_edges.isEmpty())
		return;

	QFile f(m_csvPath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QTextStream out(&f);
	out << m_header.join(',') << "\n";
	for (const EdgeRow& row : m_edges)
		out << row.cols.join(',') << "\n";
}
