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

#include "ccNodeStepperDlg.h"

// Qt
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>

ccNodeStepperDlg::ccNodeStepperDlg(QWidget* parent)
    : QDialog(parent, Qt::Tool)
    , Ui::NodeStepperDlg()
{
	setupUi(this);
	setWindowTitle(tr("Node Stepper — BatGraph F5"));

	connect(applyNameButton, &QPushButton::clicked, this, &ccNodeStepperDlg::onApplyName);
	connect(nextButton, &QPushButton::clicked, this, &ccNodeStepperDlg::onNext);
	connect(prevButton, &QPushButton::clicked, this, &ccNodeStepperDlg::onPrev);
	connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(loadNameListButton, &QPushButton::clicked, this, &ccNodeStepperDlg::onLoadNameList);
	connect(clearNameListButton, &QPushButton::clicked, this, &ccNodeStepperDlg::onClearNameList);
	connect(showLabelledCheckBox, &QCheckBox::stateChanged, this, &ccNodeStepperDlg::onShowLabelledChanged);
}

bool ccNodeStepperDlg::loadNodes(const QString& csvPath)
{
	QFile f(csvPath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::critical(this, tr("Node Stepper"), tr("Cannot open file:\n%1").arg(csvPath));
		return false;
	}

	m_csvPath = csvPath;
	m_nodes.clear();
	m_usedNames.clear();

	QTextStream in(&f);
	bool        firstLine = true;
	int         lineNo    = 0;

	while (!in.atEnd())
	{
		++lineNo;
		const QString line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;

		// Skip header
		if (firstLine)
		{
			firstLine = false;
			if (line.startsWith("label", Qt::CaseInsensitive) || line.startsWith("name", Qt::CaseInsensitive))
				continue;
		}

		const QStringList parts = line.split(',');

		// Supported formats:
		//   label, source_cloud_id, x, y, z   (F1 export — 5 columns)
		//   name, x, y, z                     (simple — 4 columns)
		NodeRow row;
		bool    ok1 = false, ok2 = false, ok3 = false;

		if (parts.size() >= 5)
		{
			row.name          = parts[0].trimmed();
			row.sourceCloudId = parts[1].trimmed();
			row.x             = parts[2].trimmed().toDouble(&ok1);
			row.y             = parts[3].trimmed().toDouble(&ok2);
			row.z             = parts[4].trimmed().toDouble(&ok3);
		}
		else if (parts.size() >= 4)
		{
			row.name = parts[0].trimmed();
			row.x    = parts[1].trimmed().toDouble(&ok1);
			row.y    = parts[2].trimmed().toDouble(&ok2);
			row.z    = parts[3].trimmed().toDouble(&ok3);
		}
		else
		{
			continue;
		}

		if (!ok1 || !ok2 || !ok3)
			continue;

		if (row.hasName())
			m_usedNames.insert(row.name);

		m_nodes.append(row);
	}

	if (m_nodes.isEmpty())
	{
		QMessageBox::warning(this, tr("Node Stepper"), tr("No valid node rows found in:\n%1").arg(csvPath));
		return false;
	}

	// Position on first visible node
	m_current = -1;
	advance(+1);

	updateDisplay();
	return true;
}

bool ccNodeStepperDlg::isVisible(int idx) const
{
	if (idx < 0 || idx >= m_nodes.size())
		return false;
	if (showLabelledCheckBox->isChecked())
		return true;
	return !m_nodes[idx].hasName();
}

bool ccNodeStepperDlg::advance(int dir)
{
	const int n     = m_nodes.size();
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

void ccNodeStepperDlg::updateDisplay()
{
	const bool hasNode = (m_current >= 0 && m_current < m_nodes.size());

	if (hasNode)
	{
		const NodeRow& row = m_nodes[m_current];
		currentNameLineEdit->setText(row.hasName() ? row.name : QString());
		coordsLineEdit->setText(QString("%1, %2, %3")
		                            .arg(row.x, 0, 'f', 4)
		                            .arg(row.y, 0, 'f', 4)
		                            .arg(row.z, 0, 'f', 4));
	}
	else
	{
		currentNameLineEdit->clear();
		coordsLineEdit->clear();
	}

	updateProgress();
	updateNameCombo();

	nextButton->setEnabled(hasNode);
	prevButton->setEnabled(hasNode);
	applyNameButton->setEnabled(hasNode);
}

void ccNodeStepperDlg::updateProgress()
{
	// Count how many visible nodes exist given current toggle state
	int total   = 0;
	int reached = 0;
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (isVisible(i))
		{
			++total;
			if (i <= m_current)
				++reached;
		}
	}
	progressLabel->setText(tr("Node %1 of %2").arg(reached).arg(total));
}

void ccNodeStepperDlg::updateNameCombo()
{
	const QString saved = nameComboBox->currentText();
	nameComboBox->clear();

	for (const QString& name : m_nameList)
	{
		if (!m_usedNames.contains(name))
			nameComboBox->addItem(name);
	}

	// Restore or pre-fill with current node's name
	if (m_current >= 0 && m_current < m_nodes.size() && m_nodes[m_current].hasName())
		nameComboBox->setCurrentText(m_nodes[m_current].name);
	else if (!saved.isEmpty())
		nameComboBox->setCurrentText(saved);
}

void ccNodeStepperDlg::onApplyName()
{
	if (m_current < 0 || m_current >= m_nodes.size())
		return;

	const QString name = nameComboBox->currentText().trimmed();
	if (name.isEmpty())
		return;

	NodeRow& row = m_nodes[m_current];
	if (row.hasName())
		m_usedNames.remove(row.name);

	row.name = name;
	m_usedNames.insert(name);

	saveNodes();
	onNext();
}

void ccNodeStepperDlg::onNext()
{
	if (!advance(+1))
	{
		QMessageBox::information(this, tr("Node Stepper"), tr("No more nodes to step through."));
		return;
	}
	updateDisplay();
}

void ccNodeStepperDlg::onPrev()
{
	if (!advance(-1))
	{
		QMessageBox::information(this, tr("Node Stepper"), tr("Already at the first node."));
		return;
	}
	updateDisplay();
}

void ccNodeStepperDlg::onLoadNameList()
{
	const QString path = QFileDialog::getOpenFileName(
	    this,
	    tr("Load name list"),
	    QString(),
	    tr("Text files (*.txt *.csv);;All files (*.*)"));
	if (path.isEmpty())
		return;

	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::critical(this, tr("Node Stepper"), tr("Cannot open file:\n%1").arg(path));
		return;
	}

	m_nameList.clear();
	QTextStream in(&f);
	while (!in.atEnd())
	{
		const QString name = in.readLine().trimmed();
		if (!name.isEmpty())
			m_nameList.append(name);
	}

	clearNameListButton->setEnabled(!m_nameList.isEmpty());
	nameListStatusLabel->setText(tr("%1 names loaded").arg(m_nameList.size()));
	updateNameCombo();
}

void ccNodeStepperDlg::onClearNameList()
{
	m_nameList.clear();
	clearNameListButton->setEnabled(false);
	nameListStatusLabel->setText(tr("No list loaded"));
	updateNameCombo();
}

void ccNodeStepperDlg::onShowLabelledChanged(int /*state*/)
{
	// Re-position current to nearest visible node in current direction
	if (m_current < 0 || !isVisible(m_current))
	{
		m_current = -1;
		advance(+1);
	}
	updateDisplay();
}

void ccNodeStepperDlg::saveNodes() const
{
	if (m_csvPath.isEmpty() || m_nodes.isEmpty())
		return;

	QFile f(m_csvPath);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QTextStream out(&f);

	const bool hasCloudId = !m_nodes.first().sourceCloudId.isEmpty();
	if (hasCloudId)
		out << "label,source_cloud_id,x,y,z\n";
	else
		out << "label,x,y,z\n";

	for (const NodeRow& row : m_nodes)
	{
		if (hasCloudId)
			out << row.name << "," << row.sourceCloudId << ","
			    << QString::number(row.x, 'f', 6) << ","
			    << QString::number(row.y, 'f', 6) << ","
			    << QString::number(row.z, 'f', 6) << "\n";
		else
			out << row.name << ","
			    << QString::number(row.x, 'f', 6) << ","
			    << QString::number(row.y, 'f', 6) << ","
			    << QString::number(row.z, 'f', 6) << "\n";
	}
}
