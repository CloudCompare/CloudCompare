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

#include "ccScalarFieldsManagerDlg.h"

#include "ui_scalarFieldsManagerDlg.h"

// local
#include "ccHistogramWindow.h"
#include "mainwindow.h"

// qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

// Qt
#include <QInputDialog>

// System
#include <cassert>

ccScalarFieldsManagerDialog::ccScalarFieldsManagerDialog(const ccHObject::Container& selectedEntities,
                                                         QWidget*                    parent /*=nullptr*/)
    : QDialog(parent)
    , m_pointCloud(nullptr)
    , m_sfCount(0)
    , m_ui(new Ui::ScalarFieldsManagerDlg)
{
	m_ui->setupUi(this);
	m_ui->sfTableWidget->verticalHeader()->setVisible(true);                     // enable row numbering
	m_ui->sfTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);    // highlight entire row on click
	m_ui->sfTableWidget->setSelectionMode(QAbstractItemView::ExtendedSelection); // ctrl/shift multi-selection
	m_ui->deleteSFButton->setEnabled(false);                                     // disable until a row is selected
	m_ui->showHistogramButton->setEnabled(false);                                // disable until a row is selected

	connect(m_ui->deleteSFButton, &QPushButton::clicked, this, &ccScalarFieldsManagerDialog::deleteSF);
	connect(m_ui->addConstantSFButton, &QPushButton::clicked, this, &ccScalarFieldsManagerDialog::addConstantSF);
	connect(m_ui->showHistogramButton, &QPushButton::clicked, this, &ccScalarFieldsManagerDialog::showHistogram);
	connect(m_ui->arithmeticButton, &QPushButton::clicked, this, &ccScalarFieldsManagerDialog::doArithmetic);

	connect(m_ui->entityComboBox, &QComboBox::currentIndexChanged, this, &ccScalarFieldsManagerDialog::onEntityChanged);

	// monitor table selections to enable buttons
	connect(m_ui->sfTableWidget->selectionModel(), &QItemSelectionModel::selectionChanged, this, [this]()
	        {
            bool hasSelection = m_ui->sfTableWidget->selectionModel()->hasSelection();
            m_ui->deleteSFButton->setEnabled(hasSelection);
            m_ui->showHistogramButton->setEnabled(hasSelection);
			m_ui->arithmeticButton->setEnabled(hasSelection); });

	// save edits when the user renames a scalar field in the table
	connect(m_ui->sfTableWidget, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item)
	        {
			// since the first column is the only editable one, 
			// save the remaning back to the point cloud
			if (item && item->column() == SFAttributes::NAME) 
			{
				this->renameSF(item->row(), item->text().trimmed());
			} });

	// filter and populate the active entities dropdown
	setSelectedEntities(selectedEntities);
}

ccScalarFieldsManagerDialog::~ccScalarFieldsManagerDialog()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccScalarFieldsManagerDialog::setSelectedEntities(const ccHObject::Container& entities)
{
	m_ui->entityComboBox->blockSignals(true);
	m_ui->entityComboBox->clear();
	m_availableClouds.clear();

	// loop through selected items and pull out only the point clouds
	for (ccHObject* entity : entities)
	{
		if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(entity);
			m_availableClouds.push_back(pc);

			// add the cloud's name to the dropdown display
			m_ui->entityComboBox->addItem(pc->getName());
		}
	}

	m_ui->entityComboBox->blockSignals(false);

	// initialize on the first cloud found
	m_ui->entityComboBox->setCurrentIndex(0);

	setActivePointCloud(!m_availableClouds.empty() ? m_availableClouds.front() : nullptr);
}

void ccScalarFieldsManagerDialog::onEntityChanged(int index)
{
	if (index >= 0 && static_cast<size_t>(index) < m_availableClouds.size())
	{
		setActivePointCloud(m_availableClouds[index]);
	}
	else
	{
		setActivePointCloud(nullptr);
	}
}

void ccScalarFieldsManagerDialog::setActivePointCloud(ccPointCloud* pc)
{
	m_pointCloud = pc;
	m_sfCount    = (m_pointCloud ? m_pointCloud->getNumberOfScalarFields() : 0);
	buildTable();
}

void ccScalarFieldsManagerDialog::updateDisplay()
{
	m_pointCloud->showSF(true);
	MainWindow::UpdateUI();
	MainWindow::RefreshAllGLWindow(false);
}

void ccScalarFieldsManagerDialog::buildTable()
{
	m_ui->sfTableWidget->blockSignals(true);
	m_ui->sfTableWidget->setSortingEnabled(false);

	m_ui->sfTableWidget->clearContents();
	m_ui->sfTableWidget->setRowCount(0);

	// iterate over scalar fields and append them to the table
	for (unsigned i = 0; i < m_sfCount; ++i)
	{
		appendSFToTable(i);
	}

	m_ui->sfTableWidget->setSortingEnabled(true);
	m_ui->sfTableWidget->blockSignals(false);
}

void ccScalarFieldsManagerDialog::appendSFToTable(int sfIdx)
{
	ccScalarField* sf = static_cast<ccScalarField*>(m_pointCloud->getScalarField(sfIdx));
	if (!sf)
	{
		return;
	}

	m_ui->sfTableWidget->blockSignals(true); // otherwise the renameSF method will be called

	// add a new row to the table
	int rowCount = m_ui->sfTableWidget->rowCount();
	m_ui->sfTableWidget->insertRow(rowCount);

	// compute statistics
	auto name = QString::fromStdString(sf->getName());
	sf->computeMinAndMax();
	auto       minVal = sf->getMin();
	auto       maxVal = sf->getMax();
	ScalarType mean   = 0;
	ScalarType var    = 0;
	sf->computeMeanAndVariance(mean, &var);
	double stdDev = std::sqrt(static_cast<double>(var));

	// Helper lambda to create read-only items with background
	auto createReadOnlyItem = [](const QString& text)
	{
		QTableWidgetItem* item = new QTableWidgetItem(text);
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
		// light grey backrgound for read-only cells
		item->setBackground(Qt::lightGray);
		item->setForeground(Qt::black);
		return item;
	};

	// SF Name (editable)
	QTableWidgetItem* nameItem = new QTableWidgetItem(name);
	nameItem->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	m_ui->sfTableWidget->setItem(sfIdx, SFAttributes::NAME, nameItem);

	// MinValue (read-only)
	m_ui->sfTableWidget->setItem(sfIdx, SFAttributes::MINVAL, createReadOnlyItem(QString::number(minVal, 'f', 6)));

	// MaxValue (read-only)
	m_ui->sfTableWidget->setItem(sfIdx, SFAttributes::MAXVAL, createReadOnlyItem(QString::number(maxVal, 'f', 6)));

	// Mean (read-only)
	m_ui->sfTableWidget->setItem(sfIdx, SFAttributes::MEAN, createReadOnlyItem(QString::number(mean, 'f', 6)));

	// Std (read-only)
	m_ui->sfTableWidget->setItem(sfIdx, SFAttributes::STD, createReadOnlyItem(QString::number(stdDev, 'f', 6)));

	m_ui->sfTableWidget->blockSignals(false);
}

void ccScalarFieldsManagerDialog::addConstantSF()
{
	QString  defaultName = tr("Constant");
	unsigned trys        = 1;
	while (m_pointCloud->getScalarFieldIndexByName(defaultName.toStdString()) >= 0 || trys > 99)
	{
		defaultName = tr("Constant") + QString(" #%1").arg(++trys);
	}

	// ask for a name
	bool    ok     = false;
	QString sfName = QInputDialog::getText(this, tr("New SF name"), tr("SF name (must be unique)"), QLineEdit::Normal, defaultName, &ok);
	if (!ok)
	{
		return;
	}

	if (ccEntityAction::sfAddConstant(m_pointCloud, sfName, false, this))
	{
		appendSFToTable(m_sfCount++);
		m_pointCloud->redrawDisplay();
	}

	updateDisplay();

	ccLog::Print(QString("[SF Manager] Added constant scalar field '%1' to '%2'").arg(sfName).arg(m_pointCloud->getName()));
}

void ccScalarFieldsManagerDialog::deleteSF()
{
	// check which row is currently selected
	int row = m_ui->sfTableWidget->currentRow();
	if (row < 0)
	{
		ccLog::Warning("Delete SF", "Please select a scalar field from the table first.");
		return;
	}

	// handle multi-row selecton
	QModelIndexList selectedRows = m_ui->sfTableWidget->selectionModel()->selectedRows();
	if (selectedRows.isEmpty())
	{
		return;
	}

	// iterate backwards to avoid index shifting issues when deleting rows
	for (int i = selectedRows.count() - 1; i >= 0; --i)
	{
		int row = selectedRows[i].row();

		// Get the SF name from the selected row
		QTableWidgetItem* nameItem = m_ui->sfTableWidget->item(row, SFAttributes::NAME);
		QString           sfName   = nameItem->text();

		// delete scalar field
		int sfIdx = m_pointCloud->getScalarFieldIndexByName(sfName.toStdString());
		m_pointCloud->deleteScalarField(sfIdx);
		m_sfCount -= 1;

		ccLog::Print(QString("[SF Manager] Deleted '%1' from '%2'").arg(sfName, m_pointCloud->getName()));
		m_ui->sfTableWidget->removeRow(row);
	}

	updateDisplay();

	// re-unable delete button
	m_ui->deleteSFButton->setEnabled(false);
}

void ccScalarFieldsManagerDialog::showHistogram()
{
	// check which row is currently selected
	int row = m_ui->sfTableWidget->currentRow();
	if (row < 0)
	{
		ccLog::Warning("Delete SF", "Please select a scalar field from the table first.");
		return;
	}

	// handle multi-row selecton
	QModelIndexList selectedRows = m_ui->sfTableWidget->selectionModel()->selectedRows();
	if (selectedRows.isEmpty())
	{
		return;
	}

	for (int i = 0; i < selectedRows.count(); ++i)
	{
		// Get the SF info and data
		int            sfIdx = selectedRows[i].row();
		ccScalarField* sf    = static_cast<ccScalarField*>(m_pointCloud->getScalarField(sfIdx));
		if (!sf)
		{
			continue;
		}

		// we display the histogram of the current scalar field
		if (sf)
		{
			ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
			hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
			hDlg->setWindowTitle(tr("Histogram [%1]").arg(m_pointCloud->getName()));

			ccHistogramWindow* histogram = hDlg->window();
			{
				unsigned numberOfPoints  = m_pointCloud->size();
				unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
				// we take the 'nearest' multiple of 4
				numberOfClasses &= (~3);
				numberOfClasses = std::max<unsigned>(4, numberOfClasses);
				numberOfClasses = std::min<unsigned>(256, numberOfClasses);

				histogram->setTitle(tr("%1 (%2 values) ").arg(QString::fromStdString(sf->getName())).arg(numberOfPoints));
				bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
				histogram->fromSF(sf, numberOfClasses, true, showNaNValuesInGrey);
				histogram->setAxisLabels(QString::fromStdString(sf->getName()), tr("Count"));
				histogram->refresh();
			}
			hDlg->show();
		}
	}
}

void ccScalarFieldsManagerDialog::renameSF(int row, const QString& newName)
{
	ccScalarField* sf = static_cast<ccScalarField*>(m_pointCloud->getScalarField(row));
	if (!sf)
	{
		return;
	}

	QString oldName = QString::fromStdString(sf->getName());
	m_pointCloud->renameScalarField(row, newName.toStdString());

	updateDisplay();

	ccLog::Print(QString("[SF Manager] Renamed '%1' to '%2' in '%3'").arg(oldName, newName, m_pointCloud->getName()));
}

void ccScalarFieldsManagerDialog::doArithmetic()
{
	ccHObject::Container selectedEntities;
	selectedEntities.push_back(m_pointCloud);

	if (ccEntityAction::sfArithmetic(selectedEntities, this))
	{
		appendSFToTable(m_sfCount++);
		m_pointCloud->redrawDisplay();
	}

	updateDisplay();
}
