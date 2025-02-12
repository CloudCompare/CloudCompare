#include "../include/ccCloudLayersDlg.h"

#include "../include/ccColorStyledDelegate.h"
#include "../include/ccMouseCircle.h"

//QT
#include <QColorDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QSettings>
#include <QSortFilterProxyModel>
#include <QWidget>

//CC
#include <ccGLWindowInterface.h>
#include <ccMainAppInterface.h>
#include <ccPointCloud.h>

static QStringList s_presets{ QObject::tr("All Points"), QObject::tr("Visible Points") };

ccCloudLayersDlg::ccCloudLayersDlg(ccMainAppInterface* app, QWidget* parent/*=nullptr*/)
	: ccOverlayDialog(parent)
	, Ui::ccCloudLayersDlg()
	, m_app(app)
	, m_helper(nullptr)
	, m_mouseCircle(new ccMouseCircle(app ? app->getActiveGLWindow() : nullptr))
{
	setupUi(this);

	setWindowTitle(tr("Cloud layers"));

	// allow resize and move window
	setWindowFlags(Qt::Tool | Qt::CustomizeWindowHint);

	// set model to tableView
	initTableView();

	// connect buttons
	connect(pbAdd, &QPushButton::clicked, this, &ccCloudLayersDlg::addClicked);
	connect(pbDelete, &QPushButton::clicked, this, &ccCloudLayersDlg::deleteClicked);
	connect(pbStart, &QPushButton::clicked, this, &ccCloudLayersDlg::startClicked);
	connect(pbPause, &QPushButton::clicked, this, &ccCloudLayersDlg::pauseClicked);
	connect(pbApply, &QPushButton::clicked, this, &ccCloudLayersDlg::applyClicked);
	connect(pbClose, &QPushButton::clicked, this, &ccCloudLayersDlg::closeClicked);

	// connect comboboxes
	connect(cbScalarField, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccCloudLayersDlg::scalarFieldIndexChanged);
	connect(cbInput, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccCloudLayersDlg::inputClassIndexChanged);
	connect(cbOutput, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccCloudLayersDlg::outputClassIndexChanged);

	// color picker
	connect(tableView, &QTableView::doubleClicked, this, &ccCloudLayersDlg::tableViewDoubleClicked);

	// asprs model changed signals
	connect(&m_asprsModel, &ccAsprsModel::codeChanged, this, &ccCloudLayersDlg::codeChanged);
	connect(&m_asprsModel, &ccAsprsModel::colorChanged, this, &ccCloudLayersDlg::colorChanged);
	connect(&m_asprsModel, &ccAsprsModel::classNamedChanged, this, &ccCloudLayersDlg::classNameChanged);

	m_mouseCircle->setVisible(false);
}

ccCloudLayersDlg::~ccCloudLayersDlg()
{
	if (m_mouseCircle)
	{
		delete m_mouseCircle;
		m_mouseCircle = nullptr;
	}
}

bool ccCloudLayersDlg::start()
{
	if (!m_helper)
	{
		return false;
	}

	m_asprsModel.load();
	updateInputOutput();
	loadSettings();

	resetUI();
	m_app->freezeUI(true);

	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::mouseMoved, this, &ccCloudLayersDlg::mouseMoved);

	return ccOverlayDialog::start();
}

void ccCloudLayersDlg::stop(bool accepted)
{
	if (m_mouseCircle && m_mouseCircle->isVisible())
	{
		pauseClicked();
	}

	setPointCloud(nullptr);

	if (m_app)
	{
		m_app->freezeUI(false);
	}

	ccOverlayDialog::stop(accepted);
}

bool ccCloudLayersDlg::setPointCloud(ccPointCloud* cloud)
{
	if (m_helper)
	{
		delete m_helper;
		m_helper = nullptr;
	}

	cbScalarField->clear();

	if (cloud)
	{
		m_helper = new ccCloudLayersHelper(m_app);

		if (!m_helper->setCloud(cloud))
		{
			// error message already issued
			delete m_helper;
			m_helper = nullptr;
			return false;
		}

		cbScalarField->blockSignals(true);
		cbScalarField->addItems(m_helper->getScalarFields());
		cbScalarField->setCurrentIndex(m_helper->getCurrentScalarFieldIndex());
		cbScalarField->blockSignals(false);
	}

	return true;
}

void ccCloudLayersDlg::resetUI()
{
	pbStart->setEnabled(true);
	pbPause->setEnabled(false);
}

void ccCloudLayersDlg::initTableView()
{
	QSortFilterProxyModel* proxyModel = new QSortFilterProxyModel(this);
	proxyModel->setSourceModel(&m_asprsModel);
	tableView->setModel(proxyModel);
	tableView->setSortingEnabled(true);
	tableView->sortByColumn(ccAsprsModel::CODE, Qt::AscendingOrder);

	// set column delegates
	tableView->setItemDelegateForColumn(ccAsprsModel::COLOR, new ccColorStyledDelegate(this));

	tableView->horizontalHeader()->setSectionResizeMode(ccAsprsModel::NAME, QHeaderView::Stretch);
	tableView->horizontalHeader()->setSectionResizeMode(ccAsprsModel::VISIBLE, QHeaderView::ResizeToContents);
	tableView->horizontalHeader()->setSectionResizeMode(ccAsprsModel::CODE, QHeaderView::ResizeToContents);
	tableView->horizontalHeader()->setSectionResizeMode(ccAsprsModel::COLOR, QHeaderView::ResizeToContents);
	tableView->horizontalHeader()->setSectionResizeMode(ccAsprsModel::COUNT, QHeaderView::ResizeToContents);
}

void ccCloudLayersDlg::saveSettings()
{
	QSettings settings;
	settings.beginGroup("qCloudLayers");
	{
		if (cbScalarField->currentIndex() >= 0)
			settings.setValue("ScalarField", cbScalarField->currentText());
		if (cbInput->currentIndex() >= 0)
			settings.setValue("InputClass", cbInput->currentText());
		if (cbOutput->currentIndex() >= 0)
			settings.setValue("OutputClass", cbOutput->currentText());

		settings.setValue("keepRGBOnExit", keepRGBColorsCheckBox->isChecked());

		settings.beginGroup("Window");
		{
			settings.setValue("geometry", saveGeometry());
		}
		settings.sync();
	}
	settings.endGroup();
}

void ccCloudLayersDlg::loadSettings()
{
	if (!m_helper)
	{
		return;
	}

	QSettings settings;
	settings.beginGroup("qCloudLayers");
	{
		QString sfName = settings.value("ScalarField").toString();

		int sfIndex = m_helper->getScalarFields().indexOf(sfName);
		if (sfIndex < 0)
		{
			// previous scalar field not found
			sfName = "Classification";
			sfIndex = m_helper->getScalarFields().indexOf(sfName);
			if (sfIndex < 0)
			{
				// we'll take the first one
				sfIndex = 0;
			}
		}

		cbScalarField->setCurrentIndex(sfIndex);

		QString inputName = settings.value("InputClass").toString();
		const ccAsprsModel::AsprsItem* item = m_asprsModel.find(inputName);

		if (!item)
		{
			// selected index is (hopefully) part of the presets
			int index = s_presets.indexOf(inputName);
			cbInput->setCurrentIndex(std::max(0, index));
		}
		else
		{
			cbInput->setCurrentIndex(m_asprsModel.indexOf(inputName) + s_presets.size());
		}

		QString outputName = settings.value("OutputClass").toString();
		int outIndex = m_asprsModel.indexOf(outputName);
		cbOutput->setCurrentIndex(outIndex == -1 ? 0 : outIndex);

		keepRGBColorsCheckBox->setChecked(settings.value("keepRGBOnExit", false).toBool());

		settings.beginGroup("Window");
		{
			restoreGeometry(settings.value("geometry").toByteArray());
		}
		settings.endGroup();
	}
	settings.endGroup();
}

void ccCloudLayersDlg::addClicked()
{
	QModelIndex index = m_asprsModel.createNewItem();
	tableView->selectRow(index.row());
	tableView->setCurrentIndex(index);

	updateInputOutput();
}

void ccCloudLayersDlg::deleteClicked()
{
	QItemSelectionModel* select = tableView->selectionModel();
	if (!select->hasSelection())
		return;

	if (QMessageBox::question(	m_associatedWin->asWidget(),
								tr("Cloud layers"),
								tr("Are you sure you want to delete this record(s)?"),
								QMessageBox::Yes,
								QMessageBox::No)
		== QMessageBox::No)
	{
		return;
	}

	QModelIndexList mapIndices = select->selectedIndexes();
	qSort(mapIndices);

	QModelIndexList sourceIndices;
	for (QModelIndex index : mapIndices)
	{
		QModelIndex sourceIndex = static_cast<QSortFilterProxyModel*>(tableView->model())->mapToSource(index);
		sourceIndices.append(sourceIndex);
	}

	ccAsprsModel::AsprsItem* to = m_asprsModel.getData().size() > 0 ? &(m_asprsModel.getData().front()) : nullptr;
	for (int i = mapIndices.size(); i > 0; --i)
	{
		ccAsprsModel::AsprsItem& from = m_asprsModel.getData()[sourceIndices[i - 1].row()];
		int affected = m_helper ? m_helper->moveItem(from, to) : 0;
		if (to)
		{
			to->count += affected;
		}

		tableView->model()->removeRows(mapIndices[i - 1].row(), 1);
	}

	updateInputOutput();

	m_associatedWin->redraw();
}

void ccCloudLayersDlg::startClicked()
{
	if (nullptr == m_app->getActiveGLWindow())
	{
		return;
	}

	m_app->getActiveGLWindow()->setPickingMode(ccGLWindowInterface::PICKING_MODE::NO_PICKING);

	//set orthographic view (as this tool doesn't work in perspective mode)
	m_app->getActiveGLWindow()->setPerspectiveState(false, true);
	m_app->getActiveGLWindow()->setInteractionMode(ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
	m_mouseCircle->setVisible(true);

	pbStart->setEnabled(false);
	pbPause->setEnabled(true);
}

void ccCloudLayersDlg::pauseClicked()
{
	if (nullptr == m_app->getActiveGLWindow())
	{
		return;
	}

	m_mouseCircle->setVisible(false);
	m_app->getActiveGLWindow()->setPickingMode(ccGLWindowInterface::PICKING_MODE::DEFAULT_PICKING);
	m_app->getActiveGLWindow()->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA);
	m_app->getActiveGLWindow()->redraw(true, false);

	pbStart->setEnabled(true);
	pbPause->setEnabled(false);
}

void ccCloudLayersDlg::applyClicked()
{
	m_asprsModel.save();
	
	saveSettings();
	
	if (m_helper)
	{
		m_helper->keepCurrentSFVisible();
		if (!keepRGBColorsCheckBox->isChecked())
		{
			m_helper->restoreCloud(false);
		}
		else
		{
			m_helper->setVisible(true);
		}
	}

	stop(true);
}

void ccCloudLayersDlg::reject()
{
	if (m_helper)
	{
		if (m_helper->modified())
		{
			if (QMessageBox::question(m_associatedWin->asWidget(),
				tr("Cloud layers"),
				tr("The cloud has been modified, are you sure you want exit?"),
				QMessageBox::Yes,
				QMessageBox::No)
				== QMessageBox::No)
			{
				return;
			}
		}

		m_helper->restoreCloud(true);
	}

	stop(false);
}

void ccCloudLayersDlg::mouseMoved(int x, int y, Qt::MouseButtons buttons)
{
	if (buttons != Qt::LeftButton)
	{
		return;
	}

	if (!m_helper)
	{
		return;
	}

	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);

	if (!m_helper->projectCloud(camera))
	{
		// error/warning message already issued
		return;
	}

	QPointF pos2D = m_app->getActiveGLWindow()->toCenteredGLCoordinates(x, y);
	CCVector2 center(static_cast<PointCoordinateType>(pos2D.x()), static_cast<PointCoordinateType>(pos2D.y()));

	int radius_px = m_mouseCircle->getRadiusPx();
	std::map<ScalarType, int> affected;
	m_helper->mouseMove(center, static_cast<PointCoordinateType>(radius_px * radius_px), affected);

	// update point counts
	for (const auto& it : affected)
	{
		auto item = m_asprsModel.find(it.first);
		if (item)
		{
			item->count += it.second;
		}
	}

	m_asprsModel.refreshData();
}

bool ccCloudLayersDlg::eventFilter(QObject* obj, QEvent* event)
{
	if (event->type() == QEvent::KeyPress)
	{
		QKeyEvent* ev = static_cast<QKeyEvent*>(event);
		if (ev->key() == Qt::Key::Key_Alt)
		{
			swapInputOutput();
		}
		if (ev->key() == Qt::Key::Key_Alt || ev->key() == Qt::Key::Key_Control || ev->key() == Qt::Key::Key_Shift)
		{
			// to avoid conflicts with the 3D window shortcuts
			m_mouseCircle->setAllowScroll(false);
		}

	}
	else if (event->type() == QEvent::KeyRelease)
	{
		QKeyEvent* ev = static_cast<QKeyEvent*>(event);
		if (ev->key() == Qt::Key::Key_Alt)
		{
			swapInputOutput();
		}
		if (ev->key() == Qt::Key::Key_Alt || ev->key() == Qt::Key::Key_Control || ev->key() == Qt::Key::Key_Shift)
		{
			m_mouseCircle->setAllowScroll(true);
		}
	}

	return false;
}

void ccCloudLayersDlg::scalarFieldIndexChanged(int index)
{
	if (m_helper)
	{
		bool proceed = true;
		if (m_helper->modified())
		{
			proceed = (QMessageBox::question(	m_associatedWin->asWidget(),
												tr("Cloud layers"),
												tr("Current modifications will be lost. Proceed?"),
												QMessageBox::Yes,
												QMessageBox::No)
						== QMessageBox::Yes);

			if (proceed)
			{
				m_helper->restoreCurrentSFValues();
			}
		}

		if (proceed && !m_helper->setScalarFieldIndexAndStoreValues(index))
		{
			ccLog::Error(tr("Not enough memory"));
			proceed = false;
		}

		if (!proceed)
		{
			// cancel change
			cbScalarField->blockSignals(true);
			cbScalarField->setCurrentIndex(m_helper->getCurrentScalarFieldIndex());
			cbScalarField->blockSignals(false);
			return;
		}

		m_helper->applyClassColors(m_asprsModel.getData());
	}

	// refresh point count
	m_asprsModel.refreshData();
}

void ccCloudLayersDlg::inputClassIndexChanged(int index)
{
	if (!m_helper)
	{
		return;
	}

	ccCloudLayersHelper::Parameters& params = m_helper->getParameters();
	if (cbInput->currentIndex() < 0)
	{
		params.anyPoints = false;
		params.visiblePoints = false;
		params.input = nullptr;

		return;
	}

	QString inputName = cbInput->itemText(cbInput->currentIndex());
	params.anyPoints = (inputName == s_presets[0]);
	params.visiblePoints = (inputName == s_presets[1]);
	params.input = nullptr;

	if (!(params.anyPoints || params.visiblePoints))
	{
		params.input = m_asprsModel.find(inputName);
	}
}

void ccCloudLayersDlg::outputClassIndexChanged(int index)
{
	if (!m_helper)
	{
		return;
	}
	ccCloudLayersHelper::Parameters& params = m_helper->getParameters();
	if (cbInput->currentIndex() < 0)
	{
		params.output = nullptr;
		return;
	}

	QString outputName = cbOutput->itemText(cbOutput->currentIndex());
	params.output = m_asprsModel.find(outputName);
}

void ccCloudLayersDlg::codeChanged(ccAsprsModel::AsprsItem item, int oldCode)
{
	if (m_helper)
	{
		m_helper->changeCode(item, static_cast<ScalarType>(oldCode));
	}
}

void ccCloudLayersDlg::colorChanged(ccAsprsModel::AsprsItem item)
{
	if (!m_helper)
	{
		return;
	}

	item.count = m_helper->applyClassColor(item, true);

	// refresh point count
	m_asprsModel.refreshData();
}

void ccCloudLayersDlg::classNameChanged(int row, QString newName)
{
	cbInput->setItemText(s_presets.size() + row, newName);
	cbOutput->setItemText(row, newName);
}

void ccCloudLayersDlg::tableViewDoubleClicked(const QModelIndex& index)
{
	if (index.column() != ccAsprsModel::COLOR)
	{
		return;
	}

	QColor currColor = index.model()->data(index, Qt::DisplayRole).value<QColor>();
	QColor color = QColorDialog::getColor(currColor, this, tr("Pick a color"), QColorDialog::DontUseNativeDialog);

	if (color.isValid() && color != currColor)
	{
		tableView->model()->setData(index, color, Qt::EditRole);
	}
}

void ccCloudLayersDlg::updateInputOutput()
{
	auto data = m_asprsModel.getData();

	cbInput->clear();
	cbInput->addItems(s_presets);
	for (int i = 0; i < data.size(); ++i)
	{
		cbInput->addItem(data[i].name);
	}

	cbOutput->clear();
	for (int i = 0; i < data.size(); ++i)
	{
		cbOutput->addItem(data[i].name);
	}
}

void ccCloudLayersDlg::swapInputOutput()
{
	int inputIndex = cbInput->currentIndex();
	int outputIndex = cbOutput->currentIndex();

	if (inputIndex < 0 || outputIndex < 0)
	{
		return;
	}

	QString text = cbInput->itemText(cbInput->currentIndex());
	bool isPreset = s_presets.contains(text);

	if (!isPreset)
	{
		cbInput->setCurrentIndex(outputIndex + s_presets.size());
		cbOutput->setCurrentIndex(inputIndex - s_presets.size());
	}

	inputIndex = cbInput->currentIndex();
	outputIndex = cbOutput->currentIndex();
}
