//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#include "stereogramDlg.h"

//Local
#include "facetsClassifier.h"

// Qt
#include <QPainter>
#include <QMainWindow>
#include <QSpacerItem>

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccColorScalesManager.h>
#include <ccColorScaleSelector.h>
#include <ccColorScaleEditorDlg.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>

//! Density grid
class FacetDensityGrid
{
public:

	//! Default constructor
	FacetDensityGrid()
		: grid(nullptr)
		, rSteps(0)
		, ddSteps(0)
		, step_R(0)
		, step_deg(0)
	{
		minMaxDensity[0] = minMaxDensity[1] = 0;
	}

	//! Destructor
	~FacetDensityGrid()
	{
		if (grid)
			delete[] grid;
	}

	//! Grid
	double* grid;

	//! Dimension (X)
	unsigned rSteps;
	//! Dimension (Y)
	unsigned ddSteps;

	//! Grid step along X (in unit)
	double step_R;
	//! Grid step along Y (in degrees)
	double step_deg;

	//! Min/max density
	double minMaxDensity[2];
};

StereogramWidget::StereogramWidget(QWidget *parent)
	: QLabel(QString(), parent)
	, m_angularStep_deg(0)
	, m_densityGrid(nullptr)
	, m_meanDipDir_deg(-1.0)
	, m_meanDip_deg(-1.0)
	, m_densityColorScale(nullptr)
	, m_densityColorScaleSteps(ccColorScale::MAX_STEPS < 256 ? ccColorScale::MAX_STEPS : 256) //DGM: we can't pass a constant initializer (MAX_STEPS) by reference
	, m_ticksFreq(3)
	, m_showHSVRing(false)
	, m_trackMouseClick(false)
	, m_clickDip_deg(0.0)
	, m_clickDipDir_deg(0.0)
	, m_clickDipSpan_deg(30.0)
	, m_clickDipDirSpan_deg(30.0)
	, m_center(0, 0)
	, m_radius(0)
{
	setVisible(true);
	QSizePolicy policy = sizePolicy();
	policy.setHeightForWidth(true);
	setSizePolicy(policy);
	//setFixedSize(256,256);
}

StereogramWidget::~StereogramWidget()
{
	if (m_densityGrid)
		delete m_densityGrid;
}

bool StereogramWidget::init(double angularStep_deg,
							ccHObject* entity,
							double resolution_deg/*=2.0*/)
{
	m_angularStep_deg = angularStep_deg;

	if (m_densityGrid)
		delete m_densityGrid;
	m_densityGrid = nullptr;

	if (!entity)
		return false;

	ccProgressDialog pDlg(true);
	pDlg.setMethodTitle(QObject::tr("Stereogram"));
	pDlg.setInfo(QObject::tr("Preparing polar display..."));
	pDlg.start();
	QApplication::processEvents();

	size_t count = 0;
	ccHObject::Container facets;
	ccHObject::Container planes;
	ccPointCloud* cloud = nullptr;

	//a set of facets or planes?
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//check for facets first
		entity->filterChildren(facets,true,CC_TYPES::FACET);
		count = facets.size();
		if (count == 0) //if no facets, look for planes instead
		{
			entity->filterChildren(planes, true, CC_TYPES::PLANE);
			count = planes.size();
		}
	}
	//or a cloud?
	else if (entity->isA(CC_TYPES::POINT_CLOUD))
	{
		cloud = static_cast<ccPointCloud*>(entity);
		if (cloud->hasNormals())
			count = cloud->size();
	}

	if (!count)
		return false;

	//pDlg.setMaximum(static_cast<int>(count));
	CCLib::NormalizedProgress nProgress(&pDlg,static_cast<unsigned>(count));

	//create the density grid
	FacetDensityGrid* densityGrid = new FacetDensityGrid();
	densityGrid->step_deg = resolution_deg;
	densityGrid->step_R = 0.02;

	//dip steps (dip in [0,90])
	//densityGrid->dSteps = static_cast<unsigned>(ceil(90.0 / densityGrid->step_deg));
	//R steps (R in [0,1])
	densityGrid->rSteps = static_cast<unsigned>(ceil(1.0 / densityGrid->step_R));
	//dip direction steps (dip dir. in [0,360])
	densityGrid->ddSteps = static_cast<unsigned>(ceil(360.0 / densityGrid->step_deg));

	unsigned cellCount = densityGrid->rSteps * densityGrid->ddSteps;
	densityGrid->grid = new double[cellCount];
	if (densityGrid->grid)
	{
		memset(densityGrid->grid,0,sizeof(double)*cellCount);

		CCVector3d Nmean(0,0,0);
		double surfaceSum = 0.0;

		for (size_t i=0; i<count; ++i)
		{
			CCVector3 N;
			double weight = 1.0;
			if (cloud) //we're using cloud normals
			{
				N = cloud->getPointNormal(static_cast<unsigned>(i));
			}
			else if (facets.size() != 0) //we're using facets
			{
				ccFacet* facet = static_cast<ccFacet*>(facets[i]);
				N = facet->getNormal();
				weight = facet->getSurface();
			}
			else if (planes.size() != 0) //we're using planes
			{
				ccPlane* p = static_cast<ccPlane*>(planes[i]);
				N = p->getNormal();
				//n.b. planes all carry equal weight
			}

			Nmean.x += static_cast<double>(N.x) * weight;
			Nmean.y += static_cast<double>(N.y) * weight;
			Nmean.z += static_cast<double>(N.z) * weight;
			surfaceSum += weight;

			PointCoordinateType dipDir = 0;
			PointCoordinateType dip = 0;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N,dip,dipDir);

			//unsigned iDip = static_cast<unsigned>(floor(static_cast<double>(dip)/densityGrid->step_deg));
			//if (iDip == densityGrid->dSteps)
			//	iDip--;
			unsigned iDipDir = static_cast<unsigned>(floor(static_cast<double>(dipDir)/densityGrid->step_deg));
			if (iDipDir == densityGrid->ddSteps)
				iDipDir--;

			double dip_rad = dip * CC_DEG_TO_RAD;
			double R = sin(dip_rad) / (1.0 + cos(dip_rad));

			unsigned iR = static_cast<unsigned>(floor(static_cast<double>(R)/densityGrid->step_R));
			if (iR == densityGrid->rSteps)
				iR--;

			densityGrid->grid[iR + iDipDir * densityGrid->rSteps] += weight;

			//pDlg.setValue(static_cast<int>(i));
			if (!nProgress.oneStep())
			{
				//process cancelled by user
				delete densityGrid;
				return false;
			}
		}

		if (surfaceSum > 0)
		{
			Nmean.normalize();
			CCVector3 N(static_cast<PointCoordinateType>(Nmean.x),
				static_cast<PointCoordinateType>(Nmean.y),
				static_cast<PointCoordinateType>(Nmean.z));

			PointCoordinateType dipDir = 0;
			PointCoordinateType dip = 0;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N,dip,dipDir);

			m_meanDipDir_deg = static_cast<double>(dipDir);
			m_meanDip_deg = static_cast<double>(dip);

			//set same value for the "filter" center
			m_clickDipDir_deg = m_meanDipDir_deg;
			m_clickDip_deg = m_meanDip_deg;
		}

		//compute min and max density
		{
			//DGM: only supported on C++x11!
			//std::pair<double*,double*> minmax = std::minmax_element(densityGrid->grid,densityGrid->grid+cellCount);
			//densityGrid->minMaxDensity[0] = *minmax.first;
			//densityGrid->minMaxDensity[1] = *minmax.second;

			densityGrid->minMaxDensity[0] = densityGrid->grid[0];
			densityGrid->minMaxDensity[1] = densityGrid->grid[0];
			for (unsigned j=1; j<cellCount; ++j)
			{
				if (densityGrid->grid[j] < densityGrid->minMaxDensity[0])
					densityGrid->minMaxDensity[0] = densityGrid->grid[j];
				else if (densityGrid->grid[j] > densityGrid->minMaxDensity[1])
					densityGrid->minMaxDensity[1] = densityGrid->grid[j];
			}
		}

		//pDlg.hide();
		pDlg.stop();
		QApplication::processEvents();
	}
	else
	{
		//not enough memory!
		delete densityGrid;
		densityGrid = nullptr;
	}

	//replace old grid by new one! (even in case of failure! See below)
	m_densityGrid = densityGrid;

	update();

	return true;
}

void StereogramWidget::mousePressEvent(QMouseEvent* e)
{
	if (m_trackMouseClick && m_radius > 0)
	{
		if (e->button() == Qt::LeftButton)
		{
			QRect contentRect = contentsRect();
			QPoint pos = e->pos() - contentRect.topLeft();
			QPoint AB = pos - m_center;
			int squareDistToCenter = AB.x() * AB.x() + AB.y() * AB.y();
			if (squareDistToCenter <= m_radius * m_radius)
			{
				//compute equivalent positions
				m_clickDip_deg = std::min(90.0, 90.0 * sqrt(static_cast<double>(squareDistToCenter)) / static_cast<double>(m_radius));
				m_clickDipDir_deg = atan2(static_cast<double>(AB.y()),static_cast<double>(AB.x())) * CC_RAD_TO_DEG;
				if (m_clickDipDir_deg < 0)
					m_clickDipDir_deg += 360.0;
				m_clickDipDir_deg += 90.0; //stereogram starts at 12 o'clock (not 3)
				if (m_clickDipDir_deg >= 360.0)
					m_clickDipDir_deg -= 360.0;

				emit pointClicked(m_clickDip_deg, m_clickDipDir_deg);
				e->accept();
				return;
			}
		}
	}

	e->ignore();
}

void StereogramWidget::enableMouseTracking(bool state, double dipSpan_deg/*=30*/, double dipDirSpan_deg/*=30*/)
{
	m_trackMouseClick = state;
	if (state)
	{
		m_clickDipSpan_deg = dipSpan_deg;
		m_clickDipDirSpan_deg = dipDirSpan_deg;
		//to give to the parent dialog some feedback about the current marker position!
		emit pointClicked(m_clickDip_deg, m_clickDipDir_deg);
	}
}

void StereogramWidget::setTrackedCenter(double dip_deg, double dipDir_deg)
{
	m_clickDip_deg = dip_deg;
	m_clickDipDir_deg = dipDir_deg;
}

void StereogramWidget::paintEvent(QPaintEvent* event)
{
	m_radius = 0; //means that nothing has been drawn (yet ;)

	QLabel::paintEvent(event);
	QPainter painter(this);
	painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing, true);

	//pen
	QPen pen;
	pen.setStyle(Qt::SolidLine);
	pen.setBrush(Qt::black);

	int diameter = std::min(width(), height());
	int halfW = width() / 2;
	int halfH = height() / 2;
	QPoint center(halfW, halfH);

	int hsvThickness = 0;
	if (m_showHSVRing)
	{
		int newDiameter = static_cast<int>(ceil(0.9 * diameter));
		hsvThickness = diameter - newDiameter;

		//TODO
		if (hsvThickness > 0)
		{
			QRect rectangle(center.x() - diameter / 2 + 1, center.y() - diameter / 2 + 1, diameter - 2, diameter - 2);
			int angle_span = static_cast<int>(m_angularStep_deg * 16.0); //see QPainter::drawPie
			QBrush brush;
			brush.setStyle(Qt::SolidPattern);
			painter.setPen(Qt::NoPen);

			//dip direction steps (dip dir. in [0,360])
			unsigned ddSteps = static_cast<unsigned>(ceil(360.0 / std::max(m_angularStep_deg,1.0)));
			for (unsigned j = 0; j < ddSteps; ++j)
			{
				double dipDir_deg = static_cast<double>(j) * m_angularStep_deg;

				//set family color
				ccColor::Rgb col;
				FacetsClassifier::GenerateSubfamilyColor(col, 90.0, dipDir_deg + 0.5 * m_angularStep_deg, 0, 1);
				brush.setColor(QColor(	static_cast<int>(col.r),
										static_cast<int>(col.g),
										static_cast<int>(col.b),
										255));
				painter.setBrush(brush);

				int angle_start = static_cast<int>((360.0 - dipDir_deg - m_angularStep_deg + 90.0) * 16.0); //see QPainter::drawPie
				painter.drawPie(rectangle, angle_start, angle_span);
			}
		}

		diameter = newDiameter; 
	}

	//outer circle
	pen.setWidth(2);
	painter.setPen(pen);
	painter.setBrush(Qt::white);
	int radius = diameter / 2 - 2;
	painter.drawEllipse(center,radius,radius);
	painter.setBrush(Qt::NoBrush);

	//keep track of the circle position
	m_radius = radius;
	m_center = center;

	//main axes
	painter.drawLine(center - QPoint(radius, 0), center + QPoint(radius, 0));
	painter.drawLine(center - QPoint(0, radius), center + QPoint(0, radius));

	//draw circles
	if (m_angularStep_deg > 0)
	{
		//dip steps (dip in [0,90])
		unsigned dSteps = static_cast<unsigned>(ceil(90.0 / m_angularStep_deg));
		//dip direction steps (dip dir. in [0,360])
		unsigned ddSteps = static_cast<unsigned>(ceil(360.0 / m_angularStep_deg));

		//draw inner circles
		pen.setWidth(1);
		pen.setColor(Qt::gray);
		painter.setPen(pen);
		for (unsigned i = 1; i < dSteps; ++i)
		{
			double dip_deg = i * m_angularStep_deg;
			if (dip_deg < 90.0)
			{
				int R = static_cast<int>(radius * (dip_deg / 90.0));
				if (R > 1)
					painter.drawEllipse(center, R - 1, R - 1);
			}
		}

		//draw rays (+ 'm_ticksFreq' times more ticks)
		int ticksFreq = std::max(m_ticksFreq, 1);
		for (unsigned j = 1; j <= ddSteps*ticksFreq; ++j)
		{
			double dipDir_deg = j * m_angularStep_deg / ticksFreq;
			if (dipDir_deg < 360.0)
			{
				QPoint X(	 static_cast<int>(sin(dipDir_deg * CC_DEG_TO_RAD) * radius),
							-static_cast<int>(cos(dipDir_deg * CC_DEG_TO_RAD) * radius) );

				if ((j % ticksFreq) == 0) //long ticks
					painter.drawLine(center, center + X);
				else
					painter.drawLine(center + X*0.93, center + X);
			}
		}
	}

	//draw density map
	if (m_densityGrid && m_densityColorScale && m_densityGrid->grid && m_densityGrid->minMaxDensity[1] != 0)
	{
		assert(m_densityColorScale);
		assert(m_densityGrid->grid);

		QBrush brush;
		brush.setStyle(Qt::SolidPattern);
		painter.setPen(Qt::NoPen);
		QPolygon poly(4);

		const double* d = m_densityGrid->grid;
		for (unsigned j = 0; j < m_densityGrid->ddSteps; ++j)
		{
			double dipDir0_rad = (j    ) * m_densityGrid->step_deg * CC_DEG_TO_RAD;
			double dipDir1_rad = (j + 1) * m_densityGrid->step_deg * CC_DEG_TO_RAD;
			double cos_dipDir0 = cos(dipDir0_rad);
			double sin_dipDir0 = sin(dipDir0_rad);
			double cos_dipDir1 = cos(dipDir1_rad);
			double sin_dipDir1 = sin(dipDir1_rad);

			for (unsigned i = 0; i < m_densityGrid->rSteps; ++i, ++d)
			{
				if (*d != 0)
				{
					double relPos = (*d)/ m_densityGrid->minMaxDensity[1];
					const ccColor::Rgb* col = m_densityColorScale->getColorByRelativePos(relPos, m_densityColorScaleSteps);
					brush.setColor(QColor(	static_cast<int>(col->r),
											static_cast<int>(col->g),
											static_cast<int>(col->b),
											255));
					painter.setBrush(brush);

					//stereographic projection
					double R0 = radius * ((i    ) * m_densityGrid->step_R);
					double R1 = radius * ((i + 1) * m_densityGrid->step_R);

					poly.setPoint(0, center + QPoint(static_cast<int>(sin_dipDir0 * R0), -static_cast<int>(cos_dipDir0 * R0)));
					poly.setPoint(1, center + QPoint(static_cast<int>(sin_dipDir0 * R1), -static_cast<int>(cos_dipDir0 * R1)));
					poly.setPoint(2, center + QPoint(static_cast<int>(sin_dipDir1 * R1), -static_cast<int>(cos_dipDir1 * R1)));
					poly.setPoint(3, center + QPoint(static_cast<int>(sin_dipDir1 * R0), -static_cast<int>(cos_dipDir1 * R0)));

					painter.drawPolygon(poly);
				}
			}
		}
	}

	//draw main 'dip direction'
	if (m_meanDipDir_deg >= 0)
	{
		pen.setWidth(2);
		pen.setColor(Qt::red);
		painter.setPen(pen);
		//draw main direction
		QPoint X(	 static_cast<int>(sin(m_meanDipDir_deg * CC_DEG_TO_RAD) * radius),
					-static_cast<int>(cos(m_meanDipDir_deg * CC_DEG_TO_RAD) * radius) );
		pen.setStyle(Qt::DashLine);
		painter.setPen(pen);
		painter.drawLine(center,center+X);

		//draw orthogonal to main direction
		QPoint Y(	static_cast<int>(cos(m_meanDipDir_deg * CC_DEG_TO_RAD) * radius),
					static_cast<int>(sin(m_meanDipDir_deg * CC_DEG_TO_RAD) * radius) );
		pen.setStyle(Qt::SolidLine);
		painter.setPen(pen);
		painter.drawLine(center-Y,center+Y);
	}

	//draw filter window around last cliked point
	if (m_trackMouseClick)
	{
		pen.setWidth(2);
		pen.setColor(Qt::magenta);
		painter.setPen(pen);
		//QBrush brush;
		//brush.setStyle(Qt::Dense6Pattern);
		//brush.setColor(Qt::red);
		//painter.setBrush(brush);
		painter.setBrush(Qt::NoBrush);

		double R0 = radius * (std::max(0.0, m_clickDip_deg - m_clickDipSpan_deg / 2) / 90.0);
		double R1 = radius * (std::min(90.0, m_clickDip_deg + m_clickDipSpan_deg / 2) / 90.0);

		//draw radial limits
		{
			QPoint X0(	 static_cast<int>(sin((m_clickDipDir_deg - m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R0),
						-static_cast<int>(cos((m_clickDipDir_deg - m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R0));
			QPoint X1(	 static_cast<int>(sin((m_clickDipDir_deg - m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R1),
						-static_cast<int>(cos((m_clickDipDir_deg - m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R1));
			painter.drawLine(center + X0, center + X1);
		}
		{
			QPoint X0(	 static_cast<int>(sin((m_clickDipDir_deg + m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R0),
						-static_cast<int>(cos((m_clickDipDir_deg + m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R0));
			QPoint X1(	 static_cast<int>(sin((m_clickDipDir_deg + m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R1),
						-static_cast<int>(cos((m_clickDipDir_deg + m_clickDipDirSpan_deg / 2) * CC_DEG_TO_RAD) * R1));
			painter.drawLine(center + X0, center + X1);
		}

		//draw concentric limits
		{
			int angle_start = static_cast<int>((360.0 - m_clickDipDir_deg - m_clickDipDirSpan_deg/2 + 90.0) * 16.0); //see QPainter::drawPie
			int angle_span = static_cast<int>(m_clickDipDirSpan_deg * 16.0); //see QPainter::drawPie

			QRectF rect0(	center.x() - R0,
							center.y() - R0,
							2*R0, 2*R0);
			painter.drawArc(rect0, angle_start, angle_span);

			QRectF rect1(	center.x() - R1,
							center.y() - R1,
							2*R1, 2*R1);
			painter.drawArc(rect1, angle_start, angle_span);
		}
	}
}

StereogramDialog::StereogramDialog(ccMainAppInterface* app)
	: QDialog( app->getMainWindow() )
	, Ui::StereogramDialog()
	, m_classifWidget(nullptr)
	, m_colorScaleSelector(nullptr)
	, m_app(app)
	, m_facetGroupUniqueID(0)
{
	setupUi(this);

	//create classification widget (stereogram)
	m_classifWidget = new StereogramWidget(this);
	{
		if (!frame->layout())
			frame->setLayout(new QHBoxLayout());
		frame->layout()->addWidget(m_classifWidget);
		m_classifWidget->setTicksFreq(ticksFreqSpinBox->value());
	}

	//add color ramp selector widget
	if (m_app)
	{
		ccColorScalesManager* csManager = m_app->getColorScalesManager();
		m_colorScaleSelector = new ccColorScaleSelector(csManager, this, QString::fromUtf8(":/CC/plugin/qFacets/images/gearIcon.png"));
		m_colorScaleSelector->init();
		ccColorScale::Shared scale = csManager->getDefaultScale(ccColorScalesManager::HIGH_CONTRAST);
		if (scale)
		{
			m_colorScaleSelector->setSelectedScale(scale->getUuid());
			m_classifWidget->setDensityColorScale(scale);
		}
		connect(m_colorScaleSelector, SIGNAL(colorScaleSelected(int)), this, SLOT(colorScaleChanged(int)));
		connect(m_colorScaleSelector, SIGNAL(colorScaleEditorSummoned()), this, SLOT(spawnColorScaleEditor()));
		//add selector to group's layout
		if (!colorRampGroupBox->layout())
			colorRampGroupBox->setLayout(new QHBoxLayout());
		colorRampGroupBox->layout()->addItem(new QSpacerItem(20, 20, QSizePolicy::Preferred, QSizePolicy::Minimum));
		colorRampGroupBox->layout()->addWidget(m_colorScaleSelector);
		colorScaleStepsSpinBox->setRange(ccColorScale::MIN_STEPS, ccColorScale::MAX_STEPS);
	}
	else
	{
		m_classifWidget->setDensityColorScale(ccColorScalesManager::GetDefaultScale());
	}

	connect(colorScaleStepsSpinBox,		SIGNAL(valueChanged(int)),				this,	SLOT(onDensityColorStepsChanged(int)));
	connect(ticksFreqSpinBox,			SIGNAL(valueChanged(int)),				this,	SLOT(onTicksFreqChanged(int)));
	connect(showHSVColorsCheckBox,		SIGNAL(toggled(bool)),					this,	SLOT(onHSVColorsToggled(bool)));

	//interactive filtering mechanism
	connect(filterFacetsGroupBox,		SIGNAL(toggled(bool)),					this,	SLOT(onFilterEnabled(bool)));
	connect(dipSpanDoubleSpinBox,		SIGNAL(valueChanged(double)),			this,	SLOT(onFilterSizeChanged(double)));
	connect(dipDirSpanDoubleSpinBox,	SIGNAL(valueChanged(double)),			this,	SLOT(onFilterSizeChanged(double)));
	connect(dipDoubleSpinBox,			SIGNAL(valueChanged(double)),			this,	SLOT(onFilterCenterChanged(double)));
	connect(dipDirDoubleSpinBox,		SIGNAL(valueChanged(double)),			this,	SLOT(onFilterCenterChanged(double)));
	connect(m_classifWidget,			SIGNAL(pointClicked(double, double)),	this,	SLOT(onPointClicked(double, double)));
	connect(exportPushButton,			SIGNAL(clicked()),						this,	SLOT(exportCurrentSelection()));
}

bool StereogramDialog::init(double angularStep_deg,
							ccHObject* facetGroup,
							double resolution_deg/*=2.0*/)
{
	if (!m_classifWidget)
		return false;

	if (!m_classifWidget->init(angularStep_deg, facetGroup, resolution_deg))
		return false;

	double meanDipDir_deg = 0.0;
	double meanDip_deg = 0.0;
	//set stereogram subtitle (i.e. mean direction)
	m_classifWidget->getMeanDir(meanDip_deg, meanDipDir_deg);
	meanDirLabel->setText(	QString("[Mean] ")
						+	ccNormalVectors::ConvertDipAndDipDirToString(	static_cast<PointCoordinateType>(meanDip_deg),
																			static_cast<PointCoordinateType>(meanDipDir_deg)) );

	//restore any old set state before forgetting it!
	bool filterModeEnabled = filterFacetsGroupBox->isChecked();
	if (m_facetGroupUniqueID != 0 && filterModeEnabled)
	{
		updateFacetsFilter(false);
	}
	m_facetGroupUniqueID = facetGroup->getUniqueID();
	if (filterModeEnabled)
	{
		updateFacetsFilter(true);
	}

	return true;
}

void StereogramDialog::closeEvent(QCloseEvent* e)
{
	//filter mode enabled?
	if (filterFacetsGroupBox->isChecked())
	{
		updateFacetsFilter(false);
	}
}

void StereogramDialog::onFilterEnabled(bool state)
{
	if (m_classifWidget)
	{
		m_classifWidget->enableMouseTracking(state, dipSpanDoubleSpinBox->value(), dipDirSpanDoubleSpinBox->value());
		m_classifWidget->update();
	}

	updateFacetsFilter(state);
}

void StereogramDialog::onFilterSizeChanged(double)
{
	if (m_classifWidget)
	{
		m_classifWidget->enableMouseTracking(filterFacetsGroupBox->isChecked(), dipSpanDoubleSpinBox->value(), dipDirSpanDoubleSpinBox->value());

		updateFacetsFilter(true);
	}
}

void StereogramDialog::onFilterCenterChanged(double)
{
	if (m_classifWidget)
	{
		m_classifWidget->setTrackedCenter(dipDoubleSpinBox->value(), dipDirDoubleSpinBox->value());
		m_classifWidget->update();

		updateFacetsFilter(true);
	}
}

void StereogramDialog::onPointClicked(double dip_deg, double dipDir_deg)
{
	//filter mode enabled?
	if (filterFacetsGroupBox->isChecked())
	{
		dipDoubleSpinBox->blockSignals(true);
		dipDirDoubleSpinBox->blockSignals(true);

		dipDoubleSpinBox->setValue(dip_deg);
		dipDirDoubleSpinBox->setValue(dipDir_deg);

		dipDoubleSpinBox->blockSignals(false);
		dipDirDoubleSpinBox->blockSignals(false);

		if (m_classifWidget)
			m_classifWidget->update();

		updateFacetsFilter(true);
	}
}

void StereogramDialog::updateFacetsFilter(bool enable)
{
	if (!m_app)
		return;

	//try to find the associated entity
	ccHObject* root = m_app->dbRootObject();
	ccHObject* entity = (root ? root->find(m_facetGroupUniqueID) : nullptr);
	if (!entity)
		return;

	PointCoordinateType dipFilter = static_cast<PointCoordinateType>(dipDoubleSpinBox->value());
	PointCoordinateType dipDirFilter = static_cast<PointCoordinateType>(dipDirDoubleSpinBox->value());

	PointCoordinateType halfDipSpan = static_cast<PointCoordinateType>(dipSpanDoubleSpinBox->value())/2;
	PointCoordinateType halfDipDirSpan = static_cast<PointCoordinateType>(dipDirSpanDoubleSpinBox->value())/2;

	//a set of facets?
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		ccHObject::Container facets;
		entity->filterChildren(facets, true, CC_TYPES::FACET);
		if (facets.empty())
			return;

		for (size_t i = 0; i < facets.size(); ++i)
		{
			ccFacet* facet = static_cast<ccFacet*>(facets[i]);

			bool visible = true;
			if (enable)
			{
				CCVector3 N = facet->getNormal();
				PointCoordinateType dip = 0;
				PointCoordinateType dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

				double dDip = fabs(dip - dipFilter);
				double dDipDir = fabs(dipDir - dipDirFilter);

				visible = (	(	dDip	<= halfDipSpan		|| dDip		>= 360.0 - halfDipSpan)
						&&	(	dDipDir	<= halfDipDirSpan	|| dDipDir	>= 360.0 - halfDipDirSpan) );
			}

			//facet->setEnabled(visible);
			ccMesh* polygon = facet->getPolygon();
			if (polygon)
			{
				polygon->setVisible(visible);
			}
			ccPolyline* polyline = facet->getContour();
			if (polyline)
			{
				//polyline->showColors(visible);
				polyline->setWidth(static_cast<PointCoordinateType>(visible ? 2 : 1));
			}
		}
	}
	//or a cloud?
	else if (entity->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		if (!cloud->hasNormals())
			return;

		if (enable)
		{
			unsigned count = cloud->size();
			if (count == 0)
				return;
			if (!cloud->resetVisibilityArray())
			{
				m_app->dispToConsole("Not enough memory!");
				return;
			}
			ccPointCloud::VisibilityTableType& visTable = cloud->getTheVisibilityArray();
			assert(!visTable.empty());

			for (unsigned i = 0; i < static_cast<unsigned>(count); ++i)
			{
				CCVector3 N = cloud->getPointNormal(i);
				PointCoordinateType dip = 0;
				PointCoordinateType dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

				double dDip = fabs(dip - dipFilter);
				double dDipDir = fabs(dipDir - dipDirFilter);

				bool visible = (	(	dDip	<= halfDipSpan		|| dDip		>= 360.0 - halfDipSpan)
								&&	(	dDipDir	<= halfDipDirSpan	|| dDipDir	>= 360.0 - halfDipDirSpan) );

				visTable[i] = (visible ? POINT_VISIBLE : POINT_HIDDEN);
			}
		}
		else
		{
			cloud->unallocateVisibilityArray();
		}
	}

	if (m_app)
		m_app->redrawAll();
}

void StereogramDialog::exportCurrentSelection()
{
	if (!m_app)
		return;

	//try to find the associated entity
	ccHObject* root = m_app->dbRootObject();
	ccHObject* entity = (root ? root->find(m_facetGroupUniqueID) : nullptr);
	if (!entity)
		return;

	PointCoordinateType dipFilter = static_cast<PointCoordinateType>(dipDoubleSpinBox->value());
	PointCoordinateType dipDirFilter = static_cast<PointCoordinateType>(dipDirDoubleSpinBox->value());

	PointCoordinateType halfDipSpan = static_cast<PointCoordinateType>(dipSpanDoubleSpinBox->value()) / 2;
	PointCoordinateType halfDipDirSpan = static_cast<PointCoordinateType>(dipDirSpanDoubleSpinBox->value()) / 2;

	QString selectionSuffix = QString(" [dip=(%1 - %2)][dipDir=(%3 - %4)]").arg(dipFilter - halfDipSpan).arg(dipFilter + halfDipSpan).arg(dipDirFilter - halfDipDirSpan).arg(dipDirFilter + halfDipDirSpan);

	//a set of facets?
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		ccHObject::Container facets;
		entity->filterChildren(facets,true,CC_TYPES::FACET);
		if (facets.empty())
			return;
		ccHObject* newGroup = new ccHObject(entity->getName() + selectionSuffix);

		for (size_t i = 0; i < facets.size(); ++i)
		{
			ccFacet* facet = static_cast<ccFacet*>(facets[i]);

			CCVector3 N = facet->getNormal();
			PointCoordinateType dip = 0;
			PointCoordinateType dipDir = 0;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

			double dDip = fabs(dip - dipFilter);
			double dDipDir = fabs(dipDir - dipDirFilter);

			bool visible = (	(	dDip	<= halfDipSpan		|| dDip		>= 360.0 - halfDipSpan)
							&&	(	dDipDir	<= halfDipDirSpan	|| dDipDir	>= 360.0 - halfDipDirSpan) );

			if (visible)
			{
				ccFacet* facetClone = facet->clone();
				if (facetClone)
					newGroup->addChild(facetClone);
			}
		}

		if (newGroup->getChildrenNumber() != 0)
		{
			m_app->addToDB(newGroup);
		}
		else
		{
			delete newGroup;
			newGroup = nullptr;
		}
	}
	//or a cloud?
	else if (entity->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
		if (!cloud->isVisibilityTableInstantiated())
			return;

		ccGenericPointCloud* subset = cloud->createNewCloudFromVisibilitySelection();
		if (subset)
		{
			if (subset->size() != 0)
			{
				subset->setName(cloud->getName() + selectionSuffix);
				m_app->addToDB(subset);
			}
			else
			{
				delete subset;
				subset = nullptr;
			}
		}
	}

	if (m_app)
		m_app->redrawAll();
}

void StereogramDialog::onDensityColorStepsChanged(int value)
{
	if (m_classifWidget && value >= ccColorScale::MIN_STEPS)
	{
		m_classifWidget->setDensityColorScaleSteps(static_cast<unsigned>(value));
		m_classifWidget->update();
	}
}

void StereogramDialog::onTicksFreqChanged(int value)
{
	if (m_classifWidget)
	{
		m_classifWidget->setTicksFreq(value);
		m_classifWidget->update();
	}
}

void StereogramDialog::onHSVColorsToggled(bool state)
{
	if (m_classifWidget)
	{
		m_classifWidget->showHSVRing(state);
		m_classifWidget->update();
	}
}

void StereogramDialog::colorScaleChanged(int)
{
	if (!m_classifWidget || !m_colorScaleSelector)
		return;

	ccColorScale::Shared colorScale = m_colorScaleSelector->getSelectedScale();
	unsigned steps = static_cast<unsigned>(colorScaleStepsSpinBox->value());

	m_classifWidget->setDensityColorScale(colorScale);
	m_classifWidget->setDensityColorScaleSteps(steps);
	m_classifWidget->update();
}

void StereogramDialog::spawnColorScaleEditor()
{
	if (!m_app || !m_app->getColorScalesManager())
		return;

	ccColorScale::Shared colorScale = (m_colorScaleSelector ? m_colorScaleSelector->getSelectedScale() : m_app->getColorScalesManager()->getDefaultScale(ccColorScalesManager::BGYR));
	ccColorScaleEditorDialog cseDlg(m_app->getColorScalesManager(), m_app, colorScale, m_app->getMainWindow());
	if (cseDlg.exec())
	{
		colorScale = cseDlg.getActiveScale();
		if (colorScale && m_colorScaleSelector)
		{
			m_colorScaleSelector->init(); //in fact it's a 're-init'
			m_colorScaleSelector->setSelectedScale(colorScale->getUuid());
		}

		//save current scale manager state to persistent settings
		m_app->getColorScalesManager()->toPersistentSettings();
	}
}