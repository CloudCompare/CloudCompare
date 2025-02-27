#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QVBoxLayout>

#include "ccCompassImport.h"

#include "ccMainAppInterface.h"

#include "ccLineation.h"
#include "ccPlane.h"
#include "ccPointCloud.h"

namespace {
	class ImportDialog : public QDialog
	{
	public:
		ImportDialog( const ccPointCloud *cloud, QWidget *parent )
			: QDialog( parent )
			, m_dipLabel( new QLabel )
			, m_dipDirLabel( new QLabel )
			, m_sizeLabel( new QLabel )
			, m_dipDirCombo( new QComboBox )
			, m_dipCombo( new QComboBox )
			, m_planeSize( nullptr )
		{
			QVBoxLayout* vbox = new QVBoxLayout;
			
			for (unsigned int i = 0; i < cloud->getNumberOfScalarFields(); ++i)
			{
				m_dipDirCombo->addItem( QString::fromStdString(cloud->getScalarFieldName(i)), i );
				m_dipCombo->addItem(QString::fromStdString(cloud->getScalarFieldName(i)), i );
			}
			
			m_planeSize = new QLineEdit( "2.0" );
			m_planeSize->setValidator( new QDoubleValidator(0.01, std::numeric_limits<double>::max(), 6) );
			
			QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
			QObject::connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
			QObject::connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
			
			vbox->addWidget(m_dipLabel);
			vbox->addWidget(m_dipCombo);
			vbox->addWidget(m_dipDirLabel);
			vbox->addWidget(m_dipDirCombo);
			vbox->addWidget(m_sizeLabel);
			vbox->addWidget(m_planeSize);
			vbox->addWidget(buttonBox);
			
			setLayout( vbox );
		}
		
		void setLabels( const QStringList &labels )
		{
			Q_ASSERT( labels.count() == 3 );
			
			m_dipLabel->setText( labels.at( 0 ) );
			m_dipDirLabel->setText( labels.at( 1 ) );
			m_sizeLabel->setText( labels.at( 2 ) );
		}
		
		int dipDirComboScalarFieldIndex()
		{
			return m_dipDirCombo->currentData().toInt();
		}
		
		int dipComboScalarFieldIndex()
		{
			return m_dipCombo->currentData().toInt();
		}
		
		double planeSize()
		{
			return m_planeSize->text().toDouble();
		}
		
	private:
		QLabel* m_dipLabel;
		QLabel* m_dipDirLabel;
		QLabel* m_sizeLabel;
		
		QComboBox* m_dipDirCombo;
		QComboBox* m_dipCombo;
		
		QLineEdit* m_planeSize;
	};
}

namespace ccCompassImport {

	//convert a point cloud containing field points (x,y,z) and dip+dip-direction scalar fields to planes for visualisation.	
	void importFoliations( ccMainAppInterface *app )
	{
		//get selected point cloud
		std::vector<ccHObject*> sel = app->getSelectedEntities();
		if (sel.empty())
		{
			app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	
		if (!sel[0]->isA(CC_TYPES::POINT_CLOUD))
		{
			app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	
		//get point cloud object
		ccPointCloud* cld = static_cast<ccPointCloud*>(sel[0]);
	
		ImportDialog foliationDialog( cld, app->getMainWindow() );
	
		foliationDialog.setLabels( {
									   QObject::tr( "Dip Field:", "ccCompassImport" ),
									   QObject::tr( "Dip-Direction Field:", "ccCompassImport" ),
									   QObject::tr( "Plane Size", "ccCompassImport" )
								   } );
		
		int result = foliationDialog.exec();
		
		if (result == QDialog::Rejected)
		{
			return; //bail!
		}
	
		//get values
		const int dipSF = foliationDialog.dipComboScalarFieldIndex();
		const int dipDirSF = foliationDialog.dipDirComboScalarFieldIndex();
		const double size = foliationDialog.planeSize();
		
		if (dipSF == dipDirSF)
		{
			app->dispToConsole("Error: Dip and Dip-Direction scalar fields must be different!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	
		//loop through points
		for (unsigned p = 0; p < cld->size(); p++)
		{
			ScalarType dip = cld->getScalarField(dipSF)->getValue(p);
			ScalarType dipdir = cld->getScalarField(dipDirSF)->getValue(p);
			CCVector3 Cd = *cld->getPoint(p);
	
			//build plane and get its orientation 
			ccPlane* plane = new ccPlane(QString("%1/%2").arg(static_cast<int>(dip), 2, 10, QChar('0')).arg(static_cast<int>(dipdir), 3, 10, QChar('0')));
			plane->showNameIn3D(true);
			cld->addChild(plane);
			app->addToDB(plane, false, true, false, false);
			CCVector3 N = plane->getNormal();
			CCVector3 C = plane->getCenter();
	
			//figure out transform (blatantly stolen from ccPlaneEditDlg::updatePlane())
			CCVector3 Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(dip, dipdir, true).toPC();
			ccGLMatrix trans;
			bool needToApplyTrans = false;
			bool needToApplyRot = false;
	
			needToApplyRot = (std::abs(N.dot(Nd) - CCCoreLib::PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon());
			needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);
	
			if (needToApplyTrans)
			{
				trans.setTranslation(-C);
				needToApplyTrans = true;
			}
			if (needToApplyRot)
			{
				ccGLMatrix rotation;
				//special case: plane parallel to XY
				if (std::abs(N.z) > CCCoreLib::PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
				{
					ccGLMatrix rotX;
					rotX.initFromParameters( CCCoreLib::DegreesToRadians( -dip ), CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
					
					ccGLMatrix rotZ;
					rotZ.initFromParameters( CCCoreLib::DegreesToRadians( dipdir ), CCVector3(0, 0, -1), CCVector3(0, 0, 0));
					rotation = rotZ * rotX;
				}
				else //general case
				{
					rotation = ccGLMatrix::FromToRotation(N, Nd);
				}
				trans = rotation * trans;
			}
			if (needToApplyTrans)
			{
				trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
			}
			if (needToApplyRot || needToApplyTrans)
			{
				plane->applyGLTransformation_recursive(&trans);
			}
			plane->setXWidth(size, false);
			plane->setYWidth(size, true);
		}
	}

	//convert a point cloud containing field points (x,y,z) and trend + plunge scalar fields to lineation vectors for visualisation.
	void importLineations( ccMainAppInterface *app )
	{
		//get selected point cloud
		std::vector<ccHObject*> sel = app->getSelectedEntities();
		if (sel.empty())
		{
			app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		if (!sel[0]->isA(CC_TYPES::POINT_CLOUD))
		{
			app->dispToConsole("Please select a point cloud containing your field data (this can be loaded from a text file)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		//get point cloud object
		ccPointCloud* cld = static_cast<ccPointCloud*>(sel[0]);

		
		ImportDialog	lineationDialog( cld, app->getMainWindow() );
	
		lineationDialog.setLabels( {
									   QObject::tr( "Trend Field:", "ccCompassImport" ),
									   QObject::tr( "Plunge Field:", "ccCompassImport" ),
									   QObject::tr( "Display Length", "ccCompassImport" )
								   } );

		int result = lineationDialog.exec();
		
		if (result == QDialog::Rejected)
		{
			return; //bail!
		}

		//get values
		const int dipSF = lineationDialog.dipComboScalarFieldIndex();
		const int dipDirSF = lineationDialog.dipDirComboScalarFieldIndex();
		const double size = lineationDialog.planeSize();

		if (dipSF == dipDirSF)
		{
			app->dispToConsole("Error: Trend and plunge scalar fields must be different!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		//loop through points
		for (unsigned p = 0; p < cld->size(); p++)
		{
			ScalarType trend = cld->getScalarField(dipSF)->getValue(p);
			ScalarType plunge = cld->getScalarField(dipDirSF)->getValue(p);
			CCVector3 Cd = *cld->getPoint(p);

			//build lineation vector
			CCVector3 l( sin( CCCoreLib::DegreesToRadians( trend ) ) * cos( CCCoreLib::DegreesToRadians( plunge ) ),
						 cos( CCCoreLib::DegreesToRadians( trend ) ) * cos( CCCoreLib::DegreesToRadians( plunge ) ),
						 -sin( CCCoreLib::DegreesToRadians( plunge ) ) );
			
			//create new point cloud to associate with lineation graphic
			ccPointCloud* points = new ccPointCloud();
			points->copyGlobalShiftAndScale(*cld); //copy global shift & scale onto new point cloud
			points->reserve(2);
			points->addPoint(Cd);
			points->addPoint(Cd + l*size);
			points->setName("verts");

			//create lineation graphic
			ccLineation* lineation = new ccLineation(points);
			lineation->addChild(points);
			lineation->addPointIndex(0);
			lineation->addPointIndex(1);
			lineation->updateMetadata();
			lineation->setName(QStringLiteral("%1->%2").arg( qRound( plunge ) ).arg( qRound( trend ) ));
			cld->addChild(lineation);
			app->addToDB(lineation, false, true, false, false);
		}
	}

}
