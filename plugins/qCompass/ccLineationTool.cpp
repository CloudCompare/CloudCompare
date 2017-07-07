#include "ccLineationTool.h"
#include "ccCompass.h"

ccLineationTool::ccLineationTool()
	: ccTool()
{
}


ccLineationTool::~ccLineationTool()
{
}

//called when a point in a point cloud gets picked while this tool is active
void ccLineationTool::pointPicked(ccHObject* insertPoint, unsigned itemIdx, ccPointCloud* cloud, const CCVector3& P)
{
	if (!m_lineation)
	{
		//no active trace -> make a new one
		m_lineation = new ccLineation(cloud);
		m_lineation_id = m_lineation->getUniqueID();
		m_lineation->setDisplay(m_window);
		m_lineation->setVisible(true);
		m_lineation->setName("Lineation");
		m_lineation->prepareDisplayForRefresh_recursive();
		insertPoint->addChild(m_lineation);
		m_app->addToDB(m_lineation, false, false, false, false);
	}

	//add point
	int index = m_lineation->addPointIndex(itemIdx);

	//is this the end point?
	if (m_lineation->size() == 2)
	{
		//calculate trace orientation (trend/plunge)
		CCVector3f dir = m_lineation->getDirection(); dir.normalize();
		float trend, plunge;
		//special case: dir is vertical
		if (dir.z > 0.9999999) //vector = 0,0,1
		{
			trend = 0;
			if (dir.z < 0)
				plunge = 90;
			else
				plunge = -90;
		}
		else //normal cases...
		{
			CCVector3f hzComp = CCVector3f(dir.x, dir.y, 0); hzComp.normalize();

			//calculate plunge: plunge = angle between vector & vector projected onto horizontal (x,y) plane
			plunge = std::acos(dir.dot(hzComp)) * (180 / M_PI); //plunge measured from horizontal (in degrees)
			if (dir.z > 0) //lineations pointing towards the sky have negative plunges
				plunge *= -1;

			//calculate trend (N.B. I have very little idea how exactly this code work, it's kinda magic)
			//[c.f. http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors ]
			CCVector3f N(0, 1, 0); //north vector
			float dot = hzComp.dot(N);
			float det = CCVector3f(0, 0, 1).dot(hzComp.cross(N));
			trend = std::atan2(det, dot) * (180 / M_PI); //heading measured clockwise from north (in degrees)
			if (trend < 0)
				trend += 360;
		}

		//store trend and plunge info
		QVariantMap* map = new QVariantMap();
		CCVector3 s = *m_lineation->getPoint(0);
		CCVector3 e = *m_lineation->getPoint(1);
		map->insert("Sx", s.x); map->insert("Sy", s.y); map->insert("Sz", s.z);
		map->insert("Ex", e.x); map->insert("Ey", e.y); map->insert("Ez", e.z);
		map->insert("Trend", trend); map->insert("Plunge", plunge);
		m_lineation->setMetaData(*map, true);

		//rename lineation
		QString trendAndPlungeStr = QString("%1->%2").arg((int)plunge, 2, 10, QChar('0')).arg((int)trend, 3, 10, QChar('0'));
		m_lineation->setName(trendAndPlungeStr);
		m_lineation->showNameIn3D(ccCompass::drawName);

		//report orientation to console for convenience
		m_app->dispToConsole(QString("[ccCompass] Lineation = " + trendAndPlungeStr), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//start new one
		m_lineation = nullptr;
	}
}

//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
void ccLineationTool::accept()
{
	cancel(); //removes any incomplete lineations
}

//called when the "Escape" is pressed, or the "Cancel" button is clicked
void ccLineationTool::cancel()
{
	if (m_lineation)
	{
		if (m_app->dbRootObject()->find(m_lineation_id) && m_lineation->size() < 2) //if lineation is incomplete and it exists in the database still
		{
			m_app->removeFromDB(m_lineation); //remove incomplete lineation
		}
		m_lineation = nullptr;
	}
}

