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
	//try retrieve active lineation (will fail if there isn't one)
	ccLineation* l = dynamic_cast<ccLineation*>(m_app->dbRootObject()->find(m_lineation_id));
	if (!l) //make a new one
	{
		//no active trace -> make a new one
		l = new ccLineation(cloud);
		m_lineation_id = l->getUniqueID();

		//set drawing properties
		l->setDisplay(m_window);
		l->setVisible(true);
		l->setName("Lineation");
		l->prepareDisplayForRefresh_recursive();

		//add to DB Tree
		insertPoint->addChild(l);
		m_app->addToDB(l, false, false, false, false);
	} 

	//add point
	int index = l->addPointIndex(itemIdx);

	//is this the end point?
	if (l->size() == 2)
	{
		//calculate trace orientation (trend/plunge)
		CCVector3f dir = l->getDirection(); dir.normalize();
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
		CCVector3 s = *l->getPoint(0);
		CCVector3 e = *l->getPoint(1);
		map->insert("Sx", s.x); map->insert("Sy", s.y); map->insert("Sz", s.z);
		map->insert("Ex", e.x); map->insert("Ey", e.y); map->insert("Ez", e.z);
		map->insert("Trend", trend); map->insert("Plunge", plunge);
		l->setMetaData(*map, true);

		//rename lineation
		float length = sqrt((s.x - e.x)*(s.x - e.x) + (s.y - e.y)*(s.y - e.y) + (s.z - e.z)*(s.z - e.z));
		QString lengthstr = QString("").asprintf("%.1f on ", length);
		QString trendAndPlungeStr = QString("%2->%3").arg((int)plunge, 2, 10, QChar('0')).arg((int)trend, 3, 10, QChar('0'));
		QString namestr = lengthstr + trendAndPlungeStr;

		l->setName(namestr);
		l->showNameIn3D(ccCompass::drawName);

		//report orientation to console for convenience
		m_app->dispToConsole(QString("[ccCompass] Lineation = " + trendAndPlungeStr), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//start new lineation
		m_lineation_id = -1;
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
	if (m_lineation_id != -1) //there is an active lineation
	{
		ccLineation* l = dynamic_cast<ccLineation*>(m_app->dbRootObject()->find(m_lineation_id));
		if (l && l->size() < 2)
		{
			m_app->removeFromDB(l); //remove incomplete lineation
			m_lineation_id = -1;
		}
	}
}

