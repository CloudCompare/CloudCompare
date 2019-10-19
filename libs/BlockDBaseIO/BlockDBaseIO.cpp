#include "BlockDBaseIO.h"

#include "LasPrjIO.h"
#include "libpq-fe.h"

#include <cfloat>

BLKDB_NAMESPACE_BEGIN

bool blkImageInfo::isValid()
{
	return _finite(posXs) && _finite(posYs) && _finite(posZs);
}

void blkImageInfo::setLevel(std::string _l)
{
	level = IMGLEVEL_UNO;
	for (size_t i = 0; i < IMGLEVEL_END; i++) {
		if (_l == g_strImageLevelName[i]) {
			level = BLOCK_ImgLevel(i);
			break;
		}
	}
}

BlockDBaseIO::BlockDBaseIO()
	: m_PcNum(0)
	, m_ImgNum(0)
	, m_camNum(0)
	, m_ptClds(nullptr)
	, m_images(nullptr)
	, m_cameras(nullptr)
{

}

BlockDBaseIO::~BlockDBaseIO()
{
	if (m_ptClds) {
		delete m_ptClds;
		m_ptClds = nullptr;
	}
	if (m_images) {
		delete m_images;
		m_images = nullptr;
	}
	if (m_cameras) {
		delete m_cameras;
		m_cameras = nullptr;
	}
}

int BlockDBaseIO::ptCldsNum() const { return m_PcNum; }
int & BlockDBaseIO::ptCldsNum() { return m_PcNum; }
int BlockDBaseIO::imagesNum() const { return m_ImgNum; }
int & BlockDBaseIO::imagesNum() { return m_ImgNum; }
int BlockDBaseIO::camerasNum() const { return m_camNum; }
int & BlockDBaseIO::camerasNum() { return m_camNum; }
blkProjHdr BlockDBaseIO::projHdr() const { return m_projHdr; }
blkProjHdr & BlockDBaseIO::projHdr() { return m_projHdr; }
// blkPtCldInfo * BlockDBaseIO::ptClds() {	return m_ptClds; }
blkImageInfo * BlockDBaseIO::images() { return m_images; }
blkCameraInfo * BlockDBaseIO::cameras() { return m_cameras; }

bool BlockDBaseIO::loadProject(const char * lpstrXmlPN)
{
	//! load from wgs

	//! parse



	return true;
}

bool BlockDBaseIO::saveProject(const char * lpstrXmlPN)
{
	return true;
}

bool getImageGPSInfo(const char* path, double& Lat, double& Lon, double& Height)
{
	LasPrjIO las_prj;
	return (las_prj.ReadImgGpsInfo(path, Lat, Lon, Height) == 1);
}

bool readPosFile(const char* path, std::vector<blkImageInfo>& images_pos)
{
	FILE *fpos;
	fpos = fopen(path, "r");
	if (!fpos) { return false; }

	int nImgNum(-1);
	char szBuf[1024];
	char char_temp[1024];
	//	CString str_temp;

	fgets(szBuf, 1024, fpos);
	sscanf(szBuf, "%d", &nImgNum);
	if (nImgNum < 0) {
		return false;
	}
	std::vector<blkImageInfo> data;
	for (size_t i = 0; i < nImgNum; i++)
	{
		blkImageInfo info;
		fgets(szBuf, 1024, fpos);
		sscanf(szBuf, "%s %lf %lf %lf %lf %lf %lf %lf",
			info.sName, &info.gps_time,
			&info.posXs, &info.posYs, &info.posZs,
			&info.posPhi, &info.posOmega, &info.posKappa);
		data.push_back(info);
	}
	images_pos = data;
	fclose(fpos);

	return true;
}

bool writePosFile(const char* path, const std::vector<blkImageInfo>& images_pos)
{
	FILE *fp;
	fp = fopen(path, "w");
	if (!fp)
	{
		return false;
	}
	fprintf(fp, "%d\n", images_pos.size());
	for (int i = 0; i < images_pos.size(); i++)
	{
		fprintf(fp, "%s %.10lf %.5lf %.5lf %.5lf %.10lf %.10lf %.10lf\n", images_pos[i].sName, images_pos[i].gps_time,
			images_pos[i].posXs, images_pos[i].posYs, images_pos[i].posZs, images_pos[i].posPhi,
			images_pos[i].posOmega, images_pos[i].posKappa);
	}

	fclose(fp);
	return true;
}

bool connectPgsDB(const char* connInfo)
{
	PGconn* Conn = PQconnectdb(connInfo);
	if (PQstatus(Conn) == CONNECTION_OK) {


	}
	return true;
}

BLKDB_NAMESPACE_END


