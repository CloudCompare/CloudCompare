#include "BlockDBaseIO.h"

#include "libpq-fe.h"

BLKDB_NAMESPACE_BEGIN

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
blkPtCldInfo * BlockDBaseIO::ptClds() {	return m_ptClds; }
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

bool connectPgsDB(const char* connInfo)
{
	PGconn* Conn = PQconnectdb(connInfo);
	if (PQstatus(Conn) == CONNECTION_OK) {


	}
	return true;
}

BLKDB_NAMESPACE_END