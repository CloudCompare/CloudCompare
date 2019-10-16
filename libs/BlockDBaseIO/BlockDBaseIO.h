#ifndef BLOCK_DBASE_IO_HEADER
#define BLOCK_DBASE_IO_HEADER

#if defined( libBlockDBaseIO_EXPORTS )
#  define BLOCKDB_IO_LIB_API __declspec(dllexport)
#else
#  define BLOCKDB_IO_LIB_API __declspec(dllimport)
#endif

#ifndef BLKDB_NAMESPACE_BEGIN
#define BLKDB_NAMESPACE_BEGIN namespace BlockDB{
#endif
#ifndef BLKDB_NAMESPACE_END
#define BLKDB_NAMESPACE_END }
#endif
//system
#include <string>

BLKDB_NAMESPACE_BEGIN

#ifndef _MAX_PATH
#define _MAX_PATH 260
#endif

#ifndef _MAX_FNAME
#define _MAX_FNAME 256
#endif

enum BLOCK_TASK_TYPE
{
	TASK_TILE,
	TASK_FILTER,
	TASK_REGIS,
	TASK_CLASS,
	TASK_BDSEG,
	TASK_RECON,
};

struct blkProjHdr
{
	char sPath[_MAX_PATH];
	char sName[_MAX_FNAME];

	int projectID;
	int GroupID;

	char camParPath[_MAX_PATH];
	char imgListPath[_MAX_PATH];
	char lasListPath[_MAX_PATH];

	char m_strProdGCDPN[_MAX_PATH];
	char m_strGcpGCDPN[_MAX_PATH];
	char m_str7parPN[_MAX_PATH];

	char m_strTaskIni[_MAX_PATH];
};

struct blkSceneInfo
{
	double getMinX() { return bound[0]; }
	double getMinY() { return bound[1]; }
	double getMinZ() { return bound[2]; }
	double getMaxX() { return bound[3]; }
	double getMaxY() { return bound[4]; }
	double getMaxZ() { return bound[5]; }
	void setMinMax(double minx, double miny, double minz, double maxx, double maxy, double maxz) {
		bound[0] = minx;
		bound[1] = miny;
		bound[2] = minz;
		bound[3] = maxx;
		bound[4] = maxy;
		bound[5] = maxz;
	}
	char sceneID[512];
	double bound[6];//minX, minY, minZ, maxX, maxY, maxZ
};

enum blkDataType {
	Blk_unset,
	Blk_PtCld,
	Blk_Image,
	Blk_Camera,
	Blk_Miscs,
};

enum BLOCK_PtCldLevel
{
	PCLEVEL_UNO,
	// ori
	PCLEVEL_STRIP,
	PCLEVEL_TILE,
	// product
	PCLEVEL_FILTER,
	PCLEVEL_CLASS,
	PCLEVEL_BUILD,
	PCLEVEL_END,
};
static const char* g_strPtCldLevelName[] = { "uno","strip","tile","filter","class","build" };

class BLOCKDB_IO_LIB_API blkDataInfo
{
public:
	blkDataInfo(){}
	~blkDataInfo(){}

	virtual blkDataType dataType() { return Blk_unset; }

	char sPath[_MAX_PATH];
	char sName[_MAX_FNAME];
	char sID[32];
	int nGroupID;
};

class BLOCKDB_IO_LIB_API blkPtCldInfo : public blkDataInfo
{
public:
	blkPtCldInfo() 
		: blkDataInfo() 
	{}
	~blkPtCldInfo() {}

	virtual blkDataType dataType() override { return Blk_PtCld; }

	BLOCK_PtCldLevel level;
	blkSceneInfo scene_info;
};

class BLOCKDB_IO_LIB_API blkCameraInfo : public blkDataInfo
{
public:
	blkCameraInfo()
		: blkDataInfo()
	{}
	~blkCameraInfo() {}

	virtual blkDataType dataType() override { return Blk_Camera; }
	
	double pixelSize;
	int width, height;
	double f, x0, y0;
	double distortionPar[8];
	double cameraBias[6];
	double R0;
};

enum BLOCK_ImgLevel {
	IMGLEVEL_UNO,
	IMGLEVEL_STRIP,
	IMGLEVEL_DOM,
	IMGLEVEL_END,
};
static const char* g_strImageLevelName[] = { "uno","strip","dom" };

class BLOCKDB_IO_LIB_API blkImageInfo : public blkDataInfo
{
public:
	blkImageInfo()
		: blkDataInfo()
	{}
	~blkImageInfo() {}

	virtual blkDataType dataType() override { return Blk_Image; }

	BLOCK_ImgLevel level;
	double gpsLat, gpsLon, gpsHeight;
	double posXs, posYs, posZs, posPhi, posOmega, posKappa;
	int stripID, attrib, cameraID, bFlag;
	blkSceneInfo scene_info;
};

class BLOCKDB_IO_LIB_API BlockDBaseIO
{
public:
	BlockDBaseIO();
	~BlockDBaseIO();

	bool loadProject(const char* lpstrXmlPN);
	bool saveProject(const char* lpstrXmlPN);

	int ptCldsNum() const;
	int& ptCldsNum();
	int imagesNum() const;
	int& imagesNum();
	int camerasNum() const;
	int& camerasNum();

	blkProjHdr projHdr() const;
	blkProjHdr& projHdr();

	blkPtCldInfo* ptClds();
	blkImageInfo* images();
	blkCameraInfo* cameras();

private:
	int m_PcNum;
	int m_ImgNum;
	int m_camNum;

	blkPtCldInfo*	m_ptClds;
	blkImageInfo*	m_images;
	blkCameraInfo*	m_cameras;
	blkProjHdr		m_projHdr;
};

BLOCKDB_IO_LIB_API bool connectPgsDB(const char* connInfo);

BLKDB_NAMESPACE_END

#endif //BLOCK_DBASE_IO_HEADER
