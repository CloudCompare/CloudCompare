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
#include <vector>

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

	blkSceneInfo& operator = (const blkSceneInfo& B) {
		strcpy(sceneID, B.sceneID);
		for ( size_t i = 0; i < 6; i++) {
			bound[i] = B.bound[i];
		}
		return *this;
	}
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
	blkDataInfo(blkDataType type = Blk_unset)
		: m_dataType(type)
	{}
	blkDataInfo(const blkDataInfo & info) {
		strcpy(sPath, info.sPath);
		strcpy(sName, info.sName);
		strcpy(sID, info.sID);
		nGroupID = info.nGroupID;
		m_dataType = info.dataType();
	}
	~blkDataInfo(){}

	virtual blkDataType dataType() const { return m_dataType; }
	virtual void fromString(std::string str) {}
	virtual std::string toString() const { std::string str; return str; }

	char sPath[_MAX_PATH];
	char sName[_MAX_FNAME];
	char sID[32];
	int nGroupID;
protected:
	blkDataType m_dataType;
};

class BLOCKDB_IO_LIB_API blkPtCldInfo : public blkDataInfo
{
public:
	blkPtCldInfo() 
		: blkDataInfo(Blk_PtCld)
	{}
	blkPtCldInfo(const blkPtCldInfo& info)
		: blkDataInfo(info) {
		level = info.level;
		scene_info = info.scene_info;
	}
	~blkPtCldInfo() {}

	BLOCK_PtCldLevel level;
	blkSceneInfo scene_info;
};

class BLOCKDB_IO_LIB_API blkCameraInfo : public blkDataInfo
{
public:
	blkCameraInfo()
		: blkDataInfo(Blk_Camera)
	{}
	blkCameraInfo(const blkCameraInfo& info)
		: blkDataInfo(info) {
		pixelSize = info.pixelSize; width = info.width;	height = info.height;
		f = info.f;	x0 = info.x0; y0 = info.y0;	R0 = info.R0;
		for (size_t i = 0; i < 8; i++)
			distortionPar[i] = info.distortionPar[i];
		for (size_t i = 0; i < 6; i++) 
			cameraBias[i] = info.cameraBias[i];
	}
	~blkCameraInfo() {}
		
	void fromString(std::string str) override {}
	std::string toString() const override { std::string str; return str; }

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
		: blkDataInfo(Blk_Image)
	{}
	blkImageInfo(const blkImageInfo & info)
		: blkDataInfo(info) {
		level = info.level;
		gpsLat = info.gpsLat; gpsLon = info.gpsLon; gpsHeight = info.gpsHeight;
		posXs = info.posXs, posYs = info.posYs, posZs = info.posZs;
		posPhi = info.posPhi, posOmega = info.posOmega, posKappa = info.posKappa;
		gps_time = info.gps_time;
		cameraName = info.cameraName;
		cameraID = info.cameraID;
		stripID = info.stripID;
		attrib = info.attrib;
		bFlag = info.bFlag;
		scene_info = info.scene_info;
	}
	~blkImageInfo() {}

	bool isValid();
	void setLevel(std::string _l);

	void fromString(std::string str) override {}
	std::string toString() const override { std::string str; return str; }

	BLOCK_ImgLevel level;
	double gpsLat, gpsLon, gpsHeight;
	double posXs, posYs, posZs, posPhi, posOmega, posKappa;
	double gps_time;
	std::string cameraName;
	int stripID, attrib, cameraID, bFlag;
	blkSceneInfo scene_info;
};

enum BLOCK_MiscAPP {
	MISCAPP_UNO,
	MISCAPP_CAM,
	MISCAPP_GCP,
	MISCAPP_DEM,
	MISCAPP_END,
};
static const char* g_strMiscAppName[] = { "uno", "CAM", "GCP", "DEM" };
class BLOCKDB_IO_LIB_API blkMiscsInfo : public blkDataInfo
{
public:
	blkMiscsInfo()
		: blkDataInfo(Blk_Miscs)
	{}
	blkMiscsInfo(const blkMiscsInfo& info)
		: blkDataInfo(info) {
		meta_app = info.meta_app;
		meta_key = info.meta_key;
		meta_value = info.meta_value;
	}
	~blkMiscsInfo() {}

	void fromString(std::string str) override {}
	std::string toString() const override { std::string str; return str; }

	BLOCK_MiscAPP meta_app;
	std::string meta_key;
	std::string meta_value;
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

	blkPtCldInfo** ptClds()  { return &m_ptClds; }
	blkPtCldInfo* ptClds() const { return m_ptClds; }
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

BLOCKDB_IO_LIB_API bool getImageGPSInfo(const char * path, double & Lat, double & Lon, double & Height);

BLOCKDB_IO_LIB_API bool readPosFile(const char * path, std::vector<blkImageInfo>& images_pos);

BLOCKDB_IO_LIB_API bool writePosFile(const char * path, const std::vector<blkImageInfo>& images_pos);

BLOCKDB_IO_LIB_API bool connectPgsDB(const char* connInfo);

BLKDB_NAMESPACE_END

#endif //BLOCK_DBASE_IO_HEADER
