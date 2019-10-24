/*
File	:		BlockDBaseIO.h
Brief	:		The header file of BlockDBase

Author	:		Xinyi Liu
Date	:		2019/10/20
E-mail	:		liuxy0319@outlook.com
*/

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
#include <map>

BLKDB_NAMESPACE_BEGIN

#ifndef _MAX_PATH
#define _MAX_PATH 260
#endif

#ifndef _MAX_FNAME
#define _MAX_FNAME 256
#endif

#define PRODUCT_DIR "Product"
#define IF_PROCESS_DIR "IF_PROCESS"
#define REGISPRJ_PATH_KEY "RegisPrjPath"

typedef std::map<std::string, std::string> strMetaMap;

enum BLOCK_TASK_TYPE
{
	TASK_TILE,
	TASK_FILTER,
	TASK_REGIS,
	TASK_CLASS,
	TASK_BDSEG,
	TASK_RECON,
	TASK_END,
};
enum BLOCK_TASK_ID
{
	TASK_ID_TILE,
	TASK_ID_FILTER,
	TASK_ID_REGIS,
	TASK_ID_CLASS,
	TASK_ID_BDSEG,
	TASK_ID_RECON,
	TASK_ID_END,
};
static const char* g_strTaskTagName[] = { "TILE", "FILTER", "REGIST", "CLASSIFY", "BDSEG", "BDRECON" };
static const char* g_strTaskDirName[] = { "Tiles", "Filtering", "Registration", "Classification", "Segmentation", "Reconstruction" };

struct blkProjHdr
{
	blkProjHdr()
		: projectID(0)
		, groupID(0)
	{
		memset(sDirPath, 0, _MAX_PATH);
		memset(sName, 0, _MAX_FNAME);
		memset(camParPath, 0, _MAX_PATH);
		memset(imgListPath, 0, _MAX_PATH);
		memset(lasListPath, 0, _MAX_PATH);
		memset(m_strProdGCDPN, 0, _MAX_PATH);
		memset(m_strGcpGCDPN, 0, _MAX_PATH);
		memset(m_str7parPN, 0, _MAX_PATH);
	}
	char sDirPath[_MAX_PATH];
	char sName[_MAX_FNAME];

	int projectID;
	int groupID;

	char lasListPath[_MAX_PATH];
	char camParPath[_MAX_PATH];
	char imgListPath[_MAX_PATH];

	char m_strProdGCDPN[_MAX_PATH];
	char m_strGcpGCDPN[_MAX_PATH];
	char m_str7parPN[_MAX_PATH];
};

struct blkSceneInfo
{
	blkSceneInfo() {
		memset(sceneID, 0, 512);
		memset(bound, 0, sizeof(double) * 6);
	}

	bool setStrSceneInfo(std::string str);
	std::string getStrSceneInfo(std::string* key = nullptr) const;

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
		, nGroupID(0)
	{
		memset(sPath, 0, _MAX_PATH);
		memset(sName, 0, _MAX_FNAME);
		memset(sID, 0, 32);
	}
	blkDataInfo(const blkDataInfo & info) {
		strcpy(sPath, info.sPath);
		strcpy(sName, info.sName);
		strcpy(sID, info.sID);
		nGroupID = info.nGroupID;
		m_dataType = info.dataType();
	}
	~blkDataInfo(){}

	virtual blkDataType dataType() const { return m_dataType; }

	virtual bool isValid() { return strlen(sName) > 0; }

	virtual void pushToMeta(strMetaMap& meta_info) const;
	virtual bool pullFromMeta(const strMetaMap& meta_info);

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
		, level(PCLEVEL_TILE)
	{}
	blkPtCldInfo(const blkPtCldInfo& info)
		: blkDataInfo(info) {
		level = info.level;
		scene_info = info.scene_info;
	}
	~blkPtCldInfo() {}

	virtual bool isValid() override;

	virtual void pushToMeta(strMetaMap& meta_info) const override;
	virtual bool pullFromMeta(const strMetaMap& meta_info) override;

	bool setStrLevel(std::string str);
	std::string getStrLevel(std::string* key = nullptr) const;
	bool setStrSceneInfo(std::string str);
	std::string getStrSceneInfo(std::string* key = nullptr) const;

	BLOCK_PtCldLevel level;
	blkSceneInfo scene_info;
};

class BLOCKDB_IO_LIB_API blkCameraInfo : public blkDataInfo
{
public:
	blkCameraInfo()
		: blkDataInfo(Blk_Camera)
		, pixelSize(0)
		, width(0)
		, height(0)
		, f(0), x0(0), y0(0)
	{
		memset(distortionPar, 0, sizeof(double) * 8);
		memset(cameraBias, 0, sizeof(double) * 6);
	}
	blkCameraInfo(const blkCameraInfo& info)
		: blkDataInfo(info) {
		pixelSize = info.pixelSize; width = info.width;	height = info.height;
		f = info.f;	x0 = info.x0; y0 = info.y0;
		for (size_t i = 0; i < 8; i++)
			distortionPar[i] = info.distortionPar[i];
		for (size_t i = 0; i < 6; i++) 
			cameraBias[i] = info.cameraBias[i];
	}
	~blkCameraInfo() {}

	virtual bool isValid() override;

	virtual void pushToMeta(strMetaMap& meta_info) const override;
	virtual bool pullFromMeta(const strMetaMap& meta_info) override;

	bool setStrPixSize(std::string str);
	std::string getStrPixSize(std::string* key = nullptr) const;
	bool setStrFrameSize(std::string str);
	std::string getStrFrameSize(std::string* key = nullptr) const;
	bool setStrFxy(std::string str);
	std::string getStrFxy(std::string* key = nullptr) const;
	bool setStrDistPar(std::string str);
	std::string getStrDistPar(std::string* key = nullptr) const;

	double pixelSize;
	int width, height;
	double f, x0, y0;
	double distortionPar[8];
	double cameraBias[6];//disabled, not used
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
		, level(IMGLEVEL_STRIP)
		, gpsLat(0), gpsLon(0), gpsHeight(0)
		, posXs(0), posYs(0), posZs(0)
		, posPhi(0), posOmega(0), posKappa(0)
		, gps_time(0)
		, stripID(0)
		, attrib(0)
		, bFlag(0)
	{}
	blkImageInfo(const blkImageInfo & info)
		: blkDataInfo(info) {
		level = info.level;
		gpsLat = info.gpsLat; gpsLon = info.gpsLon; gpsHeight = info.gpsHeight;
		posXs = info.posXs, posYs = info.posYs, posZs = info.posZs;
		posPhi = info.posPhi, posOmega = info.posOmega, posKappa = info.posKappa;
		gps_time = info.gps_time;
		cameraName = info.cameraName;
		stripID = info.stripID;
		attrib = info.attrib;
		bFlag = info.bFlag;
	}
	~blkImageInfo() {}

	virtual bool isValid() override;

	virtual void pushToMeta(strMetaMap& meta_info) const override;
	virtual bool pullFromMeta(const strMetaMap& meta_info) override;

	bool setStrLevel(std::string str);
	std::string getStrLevel(std::string* key = nullptr) const;
	bool setStrGPS(std::string str);
	std::string getStrGPS(std::string* key = nullptr) const;
	bool setStrPOS(std::string str);
	std::string getStrPOS(std::string* key = nullptr) const;
	bool setStrCamera(std::string str);
	std::string getStrCamera(std::string* key = nullptr) const;
	bool setStrExtraTag(std::string str);
	std::string getStrExtraTag(std::string* key = nullptr) const;

	BLOCK_ImgLevel level;	//level
	double gpsLat, gpsLon, gpsHeight;	//gps
	double gps_time, posXs, posYs, posZs, posPhi, posOmega, posKappa;	//pos
	std::string cameraName;		//camera
	int stripID, attrib, bFlag;	//extra int tag
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

	BLOCK_MiscAPP meta_app;
	std::string meta_key;
	std::string meta_value;
};

class BLOCKDB_IO_LIB_API BlockDBaseIO
{
public:
	BlockDBaseIO() {}
	BlockDBaseIO(const char* path);
	~BlockDBaseIO();

	bool loadProject();
	bool saveProject(bool save_regisprj = false);
	bool saveRegistrationProject(const char* path);

	void clear();

	void setPath(const char * path);

	blkProjHdr projHdr() const { return m_projHdr; }
	blkProjHdr& projHdr() { return m_projHdr; }

	std::vector<blkPtCldInfo>& getPtClds() { return m_ptClds; }
	void setPtClds(const std::vector<blkPtCldInfo> & data) { m_ptClds = data; }
	std::vector<blkImageInfo>& getImages() { return m_images; }
	void setImages(const std::vector<blkImageInfo> & data) { m_images = data; }
	std::vector<blkCameraInfo>& getCameras() { return m_cameras; }
	void setCameras(const std::vector<blkCameraInfo> & data) { m_cameras = data; }

	std::string getErrorInfo() { return m_error_info; }

	bool getMetaValue(std::string key, std::string& value);
	bool addMetaValue(std::string key, std::string value);

	blkDataInfo* addData(blkDataInfo* info);
private:

	std::vector<blkPtCldInfo>			m_ptClds;
	std::vector<blkImageInfo>			m_images;
	std::vector<blkCameraInfo>			m_cameras;
	blkProjHdr							m_projHdr;
	std::map<std::string, std::string>	m_meta_info;

	std::string m_error_info;
	char m_project_path[_MAX_PATH];
};

template<typename T1 = std::string, typename T2 = std::string>
BLOCKDB_IO_LIB_API inline bool _getMetaValue(std::map<T1,T2> meta, const T1& key, T2 & value)
{
	auto it = meta.find(key);
	if (it == meta.end()) {
		return false;
	}
	else {
		value = it->second;
		return key == it->first;
	}
}

BLOCKDB_IO_LIB_API bool loadMetaListFile(const char * path, char * prefix, std::vector<strMetaMap> & meta_info);

BLOCKDB_IO_LIB_API bool saveMetaListFile(const char * path, const char * prefix, const std::vector<strMetaMap>& meta_info, bool overwrite);

BLOCKDB_IO_LIB_API bool loadImages(std::vector<blkImageInfo>& images, std::vector<blkCameraInfo>& cameras, const char * img_list, const char * cam_list);
BLOCKDB_IO_LIB_API bool saveImages(const std::vector<blkImageInfo>& images, const std::vector<blkCameraInfo>& cameras, const char * img_list, const char * cam_list);
BLOCKDB_IO_LIB_API bool loadPoints(std::vector<blkPtCldInfo>& ptClds, const char* path);
BLOCKDB_IO_LIB_API bool savePoints(const std::vector<blkPtCldInfo>& ptClds, const char* path);
BLOCKDB_IO_LIB_API bool getImageGPSInfo(const char * path, double & Lat, double & Lon, double & Height);

BLOCKDB_IO_LIB_API bool loadPosFile(const char * path, std::vector<blkImageInfo>& images_pos);

BLOCKDB_IO_LIB_API bool savePosFile(const char * path, const std::vector<blkImageInfo>& images_pos);

BLOCKDB_IO_LIB_API bool connectPgsDB(const char* connInfo);

BLKDB_NAMESPACE_END

#endif //BLOCK_DBASE_IO_HEADER

