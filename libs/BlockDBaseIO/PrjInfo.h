#ifndef _PRJINFO_H
#define _PRJINFO_H
#include <vector>

#ifndef _PRJHEADER
#define _PRJHEADER
typedef struct tagPrjHdr{
	char PrjPath[_MAX_PATH];	//�����ļ�·��
	char RegisPrjPath[_MAX_PATH]; //��׼����·��
	int ProjectID;
	int BlockID;
	char CamParFilePath[_MAX_PATH];  //��������ļ�·��
	char ImgListPath[_MAX_PATH]; //Ӱ���б��ļ�·��
	char PosFilePath[_MAX_PATH];  //Pos�����ļ�·��
	char LasListPath[_MAX_PATH];  //�����б��ļ�·��

	//char ImgDir[_MAX_DIR];  //Ӱ��洢Ŀ¼
	//char LasDir[_MAX_DIR];   //���ƴ洢Ŀ¼

	double m_fDemGSD;  //DEM����
	tagPrjHdr()
	{
		strcpy(PrjPath, "NULL");
		strcpy(RegisPrjPath, "NULL");
		strcpy(CamParFilePath, "NULL");
		strcpy(ImgListPath, "NULL");
		strcpy(PosFilePath, "NULL");
		strcpy(LasListPath, "NULL");
		//strcpy(ImgDir, "NULL");
		//strcpy(LasDir, "NULL");

		ProjectID = 0;
		BlockID = 0;
		m_fDemGSD = 0;
	}	
}PrjHdr;
#endif

#ifndef CameraInfo_WGS
#define CameraInfo_WGS
typedef struct tagCameraData
{
	int CameraID;
	char CameraName[_MAX_PATH];
	double pixelSize;  //mm
	int width; //pixel
	int height;  //pixel
	double  f;   //mm
	double x0;  //mm
	double y0;  //mm
	double distortionPar[8];  //k0 k1 k2 k3 p1 p2 b1 b2  //mm
	double CameraBias[6]; //�����pos�µ�ƫ�ķ�����ƫ�ýǷ���
	double R0; //update
	tagCameraData()
	{
		strcpy(CameraName, "");
		f = 0;
		x0 = 0;
		y0 = 0;
		CameraID = 0;
		pixelSize = 0;
		R0 = 0;
		width = 0; height = 0;
		for (int i = 0; i < 8; i++)
			distortionPar[i] = 0;
		for (int i = 0; i < 6; i++)
			CameraBias[i] = 0;
	}
}CameraData;
typedef std::vector<CameraData> CameraInfo;

#endif // !CameraInfo_WGS
//typedef struct tagCameraInfo
//{
//	char szParPath[256]; //��������ļ�·��
//	CameraList CameraList;
//
//	tagCameraInfo()
//	{
//		strcpy(szParPath, "NULL");
//	}
//}CameraInfo;

#ifndef ImgInfo_WGS
#define ImgInfo_WGS
typedef struct tagImgData
{
	char szImgName[_MAX_PATH]; //
	char szImgPath[_MAX_PATH]; //Ӱ������·��
	int ImageID,BlockID;
	double Lat, Lon, Height;
	double Xs, Ys, Zs;
	double Phi, Omega, Kappa;
	int StripID, Attrib, CameraID, bFlag;
	double gps_time;
	tagImgData()
	{
		strcpy(szImgName, "NULL");
		strcpy(szImgPath, "NULL");
		ImageID = -1;
		BlockID = 0;
		Lat = Lon = Height = 0;
		gps_time = Xs = Ys = Zs = Phi = Omega = Kappa = 0;
		StripID = Attrib = CameraID = bFlag  = 0;
	}
}ImgData;
typedef std::vector<ImgData> ImgInfo;

#endif //!ImgInfo_WGS

//#ifndef ImgInfo_WGS
//#define ImgInfo_WGS
//typedef struct tagImgInfo
//{
//	char szImgFilePath[_MAX_PATH];   //Ӱ���б��ļ�·��
//	char szPosFilePath[_MAX_PATH];
//	char szImgDir[_MAX_PATH];  //Ӱ��洢Ŀ¼
//	VtImgList vImgList;
//
//	tagImgInfo()
//	{
//		strcpy(szImgFilePath, "NULL");
//		strcpy(szPosFilePath, "NULL");
//		strcpy(szImgDir, "NULL");
//	}
//}ImgInfo;
//
//#endif


#ifndef LasInfo_WGS
#define LasInfo_WGS
//���ƺ����ṹ��
typedef struct tagLasStrip
{
	char szPath[256]; //����·��
	char szName[128];  //��������
	int  nIdx;        //����������
	int  BlockID;
	float dzoffset; //�߳�ƫ����

	tagLasStrip()
	{
		strcpy(szPath, "NULL");
		strcpy(szName, "NULL");
		nIdx = -1;
		BlockID = 0;
		dzoffset = 0.0;
	}
}LasStrip;
typedef std::vector<LasStrip> LasInfo;

#endif //!LasInfo_WGS

//#ifndef LasInfo_WGS
//#define LasInfo_WGS
//typedef struct tagLasInfo
//{
//	char szLasFilePath[_MAX_PATH]; //�����б��ļ�
//	char szDir[_MAX_PATH];
//
//	VtLasList vStrips;
//
//	tagLasInfo()
//	{
//		strcpy(szLasFilePath, "NULL");
//		strcpy(szDir, "NULL");
//	}
//}LasInfo;
//
//#endif //!LasInfo_WGS

#ifndef GeoCorr_WGS
#define GeoCorr_WGS
typedef struct tagGeoCorr
{
	double lat; //
	double log;
	double height;
}GeoCorr;
#endif //!GeoCorr_WGS

#ifndef ProjCorr_WGS
#define ProjCorr_WGS
typedef struct tagProjCorr
{
	double x;
	double y;
}ProjCoor;
#endif //!ProjCorr_WGS


#endif // !_PRJINFO_H
