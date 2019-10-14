#pragma once
#include"PrjInfo.h"
//#include <afx.h>
#include<stdio.h>
#include<string>
#include<sstream>

#ifndef USE_LASPRJIO_AS_LIB
#ifdef _LasPrjIO_LIB_export
#define _LasPrjIO_LIB	 __declspec(dllexport)
#else
#define _LasPrjIO_LIB  __declspec(dllimport)

#ifdef _DEBUG
#ifdef _WIN64
#pragma comment(lib,"LasPrjIO_D64.lib")
#pragma message("Automatically linking with LasPrjIO_D64.lib")
#else
#pragma comment(lib,"LasPrjIO_D.lib")
#pragma message("Automatically linking with LasPrjIO_D.lib")
#endif
#else

#ifdef _WIN64
#pragma comment(lib,"LasPrjIO64.lib")
#pragma message("Automatically linking with LasPrjIO64.lib")
#else
#pragma comment(lib,"LasPrjIO.lib")
#pragma message("Automatically linking with LasPrjIO.lib")
#endif

#endif
#endif
#else
#define _LasPrjIO_LIB
#endif



class _LasPrjIO_LIB LasPrjIO
{
public:
	LasPrjIO();
	~LasPrjIO();

	bool GeneratePrjFiles();  //生成工程文件
	int LoadPrjFiles(char* lpXMLPath);  
	int LoadPrjFiles2(char* lpXMLPath); //读取工程内所有las和img
	int ReadCamFile(char* lpstrPath, CameraInfo& sCamInfo);
	int ReadPosFile(char* lpstrPath, ImgInfo& sImgInfo);	
	int WriteCamFile(char* lpstrPath, CameraInfo sCamInfo);
	bool WritePrjFile(PrjHdr sPrjHdr);

	int ReadDPrjInfo(char* lpDppFPath);
	int ReadDppFile(char* lpDppFPath,ImgInfo& vImg);
	int ReadDPCamFile(char* lpDPCamFPath,CameraInfo& vCam);
	int ReadDPImgLocFile(char* lpDPImgLocFPath,ImgInfo& vImg);
	int ReadDPVpsFile(char* lpDPVpsFPath,ImgInfo& vImg);
	int ReadDPDpiFile(char* lpDPDpiFPath,ImgData& Img,CameraInfo& vCam);

	int ReadImgGpsInfo();//读取影像GPS信息 批量
	int ReadImgGpsInfo(const char* szImgFPath, double& Lat, double& Lon, double& Height);//读取影像GPS信息

	/*投影坐标均以横轴为x轴 纵轴为y轴,坐标中未加入带号；
	纵轴已向西平移500km，即x坐标上加上500km；
	投影计算中，高斯克吕格投影有3度和6度两种分带方式，UTM投影只有6度分带；
	东经为正，西经为负，北纬为正，南纬为负，单位为度
	部分函数参数：
	nType:  2:WGS84 3: CGCS2000 
	ParaDegree: 3:三度分带 6:六度分带（UTM投影只有6度分带）
	nProjType:投影类型 1：高斯克吕格投影 2：UTM投影
	nProjNo:投影带号
	dcentralM: 投影带中央经度	*/
	
	int CalCentralLon(int nProjNo, int ParaDegree,int nProjType, double& dCentralLon);//根据投影带号计算中央经线	
	int CalCentralLon(double Lon, int ParaDegree, int nProjType, double& dCentralLon);//根据经度计算所在投影带的中央经线

	//根据点经度计算投影坐标
	void LatLonToUTMXY(double lat, double lon, int nType, int ParaDegree, double *x, double *y); //根据经纬度计算UTM投影坐标，UTM投影只有6度分带，故ParaDegree应为6
	void LatLonToGauXY(double lat, double lon, int nType, int ParaDegree, double *x, double *y);//根据经纬度计算高斯投影坐标，UTM投影只有6度分带，故ParaDegree应为6
	void LatLonToUTMXY(int nType, int ParaDegree); //根据经纬度计算UTM投影坐标 批量
	void LatLonToGauXY(int nType, int ParaDegree);//根据经纬度计算高斯投影坐标 批量

	//根据中央经线计算投影坐标
	void LatLonToUTMXY(double lat, double lon, int nType, double dcentralM, double *x, double *y); //根据中央经线计算UTM投影坐标
	void LatLonToGauXY(double lat, double lon, int nType, double dcentralM, double *x, double *y);//根据中央经线计算高斯投影坐标
	void LatLonToUTMXY(int nType, double dcentralM); //根据投影中央经线计算UTM投影坐标 批量
	void LatLonToGauXY(int nType, double dcentralM);//根据投影中央经线计算高斯投影坐标 批量

	//根据投影坐标反算大地坐标
	void UTMXYToLatLon(double x, double y, int nType, double dcentralM/*int zone*/, double& lat, double& lon);//UTM投影坐标转大地坐标
	void GauXYToLatLon(double x, double y, int nType, double dcentralM/*int zone*/, double& lat, double& lon);//高斯克吕格投影坐标转大地坐标
	void UTMXYToLatLon(int nType, double dcentralM);
	void GauXYToLatLon(int nType, double dcentralM);

	std::string getErrorMsg() { return m_error_msg; }
protected:
	
	int WriteLasList(char* lpstrPath, LasInfo sLasInfo);
	int WriteImgList(char* lpstrPath, ImgInfo ImgInfo);
	int WritePosFile(char* lpstrPath, ImgInfo sImgInfo);

	int ReadPrjFile(char* lpstrPath, PrjHdr& sPrjHdr);
	int ReadLasList(char* lpstrPath, LasInfo& sLasInfo,PrjHdr& sPrjHdr);	
	int ReadLasList(char* sLasLstPath, LasInfo& sLasInfo);
	int ReadImgList(char* lpstrPath, ImgInfo& sImgInfo,PrjHdr& sPrjHdr);
	int ReadImgList(char* sImgLstPath, ImgInfo& sImgInfo);
	void trim(char *strIn, char *strOut);

	static double RadToDeg(double rad);	
	static double FootpointLatitude(double sm_a,double sm_b,double y);
	static void MapXYToLatLon(double sm_a,double sm_b,double x, double y, double lambda0, GeoCorr &philambda);

	static double DegToRad(double deg);
	static double ArcLengthOfMeridian(double sm_a,double sm_b,double phi);
	static void MapLatLonToXY(double sm_a,double sm_b,double phi, double lambda, double lambda0, ProjCoor &xy);
	static double UTMCentralMeridian(int zone);
	
public:
	PrjHdr m_PrjHdr;
	CameraInfo m_CameraInfo;
	LasInfo m_LasInfo;
	ImgInfo m_ImgInfo;

private:
	static const int m_MaxCamCount = 10; //最大相机数
	static const int m_MaxImgCount = 10000;  //最大影像数
	static const int m_MaxLasCount = 1000; //最大点云条带数
	std::string m_error_msg;
};

