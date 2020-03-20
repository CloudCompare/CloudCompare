/*
 * ccPApi.h
 *
 *  Created on: 14 mars 2020
 *      Author: paul
 */

#ifndef CCPAPI_CCPAPI_H_
#define CCPAPI_CCPAPI_H_

#include <ccCommandLineInterface.h>
//#include <vector>
//#include <ccHObject.h>
//#include <FileIOFilter.h>

class ccPApi
{
public:

    enum CC_SHIFT_MODE
    {
        AUTO = 0, XYZ = 1
    };

    ccPApi();
    virtual ~ccPApi();

    ccPointCloud* loadPointCloud(const char *filename,
            CC_SHIFT_MODE mode = AUTO, int skip = 0, double x = 0, double y = 0,
            double z = 0);

protected: //file I/O

    //Extended file loading parameters
    struct CLLoadParameters: public FileIOFilter::LoadParameters
    {
        CLLoadParameters() :
                FileIOFilter::LoadParameters(), m_coordinatesShiftEnabled(
                        false), m_coordinatesShift(0, 0, 0)
        {
            shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
            alwaysDisplayLoadDialog = false;
            autoComputeNormals = false;
            coordinatesShiftEnabled = &m_coordinatesShiftEnabled;
            coordinatesShift = &m_coordinatesShift;
        }

        bool m_coordinatesShiftEnabled;
        CCVector3d m_coordinatesShift;
    };

protected: //members

    //! Currently opened point clouds and their filename
    std::vector<CLCloudDesc> m_clouds;

    //! Currently opened meshes and their filename
    std::vector<CLMeshDesc> m_meshes;

    //! Silent mode
    bool m_silentMode;

    //! Whether files should be automatically saved (after each process) or not
    bool m_autoSaveMode;

    //! Whether a timestamp should be automatically added to output files or not
    bool m_addTimestamp;

    //! Default numerical precision for ASCII output
    int m_precision;

    //! File loading parameters
    CLLoadParameters m_loadingParameters;

    //! Whether Global (coordinate) shift has already been defined
    bool m_coordinatesShiftWasEnabled;

    //! Global (coordinate) shift (if already defined)
    CCVector3d m_formerCoordinatesShift;

    //! Orphan entities
    ccHObject m_orphans;

};

#endif /* CCPAPI_CCPAPI_H_ */
