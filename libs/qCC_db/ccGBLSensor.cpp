//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1733                                                              $
//$LastChangedDate:: 2010-12-02 14:20:04 +0100 (jeu., 02 déc. 2010)       $
//**************************************************************************
//

#include "ccGBLSensor.h"

#include "ccIncludeGL.h"

ccGBLSensor::ccGBLSensor(CCLib::CC_SENSOR_ROTATION_ORDER rotOrder) : CCLib::GroundBasedLidarSensor(rotOrder), ccSensor()
{
    setName("Ground Based Laser Scanner");

    //graphic representation
    lockVisibility(false);
    scale=1.0;
}

void ccGBLSensor::setGraphicScale(double _scale)
{
    scale = _scale;
}

double ccGBLSensor::getGraphicScale()
{
    return scale;
}

void ccGBLSensor::updateGraphicRepresentation()
{
    glTrans.toIdentity();

    //rotation matrix
    if (rotation)
        glTrans = ccGLMatrix(*rotation,CCVector3(),sensorCenter); //rotation center = sensor center

    //translation = sensor center
    glTrans += sensorCenter;
}

void ccGBLSensor::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    //we draw here a little 3d representation of the sensor
    if (MACRO_Draw3D(context))
    {
        bool pushName = MACRO_DrawNames(context);

        if (pushName)
            glPushName(getUniqueID());

        //TODO: apply orientation!

        //sensor head
        const double halfHeadSize=0.3;
        CCVector3 minCorner(-halfHeadSize,-halfHeadSize,-halfHeadSize);
        CCVector3 maxCorner(halfHeadSize,halfHeadSize,halfHeadSize);
        minCorner*=scale;
        maxCorner*=scale;
        ccBBox bbHead(minCorner,maxCorner);
        CCVector3 headCenter(0.0,0.0,(1.0-halfHeadSize)*scale);
        bbHead += headCenter;
        bbHead.draw(ccColor::green);

        //sensor legs
        CCVector3 headConnect = headCenter - CCVector3(0.0,0.0,halfHeadSize*scale);
        glBegin(GL_LINES);
        glVertex3fv(headConnect.u);
        glVertex3f(-scale,-scale,-scale);
        glVertex3fv(headConnect.u);
        glVertex3f(-scale,scale,-scale);
        glVertex3fv(headConnect.u);
        glVertex3f(scale,0.0,-scale);
        glEnd();


        if (pushName)
            glPopName();
    }
}

/*ccBBox ccGBLSensor::getMyOwnBB()
{
}
//*/

ccBBox ccGBLSensor::getDisplayBB()
{
    CCVector3 minCorner(-1.0,-1.0,-1.0);
    CCVector3 maxCorner(1.0,1.0,1.0);
    minCorner*=scale;
    maxCorner*=scale;

    return ccBBox(minCorner,maxCorner);
}

