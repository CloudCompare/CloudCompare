//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "GroundBasedLidarSensor.h"

#include "GenericIndexedCloud.h"
#include "SimpleCloud.h"
#include "CCMiscTools.h"
#include "CCGeom.h"

using namespace CCLib;

GroundBasedLidarSensor::GroundBasedLidarSensor(CC_SENSOR_ROTATION_ORDER rotOrder)
{
    //depth buffer descriptor
    dB.zBuff=0;
    dB.h_buff=dB.l_buff=0;

    //sensor pose
    setSensorCenter(CCVector3(0.0,0.0,0.0));

    //axis/rotation matrices
    axis=rotation=0;

    //sensor parameters
    setPhiMin(0.0);
    setPhiMax(0.0);
    setThetaMin(0.0);
    setThetaMax(0.0);
    setDeltaPhi(5e-3f);      //5e-3 rad. ~= 1/12 degrees
    setDeltaTheta(5e-3f);    //5e-3 rad. ~= 1/12 degrees
    setSensorRange(1e12f);
    setRotationOrder(rotOrder);
    setSensorBase(0.0f);
    setUncertainty(0.01f);   //uncertainty = 1% (default)
}

GroundBasedLidarSensor::~GroundBasedLidarSensor()
{
    if (axis)
        delete axis;

    if (rotation)
        delete rotation;

    if (dB.zBuff)
        delete[] dB.zBuff;
}

inline void GroundBasedLidarSensor::projectPoint(const CCVector3& sourcePoint, CCVector2& destPoint, DistanceType &dist)
{
    CCVector3 P = sourcePoint - sensorCenter;
    P.x += base;

    if (rotation)
        rotation->apply(P.u);

    float norm;

    //VERSION GS (theta,phi)
    if (rotationOrder==GBL_THETA_PHI)
    {
        destPoint.x = atan2(P.x,P.y);
        norm = P.x*P.x+P.y*P.y;
        destPoint.y = atan2(P.z,sqrt(norm));
        norm += P.z*P.z;
    }
    //VERSION SOISIC (phi,theta)
    else //if (rotationOrder==GBL_PHI_THETA)
    {
        destPoint.y = -atan2(P.y,P.z);
        norm = P.y*P.y+P.z*P.z;
        destPoint.x = -atan2(sqrt(norm),P.x);
        norm += P.x*P.x;
    }

    dist = sqrt(norm);
}

SimpleCloud* GroundBasedLidarSensor::project(GenericCloud* theCloud, int& errorCode, bool getParameters)
{
    if (!theCloud)
    {
        errorCode = -1;
        return 0;
    }

    SimpleCloud* newCloud = new SimpleCloud();

    unsigned n = theCloud->size();
    if (!newCloud->reserve(n)) //not enough memory
	{
		errorCode = -4;
		delete newCloud;
		return 0;
	}
    newCloud->enableScalarField();

    CCVector2 Q;
    DistanceType dist,maxDist=0.0;

    theCloud->placeIteratorAtBegining();
    unsigned i=0;
    for (;i<n;++i)
    {
        const CCVector3 *P = theCloud->getNextPoint();
        projectPoint(*P,Q,dist);

        newCloud->addPoint(CCVector3(Q.x,Q.y,0.0));
        newCloud->setPointScalarValue(i,dist);

        if (i==0)
            maxDist = dist;
        else
            maxDist = ccMax(maxDist,dist);
    }

    if (getParameters)
    {
        PointCoordinateType Mins[3],Maxs[3];
        newCloud->getBoundingBox(Mins,Maxs);
        //on rajoute l'information capteur à la liste
        setTheta(Mins[0],Maxs[0]);
        setPhi(Mins[1],Maxs[1]);
        setSensorRange(maxDist);
    }

    //on peut créer maintenant le Z-buffer
    float dt = 1.0f/deltaTheta;
    float dp = 1.0f/deltaPhi;

    int x,y;
    dB.l_buff = int(ceil((thetaMax-thetaMin)*dt));
    dB.h_buff = int(ceil((phiMax-phiMin)*dp));

    int mSize = ccMax(dB.l_buff,dB.h_buff);
    if (mSize>2048) //too big
    {
        errorCode = -2;
        delete newCloud;
        return 0;
    }
    else if (mSize==0) //too small ;)
    {
        errorCode = -3;
        delete newCloud;
        return 0;
    }

    unsigned zBuffSize = dB.l_buff*dB.h_buff;

    //printf("Z-Buffer : [%i * %i] (%i octets)\n",dB.l_buff,dB.h_buff,zBuffSize*sizeof(DistanceType));
    if (dB.zBuff)
        delete[] dB.zBuff;
    dB.zBuff = new DistanceType[zBuffSize];
    if (!dB.zBuff) //not enough memory
    {
        errorCode = -4;
        delete newCloud;
        return 0;
    }
    memset(dB.zBuff,0,zBuffSize*sizeof(DistanceType));

    //on projette les points de newCloud (points exprimés dans le "repère" du capteur) dans le Z-Buffer
    DistanceType* _zBuffTemp;
    newCloud->placeIteratorAtBegining();
    for (i=0;i<newCloud->size();++i)
    {
        const CCVector3 *P = newCloud->getNextPoint();
        dist = newCloud->getPointScalarValue(i);

        x = int(floor((P->x-thetaMin)*dt));
        y = int(floor((P->y-phiMin)*dp));

        _zBuffTemp = dB.zBuff + (y*dB.l_buff+x);
        *_zBuffTemp = ccMax(*_zBuffTemp,dist);
    }

    //fillZBufferHoles();

    errorCode = 0;
    return newCloud;
}

int GroundBasedLidarSensor::fillZBufferHoles()
{
    if (!dB.zBuff)
        return -1; //no z-buffer !

    //new temp buffer
    unsigned tempZBuffSize = (dB.l_buff+2)*(dB.h_buff+2);
    DistanceType *zBuffTemp = new DistanceType[tempZBuffSize];
    if (!zBuffTemp)
        return -2; //not enough memory


    //RAZ : on copie le zBuffer dans un buffer plus grand (bordure de 1 case)
    int y,dx=dB.l_buff+2;
    memset(zBuffTemp,0,tempZBuffSize*sizeof(DistanceType));
    DistanceType *_zBuffTemp = zBuffTemp+dx+1; //on se positionne sur la deuxième ligne, deuxième colonne du buffer temporaire
    DistanceType *_zBuff = dB.zBuff; //on se positionne sur la première ligne du vrai buffer
    for (y=0;y<dB.h_buff;++y)
    {
        //on recopie chaque ligne à "l'intérieur" du buffer
        memcpy(_zBuffTemp,_zBuff,dB.l_buff*sizeof(DistanceType));
        _zBuffTemp += dx;
        _zBuff += dB.l_buff;
    }

    //on applique ici une petite opération de remplissage des trous et on copie les valeurs dans zBuff
    //il fautdrait peut-être faire des opérations d'erosion/dilatation plutôt ?
    DistanceType *zu,*zd;
    int nsup,x;
    for (y=0;y<dB.h_buff;++y)
    {
        zu = zBuffTemp+y*dx;
        _zBuffTemp = zu+dx;
        zd = _zBuffTemp+dx;
        for (x=0;x<dB.l_buff;++x)
        {
            if (_zBuffTemp[1]==0) //si c'est un trou !
            {
                //on compte le nombre de voisins non vides
                nsup=0;
                nsup += (zu[0]>0);
                nsup += (zu[1]>0);
                nsup += (zu[2]>0);
                nsup += (_zBuffTemp[0]>0);
                //nsup += (_zBuffTemp[1]>0); //inutile !
                nsup += (_zBuffTemp[2]>0);
                nsup += (zd[0]>0);
                nsup += (zd[1]>0);
                nsup += (zd[2]>0);

                //si y'en a plus de 3 non vides
                if (nsup>3)
                {
                    //on remplace le trou par la moyenne de ses voisins
                    dB.zBuff[x+y*dB.l_buff] = (zu[0]+zu[1]+zu[2]+
                                               _zBuffTemp[0]+_zBuffTemp[2]+
                                               zd[0]+zd[1]+zd[2])/DistanceType(nsup);
                }
            }

            ++zu;
            ++_zBuffTemp;
            ++zd;
        }
    }

    delete[] zBuffTemp;

    return 0;
}

PointCoordinateType* GroundBasedLidarSensor::projectNormals(GenericCloud* aCloud, GenericChunkedArray<3,PointCoordinateType>& theNorms)
{
    if ((!aCloud)||(!theNorms.isAllocated()))
        return 0;

    unsigned size = 3*dB.h_buff*dB.l_buff;
    PointCoordinateType* theNewNorms = new PointCoordinateType[size];
    if (!theNewNorms)
        return 0; //not enough memory
    memset(theNewNorms,0,size*sizeof(PointCoordinateType));

    float dt = 1.0f/deltaTheta;
    float dp = 1.0f/deltaPhi;
    int x,y;

    PointCoordinateType *N,*_theNewNorms;
    CCVector3 R,S;
    CCVector2 Q,S2;
    DistanceType dist1,dist2;

    aCloud->placeIteratorAtBegining();
    theNorms.placeIteratorAtBegining();
    unsigned i,n = aCloud->size();
    for (i=0;i<n;++i)
    {
        //le ième point et sa normale
        const CCVector3 *P = aCloud->getNextPoint();
        N = theNorms.getCurrentValue();

        CCVector3 U = *P - sensorCenter;
        U.x += base;

        //norme du vecteur "direction de visée laser"
        float norm = U.norm();


        //si sa norme est nulle, alors le point serait confondu avec le centre du capteur (peu probable ...)
        if (norm>1e-7)
        {
            //composante de la normale selon la direction de visée ("S.z")
            S.z = -CCVector3::vdot(N,U.u)/norm;

            if (S.z > 0.99999)
            {
                S.x = 0.0;
                S.y = 0.0;
            }
            else
            {
                //on projette le point
                projectPoint(*P,Q,dist1);

                //et l'autre extrémité du vecteur normal
                R.x = P->x + N[0];
                R.y = P->y + N[1];
                R.z = P->z + N[2];
                projectPoint(R,S2,dist2);

                //la normale projetée est égale selon les autres dimensions à la différence des deux (à un coefficient près ;)
                S.x = S2.x - Q.x;
                S.y = S2.y - Q.y;

                float coef = sqrt((1.0f-S.z*S.z)/(S.x*S.x+S.y*S.y));

                S.x = coef * S.x;
                S.y = coef * S.y;
            }
        }
        else
        {
            //sinon on se casse pas la tête, on renvoie la même normale
            S.x = N[0];
            S.y = N[1];
            S.z = N[2];
        }

        //on projette dans le Z-buffer
        x = int(floor((Q.x-thetaMin)*dt));
        y = int(floor((Q.y-phiMin)*dp));

        //on ajoute la normale transformée
        _theNewNorms = theNewNorms + 3*(y*dB.l_buff+x);
        CCVector3::vadd(_theNewNorms,S.u,_theNewNorms);

        theNorms.forwardIterator();
    }

    //on normalise tout (au cas où il y ait des accumulations)
    _theNewNorms = theNewNorms;
    for (i=0;i<unsigned(dB.h_buff*dB.l_buff);++i)
    {
        CCVector3::vnormalize(_theNewNorms);
        _theNewNorms+=3;
    }

    return theNewNorms;
}

uchar* GroundBasedLidarSensor::projectColors(GenericCloud* aCloud, GenericChunkedArray<3,uchar>& theColors)
{
    if ((!aCloud)||(!theColors.isAllocated()))
        return 0;

    unsigned size = dB.h_buff*dB.l_buff;
    uchar* theNewColorsCount = new uchar[size];
    if (!theNewColorsCount)
        return 0; //not enough memory
    memset(theNewColorsCount,0,size*sizeof(uchar));

    size *= 3;
    uchar* theNewColors = new uchar[size];
    if (!theNewColors)
    {
        delete[] theNewColorsCount;
        return 0; //not enough memory
    }
    memset(theNewColors,0,size*sizeof(uchar));

    float dt = 1.0f/deltaTheta;
    float dp = 1.0f/deltaPhi;
    int x,y,index;

    uchar *C,*_theNewColors;
    CCVector2 Q;
    DistanceType dist;

    aCloud->placeIteratorAtBegining();
    theColors.placeIteratorAtBegining();
    unsigned n = aCloud->size();
    unsigned i=0;
    for (;i<n;++i)
    {
        //on projette le point
        const CCVector3 *P = aCloud->getNextPoint();
        projectPoint(*P,Q,dist);

        //on projette dans le Z-buffer
        x = int(floor((Q.x-thetaMin)*dt));
        y = int(floor((Q.y-phiMin)*dp));
        index = y*dB.l_buff+x;
        ++theNewColorsCount[index];
        _theNewColors = theNewColors + 3*index;

        //on ajoute la couleur
        C = theColors.getCurrentValue();

        if ((_theNewColors[0]>0)||(_theNewColors[1]>0)||(_theNewColors[2]>0))
        {
            //petite moyenne "mobile" dégueux, pour éviter les overflows !
            _theNewColors[0]=uchar((int(_theNewColors[0])+int(C[0]))>>1);
            _theNewColors[1]=uchar((int(_theNewColors[1])+int(C[1]))>>1);
            _theNewColors[2]=uchar((int(_theNewColors[2])+int(C[2]))>>1);
        }
        else
        {
            _theNewColors[0]=C[0];
            _theNewColors[1]=C[1];
            _theNewColors[2]=C[2];
        }

        theColors.forwardIterator();
    }

    //on normalise tout (au cas où il y ait des accumulations)
    _theNewColors = theNewColors;
    float coef;
    for (;i<unsigned(dB.h_buff*dB.l_buff);++i)
    {
        if (theNewColorsCount[i]>1)
        {
            //on calcule betement la moyenne de chaque composante (?)
            coef = 1.0f/(float)theNewColorsCount[i];
            _theNewColors[0] = uchar(float(_theNewColors[0])*coef);
            _theNewColors[1] = uchar(float(_theNewColors[1])*coef);
            _theNewColors[2] = uchar(float(_theNewColors[2])*coef);
        }
        _theNewColors+=3;
    }

    delete[] theNewColorsCount;

    return theNewColors;
}

//test de profondeur sur le Zbuffer associé au capteur pour savoir si le point
//peut-être vu par le scanner ou non (i.e. il est caché,hors champ, ou hors de portée)
CC_VISIBILITY_TYPE GroundBasedLidarSensor::checkVisibility(const CCVector3& aPoint)
{
    //si on a pas de zBuffer, on renvoie "vu" par défaut
    if (!dB.zBuff)
        return VIEWED;

    CCVector2 q;
    DistanceType dist;
    int x,y;

    //on projette le point dans le "repère" du capteur
    projectPoint(aPoint,q,dist);

    //s'il est hors de portée
    if (dist>sensorRange)
        return OUT_OF_RANGE;

    x = int(floor((q.x-thetaMin)/deltaTheta));
    y = int(floor((q.y-phiMin)/deltaPhi));

    //s'il est hors-champ
    if (((x<0)||(x>=dB.l_buff))
            ||((y<0)||(y>=dB.h_buff)))
        return OUT_OF_FOV;

    //s'il est caché
    if (dist > dB.zBuff[x+y*dB.l_buff]*(1.0+uncertainty))
        return HIDDEN;

    return VIEWED;
}

void GroundBasedLidarSensor::setRotationMatrix(CCLib::SquareMatrix* m)
{
    if (rotation)
        delete rotation;
    rotation = m;

    if (rotation)
    {
        if (axis)
            delete axis;
        axis = new CCLib::SquareMatrix(rotation->inv());
    }
}

CCLib::SquareMatrix* GroundBasedLidarSensor::getRotationMatrix()
{
    return rotation;
}

void GroundBasedLidarSensor::setAxisMatrix(CCLib::SquareMatrix* m)
{
    if (axis)
        delete axis;
    axis = m;

    if (axis)
    {
        if (rotation)
            delete rotation;
        rotation = new CCLib::SquareMatrix(axis->inv());
    }
}

CCLib::SquareMatrix* GroundBasedLidarSensor::getAxisMatrix()
{
    return axis;
}

void GroundBasedLidarSensor::setPhi(float minV, float maxV)
{
    phiMin=minV;
    phiMax=maxV;
}

void GroundBasedLidarSensor::setPhiMin(float minV)
{
    phiMin=minV;
}

void GroundBasedLidarSensor::setPhiMax(float maxV)
{
    phiMax=maxV;
}

void GroundBasedLidarSensor::setDeltaPhi(float dPhi)
{
    deltaPhi=dPhi;
}

float GroundBasedLidarSensor::getPhiMin()
{
    return phiMin;
}

float GroundBasedLidarSensor::getPhiMax()
{
    return phiMax;
}

float GroundBasedLidarSensor::getDeltaPhi()
{
    return deltaPhi;
}

void GroundBasedLidarSensor::setTheta(float minV, float maxV)
{
    thetaMin=minV;
    thetaMax=maxV;
}

void GroundBasedLidarSensor::setThetaMin(float minV)
{
    thetaMin=minV;
}

void GroundBasedLidarSensor::setThetaMax(float maxV)
{
    thetaMax=maxV;
}

void GroundBasedLidarSensor::setDeltaTheta(float dTheta)
{
    deltaTheta=dTheta;
}

float GroundBasedLidarSensor::getThetaMin()
{
    return thetaMin;
}

float GroundBasedLidarSensor::getThetaMax()
{
    return thetaMax;
}

float GroundBasedLidarSensor::getDeltaTheta()
{
    return deltaTheta;
}

float GroundBasedLidarSensor::getSensorBase()
{
    return base;
}

void GroundBasedLidarSensor::setSensorBase(PointCoordinateType _base)
{
    base=_base;
}

DistanceType GroundBasedLidarSensor::getSensorRange()
{
    return sensorRange;
}

void GroundBasedLidarSensor::setSensorRange(DistanceType range)
{
    sensorRange = range;
}

CCVector3 GroundBasedLidarSensor::getSensorCenter()
{
    return sensorCenter;
}

void GroundBasedLidarSensor::setSensorCenter(const CCVector3& C)
{
    sensorCenter=C;
}

DistanceType GroundBasedLidarSensor::getUncertainty()
{
    return uncertainty;
}

void GroundBasedLidarSensor::setUncertainty(DistanceType u)
{
    uncertainty=u;
}

CC_SENSOR_ROTATION_ORDER GroundBasedLidarSensor::getRotationOrder()
{
    return rotationOrder;
}

void GroundBasedLidarSensor::setRotationOrder(CC_SENSOR_ROTATION_ORDER r)
{
    rotationOrder=r;
}

GroundBasedLidarSensor::DepthBuffer GroundBasedLidarSensor::getDepthBuffer()
{
    return dB;
}

