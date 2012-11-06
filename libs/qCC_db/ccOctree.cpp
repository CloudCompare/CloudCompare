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
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//

#include "ccOctree.h"

//CCLib
#include <DistanceComputationTools.h>
#include <Neighbourhood.h>

#include "ccGenericPointCloud.h"
#include "ccIncludeGL.h"

ccOctree::ccOctree(ccGenericPointCloud* aCloud) : CCLib::DgmOctree(aCloud), ccHObject("Octree")
{
    shouldBeRefreshed = true;
    setVisible(false);
    lockVisibility(false);

	glID = -1;
	displayedLevel = 1;
	displayType = DEFAULT_OCTREE_DISPLAY_TYPE;
    _associatedCloud = aCloud;
}

int ccOctree::getDisplayedLevel()
{
    return displayedLevel;
}

void ccOctree::setDisplayedLevel(int level)
{
    if (level != displayedLevel)
    {
        displayedLevel = level;
        shouldBeRefreshed = true;
    }
}

CC_OCTREE_DISPLAY_TYPE ccOctree::getDisplayType()
{
    return displayType;
}

void ccOctree::setDisplayType(CC_OCTREE_DISPLAY_TYPE type)
{
    if (displayType != type)
    {
        displayType = type;
        shouldBeRefreshed = true;
    }
}

void ccOctree::clear()
{
	if (glID>=0)
	{
		if (glIsList(glID))
			glDeleteLists(glID,1);
		glID=-1;
	}

	DgmOctree::clear();
}

ccBBox ccOctree::getMyOwnBB()
{
    CCVector3 bbMin(m_pointsMin[0],m_pointsMin[1],m_pointsMin[2]);
    CCVector3 bbMax(m_pointsMax[0],m_pointsMax[1],m_pointsMax[2]);
    return ccBBox(bbMin,bbMax);
}

ccBBox ccOctree::getDisplayBB()
{
    CCVector3 bbMin(m_dimMin[0],m_dimMin[1],m_dimMin[2]);
    CCVector3 bbMax(m_dimMax[0],m_dimMax[1],m_dimMax[2]);
    return ccBBox(bbMin,bbMax);
}

void ccOctree::multiplyBoundingBox(const PointCoordinateType multFactor)
{
	m_dimMin *= multFactor;
	m_dimMax *= multFactor;
	m_pointsMin *= multFactor;
	m_pointsMax *= multFactor;

	for (int i=0;i<=MAX_OCTREE_LEVEL;++i)
		m_cellSize[i]*=multFactor;
}

void ccOctree::translateBoundingBox(const CCVector3& T)
{
	m_dimMin[0]+=T.x; m_dimMax[0]+=T.x; m_pointsMin[0]+=T.x; m_pointsMax[0]+=T.x;
	m_dimMin[1]+=T.y; m_dimMax[1]+=T.y; m_pointsMin[1]+=T.y; m_pointsMax[1]+=T.y;
	m_dimMin[2]+=T.z; m_dimMax[2]+=T.z; m_pointsMin[2]+=T.z; m_pointsMax[2]+=T.z;
}

void ccOctree::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_thePointsAndTheirCellCodes.empty())
        return;

    if (MACRO_Draw3D(context))
    {
        bool pushName = MACRO_DrawNames(context);

        if (pushName)
            glPushName(getUniqueID());

        RenderOctreeAs(displayType,this,displayedLevel,_associatedCloud,glID,shouldBeRefreshed);

        if (shouldBeRefreshed)
            shouldBeRefreshed = false;

        if (pushName)
            glPopName();
    }
}

/*** RENDERING METHODS ***/
static GLubyte s_cubeFaceIndexes[] = {4, 5, 6, 7, 1, 2, 6, 5, 0, 1, 5, 4, 0, 3, 2, 1, 0, 4, 7, 3, 2, 3, 7, 6};
void ccOctree::RenderOctreeAs(  CC_OCTREE_DISPLAY_TYPE octreeDisplayType,
                                        CCLib::DgmOctree* theOctree,
                                        unsigned char level,
                                        ccGenericPointCloud* theAssociatedCloud,
                                        int &octreeGLListID,
                                        bool updateOctreeGLDisplay)
{
	if ((!theOctree)||(!theAssociatedCloud))
        return;

	//ccConsole::Print("[Octree::Render] Level %i (GL_ID=%i) [refresh=%i]\n",level,octreeGLListID,updateOctreeGLDisplay);

	//cet affichage demande trop de mémoire pour le stocker sous forme de liste OpenGL
	//donc on doit le générer dynamiquement
	if (octreeDisplayType==WIRE)
	{
		//au cas où la lumière soit allumée
		glPushAttrib(GL_LIGHTING_BIT);
		glDisable(GL_LIGHTING);
		glColor3ubv(ccColor::green);
		theOctree->executeFunctionForAllCellsAtLevel(level,&DrawCellAsABox,NULL);
		glPopAttrib();
	}
	else
	{
        glDrawParams glParams;
        theAssociatedCloud->getDrawingParameters(glParams);

        if (glParams.showNorms)
		{
            //DGM: Strangely, when Qt::renderPixmap is called, the OpenGL version is sometimes 1.0!
            glEnable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ccColor::darker);
            glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,ccColor::lighter);
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,ccColor::bright);
            glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,ccColor::darker);
            glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,50.0);
            glEnable(GL_LIGHTING);

            glEnable(GL_COLOR_MATERIAL);
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
        }

        if (!glParams.showColors)
            glColor3ubv(ccColor::white);

		if ((updateOctreeGLDisplay)||(octreeGLListID<0))
		{
			if (octreeGLListID<0)
				octreeGLListID = glGenLists(1);
			else if (glIsList(octreeGLListID))
				glDeleteLists(octreeGLListID,1);

			//on ne peut pas afficher de primitives graphiques pendant cette phase, sinon elles risquent
			//d'être capturée dans la liste OpenGL !
			glNewList(octreeGLListID,GL_COMPILE);

			//structure contenant les parametres additionnels
			if (octreeDisplayType == MEAN_POINTS)
			{
				void* additionalParameters[2];
				additionalParameters[0] = (void*)&glParams;
				additionalParameters[1] = (void*)theAssociatedCloud;

				glBegin(GL_POINTS);
				theOctree->executeFunctionForAllCellsAtLevel(level,&DrawCellAsAPoint,additionalParameters,0,"Render octree");
				glEnd();
			}
			else
			{
				GLfloat cs = theOctree->getCellSize(level)*0.5;
				GLfloat cubeVertices[24];
				cubeVertices[0] = -cs;	cubeVertices[1] = -cs;	cubeVertices[2] = -cs;
				cubeVertices[3] = cs;	cubeVertices[4] = -cs;	cubeVertices[5] = -cs;
				cubeVertices[6] = cs;	cubeVertices[7] = cs;	cubeVertices[8] = -cs;
				cubeVertices[9] = -cs;	cubeVertices[10] = cs;	cubeVertices[11] = -cs;
				cubeVertices[12] = -cs;	cubeVertices[13] = -cs;	cubeVertices[14] = cs;
				cubeVertices[15] = cs;	cubeVertices[16] = -cs;	cubeVertices[17] = cs;
				cubeVertices[18] = cs;	cubeVertices[19] = cs;	cubeVertices[20] = cs;
				cubeVertices[21] = -cs;	cubeVertices[22] = cs;	cubeVertices[23] = cs;

				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer (3, GL_FLOAT, 0, cubeVertices);

				void* additionalParameters[4];
				additionalParameters[0] = (void*)&glParams;
				additionalParameters[1] = (void*)theAssociatedCloud;
				additionalParameters[2] = (void*)cubeVertices;
				additionalParameters[3] = (void*)s_cubeFaceIndexes;

				theOctree->executeFunctionForAllCellsAtLevel(level,&DrawCellAsAPlainCube,additionalParameters,0);

				glDisableClientState(GL_VERTEX_ARRAY);
			}

			glEndList();
		}

		glCallList(octreeGLListID);

        if (glParams.showNorms)
		{
            glDisable(GL_COLOR_MATERIAL);
            glDisable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
            glDisable(GL_LIGHTING);
        }
	}
}


//FONCTION "CELLULAIRE" D'AFFICHAGE "FIL DE FER"
bool ccOctree::DrawCellAsABox(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//tableaux de coordonnées "temporaires"
	CCVector3 bbMin,bbMax;

	cell.parentOctree->computeCellLimits(cell.truncatedCode,cell.level,bbMin.u,bbMax.u,true);

	//ccConsole::Print("bbMin=(%f,%f,%f)",bbMin.x,bbMin.y,bbMin.z);
	//ccConsole::Print("bbMax=(%f,%f,%f)",bbMax.x,bbMax.y,bbMax.z);

    glColor3ubv(ccColor::green);
    glBegin(GL_LINE_LOOP);
    glVertex3fv(bbMin.u);
    glVertex3f(bbMax.x,bbMin.y,bbMin.z);
    glVertex3f(bbMax.x,bbMax.y,bbMin.z);
    glVertex3f(bbMin.x,bbMax.y,bbMin.z);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(bbMin.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMin.y,bbMax.z);
    glVertex3fv(bbMax.u);
    glVertex3f(bbMin.x,bbMax.y,bbMax.z);
    glEnd();

    glBegin(GL_LINES);
    glVertex3fv(bbMin.u);
    glVertex3f(bbMin.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMin.y,bbMin.z);
    glVertex3f(bbMax.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMax.y,bbMin.z);
    glVertex3fv(bbMax.u);
    glVertex3f(bbMin.x,bbMax.y,bbMin.z);
    glVertex3f(bbMin.x,bbMax.y,bbMax.z);
    glEnd();

	return true;
}

//FONCTION "CELLULAIRE" D'AFFICHAGE "POINT MOYEN"
bool ccOctree::DrawCellAsAPoint(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	glDrawParams* glParams						= (glDrawParams*)additionalParameters[0];
	ccGenericPointCloud* theAssociatedCloud		= (ccGenericPointCloud*)additionalParameters[1];

	if (glParams->showSF)
	{
        DistanceType dist = CCLib::DistanceComputationTools::computeMeanDist(cell.points);
	    const colorType* col = theAssociatedCloud->getDistanceColor(dist);
        glColor3ubv(col ? col : ccColor::lightGrey);
	}
	else
	{
		if (glParams->showColors)
		{
			colorType col[3];
			ComputeAverageColor(cell.points,theAssociatedCloud,col);
			glColor3ubv(col); // on peut mettre directement le pointeur ?
		}

		if (glParams->showNorms)
		{
			GLfloat N[3];
			ComputeAverageNorm(cell.points,theAssociatedCloud,N);
			glNormal3fv(N);
		}
	}

	const CCVector3* gravityCenter = CCLib::Neighbourhood(cell.points).getGravityCenter();
	glVertex3fv(gravityCenter->u);

	return true;
}


//FONCTION "CELLULAIRE" D'AFFICHAGE "CUBE MOYEN"
bool ccOctree::DrawCellAsAPlainCube(const CCLib::DgmOctree::octreeCell& cell, void** additionalParameters)
{
	//variables additionnelles
	glDrawParams* glParams						= (glDrawParams*)additionalParameters[0];
	ccGenericPointCloud* theAssociatedCloud	    = (ccGenericPointCloud*)additionalParameters[1];
	//GLfloat* cubeVertexesCoords				= (GLfloat*)additionalParameters[2];
	GLubyte* cubeVertexesIndexes				= (GLubyte*)additionalParameters[3];

	//GLfloat* gravityCenter = (GLfloat*)cell.points->getGeometricalElement(GRAVITY_CENTER);
	GLfloat cellCenter[3];
	cell.parentOctree->computeCellCenter(cell.truncatedCode,cell.level,cellCenter,true);

	if (glParams->showSF)
	{
        DistanceType dist = CCLib::DistanceComputationTools::computeMeanDist(cell.points);
	    const colorType* col = theAssociatedCloud->getDistanceColor(dist);
        glColor3ubv(col ? col : ccColor::lightGrey);
	}
	else
	{
		if (glParams->showColors)
		{
			colorType col[3];
			ComputeAverageColor(cell.points,theAssociatedCloud,col);
			glColor3ubv(col); // on peut mettre directement le pointeur ?
		}

		if (glParams->showNorms)
		{
			GLfloat N[3];
			ComputeAverageNorm(cell.points,theAssociatedCloud,N);
			glNormal3fv(N);
		}
	}

	glPushMatrix();
	glTranslatef(cellCenter[0],cellCenter[1],cellCenter[2]);
	glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cubeVertexesIndexes);
	glPopMatrix();

	return true;
}

void ccOctree::ComputeAverageColor(CCLib::ReferenceCloud* subset, ccGenericPointCloud* sourceList, colorType meanCol[])
{
	if ((!subset)||(!sourceList))
        return;

	if (subset->size()==0)
        return;

    assert(sourceList->hasColors());

	double Rsum=0.0;
	double Gsum=0.0;
	double Bsum=0.0;

	unsigned i,n=subset->size();

	subset->placeIteratorAtBegining();
	for (i=0;i<n;++i)
	{
		//pointeur sur la couleur du point courant d'index theIterator
		const colorType* _theColors = sourceList->getPointColor(subset->getCurrentPointGlobalIndex());

		Rsum += double(*_theColors);
		++_theColors;
		Gsum += double(*_theColors);
		++_theColors;
		Bsum += double(*_theColors);
	}

	double coef =  1.0 / double(n);

	meanCol[0] = colorType( Rsum * coef);
	meanCol[1] = colorType( Gsum * coef);
	meanCol[2] = colorType( Bsum * coef);
}

void ccOctree::ComputeAverageNorm(CCLib::ReferenceCloud* subset, ccGenericPointCloud* cloud, float norm[])
{
	if (!subset || !cloud)
        return;

	unsigned n=subset->size();
	if (n==0)
        return;

	assert(cloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCLib::GenericIndexedCloud*>(cloud));

	norm[0] = norm[1] = norm[2] = 0.0;

	//calcul des sommes
	subset->placeIteratorAtBegining();
	for (unsigned i=0;i<n;++i)
	{
		const PointCoordinateType* N = cloud->getPointNormal(subset->getCurrentPointGlobalIndex());
		CCVector3::vadd(norm,N,norm);
		subset->forwardIterator();
	}

	CCVector3::vnormalize(norm);
}

void ccOctree::ComputeRobustAverageNorm(CCLib::ReferenceCloud* subset, ccGenericPointCloud* cloud, PointCoordinateType norm[])
{
	if (!subset || !cloud)
        return;

	unsigned n=subset->size();
	if (n==0)
        return;

	assert(cloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCLib::GenericIndexedCloud*>(cloud));

	norm[0] = norm[1] = norm[2] = 0.0;

	unsigned i=0;
	CCVector3 Nplane;
	subset->placeIteratorAtBegining();

	//on chope la norme du plan interpolant aux moindres carrés
	const PointCoordinateType* Nlsq = CCLib::Neighbourhood(subset).getLSQPlane();
	if (Nlsq)
	{
		Nplane = CCVector3(Nlsq);
	}
	else //si elle n'existe pas (pas assez de points ?)
	{
		//alors on prend le premier vecteur normal
		const PointCoordinateType* N = cloud->getPointNormal(subset->getCurrentPointGlobalIndex());
		Nplane = CCVector3(N);
		subset->forwardIterator();
		++i;
	}

	//maintenant on fait la somme des autres vecteurs normaux, en faisant attention au sens global
	float ps;
	for (;i<n;++i)
	{
		const PointCoordinateType* N = cloud->getPointNormal(subset->getCurrentPointGlobalIndex());
		//calcul du produit scalaire entre la ième normale et la normale du plan aux moindres carrés
		//(pour savoir de quel côté pointe la normale)
		ps = CCVector3::vdot(N,Nplane.u);
		if (ps<0.0)
		{
			CCVector3::vsubstract(norm,N,norm);
		}
		else
		{
			CCVector3::vadd(norm,N,norm);
		}

		subset->forwardIterator();
	}

	//norme moyenne
	CCVector3::vnormalize(norm);
}
