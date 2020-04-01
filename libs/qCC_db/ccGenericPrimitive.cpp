//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
#include "ccIncludeGL.h"

#include "ccGenericPrimitive.h"
#include "ccPointCloud.h"

ccGenericPrimitive::ccGenericPrimitive(	QString name/*=QString()*/,
										const ccGLMatrix* transMat/*=0*/,
										unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccMesh(new ccPointCloud("vertices"), uniqueID)
	, m_drawPrecision(0)
{
	setName(name);
	showNormals(true);

	ccPointCloud* vert = vertices();
	assert(vert);
	addChild(vert);
	vert->setEnabled(false);
	//we don't want the user to transform the vertices for instance (as they are only temporary)
	vert->setLocked(true);

	if (transMat)
		m_transformation = *transMat;
}

void ccGenericPrimitive::setColor(const ccColor::Rgb& col)
{
	if (m_associatedCloud)
	{
		static_cast<ccPointCloud*>(m_associatedCloud)->setColor(col);
	}
}

ccPointCloud* ccGenericPrimitive::vertices()
{
	return static_cast<ccPointCloud*>(m_associatedCloud);
}

const ccGenericPrimitive& ccGenericPrimitive::operator += (const ccGenericPrimitive& prim)
{
	ccPointCloud* verts = vertices();
	unsigned vertCount = verts->size();
	unsigned facesCount = size();
	unsigned triFacesNormCount = (m_triNormals ? m_triNormals->currentSize() : 0);

	//count new number of vertices & faces
	unsigned newVertCount = vertCount + prim.getAssociatedCloud()->size();
	unsigned newFacesCount = facesCount + prim.size();
	bool primHasVertNorms = prim.getAssociatedCloud()->hasNormals();
	bool primHasFaceNorms = prim.hasTriNormals();

	//reserve memory
	if (	verts->reserve(newVertCount)
		&&	(!primHasVertNorms || verts->reserveTheNormsTable())
		&&	reserve(newFacesCount)
		&&	(!primHasFaceNorms || m_triNormalIndexes || reservePerTriangleNormalIndexes()))
	{
		//copy vertices & normals
		ccGenericPointCloud* cloud = prim.getAssociatedCloud();
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			verts->addPoint(*cloud->getPoint(i));
			if (primHasVertNorms)
			{
				verts->addNormIndex(cloud->getPointNormalIndex(i));
			}
		}

		//copy face normals
		if (primHasFaceNorms)
		{
			const NormsIndexesTableType* primNorms = prim.getTriNormsTable();
			assert(primNorms);
			unsigned primTriNormCount = primNorms->currentSize();

			NormsIndexesTableType* normsTable = (m_triNormals ? m_triNormals : new NormsIndexesTableType());
			if (!normsTable || !normsTable->reserveSafe(triFacesNormCount + primTriNormCount))
			{
				ccLog::Error("[ccGenericPrimitive::operator +] Not enough memory!");
				return *this;
			}

			//attach table if not done already
			if (!m_triNormals)
			{
				setTriNormsTable(normsTable);
				assert(m_triNormals);
			}

			for (unsigned i = 0; i < primTriNormCount; ++i)
			{
				normsTable->addElement(primNorms->getValue(i));
			}
		}

		//copy faces
		for (unsigned i = 0; i < prim.size(); ++i)
		{
			const CCLib::VerticesIndexes* tsi = prim.getTriangleVertIndexes(i);
			addTriangle(vertCount + tsi->i1, vertCount + tsi->i2, vertCount + tsi->i3);
			if (primHasFaceNorms)
			{
				const Tuple3i& normIndexes = prim.m_triNormalIndexes->at(i);
				addTriangleNormalIndexes(triFacesNormCount + normIndexes.u[0], triFacesNormCount + normIndexes.u[1], triFacesNormCount + normIndexes.u[2]);
			}
		}
	}
	else
	{
		ccLog::Error("[ccGenericPrimitive::operator +] Not enough memory!");
	}

	return *this;
}

bool ccGenericPrimitive::toFile_MeOnly(QFile& out) const
{
	if (!ccMesh::toFile_MeOnly(out))
		return false;

	//Transformation matrix backup (dataVersion>=21)
	if (!m_transformation.toFile(out))
		return false;

	//'drawing precision' (dataVersion>=21))
	if (out.write((const char*)&m_drawPrecision,sizeof(unsigned)) < 0)
		return WriteError();

	return true;
}

bool ccGenericPrimitive::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccMesh::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//HACK: first, we have to remove any 'wrongly' associated vertices cloud!
	//(this is in fact the default one - automatically created on construction)
	//while the true vertices come as a child (at least it should;)
	if (getChildrenNumber() && getChild(0)->isKindOf(CC_TYPES::POINT_CLOUD) && getChild(0) != m_associatedCloud)
		removeChild(0);

	//Transformation matrix backup (dataVersion>=21)
	if (!m_transformation.fromFile(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//'drawing precision' (dataVersion>=21))
	if (in.read((char*)&m_drawPrecision, sizeof(unsigned)) < 0)
		return ReadError();

	return true;
}

bool ccGenericPrimitive::setDrawingPrecision(unsigned steps)
{
	if (m_drawPrecision == steps)
		return true;
	if (steps < MIN_DRAWING_PRECISION)
		return false;

	m_drawPrecision = steps;

	return updateRepresentation();
}

bool ccGenericPrimitive::updateRepresentation()
{
	bool success = buildUp();
	if (success)
	{
		applyTransformationToVertices();
	}

	return success;
}

void ccGenericPrimitive::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccMesh::applyGLTransformation(trans);

	//we update the vertices transformation
	m_transformation = trans * m_transformation;
}

const ccGLMatrix& ccGenericPrimitive::getGLTransformationHistory() const
{
	return m_transformation;
}

void ccGenericPrimitive::applyTransformationToVertices()
{
	//we apply associated transformation but as a call 
	//to 'applyGLTransformation_recursive' will multiply
	//this matrix by the new one, we must set the
	//m_transformation matrix to identity first! (tricky, isn't it?)
	ccGLMatrix oldTrans = m_transformation;
	m_transformation.toIdentity();
	setGLTransformation(oldTrans);
	applyGLTransformation_recursive();
}

bool ccGenericPrimitive::init(unsigned vertCount, bool vertNormals, unsigned faceCounts, unsigned faceNormCounts)
{
	ccPointCloud* verts = vertices();
	assert(verts);
	if (!verts)
		return false;

	/*** clear existing structures ***/

	//clear vertices & normals
	bool keepOtherProps = (vertCount ==  verts->size());
	if (keepOtherProps)
	{
		verts->unallocatePoints();
		verts->unallocateNorms();
	}
	else
	{
		verts->clear();
	}

	//clear triangles indexes
	assert(m_triVertIndexes);
	m_triVertIndexes->clear();

	//clear per triangle normals
	removePerTriangleNormalIndexes();
	if (m_triNormals)
		m_triNormals->clear();
	//DGM: if we do this we'll have issues with the DB tree depending on where when we call this method!
	//{
	//	removeChild(m_triNormals);
	//	setTriNormsTable(0);
	//	assert(!m_triNormals);
	//}

	/*** init necessary structures ***/

	if (vertCount && !verts->reserve(vertCount))
		return false;

	if (vertNormals && !verts->reserveTheNormsTable())
	{
		verts->clear();
		return false;
	}

	if (faceCounts && !reserve(faceCounts))
	{
		verts->clear();
		return false;
	}

	if (faceNormCounts)
	{
		NormsIndexesTableType* normsTable = (m_triNormals ? m_triNormals : new NormsIndexesTableType());
		if (!normsTable || !normsTable->reserveSafe(faceNormCounts) || !reservePerTriangleNormalIndexes())
		{
			verts->clear();
			m_triVertIndexes->clear();
			delete normsTable;
			return false;
		}

		//attach table if not done already
		if (!m_triNormals)
		{
			setTriNormsTable(normsTable);
			assert(m_triNormals);
		}
	}

	return true;
}

ccGenericPrimitive* ccGenericPrimitive::finishCloneJob(ccGenericPrimitive* primitive) const
{
	if (primitive)
	{
		//'clone' vertices (everything but the points that are already here)
		if (primitive->m_associatedCloud && m_associatedCloud && m_associatedCloud->size() == primitive->m_associatedCloud->size())
		{
			primitive->m_associatedCloud = m_associatedCloud->clone(primitive->m_associatedCloud);
			primitive->m_associatedCloud->setName(m_associatedCloud->getName());
		}

		primitive->showNormals(normalsShown());
		primitive->showColors(colorsShown());
		primitive->showSF(sfShown());
		//primitive->showMaterials(materialsShown());
		//primitive->setName(getName()+QString(".clone"));
		primitive->setVisible(isVisible());
		primitive->setEnabled(isEnabled());
		primitive->importParametersFrom(this);
	}
	else
	{
		//if the calling primitive provide a null pointer, it means that the cloned version creation failed!
		ccLog::Warning("[ccGenericPrimitive::clone] Not enough memory!");
	}

	return primitive;
}