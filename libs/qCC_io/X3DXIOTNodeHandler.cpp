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

#ifdef CC_X3D_SUPPORT

#include "X3DXIOTNodeHandler.h"

//qCC_db
#include <ccHObject.h>
#include <ccMesh.h>
#include <ccPointCloud.h>

//XIOT
//#include <xiot/X3DLoader.h>
#include <xiot/X3DAttributes.h>
//#include <xiot/X3DParseException.h>

//System (for test)
#include <iostream>

X3DXIOTNodeHandler::X3DXIOTNodeHandler(ccHObject* root)
	: X3DDefaultNodeHandler()
	, m_root(root)
	, m_currentLeaf(m_root)
{
	assert(root);
}

X3DXIOTNodeHandler::~X3DXIOTNodeHandler()
{
}

int X3DXIOTNodeHandler::startShape(const X3DAttributes &attr)
{
#ifdef _DEBUG
	std::cout << "Start Shape event" << std::endl;
	for (size_t i = 0; i < static_cast<int>(attr.getLength()); i++)
		std::cout << attr.getAttributeName(static_cast<int>(i)) << std::endl;
	if (attr.isDEF())
		std::cout << attr.getDEF() << std::endl;
#endif
	return CONTINUE;
}

int X3DXIOTNodeHandler::endShape()
{
#ifdef _DEBUG
	std::cout << "End Shape event" << std::endl;
#endif
	return CONTINUE;
}

int X3DXIOTNodeHandler::startUnhandled(const char* nodeName, const X3DAttributes& attr)
{
#ifdef _DEBUG
	std::cout << "Unhandled Start Node event: " << nodeName << std::endl;
#endif
	return CONTINUE;
}

int X3DXIOTNodeHandler::endUnhandled(const char* nodeName)
{
#ifdef _DEBUG
	std::cout << "Unhandled End Node event: " << nodeName << std::endl;
#endif
	return CONTINUE;
}

int X3DXIOTNodeHandler::startTransform(const X3DAttributes &attr)
{
#ifdef _DEBUG
	std::cout << "Start Transform" << std::endl;
#endif

	// Get all the X3D tranformation data
	SFVec3f translation;
	SFRotation rotation;

	int index = attr.getAttributeIndex(ID::translation);
	if (index != -1)
	{
		attr.getSFVec3f(index,translation);
	}
	index = attr.getAttributeIndex(ID::rotation);
	if (index != -1)
	{
		attr.getSFRotation(index,rotation);
	}

	assert(m_currentLeaf);
	if (m_currentLeaf)
	{
		ccGLMatrix mat;
		mat.initFromParameters(rotation.angle,
			CCVector3(rotation.x,rotation.y,rotation.z),
			CCVector3(translation.x,translation.y,translation.z));
		m_currentLeaf->setGLTransformation(mat);
	}

	return CONTINUE;
}

int X3DXIOTNodeHandler::startCoordinate(const X3DAttributes &attr)
{
	int index = attr.getAttributeIndex(ID::point);
	if (index == -1)
	{
		//TODO: warn user instead
		//throw std::runtime_error("No points given within Coordinate node");
		return SKIP_CHILDREN;
	}

	MFVec3f points;
	attr.getMFVec3f(index,points);

	unsigned count = points.size();
	if (count == 0)
		return SKIP_CHILDREN;

	ccPointCloud* cloud = new ccPointCloud(attr.isDEF() ? attr.getDEF().c_str() : 0);
	if (!cloud->reserve(count))
	{
		//not enough memory!
		delete cloud;
		return SKIP_CHILDREN;
	}

	for (MFVec3f::const_iterator it=points.begin(); it!=points.end(); ++it)
		cloud->addPoint(CCVector3(it->x,it->y,it->z));

	assert(m_currentLeaf);
	m_currentLeaf->addChild(cloud);

	//cloud = parent vertices?
	if (m_currentLeaf->isKindOf(CC_TYPES::MESH))
	{
		ccMesh* mesh = ccHObjectCaster::ToMesh(m_currentLeaf);
		if (mesh && mesh->getAssociatedCloud() == 0) //FIXME DGM: weird
		{
			cloud->setVisible(false);
		}
	}

	m_currentLeaf = cloud;

	return CONTINUE;
}

int X3DXIOTNodeHandler::endCoordinate()
{
	if (m_currentLeaf && m_currentLeaf->getParent())
	{
		ccHObject* parent = m_currentLeaf->getParent();

		assert(m_currentLeaf->isKindOf(CC_TYPES::POINT_CLOUD));
		m_currentLeaf=parent;
	}
	else
		assert(false);

	return CONTINUE;
}

int X3DXIOTNodeHandler::startNormal(const X3DAttributes &attr)
{
	//if(!_currentGeometry)
	//	throw std::runtime_error("Normal currently only supported for IndexedFaceSets");
	//int index = attr.getAttributeIndex(ID::vector);
	//if (index != -1)
	//	_currentGeometry->setNormals(attr.getMFVec3f(index));
	//else
	//	throw std::runtime_error("No points given within Coordinate node");
	return CONTINUE;
}

int X3DXIOTNodeHandler::startColor(const X3DAttributes &attr)
{
	//if(!_currentGeometry)
	//	throw std::runtime_error("Color currently only supported for IndexedFaceSets");
	//int index = attr.getAttributeIndex(ID::color);
	//if (index != -1)
	//	_currentGeometry->setColors(attr.getMFColor(index));
	//else
	//	throw std::runtime_error("No color given within Color node");
	return CONTINUE;
}

int X3DXIOTNodeHandler::startIndexedFaceSet(const X3DAttributes &attr)
{
	std::cout << "Start IndexedFaceSet" << std::endl;

	ccMesh* mesh = new ccMesh(0);
	unsigned realFaceNumber=0;

	if (attr.isDEF())
		mesh->setName(attr.getDEF().c_str());

	//Vertices indexes
	int vertIndex = attr.getAttributeIndex(ID::coordIndex);
	if (vertIndex != -1)
	{
		MFInt32 streamIndexes;
		attr.getMFInt32(vertIndex,streamIndexes);

		int triIndexes[3];
		int pos=0;
		for (MFInt32::const_iterator it = streamIndexes.begin(); it != streamIndexes.end(); ++it)
		{
			if (*it == -1) //end of polygon
			{
				if (pos<3)
				{
					//incomplete dataset?
					//TODO: warn user
				}
				pos = 0;
				++realFaceNumber;
			}
			else //new vertex index
			{
				if (pos<3)
				{
					triIndexes[pos]=*it;
				}
				else
				{
					//FIXME: simplistic fan triangulation (hum, hum)
					triIndexes[1]=triIndexes[2];
					triIndexes[2]=*it;
				}
				++pos;
			}

			//new face
			if (pos==3)
			{
				//we check that we are at the end of the polygon (we don't handle non triangular meshes yet!)
				if (it+1 == streamIndexes.end() || *(it+1)==-1)
				{
					//we must reserve some more space for storage!
					if (mesh->size() == mesh->maxSize())
					{
						if (!mesh->reserve(mesh->maxSize() + 100))
						{
							delete mesh;
							return ABORT; //not enough memory!
						}
					}
					mesh->addTriangle(triIndexes[0],triIndexes[1],triIndexes[2]);
				}
				else
				{
					//TODO: we don't handle non triangle faces yet!
				}
			}
		}

		//unhandled type of mesh
		if (mesh->size() == 0)
		{
			delete mesh;
			return SKIP_CHILDREN;
		}
		else
		{
			mesh->shrinkToFit();
		}
	}

	//Normals (per face)
	int normIndex = attr.getAttributeIndex(ID::normalIndex);
	if (normIndex != -1)
	{
		//per-triangle normals!
		if (mesh->size() == realFaceNumber)
		{
			if (mesh->reservePerTriangleNormalIndexes())
			{
				MFInt32 perFaceNormIndexes;
				attr.getMFInt32(normIndex,perFaceNormIndexes);
				for (unsigned i=0;i<perFaceNormIndexes.size();++i)
					mesh->addTriangleNormalIndexes(i,i,i);

				mesh->showTriNorms(true);
			}
			else
			{
				//TODO: not enough memory!
			}
		}
		else
		{
			//TODO: we can't load per-face normals with non-triangular meshes yet!
		}
	}

	//Normals (per vertex)
	//	normIndex = attr.getAttributeIndex(ID::normalPerVertex);
	//	bool perVertexNormals=(normIndex != -1 ? attr.getSFBool(normIndex) : true);
	//DGM: in fact we don't care

	assert(m_currentLeaf);
	mesh->setVisible(true);
	m_currentLeaf->addChild(mesh);
	m_currentLeaf = mesh;

	return CONTINUE;
}

int X3DXIOTNodeHandler::endIndexedFaceSet()
{
	if (m_currentLeaf && m_currentLeaf->getParent())
	{
		ccHObject* parent = m_currentLeaf->getParent();

		assert(m_currentLeaf->isA(CC_TYPES::MESH));
		if (m_currentLeaf->isA(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(m_currentLeaf);
			if (!mesh->getAssociatedCloud() || mesh->getAssociatedCloud()->size()==0)
			{
				//TODO: warn that mesh has no vertices!
				parent->removeChild(mesh);
			}
			else
			{
				//in case per-triangle normal loading has failed
				if (mesh->arePerTriangleNormalsEnabled() && !mesh->hasTriNormals())
					mesh->removePerTriangleNormalIndexes();

				//mesh without normals?
				//DGM: normals can be per-vertex or per-triangle so it's better to let the user do it himself later
				//Moreover it's not always good idea if the user doesn't want normals (especially in ccViewer!)
				//if (!mesh->hasNormals())
				//	mesh->computeNormals();
				mesh->showNormals(mesh->hasNormals());
			}
		}
		m_currentLeaf = parent;
	}
	else
		assert(false);

	return CONTINUE;
}

//int X3DXIOTNodeHandler::startSphere(const X3DAttributes &attr)
//{
//	std::cout << "Start Sphere\n";
//
//	int index = attr.getAttributeIndex(ID::radius);
//	float radius = index == -1 ? 1.0 : attr.getSFFloat(index);
//
//	const string name = createUniqueName(attr, "sphere");
//	GeomUtils::createSphere(name , radius, 20, 20, true, false);
//
//	_currentMesh = MeshManager::getSingleton().getByName(name);
//	_currentEntity = _sceneManager->createEntity(_shapeName, name);
//	return CONTINUE;
//}
//
//int X3DXIOTNodeHandler::startBox(const X3DAttributes &attr) {
//	std::cout << "Start Box\n";
//	_currentEntity = _sceneManager->createEntity(createUniqueName(attr, "shape"), "cube.mesh");
//
//	return CONTINUE;
//}
//
//int X3DXIOTNodeHandler::startMaterial(const X3DAttributes &attr) {
//	std::cout << "Start Material" << std::endl; 
//	if (!_currentMaterial.isNull())
//	{
//		Pass* pass = _currentMaterial->getTechnique(0)->getPass(0);
//		int index = attr.getAttributeIndex(ID::ambientIntensity);
//		float ambientIntensity = (index == -1) ? 0.2f : attr.getSFFloat(index);
//		index = attr.getAttributeIndex(ID::transparency);
//		float transparency = (index == -1) ? 0.0f : attr.getSFFloat(index);
//		
//		SFColor diffuseColor;
//		index = attr.getAttributeIndex(ID::diffuseColor);
//		if (index != -1)
//		{
//			diffuseColor = attr.getSFColor(index);
//		}
//		else 
//		{
//			diffuseColor.r = diffuseColor.g = diffuseColor.b = 0.8;
//		}
//
//		SFColor specularColor;
//		index = attr.getAttributeIndex(ID::specularColor);
//		if (index != -1)
//		{
//			specularColor = attr.getSFColor(index);
//		}
//
//			SFColor emissiveColor;
//		index = attr.getAttributeIndex(ID::emissiveColor);
//		if (index != -1)
//		{
//			emissiveColor = attr.getSFColor(index);
//		}
//		
//		index = attr.getAttributeIndex(ID::shininess);
//		float shininess = (index == -1) ? 0.2f : attr.getSFFloat(index);
//		shininess = Math::Clamp(shininess * 128.0f, 0.0f, 128.0f);
//
//	pass->setAmbient(	ambientIntensity * diffuseColor.r,
//						ambientIntensity * diffuseColor.g,
//						ambientIntensity * diffuseColor.b);
//	pass->setDiffuse(	diffuseColor.r,
//						diffuseColor.g,
//						diffuseColor.b,
//						1.0f - transparency);
//	pass->setSpecular(specularColor.r,
//						specularColor.g,
//						specularColor.b,
//						1.0f - transparency);
//
//	pass->setSelfIllumination(emissiveColor.r,
//								emissiveColor.g,
//								emissiveColor.b);
//
//	ass->setShininess( shininess );
//	pass->setLightingEnabled(true);
//	}
//
//	return CONTINUE;
//}
//

//int X3DXIOTNodeHandler::startIndexedLineSet(const X3DAttributes &attr) {
//	
//	_currentGeometry = new IndexedGeometry(createUniqueName(attr, "indexedLineSet"));
//
//	int index = attr.getAttributeIndex(ID::coordIndex);
//	if (index != -1)
//		_currentGeometry->setCoordIndex(attr.getMFInt32(index));
//
//	index = attr.getAttributeIndex(ID::colorPerVertex);
//	_currentGeometry->setColorPerVertex(index != -1 ? attr.getSFBool(index) : true);
//
//	return CONTINUE;
//}
//
//int X3DXIOTNodeHandler::endIndexedLineSet() {
//	std::cout << "End IndexedLineSet" << std::endl;
// 
//	_currentManualObject = _sceneManager->createManualObject(_shapeName);
//	_currentGeometry->createIndexedLineSet(_currentManualObject);
//	delete _currentGeometry;
//	_currentGeometry = NULL;
//
//	return CONTINUE;
//}

#endif

