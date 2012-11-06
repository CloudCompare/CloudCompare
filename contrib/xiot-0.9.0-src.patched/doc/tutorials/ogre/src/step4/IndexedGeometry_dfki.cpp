#include "IndexedGeometry.h"
#include "Ogre.h"

using namespace Ogre;
using namespace XIOT;

#include <cassert>

	struct face {
	std::vector<int> indices;
	};

	// OGRE face
struct triangle {
	int vertices[3];
};
struct vertex {
	int pos, tc, normal, colour;

	bool operator<(const vertex &v) const {
		if (pos < v.pos) return true;
		if (pos > v.pos) return false;
		if (normal < v.normal) return true;
		if (normal > v.normal) return false;
		if (tc < v.tc) return true;
		if (tc > v.tc) return false;
		return colour < v.colour;
	}
};

int getIndex(const std::vector<int> &coordIndex, const std::vector<int> &vec, bool perVertex, int facenr, int index)
{
	if (!perVertex) {
		if (!vec.empty())
			return vec[facenr];
		else
			return facenr;
	} else {
		if (!vec.empty())
			return vec[index];
		else
			return coordIndex[index];
	}
}




typedef std::map<vertex, int> VertMap;

IndexedGeometry::IndexedGeometry(const std::string &name) : _name(name)
{
	this->_colorPerVertex = true;
	this->_normalPerVertex = true;
	this->_ccw = true;
}

IndexedGeometry::~IndexedGeometry()
{
	
}

	// Flags
void IndexedGeometry::setNormalPerVertex(bool normalPerVertex) { _normalPerVertex = normalPerVertex; }
void IndexedGeometry::setColorPerVertex(bool colorPerVertex){ _colorPerVertex = colorPerVertex; }

// Indices
void IndexedGeometry::setCoordIndex(const std::vector<int> &coordIndex){ _coordIndex = coordIndex; }
void IndexedGeometry::setNormalIndex(const std::vector<int> &normalIndex){ _normalIndex = normalIndex; }
void IndexedGeometry::setColorIndex(const std::vector<int> &colorIndex){ _colorIndex = colorIndex; }
void IndexedGeometry::setTexCoordIndex(const std::vector<int> &texCoordIndex){ _texCoordIndex = texCoordIndex; }

// Data
void IndexedGeometry::setCoords(const std::vector<SFVec3f> &coords){ _coords = coords; }
void IndexedGeometry::setNormals(const std::vector<SFVec3f> &normals){ _normals = normals; }
void IndexedGeometry::setTexCoords(const std::vector<SFVec2f> &texCoords){ _texCoords = texCoords; }
void IndexedGeometry::setColors(const std::vector<SFColor> &colors) { _colors = colors; }



// Generate normals for polygon meshes
void IndexedGeometry::createIndexedFaceSet()
{
	if (_coords.empty())
		throw std::runtime_error("No coordinates given.");

	if (_coordIndex.empty())
		throw std::runtime_error("No coordinate index given.");

	MeshPtr mesh = MeshManager::getSingleton().createManual(_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *sub = mesh->createSubMesh();

	bool hasTextureCoordinates = !_texCoords.empty();
	bool hasPointColors = !_colors.empty() && _colorPerVertex;
	bool hasPointNormals = !_normals.empty() && _normalPerVertex;
	bool hasCellColors = !_colors.empty() && !hasPointColors;
	bool hasCellNormals = !_normals.empty() && !hasPointNormals;
	bool calcPointNormals = _normals.empty() && _normalPerVertex;

	std::vector<face> faces;
	face f;
	for (std::vector<int>::const_iterator I=_coordIndex.begin(); I !=_coordIndex.end(); ++I) {
		if (*I == -1) {
			faces.resize(faces.size()+1);
			faces.back().indices.swap(f.indices);
		} else
			f.indices.push_back(I - _coordIndex.begin());
	}
	if (!f.indices.empty()) {
		faces.resize(faces.size()+1);
		faces.back().indices.swap(f.indices);
	}


	std::vector<vertex> vertices;
	std::vector<triangle> triangles;


	VertMap vertexMap;

	// triangulate and expand vertices
	for (std::vector<face>::const_iterator f=faces.begin(), e=faces.end(); f!=e; ++f) {
		int faceNr = f - faces.begin();
		int triVertNr = 0;
		int triVerts[2] = { -1, -1 };
		for (std::vector<int>::const_iterator i = f->indices.begin(), e=f->indices.end(); i!=e; ++i, ++triVertNr) {
			int triVertNr = i - f->indices.begin();
			int index = *i;

			vertex vert;

			// get full indices for vertex data
			vert.pos = _coordIndex[index];
			vert.normal = !_normals.empty() ? getIndex(_coordIndex, _normalIndex,
				_normalPerVertex, faceNr, index) : 0;
			vert.colour = !_colors.empty() ? getIndex(_coordIndex, _colorIndex,
				_colorPerVertex, faceNr, index) : 0;
			vert.tc = hasTextureCoordinates ? getIndex(_coordIndex, _texCoordIndex,
				true, faceNr, index) : 0;

			// avoid duplication
			//int nvert = vertexMap.size();
			//int &vpos = vertexMap[vert];
			//if (nvert != vertexMap.size()) {
				int vpos = vertices.size();
				vertices.push_back(vert);
			//}

			// emit triangle (maybe)
			if (triVertNr == 0)
				triVerts[0] = vpos;
			else if (triVertNr == 1)
				triVerts[1] = vpos;
			else {
				triangle t;
				t.vertices[0] = triVerts[0];
				t.vertices[1] = triVerts[1];
				t.vertices[2] = vpos;

				if (!_ccw)
					std::swap(t.vertices[1], t.vertices[2]);

				triangles.push_back(t);

				triVerts[1] = vpos;
			}
		}
	}
	
	// createOgreMesh
	int nvertices = vertices.size();
	int nfaces = triangles.size();

	VertexData* vertexData = new VertexData();
	sub->vertexData = vertexData;

	IndexData* indexData = sub->indexData;
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	if (hasPointNormals)
	{
		// normals
		vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
		currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	}
	// two dimensional texture coordinates
	if (hasTextureCoordinates)
	{
		vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
		currOffset += VertexElement::getTypeSize(VET_FLOAT2);
	}

	std::cout << std::endl;

	// allocate index buffer
	indexData->indexCount = nfaces * 3;
	indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	std::cout << triangles.size() << ": ";
	for(std::vector<triangle>::const_iterator T = triangles.begin(); T != triangles.end(); T++)
	{
		const triangle &t = (*T);
		*pIndices++ = t.vertices[0];
		*pIndices++ = t.vertices[1];
		*pIndices++ = t.vertices[2];
		std::cout << t.vertices[0] << " " << t.vertices[1] << " "  << t.vertices[2] << " ";
	}
	std::cout << std::endl;

	// allocate the vertex buffer
	vertexData->vertexCount = nvertices;
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	AxisAlignedBox aabox;
	std::cout << vertices.size() << ": ";
	for(std::vector<vertex>::const_iterator V = vertices.begin(); V != vertices.end(); V++)
	{

		const vertex &v = (*V);
		SFVec3f pos = _coords.at(v.pos);
		*pVertex++ = pos.x;
		*pVertex++ = pos.y;
		*pVertex++ = pos.z;
		aabox.merge(Vector3(pos.x, pos.y, pos.z));
		std::cout << pos.x << " " << pos.y << " "  << pos.z << " " << std::endl;
		//std::cout << v.pos << " ";
		if (hasPointNormals)
		{
			const SFVec3f normal = _normals.at(v.normal);
			*pVertex++ = normal.x;
			*pVertex++ = normal.y;
			*pVertex++ = normal.z;
		}
	}

std::cout << std::endl;
	// Unlock
	vBuf->unlock();
	iBuf->unlock();

	sub->useSharedVertices = false;

	// the original code was missing this line:
	mesh->_setBounds(aabox);
	mesh->_setBoundingSphereRadius((aabox.getMaximum()-aabox.getMinimum()).length()/2.0);
	// this line makes clear the mesh is loaded (avoids memory leaks)
	mesh->load();

}

void IndexedGeometry::createIndexedLineSet(Ogre::ManualObject* manualObject)
{
	if (_coords.empty())
		throw std::runtime_error("No coordinates given.");

	if (_coordIndex.empty())
		throw std::runtime_error("No coordinate index given.");

	MaterialPtr myManualObjectMaterial = MaterialManager::getSingleton().create("_X3DLINEDEFAULT","X3DRENDERER"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(false); 

	bool hasColors = !_colors.empty();
	bool hasVertexColors = hasColors && _colorPerVertex;

	bool inStrip = false;
	int stripCount = 0;
	for(std::vector<int>::const_iterator I = _coordIndex.begin();
		I != _coordIndex.end();
		I++)
	{
		if (!inStrip)
		{
			manualObject->begin("_X3DLINEDEFAULT", Ogre::RenderOperation::OT_LINE_STRIP);
		}
		if (*I == -1)
		{
			manualObject->end();
			inStrip = false;
			stripCount++;
		} else {
			SFVec3f pos = _coords.at(*I);
			manualObject->position(pos.x, pos.y, pos.z);
			if (hasColors)
			{
				if (hasVertexColors)
				{
					SFColor colour = _colorIndex.empty() ? _colors.at(*I) : _colors.at(_colorIndex.at(*I));
					manualObject->colour(colour.r, colour.g, colour.b);
				}
				else
				{
					SFColor colour = _colorIndex.empty() ? _colors.at(stripCount) : _colors.at(_colorIndex.at(stripCount));
					manualObject->colour(colour.r, colour.g, colour.b);
				}
			}
			inStrip = true;
		}
	}
	
	
}

