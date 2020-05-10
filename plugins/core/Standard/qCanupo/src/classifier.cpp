//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

//This file is directly inspired of the equivalently named file in the
//original CANUPO project, by N. Brodu and D. Lague.

#include "classifier.h"

//Qt
#include <QFile>

//system
#include <assert.h>
#include <limits.h>

Classifier::Classifier()
	: class1(0)
	, class2(0)
	, absMaxXY(0)
	, axisScaleRatio(1.0f)
	, descriptorID(DESC_INVALID)
	, dimPerScale(0)
{}

bool Classifier::checkRefPoints()
{
	// dot product with (+1,+1) vector gives the classification sign
	if (refPointPos.x + refPointPos.y < 0)
	{
		std::swap(refPointPos, refPointNeg);
	}

	return (refPointPos.x + refPointPos.y >= 0);
}

float Classifier::classify2D_checkcondnum(const Point2D& P, const Point2D& R, float& condnumber) const
{
	condnumber = 0;
	if (path.size() < 2)
	{
		assert(false);
		return 0;
	}

	// consider each path segment as a mini-classifier
	// the segment PR[Pt-->Refpt] and each path's segment cross
	// iff each one classifies the end point of the other in different classes
	Point2D PR = R-P;
	Point2D u = PR; u.normalize();

	unsigned numcross = 0;

	//we'll also look for the distance between P and the nearest segment
	float closestSquareDist = -1.0f;

	size_t segCount = path.size() - 1;
	for (size_t i=0; i<segCount; ++i)
	{
		//current path segment (or half-line!)
		Point2D AP = P - path[i];
		Point2D AB = path[i+1] - path[i];
		Point2D v = AB; v.normalize();

		condnumber = std::max<float>(condnumber, fabs(v.dot(u)));

		// Compute whether PR[Pt-->Refpt] and that segment cross
		float denom = (u.x*v.y-v.x*u.y);
		if (denom != 0)
		{
			// 1. check whether the given pt and the refpt are on different sides of the classifier line
			//we search for alpha and beta so that
			// P + alpha * PR = A + beta * AB
			float alpha = (AP.y * v.x - AP.x * v.y)/denom;
			bool pathIntersects = (alpha >= 0 && alpha*alpha <= PR.norm2());
			if (pathIntersects)
			{
				float beta = (AP.y * u.x - AP.x * u.y)/denom;

				// first and last lines are projected to infinity
				bool refSegIntersects = ((i == 0 || beta >= 0) && (i+1 == segCount || beta*beta < AB.norm2())); //not "beta*beta <= AB.norm2()" because the equality case will be managed by the next segment!

				// crossing iif each segment/line separates the other
				if (refSegIntersects)
					numcross++;
			}
		}

		// closest distance from the point to that segment
		// 1. projection of the point of the line
		float squareDistToSeg = 0;
		float distAH = v.dot(AP);
		if ((i == 0 || distAH >= 0.0) && (i+1 == segCount || distAH <= AB.norm()))
		{
			// 2. Is the projection within the segment limit? yes => closest
			Point2D PH = (path[i] + v * distAH) - P;
			squareDistToSeg = PH.norm2();
		}
		else
		{
			// 3. otherwise closest is the minimum of the distance to the segment ends
			Point2D BP = P - path[i+1];
			squareDistToSeg = std::min( AP.norm2(), BP.norm2() );
		}

		if (closestSquareDist < 0 || squareDistToSeg < closestSquareDist)
		{
			closestSquareDist = squareDistToSeg;
		}
	}

	assert(closestSquareDist >= 0);
	float deltaNorm = sqrt(closestSquareDist);

	return ((numcross & 1) == 0 ? deltaNorm : -deltaNorm);
}

float Classifier::classify2D(const Point2D& P) const
{
	float condpos = 0.0f;
	float condneg = 0.0f;
	float predpos = classify2D_checkcondnum(P,refPointPos,condpos);
	float predneg = classify2D_checkcondnum(P,refPointNeg,condneg);

	// normal nearly aligned = bad conditionning, the lower the dot prod the better
	return condpos < condneg ? predpos : -predneg;
}

Classifier::Point2D Classifier::project(const CorePointDesc& mscdata) const
{
	assert(weightsAxis1.size() == weightsAxis2.size());
	assert(weightsAxis1.size() > 1);

	//There may be less weights than parameters in the descriptor
	//if we use a descriptor computed with more (bigger) scales.
	//In this case we assume the matching scales are all at the end!
	//(i.e. the smallest)
	size_t weightCount = weightsAxis1.size()-1;
	size_t paramCount = mscdata.params.size();
	assert(weightCount <= paramCount);
	unsigned shift = static_cast<unsigned>(paramCount - weightCount);

	Point2D P( weightsAxis1.back(), weightsAxis2.back() );

	for (size_t i=0; i<weightCount; ++i)
	{
		P.x += weightsAxis1[i] * mscdata.params[shift+i];
		P.y += weightsAxis2[i] * mscdata.params[shift+i];
	}

	return P;
}

float Classifier::classify(const CorePointDesc& mscdata) const
{
	Point2D P = project(mscdata);
	return classify2D(P);
}

bool Classifier::Load(	QString filename,
						std::vector<Classifier>& classifiers,
						std::vector<float>& scales,
						QString& error,
						FileHeader* header/*=0*/,
						bool headerOnly/*=false*/)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		error = QString("Failed to open input classifier file!");
		return false;
	}

	scales.clear();
	classifiers.clear();

	//DGM: sadly we can't use a stream as data in prm files are saved in a strange way
	//QDataStream stream(&file);
	//--> changing the stream 'byte order' doesn't help :(
	//stream.setByteOrder(QDataStream::LittleEndian);

	// scales count
	unsigned nscales;
	file.read(reinterpret_cast<char*>(&nscales), sizeof(unsigned));

	unsigned dimPerScale = 2;
	unsigned descriptorID = 1;

	if (nscales == 9999)
	{
		//we are loading a new 'prm' file (generated by qCanupo)
		//descriptor ID
		file.read(reinterpret_cast<char*>(&descriptorID), sizeof(unsigned));
		//dimension per scale
		file.read(reinterpret_cast<char*>(&dimPerScale), sizeof(unsigned));
		//and now the real number of scales
		file.read(reinterpret_cast<char*>(&nscales), sizeof(unsigned));
	}

	// values count
	try
	{
		scales.resize(nscales);
	}
	catch (const std::bad_alloc&)
	{
		error = QString("Not enough memory!");
		return false;
	}

	// read scale values
	{
		for (unsigned s=0; s<nscales; ++s)
			file.read(reinterpret_cast<char*>(&scales[s]), sizeof(float));
	}

	// number of classifiers
	unsigned nclassifiers;
	file.read(reinterpret_cast<char*>(&nclassifiers), sizeof(unsigned));

	if (header)
	{
		header->classifierCount = nclassifiers;
		header->dimPerScale = dimPerScale;
		header->descID = descriptorID;
	}
	
	if (headerOnly)
	{
		//we can stop here
		return true;
	}

	// reserve classifiers array
	try
	{
		classifiers.resize(nclassifiers);
	}
	catch (const std::bad_alloc&)
	{
		error = QString("Not enough memory!");
		return false;
	}

	// read classifiers
	try
	{
		const unsigned fdim = nscales*dimPerScale;
		for (unsigned ci=0; ci<nclassifiers; ++ci)
		{
			Classifier& classifier = classifiers[ci];

			classifier.dimPerScale = dimPerScale;
			classifier.descriptorID = descriptorID;

			classifier.scales = scales; //all classifiers inside a file have the same scales!
			file.read(reinterpret_cast<char*>(&classifier.class1), sizeof(int));
			file.read(reinterpret_cast<char*>(&classifier.class2), sizeof(int));

			classifier.weightsAxis1.resize(fdim+1);
			{
				for (unsigned i=0; i<=fdim; ++i)
					file.read(reinterpret_cast<char*>(&classifier.weightsAxis1[i]), sizeof(float));
			}
			classifier.weightsAxis2.resize(fdim+1);
			{
				for (unsigned i=0; i<=fdim; ++i)
					file.read(reinterpret_cast<char*>(&classifier.weightsAxis2[i]), sizeof(float));
			}

			unsigned pathsize;
			file.read(reinterpret_cast<char*>(&pathsize), sizeof(unsigned));
			classifier.path.resize(pathsize);
			{
				for (unsigned i=0; i<pathsize; ++i)
				{
					file.read(reinterpret_cast<char*>(&classifier.path[i].x), sizeof(float));
					file.read(reinterpret_cast<char*>(&classifier.path[i].y), sizeof(float));
				}
			}

			file.read(reinterpret_cast<char*>(&classifier.refPointPos.x),	sizeof(float));
			file.read(reinterpret_cast<char*>(&classifier.refPointPos.y),	sizeof(float));
			file.read(reinterpret_cast<char*>(&classifier.refPointNeg.x),	sizeof(float));
			file.read(reinterpret_cast<char*>(&classifier.refPointNeg.y),	sizeof(float));
			file.read(reinterpret_cast<char*>(&classifier.absMaxXY),		sizeof(float));
			file.read(reinterpret_cast<char*>(&classifier.axisScaleRatio),	sizeof(float));

			if (!classifier.checkRefPoints())
			{
				//DGM: strange test, that fails for valid classifiers!
				//error = QString("Invalid reference points in the classifier");
				//return false;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		error = QString("Not enough memory!");
		return false;
	}

	return true;
}

bool Classifier::save(	QString filename,
						QString& error)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly))
	{
		error = QString("Failed to open output file!");
		return false;
	}

	//DGM: sadly we can't use a stream as data in prm files are saved in a strange way
	//see Classifier::Load

	if (descriptorID != DESC_DIMENSIONALITY || dimPerScale != 2)
	{
		//we code the 'new' PRM files with 9999 in place of the number of scales!
		const unsigned headerCode = 9999;
		file.write(reinterpret_cast<const char*>(&headerCode), sizeof(unsigned));

		//descriptor ID
		file.write(reinterpret_cast<const char*>(&descriptorID), sizeof(unsigned));
		//dimension per scale
		file.write(reinterpret_cast<const char*>(&dimPerScale), sizeof(unsigned));
	}

	// number of scales
	unsigned nscales = static_cast<unsigned>(scales.size());
	file.write(reinterpret_cast<const char*>(&nscales), sizeof(unsigned));

	// write scale values
	{
		for (unsigned s=0; s<nscales; ++s)
			file.write(reinterpret_cast<const char*>(&scales[s]), sizeof(float));
	}

	// number of 2-class classifiers = 1!
	unsigned nclassifiers = 1;
	file.write(reinterpret_cast<const char*>(&nclassifiers), sizeof(unsigned));

	// write classifier
	const unsigned fdim = nscales*dimPerScale;
	file.write(reinterpret_cast<const char*>(&class1), sizeof(int));
	file.write(reinterpret_cast<const char*>(&class2), sizeof(int));

	//weightsAxis1
	{
		for (unsigned i=0; i<=fdim; ++i)
			file.write(reinterpret_cast<const char*>(&weightsAxis1[i]), sizeof(float));
	}
	//weightsAxis2
	{
		for (unsigned i=0; i<=fdim; ++i)
			file.write(reinterpret_cast<const char*>(&weightsAxis2[i]), sizeof(float));
	}

	unsigned pathsize = static_cast<unsigned>(path.size());
	file.write(reinterpret_cast<const char*>(&pathsize), sizeof(unsigned));
	{
		for (unsigned i=0; i<pathsize; ++i)
		{
			file.write(reinterpret_cast<const char*>(&path[i].x), sizeof(float));
			file.write(reinterpret_cast<const char*>(&path[i].y), sizeof(float));
		}
	}

	file.write(reinterpret_cast<const char*>(&refPointPos.x),	sizeof(float));
	file.write(reinterpret_cast<const char*>(&refPointPos.y),	sizeof(float));
	file.write(reinterpret_cast<const char*>(&refPointNeg.x),	sizeof(float));
	file.write(reinterpret_cast<const char*>(&refPointNeg.y),	sizeof(float));
	file.write(reinterpret_cast<const char*>(&absMaxXY),		sizeof(float));
	file.write(reinterpret_cast<const char*>(&axisScaleRatio),	sizeof(float));

	file.close();

	return true;
}
