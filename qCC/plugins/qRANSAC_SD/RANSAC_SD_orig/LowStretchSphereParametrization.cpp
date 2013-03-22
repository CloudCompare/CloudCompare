#include "LowStretchSphereParametrization.h"
#include "Bitmap.h"

#if !defined(_WIN32) && !defined(WIN32)
#include <unistd.h>
#endif

LowStretchSphereParametrization::LowStretchSphereParametrization(
	const Sphere &sphere)
: m_sphere(&sphere)
{
	m_frame.Canonical();
}

void LowStretchSphereParametrization::Shape(const Sphere &sphere)
{
	m_sphere = &sphere;
	m_frame.Canonical();
}

void LowStretchSphereParametrization::WrapComponents(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, size_t uextent, size_t vextent,
	MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	// wraps are necessary only in v direction
	// relabel the components
	MiscLib::Vector< std::pair< int, size_t > > tempLabels(*labels);
	// wrap along v
	float vstartPrev, vendPrev, vstart = 0, vend = 0, vstartNext = 0, vendNext = 0;
	size_t vsPrev, vePrev, vs = 0, ve = 0, vsNext = 0, veNext = 0;
	float uangle = (bbox.Min()[0] + .5f * epsilon) / m_sphere->Radius();
	float radius = std::sin(uangle) * m_sphere->Radius();
	vstartNext = float(-M_PI) * radius;
	vendNext = float(M_PI) * radius;
	vsNext =  std::min(vextent - 1, (size_t)std::max((intptr_t)0,
		(intptr_t)((vstartNext - bbox.Min()[1]) / epsilon)));
	veNext = (size_t)std::max((intptr_t)0, std::min((intptr_t)vextent - 1,
		(intptr_t)((vendNext - bbox.Min()[1]) / epsilon)));
	for(size_t u = 0; u < uextent; ++u)
	{
		vstartPrev = vstart;		vstart = vstartNext;
		vendPrev = vend;			vend = vendNext;
		vsPrev = vs;				vs = vsNext;
		vePrev = ve;				ve = veNext;
		// determine starting position in the next column
		if(u < uextent - 1)
		{
			float uangleNext = ((u + 1.5f) * epsilon + bbox.Min()[0]) / m_sphere->Radius();
			float radiusNext = std::sin(uangle) * m_sphere->Radius();
			vstartNext = float(-M_PI) * radius;
			vendNext = float(M_PI) * radius;
			vsNext =  std::min(vextent - 1, (size_t)std::max((intptr_t)0,
				(intptr_t)((vstartNext - bbox.Min()[1]) / epsilon)));
			veNext = (size_t)std::max((intptr_t)0, std::min((intptr_t)vextent - 1,
				(intptr_t)((vendNext - bbox.Min()[1]) / epsilon)));
		}
		if(vstart <= bbox.Min()[1] - epsilon
			|| vend >= bbox.Max()[1] + epsilon
			|| !(*componentImg)[vs * uextent + u])
			continue;
		// check the three neighbors on the other side
		if((*componentImg)[ve * uextent + u])
			AssociateLabel((*componentImg)[vs * uextent + u],
				(*componentImg)[ve * uextent + u], &tempLabels);
		if(u > 0
			&& vstartPrev > bbox.Min()[1] - epsilon
			&& vendPrev < bbox.Min()[1] + epsilon
			&& (*componentImg)[vePrev * uextent + u - 1])
			AssociateLabel((*componentImg)[vs * uextent + u],
				(*componentImg)[vePrev * uextent + u - 1], &tempLabels);
		if(u < uextent - 1
			&& vstartNext > bbox.Min()[1] - epsilon
			&& vendNext < bbox.Min()[1] + epsilon
			&& (*componentImg)[veNext * uextent + u + 1])
			AssociateLabel((*componentImg)[vs * uextent + u],
				(*componentImg)[veNext * uextent + u + 1], &tempLabels);
	}

	// condense labels
	for(size_t i = tempLabels.size() - 1; i > 0; --i)
		tempLabels[i].first = ReduceLabel(i, tempLabels);
	MiscLib::Vector< int > condensed(tempLabels.size());
	labels->clear();
	labels->reserve(condensed.size());
	int count = 0;
    for(size_t i = 0; i < tempLabels.size(); ++i)
		if(i == tempLabels[i].first)
		{
			labels->push_back(std::make_pair(count, tempLabels[i].second));
			condensed[i] = count;
			++count;
		}
		else
			(*labels)[condensed[tempLabels[i].first]].second
				+= tempLabels[i].second;
	// set new component ids
	for(size_t i = 0; i < componentImg->size(); ++i)
		(*componentImg)[i] =
			condensed[tempLabels[(*componentImg)[i]].first];
}

size_t LowStretchSphereParametrization::SerializedSize()
{
	return sizeof(GfxTL::Frame< 3, float >::PointType) + sizeof(float);
}

void LowStretchSphereParametrization::Serialize(std::ostream *o, bool binary) const
{
	// defer rotation from frame
	GfxTL::Frame< 3, float > nframe;
	nframe.FromNormal(m_frame[2]);
	GfxTL::VectorXD< 2, float > t;
	nframe.ToTangent(m_frame[0], &t);
	for(unsigned int i = 0; i < 2; ++i)
		t[i] = GfxTL::Math< float >::Clamp(t[i], -1, 1);
	float angle = std::atan2(t[1], t[0]);
	// write normal and dummy rotation
	if(binary)
	{
		o->write((char *)&m_frame[2],
			sizeof(GfxTL::Frame< 3, float >::PointType));
		o->write((char *)&angle, sizeof(angle));
	}
	else
	{
		for(unsigned int i = 0; i < 3; ++i)
			(*o) << m_frame[2][i] << " ";
		(*o) << angle << " ";
	}
}

void LowStretchSphereParametrization::Deserialize(std::istream *i, bool binary)
{
	float rot;
	GfxTL::Frame< 3, float >::PointType normal;
	if(binary)
	{
		i->read((char *)&normal, sizeof(normal));
		i->read((char *)&rot, sizeof(rot));
	}
	else
	{
		for(unsigned int j = 0; j < 3; ++j)
			(*i) >> normal[j];
		(*i) >> rot;
	}
	m_frame.FromNormal(normal);
	m_frame.RotateOnNormal(rot);
}
