#include "LowStretchTorusParametrization.h"
#include "Bitmap.h"

#if !defined(_WIN32) && !defined(WIN32)
#include <unistd.h>
#endif

LowStretchTorusParametrization::LowStretchTorusParametrization(
	const Torus &torus)
: m_torus(&torus)
{
	m_hcs.FromNormal(m_torus->AxisDirection());
	m_minorFrame.Canonical();
}

void LowStretchTorusParametrization::Shape(const Torus &torus)
{
	m_torus = &torus;
	m_hcs.FromNormal(m_torus->AxisDirection());
	m_minorFrame.Canonical();
}

void LowStretchTorusParametrization::WrapComponents(
	const GfxTL::AABox< GfxTL::Vector2Df > &bbox, float epsilon,
	size_t uextent, size_t vextent, MiscLib::Vector< int > *componentImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels) const
{
	// there are wraps along both u and v direction
	// the first pixel of each column/row is copied to the last element
	// and components are merged accordingly

	// relabel the components
	MiscLib::Vector< std::pair< int, size_t > > tempLabels(*labels);

	// first wrap along v (minor radius)
	// but only if necessary
	if(bbox.Max()[1] - bbox.Min()[1]
		>= 2 * M_PI * m_torus->MinorRadius() - 2 * epsilon)
	{
		for(size_t u = 0; u < uextent; ++u)
		{
			if(!(*componentImg)[u])
				continue;
			// check the three neighbors on the other side
			if((*componentImg)[(vextent - 1) * uextent + u])
				AssociateLabel((*componentImg)[u],
					(*componentImg)[(vextent - 1) * uextent + u], &tempLabels);
			if(u > 0 && u < uextent - 1 && (*componentImg)[(vextent - 1) * uextent + u + 1])
				AssociateLabel((*componentImg)[u],
					(*componentImg)[(vextent - 1) * uextent + u + 1], &tempLabels);
			if(u > 0 && (*componentImg)[(vextent - 1) * uextent + u - 1])
				AssociateLabel((*componentImg)[u],
					(*componentImg)[(vextent - 1) * uextent + u - 1], &tempLabels);
		}
	}

	// wrap along u (major radius)
	float minorRot = MinorFrameRotation(); // get the rotation of the minor frame
	float ustartPrev, uendPrev, ustart = 0, uend = 0, ustartNext = 0, uendNext = 0;
	size_t usPrev, uePrev, us = 0, ue = 0, usNext = 0, ueNext = 0;
	float vangle = (bbox.Min()[1] + .5f * epsilon) / m_torus->MinorRadius();
	float majorRadius = m_torus->MajorRadius()
		+ std::cos(vangle + minorRot) * m_torus->MinorRadius();
	ustartNext = float(-M_PI) * majorRadius;
	uendNext = float(M_PI) * majorRadius;
	usNext =  std::min(uextent - 1, (size_t)std::max((intptr_t)0,
		(intptr_t)((ustartNext - bbox.Min()[0]) / epsilon)));
	ueNext = (size_t)std::max((intptr_t)0, std::min((intptr_t)uextent - 1,
		(intptr_t)((uendNext - bbox.Min()[0]) / epsilon)));
	for(size_t v = 0; v < vextent; ++v)
	{
		ustartPrev = ustart;		ustart = ustartNext;
		uendPrev = uend;			uend = uendNext;
		usPrev = us;				us = usNext;
		uePrev = ue;				ue = ueNext;
		// determine starting position in the next column
		if(v < vextent - 1)
		{
			float vangle = ((v + 1.5f) * epsilon + bbox.Min()[1]) / m_torus->MinorRadius();
			float majorRadius = m_torus->MajorRadius()
				+ std::cos(vangle + minorRot) * m_torus->MinorRadius();
			ustartNext = float(-M_PI) * majorRadius;
			uendNext = float(M_PI) * majorRadius;
			usNext =  std::min(uextent - 1, (size_t)std::max((intptr_t)0,
				(intptr_t)((ustartNext - bbox.Min()[0]) / epsilon)));
			ueNext = (size_t)std::max((intptr_t)0, std::min((intptr_t)uextent - 1,
				(intptr_t)((uendNext - bbox.Min()[0]) / epsilon)));
		}
		if(ustart <= bbox.Min()[0] - epsilon
			|| uend >= bbox.Max()[0] + epsilon
			|| !(*componentImg)[v * uextent + us])
			continue;
		// check the three neighbors on the other side
		if((*componentImg)[v * uextent + ue])
			AssociateLabel((*componentImg)[v * uextent + us],
				(*componentImg)[v * uextent + ue], &tempLabels);
		if(v > 0
			&& ustartPrev > bbox.Min()[0] - epsilon
			&& uendPrev < bbox.Min()[0] + epsilon
			&& (*componentImg)[(v - 1) * uextent + uePrev])
			AssociateLabel((*componentImg)[v * uextent + us],
				(*componentImg)[(v - 1) * uextent + uePrev], &tempLabels);
		if(v < vextent - 1
			&& ustartNext > bbox.Min()[0] - epsilon
			&& uendNext < bbox.Min()[0] + epsilon
			&& (*componentImg)[(v + 1) * uextent + ueNext])
			AssociateLabel((*componentImg)[v * uextent + us],
				(*componentImg)[(v + 1) * uextent + ueNext], &tempLabels);
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

size_t LowStretchTorusParametrization::SerializedSize()
{
	return 2 * sizeof(float);
}

void LowStretchTorusParametrization::Serialize(std::ostream *o, bool binary) const
{
	float rot = 0;
	if(binary)
	{
		rot = MajorFrameRotation();
		o->write((char *)&rot, sizeof(rot));
		rot = MinorFrameRotation();
		o->write((char *)&rot, sizeof(rot));
	}
	else
		*o << MajorFrameRotation() << " " << MinorFrameRotation() << " ";
}

void LowStretchTorusParametrization::Deserialize(std::istream *i, bool binary)
{
	float majorRot, minorRot;
	if(binary)
	{
		i->read((char *)&majorRot, sizeof(majorRot));
		i->read((char *)&minorRot, sizeof(minorRot));
	}
	else
		*i >> majorRot >> minorRot;
	GfxTL::Frame< 3, float > nframe;
	nframe.FromNormal(m_torus->AxisDirection());
	nframe.RotateOnNormal(majorRot);
	m_hcs[0] = nframe[0];
	m_hcs[1] = nframe[1];
	m_minorFrame.Canonical();
	m_minorFrame.RotateFrame(minorRot);
}

float LowStretchTorusParametrization::MinorFrameRotation() const
{
	return std::atan2(GfxTL::Math< float >::Clamp(m_minorFrame[0][1], -1, 1),
		GfxTL::Math< float >::Clamp(m_minorFrame[0][0], -1, 1));
}

float LowStretchTorusParametrization::MajorFrameRotation() const
{
	// defer rotation from frame
	GfxTL::Frame< 3, float > nframe;
	nframe.FromNormal(m_torus->AxisDirection());
	GfxTL::VectorXD< 2, float > t;
	nframe.ToTangent(m_hcs[0], &t);
	for(unsigned int i = 0; i < 2; ++i)
		t[i] = GfxTL::Math< float >::Clamp(t[i], -1, 1);
	return std::atan2(t[1], t[0]);
}
