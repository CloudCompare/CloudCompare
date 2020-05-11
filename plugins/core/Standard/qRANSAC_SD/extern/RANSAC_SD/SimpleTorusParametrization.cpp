#include "SimpleTorusParametrization.h"

SimpleTorusParametrization::SimpleTorusParametrization(
	const Torus &torus)
: m_torus(&torus)
{
	m_hcs.FromNormal(m_torus->AxisDirection());
}

void SimpleTorusParametrization::Shape(const Torus &torus)
{
	m_torus = &torus;
	m_hcs.FromNormal(m_torus->AxisDirection());
}

void SimpleTorusParametrization::WrapBitmap(const GfxTL::AABox< GfxTL::Vector2Df > &bbox,
	float epsilon, bool *uwrap, bool *vwrap) const
{
	if(bbox.Max()[0] - bbox.Min()[0] >= 2 * M_PI * 
		((m_torus->MajorRadius() < m_torus->MinorRadius() * 2) ?
			m_torus->MajorRadius() + m_torus->MinorRadius() : m_torus->MajorRadius()) - 2 * epsilon)
		*uwrap = true; // wrap around major component
	else
		*uwrap = false;
	if(bbox.Max()[1] - bbox.Min()[1] >= 2 * M_PI * m_torus->MinorRadius() - 2 * epsilon)
		*vwrap = true; // wrap around minor component
	else
		*vwrap = false;
}

size_t SimpleTorusParametrization::SerializedSize()
{
	return sizeof(float);
}

void SimpleTorusParametrization::Serialize(std::ostream *o, bool binary) const
{
	// for now only output placeholder for rotation of hcs
	float rot = 0;
	if(binary)
		o->write((char *)&rot, sizeof(rot));
	else
		*o << rot << " ";
}

void SimpleTorusParametrization::Deserialize(std::istream *i, bool binary)
{
	// read the placeholder
	float rot;
	if(binary)
		i->read((char *)&rot, sizeof(rot));
	else
		*i >> rot;
}
