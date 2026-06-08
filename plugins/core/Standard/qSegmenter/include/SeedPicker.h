#pragma once

#include <ccMainAppInterface.h>
#include <ccPickingListener.h>
#include <ccPointCloud.h>
#include <vector>

class SeedPicker : public ccPickingListener
{
  public:
	SeedPicker(ccMainAppInterface* app);
	virtual ~SeedPicker();

	void startListening();
	void stopListening();

	// UI Trigger for the algorithm
	int runRegionGrowing(double searchRadius, double tauThreshold, double ws, double wc);
	// Getters / Setters
	void setPositiveMode(bool isPositive)
	{
		m_isPositive = isPositive;
	}

	void setAlgorithmParameters(double radius, double tau)
	{
		m_searchRadius = radius;
		m_tauThreshold = tau;
	}

  protected:
	// Only declared ONCE here
	virtual void onItemPicked(const PickedItem& pi) override;

  private:
	ccMainAppInterface* m_app;
	ccPointCloud*       m_targetCloud  = nullptr;
	ccPointCloud*       m_previewCloud = nullptr;

	std::vector<unsigned int> m_positiveSeeds;
	std::vector<unsigned int> m_negativeSeeds;

	// The compiler couldn't find this, so we ensure it is here in private:
	std::vector<unsigned int> m_segmentedIndices;

	double m_searchRadius = 0.1;  // Default radius (adjust based on your sphere's scale)
	double m_tauThreshold = 30.0; // Default RGB distance (0-255 scale)

	bool m_isPositive = true;

	ccPointCloud* m_posMarkerCloud = nullptr;
	ccPointCloud* m_negMarkerCloud = nullptr;

    std::vector<ccColor::Rgba> m_originalColors;
};