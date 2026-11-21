// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "ccXRayExposure.h"

// System
#include <cstdio>
#include <cstdlib>

static void ExpectNear(const char* name, double actual, double expected)
{
	if (!std::isfinite(actual) || std::abs(actual - expected) > 1.0e-6 * std::max(1.0, std::abs(expected)))
	{
		std::fprintf(stderr, "%s: expected %.9g, got %.9g\n", name, expected, actual);
		std::exit(EXIT_FAILURE);
	}
}

int main()
{
	CCVector3d  minimum{-0.5, -0.5, -2.0};
	CCVector3d  maximum{0.5, 0.5, -2.0};
	ccGLMatrixd view;
	ccGLMatrixd projection;

	auto area = [&]()
	{
		return ccXRayExposure::ProjectedAreaRatio(minimum, maximum, view, projection);
	};

	view.toIdentity();
	projection.toIdentity();
	ExpectNear("orthographic quarter-screen footprint", area(), 0.25);

	projection.data()[0] = projection.data()[5] = 0.5;
	ExpectNear("half-width and half-height give quarter contribution", area(), 0.0625);
	view.data()[12] = 100;
	view.data()[13] = -100;
	ExpectNear("panning offscreen does not change exposure", area(), 0.0625);
	view.data()[14] = -100;
	ExpectNear("orthographic camera distance does not change exposure", area(), 0.0625);

	view.toIdentity();
	projection.toIdentity();
	projection.data()[0] = 0.5;
	ExpectNear("viewport aspect ratio is included", area(), 0.125);
	projection.toIdentity();
	view.data()[0] = view.data()[5] = 2.0;
	ExpectNear("entity scale is included", area(), 1.0);

	view.toIdentity();
	projection.toZero();
	projection.data()[0]  = 1;
	projection.data()[5]  = 1;
	projection.data()[10] = -1;
	projection.data()[11] = -1;
	projection.data()[14] = -0.2;
	double nearArea       = area();
	ExpectNear("perspective projection", nearArea, 0.0625);
	view.data()[14] = -2;
	ExpectNear("twice the perspective distance gives quarter contribution", area(), nearArea / 4);
	view.data()[14] = 2;
	ExpectNear("eye-plane division is bounded", area(), 64);

	view.toIdentity();
	projection.toIdentity();
	CCVector3d point{0, 0, 0};
	double     pointArea = ccXRayExposure::ProjectedAreaRatio(point, point, view, projection);
	if (!(pointArea > 0 && std::isfinite(pointArea)))
	{
		std::fprintf(stderr, "degenerate box must have finite positive exposure\n");
		return EXIT_FAILURE;
	}

	const double base = ccXRayExposure::Strength(0.25, 0);
	ExpectNear("one EV doubles intensity", ccXRayExposure::Strength(0.25, 1), base * 2);
	ExpectNear("minus one EV halves intensity", ccXRayExposure::Strength(0.25, -1), base / 2);
	ExpectNear("default exposure is one eighth of zero EV", ccXRayExposure::Strength(0.25, -3), base / 8);
	std::puts("X-ray exposure tests passed.");
	return EXIT_SUCCESS;
}
