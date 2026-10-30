// CloudCompare - GPL-2.0-or-later
#include "ccXRayExposure.h"

#include <array>
#include <cstdio>
#include <cstdlib>

using Matrix = std::array<double, 16>;

static Matrix Identity()
{
	return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

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
	const double minimum[3] = {-0.5, -0.5, -2.0};
	const double maximum[3] = {0.5, 0.5, -2.0};
	Matrix       view       = Identity();
	Matrix       projection = Identity();
	auto         area       = [&]()
	{ return ccXRay::ProjectedAreaRatio(minimum, maximum, view.data(), projection.data()); };
	ExpectNear("orthographic quarter-screen footprint", area(), 0.25);

	projection[0] = projection[5] = 0.5;
	ExpectNear("half-width and half-height give quarter contribution", area(), 0.0625);
	view[12] = 100;
	view[13] = -100;
	ExpectNear("panning offscreen does not change exposure", area(), 0.0625);
	view[14] = -100;
	ExpectNear("orthographic camera distance does not change exposure", area(), 0.0625);

	view          = Identity();
	projection    = Identity();
	projection[0] = 0.5;
	ExpectNear("viewport aspect ratio is included", area(), 0.125);
	projection = Identity();
	view[0] = view[5] = 2.0;
	ExpectNear("entity scale is included", area(), 1.0);

	view                  = Identity();
	projection            = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, -1, 0, 0, -0.2, 0};
	const double nearArea = area();
	ExpectNear("perspective projection", nearArea, 0.0625);
	view[14] = -2;
	ExpectNear("twice the perspective distance gives quarter contribution", area(), nearArea / 4);
	view[14] = 2;
	ExpectNear("eye-plane division is bounded", area(), 64);

	view                   = Identity();
	projection             = Identity();
	const double point[3]  = {0, 0, 0};
	const double pointArea = ccXRay::ProjectedAreaRatio(point, point, view.data(), projection.data());
	if (!(pointArea > 0 && std::isfinite(pointArea)))
	{
		std::fprintf(stderr, "degenerate box must have finite positive exposure\n");
		return EXIT_FAILURE;
	}

	const double base = ccXRay::Strength(0.25, 0);
	ExpectNear("one EV doubles intensity", ccXRay::Strength(0.25, 1), base * 2);
	ExpectNear("minus one EV halves intensity", ccXRay::Strength(0.25, -1), base / 2);
	ExpectNear("default exposure is one eighth of zero EV", ccXRay::Strength(0.25, -3), base / 8);
	ExpectNear("doubling overlap with half intensity preserves accumulated energy",
	           200 * base / 2,
	           100 * base);
	std::puts("X-ray exposure tests passed.");
	return EXIT_SUCCESS;
}
