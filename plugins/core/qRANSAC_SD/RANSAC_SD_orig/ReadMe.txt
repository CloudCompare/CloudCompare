###################################################################
#                                                                 #
#    PrimitiveShapes - software for extracting primitive shapes   #
#                      (planes, spheres, cylinders, cones, tori)  #
#                      from point-clouds via RANSAC               #
#    Version 1.1                                                  #
#                                                                 #
#    Ruwen Schnabel (schnabel@cs.uni-bonn.de)                     #
#    Roland Wahl (wahl@cs.uni-bonn.de)                            #
#    2006-2009                                                    #
#                                                                 #
###################################################################

1. Introduction.

Thank you for taking interest in our work and downloading this
software. This library implements the algorithm described in the paper

	R. Schnabel, R. Wahl, R. Klein
	"Efficient RANSAC for Point-Cloud Shape Detection",
	in Computer Graphics Forum, Vol. 26, No. 2, pages 214-226,
	Blackwell Publishing, June 2007

If you use this software you should cite the aforementioned paper
in any resulting publication.

Please send questions, comments or bug reports to
Ruwen Schnabel (schnabel@cs.uni-bonn.de)

The library has been tested using Visual Studio 2005 but it should
work with gcc under Linux as well

##################################################################

2. License & disclaimer.

    Copyright 2009 Ruwen Schnabel (schnabel@cs.uni-bonn.de),
                   Roland Wahl (wahl@cs.uni-bonn.de).

    This software may be used for research purposes only.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

##################################################################

3. Example usage.

This section shows how to use the library to detect the shapes in a point-cloud

////////////////////////////////////////////////////////////////////////
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

PointCloud pc;

// fill or load point cloud from file
// ...
// don't forget to set the bbox in pc

RansacShapeDetector::Options ransacOptions;
ransacOptions.m_epsilon = .01f * pc.getScale(); // set distance threshold to .01f of bounding box width
	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
	// NOTE: This threshold is NOT multiplied internally!
ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
ransacOptions.m_minSupport = 500; // this is the minimal numer of points required for a primitive
ransacOptoins.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

RansacShapeDetector detector(ransacOptions); // the detector object

// set which primitives are to be detected by adding the respective constructors
detector.Add(new PlanePrimitiveShapeConstructor());
detector.Add(new SpherePrimitiveShapeConstructor());
detector.Add(new CylinderPrimitiveShapeConstructor());
detector.Add(new ConePrimitiveShapeConstructor());
detector.Add(new TorusPrimitiveShapeConstructor());

MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
