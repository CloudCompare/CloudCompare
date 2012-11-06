/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/

#ifndef X3DTYPES_H
#define X3DTYPES_H

#include <xiot/XIOTConfig.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>

/**
 * All of the methods of the x3dLoader library  reside in the X3D namespace in order
 * to prevent name conflicts.
 */
namespace XIOT {
  /**
   * Return value for element callbacks that indicates
   * the parser should continue normally.
   */
  static const int CONTINUE = 1;
  /**
   * Return value for element callbacks that indicates
   * the parser should abort.
   * @warning Not implemented yet.
   */
  static const int ABORT = 2;
  /**
   * Return value for element callbacks that indicates
   * the parser should suppress all callbacks for
   * child nodes until the corresponding end callback.
   */
  static const int SKIP_CHILDREN = 3;

  struct XIOT_EXPORT Property {
		static const char* FloatEncodingAlgorithm; // "http://www.web3d.org/x3d/properties/fi/FloatEncodingAlgorithm";
		static const char* IntEncodingAlgorithm; // "http://www.web3d.org/x3d/properties/fi/IntEncodingAlgorithm";
  };

  struct XIOT_EXPORT Encoder {
		static const char* BuiltIn; // 0;
		static const char* DeltazlibIntArrayEncoder; // "encoder://web3d.org/DeltazlibIntArrayEncoder";
		static const char* QuantizedzlibFloatArrayEncoder; // "encoder://web3d.org/QuantizedzlibFloatArrayEncoder";
  };

  enum X3DProfile {
      Core = 0,
      Full,
      Immersive,
      Interactive,
      Interchange,
      MPEG4Interactive
    };

  enum X3DVersion {
      VERSION_3_0,
      VERSION_3_1,
      VERSION_3_2,
    };


  /**
   * \defgroup x3ddatatypes X3D Data Types
   */

  typedef std::vector<float> MFFloat;
  typedef std::vector<int> MFInt32;
  
  
  typedef std::string SFString;
  typedef std::vector<SFString> MFString;


  /**
   * The SFVec2f field specifies a two-dimensional (2D) vector.
   *
   * A 2-element tuple that is represented by single-precision floating point x,y coordinates
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFVec2fAndMFVec2f">X3D spec. 5.3.17</a>
   * @ingroup x3ddatatypes
   */
  struct SFVec2f {
	  float x, y;	  
	  SFVec2f() : x(0.0f), y(0.0f) {};
	  SFVec2f(float X, float Y) : x(X), y(Y) {};
	  template <class C>
		explicit SFVec2f(const C& c) : x(c[0]), y(c[1]) {};
    /// Bracket operator.
    inline float &operator[](int i) {return (&x)[i];};
    /// Bracket operator.
    inline const float &operator[](int i) const {return (&x)[i];};
  };

  typedef std::vector<SFVec2f> MFVec2f;

  /**
   * The SFVec3f field specifies a three-dimensional (3D) vector.
   *
   * A 3-element tuple that is represented by single-precision floating point x,y,z coordinates
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFVec3fAndMFVec3f">X3D spec. 5.3.19</a>
   * @ingroup x3ddatatypes
   */
  struct SFVec3f {
	  float x, y, z;
  	  SFVec3f() : x(0.0f), y(0.0f), z(0.0f) {};
	  SFVec3f(float X, float Y, float Z) : x(X), y(Y), z(Z) {};
  	  template <class C>
		explicit SFVec3f(const C& c) : x(c[0]), y(c[1]), z(c[2]) {};
    /// Bracket operator.
    inline float &operator[](int i) {return (&x)[i];};
    /// Bracket operator.
    inline const float &operator[](int i) const {return (&x)[i];};

  };

  typedef std::vector<SFVec3f> MFVec3f;
  
  /**
   * The SFRotation field specifies one arbitrary rotation. 
   *
   * An SFRotation is written to the X3D file as four floating point values. 
   * The first three values specify a normalized rotation axis vector about
   * which the rotation takes place. The fourth value specifies the amount 
   * of right-handed rotation about that axis in radians. 
   *
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFRotationAndMFRotation">X3D spec. 5.3.13</a>
   * @ingroup x3ddatatypes
   */
  struct SFRotation {
	  float x, y, z, angle;	  
   	  SFRotation() : x(0.0f), y(0.0f), z(1.0f), angle(0.0f) {};
	  SFRotation(float X, float Y, float Z, float Angle) : x(X), y(Y), z(Z), angle(Angle) {};
	  template <class C>
		explicit SFRotation(const C& c) : x(c[0]), y(c[1]), z(c[2]), angle(c[3]) {};
    /// Bracket operator.
    inline float &operator[](int i) {return (&x)[i];};
    /// Bracket operator.
    inline const float &operator[](int i) const {return (&x)[i];};

  };

  typedef std::vector<SFRotation> MFRotation;


  /**
   * The SFColor field specifies one RGB (red-green-blue) colour triple. 
   *
   * A 3-element tuple that is represented by single-precision floating point 
   * r,g,b RGB triple in the range 0.0 to 1.0.
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFColorAndMFColor">X3D spec. 5.3.2</a>
   * @ingroup x3ddatatypes
   */
  struct SFColor {
	  float r, g, b;	  
	  SFColor() : r(0.0f), g(0.0f), b(0.0f) { 
		  //std::cout << "Creating SFColor" << std::endl;
	  };
	  SFColor(float R, float G, float B) : r(R), g(G), b(B) {};
	
	  SFColor(const SFColor &other) : r(other.r), g(other.g), b(other.b) {
		  //std::cout << "Copying SFColor" << std::endl;
	  };
	  //SFColor(float R, float G, float B) : r(R), g(G), b(B) {std::cout << "Creating SFColor" << std::endl;};
	  template <class C>
		explicit SFColor(const C& c) : r(c[0]), g(c[1]), b(c[2]) {
			//std::cout << "Creating SFColor" << std::endl;
		}
    /// Bracket operator.
    inline float &operator[](int i) {return (&r)[i];};
    /// Bracket operator.
    inline const float &operator[](int i) const {return (&r)[i];};

  };

  typedef std::vector<SFColor> MFColor;

  /**
   * The SFColorRGBA field specifies one RGBA (red-green-blue-alpha) colour
   * quadruple that includes alpha (opacity) information. 
   *
   * Each colour is written to the X3D file as an RGBA quadruple of floating 
   * point numbers in the range 0.0 to 1.0. Alpha values range from 0.0 
   * (fully transparent) to 1.0 (fully opaque).
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFColorRGBAAndMFColorRGBA">X3D spec. 5.3.3</a>
   * @ingroup x3ddatatypes
   */
  struct SFColorRGBA {
	  float r, g, b, a;	  
 	  SFColorRGBA() : r(0.0f), g(0.0f), b(0.0f), a(0.0f) {};
	  SFColorRGBA(float R, float G, float B, float A) : r(R), g(G), b(B), a(A) {};
	  template <class C>
		explicit SFColorRGBA(const C& c) : r(c[0]), g(c[1]), b(c[2]), a(c[3]) {};
    /// Bracket operator.
    inline float &operator[](int i) {return (&r)[i];};
    /// Bracket operator.
    inline const float &operator[](int i) const {return (&r)[i];};

  };

  typedef std::vector<SFColorRGBA> MFColorRGBA;

  /**
   * The SFImage field specifies a single uncompressed 2-dimensional pixel image. 
   *
   * SFImage fields contain three integers representing the width, height
   * and number of components in the image, followed by widthxheight hexadecimal
   * or integer values representing the pixels in the image.
   * @see <a href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/fieldsDef.html#SFImageAndMFImage">X3D spec 5.3.6</a>
   * @ingroup x3ddatatypes
   */
  typedef std::vector<unsigned int> SFImage;
  typedef std::vector<SFImage> MFImage;


  /**
   * Utility class that defines node and attribute ids
   *
   * The X3DTypes class provides static functions for obtaining an 
   * attribute's or element's string representation by its ID 
   * or vice versa.
   */
  class XIOT_EXPORT X3DTypes {
	friend class X3DParserVocabulary;

  public:
	  static const char* getElementByID(int id);
	  static const char* getAttributeByID(int id);
	  static int getElementID(const std::string &elementStr);
	  static int getAttributeID(const std::string &attributeStr);

	  static void initMaps();

    static const char* getProfileString(X3DProfile profile);
    static const char* getVersionString(X3DVersion version);

  private:
	  static std::map<std::string, int>	elementFromStringMap;
	  static std::map<std::string, int>	attributeFromStringMap;
	  static std::map<int, std::string>	elementFromIDMap;
	  static std::map<int, std::string>	attributeFromIDMap;
  };


  namespace ID {
  /* Elements */
  enum X3DElement
    {
		Shape = 0,
		Appearance = 1,
		Material = 2,
		IndexedFaceSet = 3,
		ProtoInstance = 4,
		Transform = 5,
		ImageTexture = 6,
		TextureTransform = 7,
		Coordinate = 8,
		Normal = 9,
		Color = 10,
		ColorRGBA = 11,
		TextureCoordinate = 12,
		ROUTE = 13,
		fieldValue = 14,
		Group = 15,
		LOD = 16,
		Switch = 17,
		Script = 18,
		IndexedTriangleFanSet = 19,
		IndexedTriangleSet = 20,
		IndexedTriangleStripSet = 21,
		MultiTexture = 22,
		MultiTextureCoordinate = 23,
		MultiTextureTransform = 24,
		IndexedLineSet = 25,
		PointSet = 26,
		StaticGroup = 27,
		Sphere = 28,
		Box = 29,
		Cone = 30,
		Anchor = 31,
		Arc2D = 32,
		ArcClose2D = 33,
		AudioClip = 34,
		Background = 35,
		Billboard = 36,
		BooleanFilter = 37,
		BooleanSequencer = 38,
		BooleanToggle = 39,
		BooleanTrigger = 40,
		Circle2D = 41,
		Collision = 42,
		ColorInterpolator = 43,
		Contour2D = 44,
		ContourPolyline2D = 45,
		CoordinateDouble = 46,
		CoordinateInterpolator = 47,
		CoordinateInterpolator2D = 48,
		Cylinder = 49,
		CylinderSensor = 50,
		DirectionalLight = 51,
		Disk2D = 52,
		EXPORT = 53,
		ElevationGrid = 54,
		EspduTransform = 55,
		ExternProtoDeclare = 56,
		Extrusion = 57,
		FillProperties = 58,
		Fog = 59,
		FontStyle = 60,
		GeoCoordinate = 61,
		GeoElevationGrid = 62,
		GeoLOD = 63,
		GeoLocation = 64,
		GeoMetadata = 65,
		GeoOrigin = 66,
		GeoPositionInterpolator = 67,
		GeoTouchSensor = 68,
		GeoViewpoint = 69,
		HAnimDisplacer = 70,
		HAnimHumanoid = 71,
		HAnimJoint = 72,
		HAnimSegment = 73,
		HAnimSite = 74,
		IMPORT = 75,
		IS = 76,
		Inline = 77,
		IntegerSequencer = 78,
		IntegerTrigger = 79,
		KeySensor = 80,
		LineProperties = 81,
		LineSet = 82,
		LoadSensor = 83,
		MetadataDouble = 84,
		MetadataFloat = 85,
		MetadataInteger = 86,
		MetadataSet = 87,
		MetadataString = 88,
		MovieTexture = 89,
		NavigationInfo = 90,
		NormalInterpolator = 91,
		NurbsCurve = 92,
		NurbsCurve2D = 93,
		NurbsOrientationInterpolator = 94,
		NurbsPatchSurface = 95,
		NurbsPositionInterpolator = 96,
		NurbsSet = 97,
		NurbsSurfaceInterpolator = 98,
		NurbsSweptSurface = 99,
		NurbsSwungSurface = 100,
		NurbsTextureCoordinate = 101,
		NurbsTrimmedSurface = 102,
		OrientationInterpolator = 103,
		PixelTexture = 104,
		PlaneSensor = 105,
		PointLight = 106,
		Polyline2D = 107,
		Polypoint2D = 108,
		PositionInterpolator = 109,
		PositionInterpolator2D = 110,
		ProtoBody = 111,
		ProtoDeclare = 112,
		ProtoInterface = 113,
		ProximitySensor = 114,
		ReceiverPdu = 115,
		Rectangle2D = 116,
		ScalarInterpolator = 117,
		Scene = 118,
		SignalPdu = 119,
		Sound = 120,
		SphereSensor = 121,
		SpotLight = 122,
		StringSensor = 123,
		Text = 124,
		TextureBackground = 125,
		TextureCoordinateGenerator = 126,
		TimeSensor = 127,
		TimeTrigger = 128,
		TouchSensor = 129,
		TransmitterPdu = 130,
		TriangleFanSet = 131,
		TriangleSet = 132,
		TriangleSet2D = 133,
		TriangleStripSet = 134,
		Viewpoint = 135,
		VisibilitySensor = 136,
		WorldInfo = 137,
		X3D = 138,
		component = 139,
		connect = 140,
		field = 141,
		head = 142,
		humanoidBodyType = 143,
		meta = 144,
		CADAssembly,
		CADFace,
		CADLayer,
		CADPart,
		ComposedCubeMapTexture,
		ComposedShader,
		ComposedTexture3D,
		FloatVertexAttribute,
		FogCoordinate,
		GeneratedCubeMapTexture,
		ImageCubeMapTexture,
		ImageTexture3D,
		IndexedQuadSet,
		LocalFog,
		Matrix3VertexAttribute,
		Matrix4VertexAttribute,
		PackagedShader,
		PixelTexture3D,
		ProgramShader,
		QuadSet,
		ShaderPart,
		ShaderProgram,
		TextureCoordinate3D,
		TextureCoordinate4D,
		TextureTransform3D,
		TextureTransformMatrix3D,
		X3DELEMENT_COUNT
    };

  /* Attributes */
  enum X3DAttribute
    {
    DEF = 0,
    USE = 1,
    containerField = 2,
    fromNode = 3,
    fromField = 4,
    toNode = 5,
    toField = 6,
    name = 7,
    value = 8,
    color = 9,
    colorIndex = 10,
    coordIndex = 11,
    texCoordIndex = 12,
    normalIndex = 13,
    colorPerVertex = 14,
    normalPerVertex = 15,
    rotation = 16,
    scale = 17,
    center = 18,
    scaleOrientation = 19,
    translation = 20,
    url = 21,
    repeatS = 22,
    repeatT = 23,
    point = 24,
    vector = 25,
    range = 26,
    ambientIntensity = 27,
    diffuseColor = 28,
    emissiveColor = 29,
    shininess = 30,
    specularColor = 31,
    transparency = 32,
    whichChoice = 33,
    index = 34,
    mode = 35,
    source = 36,
    function = 37,
    alpha = 38,
    vertexCount = 39,
    radius = 40,
    size = 41,
    height = 42,
    solid = 43,
    ccw = 44,
    key = 45,
    keyValue = 46,
    enabled = 47,
    direction = 48,
    position = 49,
    orientation = 50,
    bboxCenter = 51,
    bboxSize = 52,
    AS = 53,
    InlineDEF = 54,
    accessType = 55,
    actionKeyPress = 56,
    actionKeyRelease = 57,
    address = 58,
    altKey = 59,
    antennaLocation = 60,
    antennaPatternLength = 61,
    antennaPatternType = 62,
    applicationID = 63,
    articulationParameterArray = 64,
    articulationParameterChangeIndicatorArray = 65,
    articulationParameterCount = 66,
    articulationParameterDesignatorArray = 67,
    articulationParameterIdPartAttachedArray = 68,
    articulationParameterTypeArray = 69,
    attenuation = 70,
    autoOffset = 71,
    avatarSize = 72,
    axisOfRotation = 73,
    backUrl = 74,
    beamWidth = 75,
    beginCap = 76,
    bindTime = 77,
    bottom = 78,
    bottomRadius = 79,
    bottomUrl = 80,
    centerOfMass = 81,
    centerOfRotation = 82,
    child1Url = 83,
    child2Url = 84,
    child3Url = 85,
    child4Url = 86,
    _class = 87,
    closureType = 88,
    collideTime = 89,
    content = 90,
    controlKey = 91,
    controlPoint = 92,
    convex = 93,
    coordinateSystem = 94,
    copyright = 95,
    creaseAngle = 96,
    crossSection = 97,
    cryptoKeyID = 98,
    cryptoSystem = 99,
    cutOffAngle = 100,
    cycleInterval = 101,
    cycleTime = 102,
    data = 103,
    dataFormat = 104,
    dataLength = 105,
    dataUrl = 106,
    date = 107,
    deadReckoning = 108,
    deletionAllowed = 109,
    description = 110,
    detonateTime = 111,
    dir = 112,
    directOutput = 113,
    diskAngle = 114,
    displacements = 115,
    documentation = 116,
    elapsedTime = 117,
    ellipsoid = 118,
    encodingScheme = 119,
    endAngle = 120,
    endCap = 121,
    enterTime = 122,
    enteredText = 123,
    entityCategory = 124,
    entityCountry = 125,
    entityDomain = 126,
    entityExtra = 127,
    entityID = 128,
    entityKind = 129,
    entitySpecific = 130,
    entitySubCategory = 131,
    exitTime = 132,
    extent = 133,
    family = 134,
    fanCount = 135,
    fieldOfView = 136,
    filled = 137,
    finalText = 138,
    fireMissionIndex = 139,
    fired1 = 140,
    fired2 = 141,
    firedTime = 142,
    firingRange = 143,
    firingRate = 144,
    fogType = 145,
    forceID = 146,
    frequency = 147,
    frontUrl = 148,
    fuse = 149,
    geoCoords = 150,
    geoGridOrigin = 151,
    geoSystem = 152,
    groundAngle = 153,
    groundColor = 154,
    hatchColor = 155,
    hatchStyle = 156,
    hatched = 157,
    headlight = 158,
    horizontal = 159,
    horizontalDatum = 160,
    http_equiv = 161,
    image = 162,
    importedDEF = 163,
    info = 164,
    innerRadius = 165,
    inputFalse = 166,
    inputNegate = 167,
    inputSource = 168,
    inputTrue = 169,
    integerKey = 170,
    intensity = 171,
    jump = 172,
    justify = 173,
    keyPress = 174,
    keyRelease = 175,
    knot = 176,
    lang = 177,
    language = 178,
    leftToRight = 179,
    leftUrl = 180,
    length = 181,
    lengthOfModulationParameters = 182,
    level = 183,
    limitOrientation = 184,
    lineSegments = 185,
    linearAcceleration = 186,
    linearVelocity = 187,
    linetype = 188,
    linewidthScaleFactor = 189,
    llimit = 190,
    load = 191,
    loadTime = 192,
    localDEF = 193,
    location = 194,
    loop = 195,
    marking = 196,
    mass = 197,
    maxAngle = 198,
    maxBack = 199,
    maxExtent = 200,
    maxFront = 201,
    maxPosition = 202,
    metadataFormat = 203,
    minAngle = 204,
    minBack = 205,
    minFront = 206,
    minPosition = 207,
    modulationTypeDetail = 208,
    modulationTypeMajor = 209,
    modulationTypeSpreadSpectrum = 210,
    modulationTypeSystem = 211,
    momentsOfInertia = 212,
    multicastRelayHost = 213,
    multicastRelayPort = 214,
    munitionApplicationID = 215,
    munitionEndPoint = 216,
    munitionEntityID = 217,
    munitionQuantity = 218,
    munitionSiteID = 219,
    munitionStartPoint = 220,
    mustEvaluate = 221,
    navType = 222,
    networkMode = 223,
    next = 224,
    nodeField = 225,
    offset = 226,
    on = 227,
    order = 228,
    originator = 229,
    outerRadius = 230,
    parameter = 231,
    pauseTime = 232,
    pitch = 233,
    points = 234,
    port = 235,
    power = 236,
    previous = 237,
    priority = 238,
    profile = 239,
    progress = 240,
    protoField = 241,
    radioEntityTypeCategory = 242,
    radioEntityTypeCountry = 243,
    radioEntityTypeDomain = 244,
    radioEntityTypeKind = 245,
    radioEntityTypeNomenclature = 246,
    radioEntityTypeNomenclatureVersion = 247,
    radioID = 248,
    readInterval = 249,
    receivedPower = 250,
    receiverState = 251,
    reference = 252,
    relativeAntennaLocation = 253,
    resolution = 254,
    resumeTime = 255,
    rightUrl = 256,
    rootUrl = 257,
    rotateYUp = 258,
    rtpHeaderExpected = 259,
    sampleRate = 260,
    samples = 261,
    shiftKey = 262,
    side = 263,
    siteID = 264,
    skinCoordIndex = 265,
    skinCoordWeight = 266,
    skyAngle = 267,
    skyColor = 268,
    spacing = 269,
    spatialize = 270,
    speed = 271,
    speedFactor = 272,
    spine = 273,
    startAngle = 274,
    startTime = 275,
    stiffness = 276,
    stopTime = 277,
    string = 278,
    stripCount = 279,
    style = 280,
    summary = 281,
    tdlType = 282,
    tessellation = 283,
    tessellationScale = 284,
    time = 285,
    timeOut = 286,
    timestamp = 287,
    title = 288,
    toggle = 289,
    top = 290,
    topToBottom = 291,
    topUrl = 292,
    touchTime = 293,
    transmitFrequencyBandwidth = 294,
    transmitState = 295,
    transmitterApplicationID = 296,
    transmitterEntityID = 297,
    transmitterRadioID = 298,
    transmitterSiteID = 299,
    transparent = 300,
    triggerTime = 301,
    triggerTrue = 302,
    triggerValue = 303,
    type = 304,
    uDimension = 305,
    uKnot = 306,
    uOrder = 307,
    uTessellation = 308,
    ulimit = 309,
    vDimension = 310,
    vKnot = 311,
    vOrder = 312,
    vTessellation = 313,
    version = 314,
    verticalDatum = 315,
    vertices = 316,
    visibilityLimit = 317,
    visibilityRange = 318,
    warhead = 319,
    weight = 320,
    whichGeometry = 321,
    writeInterval = 322,
    xDimension = 323,
    xSpacing = 324,
    yScale = 325,
    zDimension = 326,
    zSpacing = 327,
	visible = 328,
	repeatR = 329,
	texture = 330,
	back = 331,
	front = 332,
	left = 333,
	right = 334,
	parts = 335,
	isSelected = 336,
	isValid = 337,
	numComponents = 338,
	depth = 339,
	update = 340,
	fogCoord = 341,
	texCoord = 342,
	activate = 343,
	programs = 344,
	matrix = 345,
	X3DATTRIBUTE_COUNT
    };
	
	}; // namespace ID
	



};

#endif

