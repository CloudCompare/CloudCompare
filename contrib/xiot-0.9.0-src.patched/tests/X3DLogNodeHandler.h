#ifndef X3D_X3DLogNodeHandler_H
#define X3D_X3DLogNodeHandler_H

#include <xiot/X3DDefaultNodeHandler.h>

using namespace XIOT;

/**
 * Handler that logs all library events to a file.
 *
 * This is yet another class that derives from the X3DDefaultNodeHandler. 
 * It overrides each callback function to create a log file,
 * which reports every start and end of a XML node/event. 
 * Plus, it will count the number of  events - thus nodes - 
 * occuring throughout the file import.
 * @see X3DDefaultNodeHandler
 */
class X3DLogNodeHandler : public X3DDefaultNodeHandler {

public:

  X3DLogNodeHandler(std::string fileName);
  ~X3DLogNodeHandler();

  std::string getAttributesAsString(const X3DAttributes &attr);

  void startDocument();
  void endDocument();

  int startShape(const X3DAttributes &attr);
  int endShape();

  int startAppearance(const X3DAttributes &attr);
  int endAppearance();

  int startMaterial(const X3DAttributes &attr);
  int endMaterial();

  int startIndexedFaceSet(const X3DAttributes &attr);
  int endIndexedFaceSet();

  int startProtoInstance(const X3DAttributes &attr);
  int endProtoInstance();

  int startTransform(const X3DAttributes &attr);
  int endTransform();

  int startImageTexture(const X3DAttributes &attr);
  int endImageTexture();

  int startTextureTransform(const X3DAttributes &attr);
  int endTextureTransform();

  int startCoordinate(const X3DAttributes &attr);
  int endCoordinate();

  int startNormal(const X3DAttributes &attr);
  int endNormal();

  int startColor(const X3DAttributes &attr);
  int endColor();

  int startColorRGBA(const X3DAttributes &attr);
  int endColorRGBA();

  int startTextureCoordinate(const X3DAttributes &attr);
  int endTextureCoordinate();

  int startROUTE(const X3DAttributes &attr);
  int endROUTE();

  int startfieldValue(const X3DAttributes &attr);
  int endfieldValue();

  int startGroup(const X3DAttributes &attr);
  int endGroup();

  int startLOD(const X3DAttributes &attr);
  int endLOD();

  int startSwitch(const X3DAttributes &attr);
  int endSwitch();

  int startScript(const X3DAttributes &attr);
  int endScript();

  int startIndexedTriangleFanSet(const X3DAttributes &attr);
  int endIndexedTriangleFanSet();

  int startIndexedTriangleSet(const X3DAttributes &attr);
  int endIndexedTriangleSet();

  int startIndexedTriangleStripSet(const X3DAttributes &attr);
  int endIndexedTriangleStripSet();

  int startMultiTexture(const X3DAttributes &attr);
  int endMultiTexture();

  int startMultiTextureCoordinate(const X3DAttributes &attr);
  int endMultiTextureCoordinate();

  int startMultiTextureTransform(const X3DAttributes &attr);
  int endMultiTextureTransform();

  int startIndexedLineSet(const X3DAttributes &attr);
  int endIndexedLineSet();

  int startPointSet(const X3DAttributes &attr);
  int endPointSet();

  int startStaticGroup(const X3DAttributes &attr);
  int endStaticGroup();

  int startSphere(const X3DAttributes &attr);
  int endSphere();

  int startBox(const X3DAttributes &attr);
  int endBox();

  int startCone(const X3DAttributes &attr);
  int endCone();

  int startAnchor(const X3DAttributes &attr);
  int endAnchor();

  int startArc2D(const X3DAttributes &attr);
  int endArc2D();

  int startArcClose2D(const X3DAttributes &attr);
  int endArcClose2D();

  int startAudioClip(const X3DAttributes &attr);
  int endAudioClip();

  int startBackground(const X3DAttributes &attr);
  int endBackground();

  int startBillboard(const X3DAttributes &attr);
  int endBillboard();

  int startBooleanFilter(const X3DAttributes &attr);
  int endBooleanFilter();

  int startBooleanSequencer(const X3DAttributes &attr);
  int endBooleanSequencer();

  int startBooleanToggle(const X3DAttributes &attr);
  int endBooleanToggle();

  int startBooleanTrigger(const X3DAttributes &attr);
  int endBooleanTrigger();

  int startCircle2D(const X3DAttributes &attr);
  int endCircle2D();

  int startCollision(const X3DAttributes &attr);
  int endCollision();

  int startColorInterpolator(const X3DAttributes &attr);
  int endColorInterpolator();

  int startContour2D(const X3DAttributes &attr);
  int endContour2D();

  int startContourPolyline2D(const X3DAttributes &attr);
  int endContourPolyline2D();

  int startCoordinateDouble(const X3DAttributes &attr);
  int endCoordinateDouble();

  int startCoordinateInterpolator(const X3DAttributes &attr);
  int endCoordinateInterpolator();

  int startCoordinateInterpolator2D(const X3DAttributes &attr);
  int endCoordinateInterpolator2D();

  int startCylinder(const X3DAttributes &attr);
  int endCylinder();

  int startCylinderSensor(const X3DAttributes &attr);
  int endCylinderSensor();

  int startDirectionalLight(const X3DAttributes &attr);
  int endDirectionalLight();

  int startDisk2D(const X3DAttributes &attr);
  int endDisk2D();

  int startEXPORT(const X3DAttributes &attr);
  int endEXPORT();

  int startElevationGrid(const X3DAttributes &attr);
  int endElevationGrid();

  int startEspduTransform(const X3DAttributes &attr);
  int endEspduTransform();

  int startExternProtoDeclare(const X3DAttributes &attr);
  int endExternProtoDeclare();

  int startExtrusion(const X3DAttributes &attr);
  int endExtrusion();

  int startFillProperties(const X3DAttributes &attr);
  int endFillProperties();

  int startFog(const X3DAttributes &attr);
  int endFog();

  int startFontStyle(const X3DAttributes &attr);
  int endFontStyle();

  int startGeoCoordinate(const X3DAttributes &attr);
  int endGeoCoordinate();

  int startGeoElevationGrid(const X3DAttributes &attr);
  int endGeoElevationGrid();

  int startGeoLOD(const X3DAttributes &attr);
  int endGeoLOD();

  int startGeoLocation(const X3DAttributes &attr);
  int endGeoLocation();

  int startGeoMetadata(const X3DAttributes &attr);
  int endGeoMetadata();

  int startGeoOrigin(const X3DAttributes &attr);
  int endGeoOrigin();

  int startGeoPositionInterpolator(const X3DAttributes &attr);
  int endGeoPositionInterpolator();

  int startGeoTouchSensor(const X3DAttributes &attr);
  int endGeoTouchSensor();

  int startGeoViewpoint(const X3DAttributes &attr);
  int endGeoViewpoint();

  int startHAnimDisplacer(const X3DAttributes &attr);
  int endHAnimDisplacer();

  int startHAnimHumanoid(const X3DAttributes &attr);
  int endHAnimHumanoid();

  int startHAnimJoint(const X3DAttributes &attr);
  int endHAnimJoint();

  int startHAnimSegment(const X3DAttributes &attr);
  int endHAnimSegment();

  int startHAnimSite(const X3DAttributes &attr);
  int endHAnimSite();

  int startIMPORT(const X3DAttributes &attr);
  int endIMPORT();

  int startIS(const X3DAttributes &attr);
  int endIS();

  int startInline(const X3DAttributes &attr);
  int endInline();

  int startIntegerSequencer(const X3DAttributes &attr);
  int endIntegerSequencer();

  int startIntegerTrigger(const X3DAttributes &attr);
  int endIntegerTrigger();

  int startKeySensor(const X3DAttributes &attr);
  int endKeySensor();

  int startLineProperties(const X3DAttributes &attr);
  int endLineProperties();

  int startLineSet(const X3DAttributes &attr);
  int endLineSet();

  int startLoadSensor(const X3DAttributes &attr);
  int endLoadSensor();

  int startMetadataDouble(const X3DAttributes &attr);
  int endMetadataDouble();

  int startMetadataFloat(const X3DAttributes &attr);
  int endMetadataFloat();

  int startMetadataInteger(const X3DAttributes &attr);
  int endMetadataInteger();

  int startMetadataSet(const X3DAttributes &attr);
  int endMetadataSet();

  int startMetadataString(const X3DAttributes &attr);
  int endMetadataString();

  int startMovieTexture(const X3DAttributes &attr);
  int endMovieTexture();

  int startNavigationInfo(const X3DAttributes &attr);
  int endNavigationInfo();

  int startNormalInterpolator(const X3DAttributes &attr);
  int endNormalInterpolator();

  int startNurbsCurve(const X3DAttributes &attr);
  int endNurbsCurve();

  int startNurbsCurve2D(const X3DAttributes &attr);
  int endNurbsCurve2D();

  int startNurbsOrientationInterpolator(const X3DAttributes &attr);
  int endNurbsOrientationInterpolator();

  int startNurbsPatchSurface(const X3DAttributes &attr);
  int endNurbsPatchSurface();

  int startNurbsPositionInterpolator(const X3DAttributes &attr);
  int endNurbsPositionInterpolator();

  int startNurbsSet(const X3DAttributes &attr);
  int endNurbsSet();

  int startNurbsSurfaceInterpolator(const X3DAttributes &attr);
  int endNurbsSurfaceInterpolator();

  int startNurbsSweptSurface(const X3DAttributes &attr);
  int endNurbsSweptSurface();

  int startNurbsSwungSurface(const X3DAttributes &attr);
  int endNurbsSwungSurface();

  int startNurbsTextureCoordinate(const X3DAttributes &attr);
  int endNurbsTextureCoordinate();

  int startNurbsTrimmedSurface(const X3DAttributes &attr);
  int endNurbsTrimmedSurface();

  int startOrientationInterpolator(const X3DAttributes &attr);
  int endOrientationInterpolator();

  int startPixelTexture(const X3DAttributes &attr);
  int endPixelTexture();

  int startPlaneSensor(const X3DAttributes &attr);
  int endPlaneSensor();

  int startPointLight(const X3DAttributes &attr);
  int endPointLight();

  int startPolyline2D(const X3DAttributes &attr);
  int endPolyline2D();

  int startPolypoint2D(const X3DAttributes &attr);
  int endPolypoint2D();

  int startPositionInterpolator(const X3DAttributes &attr);
  int endPositionInterpolator();

  int startPositionInterpolator2D(const X3DAttributes &attr);
  int endPositionInterpolator2D();

  int startProtoBody(const X3DAttributes &attr);
  int endProtoBody();

  int startProtoDeclare(const X3DAttributes &attr);
  int endProtoDeclare();

  int startProtoInterface(const X3DAttributes &attr);
  int endProtoInterface();

  int startProximitySensor(const X3DAttributes &attr);
  int endProximitySensor();

  int startReceiverPdu(const X3DAttributes &attr);
  int endReceiverPdu();

  int startRectangle2D(const X3DAttributes &attr);
  int endRectangle2D();

  int startScalarInterpolator(const X3DAttributes &attr);
  int endScalarInterpolator();

  int startScene(const X3DAttributes &attr);
  int endScene();

  int startSignalPdu(const X3DAttributes &attr);
  int endSignalPdu();

  int startSound(const X3DAttributes &attr);
  int endSound();

  int startSphereSensor(const X3DAttributes &attr);
  int endSphereSensor();

  int startSpotLight(const X3DAttributes &attr);
  int endSpotLight();

  int startStringSensor(const X3DAttributes &attr);
  int endStringSensor();

  int startText(const X3DAttributes &attr);
  int endText();

  int startTextureBackground(const X3DAttributes &attr);
  int endTextureBackground();

  int startTextureCoordinateGenerator(const X3DAttributes &attr);
  int endTextureCoordinateGenerator();

  int startTimeSensor(const X3DAttributes &attr);
  int endTimeSensor();

  int startTimeTrigger(const X3DAttributes &attr);
  int endTimeTrigger();

  int startTouchSensor(const X3DAttributes &attr);
  int endTouchSensor();

  int startTransmitterPdu(const X3DAttributes &attr);
  int endTransmitterPdu();

  int startTriangleFanSet(const X3DAttributes &attr);
  int endTriangleFanSet();

  int startTriangleSet(const X3DAttributes &attr);
  int endTriangleSet();

  int startTriangleSet2D(const X3DAttributes &attr);
  int endTriangleSet2D();

  int startTriangleStripSet(const X3DAttributes &attr);
  int endTriangleStripSet();

  int startViewpoint(const X3DAttributes &attr);
  int endViewpoint();

  int startVisibilitySensor(const X3DAttributes &attr);
  int endVisibilitySensor();

  int startWorldInfo(const X3DAttributes &attr);
  int endWorldInfo();

  int startX3D(const X3DAttributes &attr);
  int endX3D();

  int startcomponent(const X3DAttributes &attr);
  int endcomponent();

  int startconnect(const X3DAttributes &attr);
  int endconnect();

  int startfield(const X3DAttributes &attr);
  int endfield();

  int starthead(const X3DAttributes &attr);
  int endhead();

  int starthumanoidBodyType(const X3DAttributes &attr);
  int endhumanoidBodyType();

  int startmeta(const X3DAttributes &attr);
  int endmeta();

  int startCADAssembly(const X3DAttributes &attr);
  int endCADAssembly();

  int startCADFace(const X3DAttributes &attr);
  int endCADFace();

  int startCADLayer(const X3DAttributes &attr);
  int endCADLayer();

  int startCADPart(const X3DAttributes &attr);
  int endCADPart();

  int startComposedCubeMapTexture(const X3DAttributes &attr);
  int endComposedCubeMapTexture();

  int startComposedShader(const X3DAttributes &attr);
  int endComposedShader();

  int startComposedTexture3D(const X3DAttributes &attr);
  int endComposedTexture3D();

  int startFloatVertexAttribute(const X3DAttributes &attr);
  int endFloatVertexAttribute();

  int startFogCoordinate(const X3DAttributes &attr);
  int endFogCoordinate();

  int startGeneratedCubeMapTexture(const X3DAttributes &attr);
  int endGeneratedCubeMapTexture();

  int startImageCubeMapTexture(const X3DAttributes &attr);
  int endImageCubeMapTexture();

  int startImageTexture3D(const X3DAttributes &attr);
  int endImageTexture3D();

  int startIndexedQuadSet(const X3DAttributes &attr);
  int endIndexedQuadSet();

  int startLocalFog(const X3DAttributes &attr);
  int endLocalFog();

  int startMatrix3VertexAttribute(const X3DAttributes &attr);
  int endMatrix3VertexAttribute();

  int startMatrix4VertexAttribute(const X3DAttributes &attr);
  int endMatrix4VertexAttribute();

  int startPackagedShader(const X3DAttributes &attr);
  int endPackagedShader();

  int startPixelTexture3D(const X3DAttributes &attr);
  int endPixelTexture3D();

  int startProgramShader(const X3DAttributes &attr);
  int endProgramShader();

  int startQuadSet(const X3DAttributes &attr);
  int endQuadSet();

  int startShaderPart(const X3DAttributes &attr);
  int endShaderPart();

  int startShaderProgram(const X3DAttributes &attr);
  int endShaderProgram();

  int startTextureCoordinate3D(const X3DAttributes &attr);
  int endTextureCoordinate3D();

  int startTextureCoordinate4D(const X3DAttributes &attr);
  int endTextureCoordinate4D();

  int startTextureTransform3D(const X3DAttributes &attr);
  int endTextureTransform3D();

  int startTextureTransformMatrix3D(const X3DAttributes &attr);
  int endTextureTransformMatrix3D();

  int startUnknown(const char* nodeName, const X3DAttributes &attr);
  int endUnknown(int id, const char* nodeName);

  private:
  
	  int iCounter;
	  FILE *fp;

};

#endif

