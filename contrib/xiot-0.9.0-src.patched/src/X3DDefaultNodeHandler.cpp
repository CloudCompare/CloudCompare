#include <xiot/X3DDefaultNodeHandler.h>


namespace XIOT {

X3DDefaultNodeHandler::X3DDefaultNodeHandler()
{
}

X3DDefaultNodeHandler::~X3DDefaultNodeHandler()
{
}

void X3DDefaultNodeHandler::startDocument()
{
	// Do nothing
}

void X3DDefaultNodeHandler::endDocument()
{
	// Do nothing
}


// START GENERATED
int X3DDefaultNodeHandler::startShape(const X3DAttributes &attr) {
  return startUnhandled("Shape", attr);
}

int X3DDefaultNodeHandler::endShape() {
  return endUnhandled("Shape");
}

int X3DDefaultNodeHandler::startAppearance(const X3DAttributes &attr) {
  return startUnhandled("Appearance", attr);
}

int X3DDefaultNodeHandler::endAppearance() {
  return endUnhandled("Appearance");
}

int X3DDefaultNodeHandler::startMaterial(const X3DAttributes &attr) {
  return startUnhandled("Material", attr);
}

int X3DDefaultNodeHandler::endMaterial() {
  return endUnhandled("Material");
}

int X3DDefaultNodeHandler::startIndexedFaceSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedFaceSet", attr);
}

int X3DDefaultNodeHandler::endIndexedFaceSet() {
  return endUnhandled("IndexedFaceSet");
}

int X3DDefaultNodeHandler::startProtoInstance(const X3DAttributes &attr) {
  return startUnhandled("ProtoInstance", attr);
}

int X3DDefaultNodeHandler::endProtoInstance() {
  return endUnhandled("ProtoInstance");
}

int X3DDefaultNodeHandler::startTransform(const X3DAttributes &attr) {
  return startUnhandled("Transform", attr);
}

int X3DDefaultNodeHandler::endTransform() {
  return endUnhandled("Transform");
}

int X3DDefaultNodeHandler::startImageTexture(const X3DAttributes &attr) {
  return startUnhandled("ImageTexture", attr);
}

int X3DDefaultNodeHandler::endImageTexture() {
  return endUnhandled("ImageTexture");
}

int X3DDefaultNodeHandler::startTextureTransform(const X3DAttributes &attr) {
  return startUnhandled("TextureTransform", attr);
}

int X3DDefaultNodeHandler::endTextureTransform() {
  return endUnhandled("TextureTransform");
}

int X3DDefaultNodeHandler::startCoordinate(const X3DAttributes &attr) {
  return startUnhandled("Coordinate", attr);
}

int X3DDefaultNodeHandler::endCoordinate() {
  return endUnhandled("Coordinate");
}

int X3DDefaultNodeHandler::startNormal(const X3DAttributes &attr) {
  return startUnhandled("Normal", attr);
}

int X3DDefaultNodeHandler::endNormal() {
  return endUnhandled("Normal");
}

int X3DDefaultNodeHandler::startColor(const X3DAttributes &attr) {
  return startUnhandled("Color", attr);
}

int X3DDefaultNodeHandler::endColor() {
  return endUnhandled("Color");
}

int X3DDefaultNodeHandler::startColorRGBA(const X3DAttributes &attr) {
  return startUnhandled("ColorRGBA", attr);
}

int X3DDefaultNodeHandler::endColorRGBA() {
  return endUnhandled("ColorRGBA");
}

int X3DDefaultNodeHandler::startTextureCoordinate(const X3DAttributes &attr) {
  return startUnhandled("TextureCoordinate", attr);
}

int X3DDefaultNodeHandler::endTextureCoordinate() {
  return endUnhandled("TextureCoordinate");
}

int X3DDefaultNodeHandler::startROUTE(const X3DAttributes &attr) {
  return startUnhandled("ROUTE", attr);
}

int X3DDefaultNodeHandler::endROUTE() {
  return endUnhandled("ROUTE");
}

int X3DDefaultNodeHandler::startfieldValue(const X3DAttributes &attr) {
  return startUnhandled("fieldValue", attr);
}

int X3DDefaultNodeHandler::endfieldValue() {
  return endUnhandled("fieldValue");
}

int X3DDefaultNodeHandler::startGroup(const X3DAttributes &attr) {
  return startUnhandled("Group", attr);
}

int X3DDefaultNodeHandler::endGroup() {
  return endUnhandled("Group");
}

int X3DDefaultNodeHandler::startLOD(const X3DAttributes &attr) {
  return startUnhandled("LOD", attr);
}

int X3DDefaultNodeHandler::endLOD() {
  return endUnhandled("LOD");
}

int X3DDefaultNodeHandler::startSwitch(const X3DAttributes &attr) {
  return startUnhandled("Switch", attr);
}

int X3DDefaultNodeHandler::endSwitch() {
  return endUnhandled("Switch");
}

int X3DDefaultNodeHandler::startScript(const X3DAttributes &attr) {
  return startUnhandled("Script", attr);
}

int X3DDefaultNodeHandler::endScript() {
  return endUnhandled("Script");
}

int X3DDefaultNodeHandler::startIndexedTriangleFanSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedTriangleFanSet", attr);
}

int X3DDefaultNodeHandler::endIndexedTriangleFanSet() {
  return endUnhandled("IndexedTriangleFanSet");
}

int X3DDefaultNodeHandler::startIndexedTriangleSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedTriangleSet", attr);
}

int X3DDefaultNodeHandler::endIndexedTriangleSet() {
  return endUnhandled("IndexedTriangleSet");
}

int X3DDefaultNodeHandler::startIndexedTriangleStripSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedTriangleStripSet", attr);
}

int X3DDefaultNodeHandler::endIndexedTriangleStripSet() {
  return endUnhandled("IndexedTriangleStripSet");
}

int X3DDefaultNodeHandler::startMultiTexture(const X3DAttributes &attr) {
  return startUnhandled("MultiTexture", attr);
}

int X3DDefaultNodeHandler::endMultiTexture() {
  return endUnhandled("MultiTexture");
}

int X3DDefaultNodeHandler::startMultiTextureCoordinate(const X3DAttributes &attr) {
  return startUnhandled("MultiTextureCoordinate", attr);
}

int X3DDefaultNodeHandler::endMultiTextureCoordinate() {
  return endUnhandled("MultiTextureCoordinate");
}

int X3DDefaultNodeHandler::startMultiTextureTransform(const X3DAttributes &attr) {
  return startUnhandled("MultiTextureTransform", attr);
}

int X3DDefaultNodeHandler::endMultiTextureTransform() {
  return endUnhandled("MultiTextureTransform");
}

int X3DDefaultNodeHandler::startIndexedLineSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedLineSet", attr);
}

int X3DDefaultNodeHandler::endIndexedLineSet() {
  return endUnhandled("IndexedLineSet");
}

int X3DDefaultNodeHandler::startPointSet(const X3DAttributes &attr) {
  return startUnhandled("PointSet", attr);
}

int X3DDefaultNodeHandler::endPointSet() {
  return endUnhandled("PointSet");
}

int X3DDefaultNodeHandler::startStaticGroup(const X3DAttributes &attr) {
  return startUnhandled("StaticGroup", attr);
}

int X3DDefaultNodeHandler::endStaticGroup() {
  return endUnhandled("StaticGroup");
}

int X3DDefaultNodeHandler::startSphere(const X3DAttributes &attr) {
  return startUnhandled("Sphere", attr);
}

int X3DDefaultNodeHandler::endSphere() {
  return endUnhandled("Sphere");
}

int X3DDefaultNodeHandler::startBox(const X3DAttributes &attr) {
  return startUnhandled("Box", attr);
}

int X3DDefaultNodeHandler::endBox() {
  return endUnhandled("Box");
}

int X3DDefaultNodeHandler::startCone(const X3DAttributes &attr) {
  return startUnhandled("Cone", attr);
}

int X3DDefaultNodeHandler::endCone() {
  return endUnhandled("Cone");
}

int X3DDefaultNodeHandler::startAnchor(const X3DAttributes &attr) {
  return startUnhandled("Anchor", attr);
}

int X3DDefaultNodeHandler::endAnchor() {
  return endUnhandled("Anchor");
}

int X3DDefaultNodeHandler::startArc2D(const X3DAttributes &attr) {
  return startUnhandled("Arc2D", attr);
}

int X3DDefaultNodeHandler::endArc2D() {
  return endUnhandled("Arc2D");
}

int X3DDefaultNodeHandler::startArcClose2D(const X3DAttributes &attr) {
  return startUnhandled("ArcClose2D", attr);
}

int X3DDefaultNodeHandler::endArcClose2D() {
  return endUnhandled("ArcClose2D");
}

int X3DDefaultNodeHandler::startAudioClip(const X3DAttributes &attr) {
  return startUnhandled("AudioClip", attr);
}

int X3DDefaultNodeHandler::endAudioClip() {
  return endUnhandled("AudioClip");
}

int X3DDefaultNodeHandler::startBackground(const X3DAttributes &attr) {
  return startUnhandled("Background", attr);
}

int X3DDefaultNodeHandler::endBackground() {
  return endUnhandled("Background");
}

int X3DDefaultNodeHandler::startBillboard(const X3DAttributes &attr) {
  return startUnhandled("Billboard", attr);
}

int X3DDefaultNodeHandler::endBillboard() {
  return endUnhandled("Billboard");
}

int X3DDefaultNodeHandler::startBooleanFilter(const X3DAttributes &attr) {
  return startUnhandled("BooleanFilter", attr);
}

int X3DDefaultNodeHandler::endBooleanFilter() {
  return endUnhandled("BooleanFilter");
}

int X3DDefaultNodeHandler::startBooleanSequencer(const X3DAttributes &attr) {
  return startUnhandled("BooleanSequencer", attr);
}

int X3DDefaultNodeHandler::endBooleanSequencer() {
  return endUnhandled("BooleanSequencer");
}

int X3DDefaultNodeHandler::startBooleanToggle(const X3DAttributes &attr) {
  return startUnhandled("BooleanToggle", attr);
}

int X3DDefaultNodeHandler::endBooleanToggle() {
  return endUnhandled("BooleanToggle");
}

int X3DDefaultNodeHandler::startBooleanTrigger(const X3DAttributes &attr) {
  return startUnhandled("BooleanTrigger", attr);
}

int X3DDefaultNodeHandler::endBooleanTrigger() {
  return endUnhandled("BooleanTrigger");
}

int X3DDefaultNodeHandler::startCircle2D(const X3DAttributes &attr) {
  return startUnhandled("Circle2D", attr);
}

int X3DDefaultNodeHandler::endCircle2D() {
  return endUnhandled("Circle2D");
}

int X3DDefaultNodeHandler::startCollision(const X3DAttributes &attr) {
  return startUnhandled("Collision", attr);
}

int X3DDefaultNodeHandler::endCollision() {
  return endUnhandled("Collision");
}

int X3DDefaultNodeHandler::startColorInterpolator(const X3DAttributes &attr) {
  return startUnhandled("ColorInterpolator", attr);
}

int X3DDefaultNodeHandler::endColorInterpolator() {
  return endUnhandled("ColorInterpolator");
}

int X3DDefaultNodeHandler::startContour2D(const X3DAttributes &attr) {
  return startUnhandled("Contour2D", attr);
}

int X3DDefaultNodeHandler::endContour2D() {
  return endUnhandled("Contour2D");
}

int X3DDefaultNodeHandler::startContourPolyline2D(const X3DAttributes &attr) {
  return startUnhandled("ContourPolyline2D", attr);
}

int X3DDefaultNodeHandler::endContourPolyline2D() {
  return endUnhandled("ContourPolyline2D");
}

int X3DDefaultNodeHandler::startCoordinateDouble(const X3DAttributes &attr) {
  return startUnhandled("CoordinateDouble", attr);
}

int X3DDefaultNodeHandler::endCoordinateDouble() {
  return endUnhandled("CoordinateDouble");
}

int X3DDefaultNodeHandler::startCoordinateInterpolator(const X3DAttributes &attr) {
  return startUnhandled("CoordinateInterpolator", attr);
}

int X3DDefaultNodeHandler::endCoordinateInterpolator() {
  return endUnhandled("CoordinateInterpolator");
}

int X3DDefaultNodeHandler::startCoordinateInterpolator2D(const X3DAttributes &attr) {
  return startUnhandled("CoordinateInterpolator2D", attr);
}

int X3DDefaultNodeHandler::endCoordinateInterpolator2D() {
  return endUnhandled("CoordinateInterpolator2D");
}

int X3DDefaultNodeHandler::startCylinder(const X3DAttributes &attr) {
  return startUnhandled("Cylinder", attr);
}

int X3DDefaultNodeHandler::endCylinder() {
  return endUnhandled("Cylinder");
}

int X3DDefaultNodeHandler::startCylinderSensor(const X3DAttributes &attr) {
  return startUnhandled("CylinderSensor", attr);
}

int X3DDefaultNodeHandler::endCylinderSensor() {
  return endUnhandled("CylinderSensor");
}

int X3DDefaultNodeHandler::startDirectionalLight(const X3DAttributes &attr) {
  return startUnhandled("DirectionalLight", attr);
}

int X3DDefaultNodeHandler::endDirectionalLight() {
  return endUnhandled("DirectionalLight");
}

int X3DDefaultNodeHandler::startDisk2D(const X3DAttributes &attr) {
  return startUnhandled("Disk2D", attr);
}

int X3DDefaultNodeHandler::endDisk2D() {
  return endUnhandled("Disk2D");
}

int X3DDefaultNodeHandler::startEXPORT(const X3DAttributes &attr) {
  return startUnhandled("EXPORT", attr);
}

int X3DDefaultNodeHandler::endEXPORT() {
  return endUnhandled("EXPORT");
}

int X3DDefaultNodeHandler::startElevationGrid(const X3DAttributes &attr) {
  return startUnhandled("ElevationGrid", attr);
}

int X3DDefaultNodeHandler::endElevationGrid() {
  return endUnhandled("ElevationGrid");
}

int X3DDefaultNodeHandler::startEspduTransform(const X3DAttributes &attr) {
  return startUnhandled("EspduTransform", attr);
}

int X3DDefaultNodeHandler::endEspduTransform() {
  return endUnhandled("EspduTransform");
}

int X3DDefaultNodeHandler::startExternProtoDeclare(const X3DAttributes &attr) {
  return startUnhandled("ExternProtoDeclare", attr);
}

int X3DDefaultNodeHandler::endExternProtoDeclare() {
  return endUnhandled("ExternProtoDeclare");
}

int X3DDefaultNodeHandler::startExtrusion(const X3DAttributes &attr) {
  return startUnhandled("Extrusion", attr);
}

int X3DDefaultNodeHandler::endExtrusion() {
  return endUnhandled("Extrusion");
}

int X3DDefaultNodeHandler::startFillProperties(const X3DAttributes &attr) {
  return startUnhandled("FillProperties", attr);
}

int X3DDefaultNodeHandler::endFillProperties() {
  return endUnhandled("FillProperties");
}

int X3DDefaultNodeHandler::startFog(const X3DAttributes &attr) {
  return startUnhandled("Fog", attr);
}

int X3DDefaultNodeHandler::endFog() {
  return endUnhandled("Fog");
}

int X3DDefaultNodeHandler::startFontStyle(const X3DAttributes &attr) {
  return startUnhandled("FontStyle", attr);
}

int X3DDefaultNodeHandler::endFontStyle() {
  return endUnhandled("FontStyle");
}

int X3DDefaultNodeHandler::startGeoCoordinate(const X3DAttributes &attr) {
  return startUnhandled("GeoCoordinate", attr);
}

int X3DDefaultNodeHandler::endGeoCoordinate() {
  return endUnhandled("GeoCoordinate");
}

int X3DDefaultNodeHandler::startGeoElevationGrid(const X3DAttributes &attr) {
  return startUnhandled("GeoElevationGrid", attr);
}

int X3DDefaultNodeHandler::endGeoElevationGrid() {
  return endUnhandled("GeoElevationGrid");
}

int X3DDefaultNodeHandler::startGeoLOD(const X3DAttributes &attr) {
  return startUnhandled("GeoLOD", attr);
}

int X3DDefaultNodeHandler::endGeoLOD() {
  return endUnhandled("GeoLOD");
}

int X3DDefaultNodeHandler::startGeoLocation(const X3DAttributes &attr) {
  return startUnhandled("GeoLocation", attr);
}

int X3DDefaultNodeHandler::endGeoLocation() {
  return endUnhandled("GeoLocation");
}

int X3DDefaultNodeHandler::startGeoMetadata(const X3DAttributes &attr) {
  return startUnhandled("GeoMetadata", attr);
}

int X3DDefaultNodeHandler::endGeoMetadata() {
  return endUnhandled("GeoMetadata");
}

int X3DDefaultNodeHandler::startGeoOrigin(const X3DAttributes &attr) {
  return startUnhandled("GeoOrigin", attr);
}

int X3DDefaultNodeHandler::endGeoOrigin() {
  return endUnhandled("GeoOrigin");
}

int X3DDefaultNodeHandler::startGeoPositionInterpolator(const X3DAttributes &attr) {
  return startUnhandled("GeoPositionInterpolator", attr);
}

int X3DDefaultNodeHandler::endGeoPositionInterpolator() {
  return endUnhandled("GeoPositionInterpolator");
}

int X3DDefaultNodeHandler::startGeoTouchSensor(const X3DAttributes &attr) {
  return startUnhandled("GeoTouchSensor", attr);
}

int X3DDefaultNodeHandler::endGeoTouchSensor() {
  return endUnhandled("GeoTouchSensor");
}

int X3DDefaultNodeHandler::startGeoViewpoint(const X3DAttributes &attr) {
  return startUnhandled("GeoViewpoint", attr);
}

int X3DDefaultNodeHandler::endGeoViewpoint() {
  return endUnhandled("GeoViewpoint");
}

int X3DDefaultNodeHandler::startHAnimDisplacer(const X3DAttributes &attr) {
  return startUnhandled("HAnimDisplacer", attr);
}

int X3DDefaultNodeHandler::endHAnimDisplacer() {
  return endUnhandled("HAnimDisplacer");
}

int X3DDefaultNodeHandler::startHAnimHumanoid(const X3DAttributes &attr) {
  return startUnhandled("HAnimHumanoid", attr);
}

int X3DDefaultNodeHandler::endHAnimHumanoid() {
  return endUnhandled("HAnimHumanoid");
}

int X3DDefaultNodeHandler::startHAnimJoint(const X3DAttributes &attr) {
  return startUnhandled("HAnimJoint", attr);
}

int X3DDefaultNodeHandler::endHAnimJoint() {
  return endUnhandled("HAnimJoint");
}

int X3DDefaultNodeHandler::startHAnimSegment(const X3DAttributes &attr) {
  return startUnhandled("HAnimSegment", attr);
}

int X3DDefaultNodeHandler::endHAnimSegment() {
  return endUnhandled("HAnimSegment");
}

int X3DDefaultNodeHandler::startHAnimSite(const X3DAttributes &attr) {
  return startUnhandled("HAnimSite", attr);
}

int X3DDefaultNodeHandler::endHAnimSite() {
  return endUnhandled("HAnimSite");
}

int X3DDefaultNodeHandler::startIMPORT(const X3DAttributes &attr) {
  return startUnhandled("IMPORT", attr);
}

int X3DDefaultNodeHandler::endIMPORT() {
  return endUnhandled("IMPORT");
}

int X3DDefaultNodeHandler::startIS(const X3DAttributes &attr) {
  return startUnhandled("IS", attr);
}

int X3DDefaultNodeHandler::endIS() {
  return endUnhandled("IS");
}

int X3DDefaultNodeHandler::startInline(const X3DAttributes &attr) {
  return startUnhandled("Inline", attr);
}

int X3DDefaultNodeHandler::endInline() {
  return endUnhandled("Inline");
}

int X3DDefaultNodeHandler::startIntegerSequencer(const X3DAttributes &attr) {
  return startUnhandled("IntegerSequencer", attr);
}

int X3DDefaultNodeHandler::endIntegerSequencer() {
  return endUnhandled("IntegerSequencer");
}

int X3DDefaultNodeHandler::startIntegerTrigger(const X3DAttributes &attr) {
  return startUnhandled("IntegerTrigger", attr);
}

int X3DDefaultNodeHandler::endIntegerTrigger() {
  return endUnhandled("IntegerTrigger");
}

int X3DDefaultNodeHandler::startKeySensor(const X3DAttributes &attr) {
  return startUnhandled("KeySensor", attr);
}

int X3DDefaultNodeHandler::endKeySensor() {
  return endUnhandled("KeySensor");
}

int X3DDefaultNodeHandler::startLineProperties(const X3DAttributes &attr) {
  return startUnhandled("LineProperties", attr);
}

int X3DDefaultNodeHandler::endLineProperties() {
  return endUnhandled("LineProperties");
}

int X3DDefaultNodeHandler::startLineSet(const X3DAttributes &attr) {
  return startUnhandled("LineSet", attr);
}

int X3DDefaultNodeHandler::endLineSet() {
  return endUnhandled("LineSet");
}

int X3DDefaultNodeHandler::startLoadSensor(const X3DAttributes &attr) {
  return startUnhandled("LoadSensor", attr);
}

int X3DDefaultNodeHandler::endLoadSensor() {
  return endUnhandled("LoadSensor");
}

int X3DDefaultNodeHandler::startMetadataDouble(const X3DAttributes &attr) {
  return startUnhandled("MetadataDouble", attr);
}

int X3DDefaultNodeHandler::endMetadataDouble() {
  return endUnhandled("MetadataDouble");
}

int X3DDefaultNodeHandler::startMetadataFloat(const X3DAttributes &attr) {
  return startUnhandled("MetadataFloat", attr);
}

int X3DDefaultNodeHandler::endMetadataFloat() {
  return endUnhandled("MetadataFloat");
}

int X3DDefaultNodeHandler::startMetadataInteger(const X3DAttributes &attr) {
  return startUnhandled("MetadataInteger", attr);
}

int X3DDefaultNodeHandler::endMetadataInteger() {
  return endUnhandled("MetadataInteger");
}

int X3DDefaultNodeHandler::startMetadataSet(const X3DAttributes &attr) {
  return startUnhandled("MetadataSet", attr);
}

int X3DDefaultNodeHandler::endMetadataSet() {
  return endUnhandled("MetadataSet");
}

int X3DDefaultNodeHandler::startMetadataString(const X3DAttributes &attr) {
  return startUnhandled("MetadataString", attr);
}

int X3DDefaultNodeHandler::endMetadataString() {
  return endUnhandled("MetadataString");
}

int X3DDefaultNodeHandler::startMovieTexture(const X3DAttributes &attr) {
  return startUnhandled("MovieTexture", attr);
}

int X3DDefaultNodeHandler::endMovieTexture() {
  return endUnhandled("MovieTexture");
}

int X3DDefaultNodeHandler::startNavigationInfo(const X3DAttributes &attr) {
  return startUnhandled("NavigationInfo", attr);
}

int X3DDefaultNodeHandler::endNavigationInfo() {
  return endUnhandled("NavigationInfo");
}

int X3DDefaultNodeHandler::startNormalInterpolator(const X3DAttributes &attr) {
  return startUnhandled("NormalInterpolator", attr);
}

int X3DDefaultNodeHandler::endNormalInterpolator() {
  return endUnhandled("NormalInterpolator");
}

int X3DDefaultNodeHandler::startNurbsCurve(const X3DAttributes &attr) {
  return startUnhandled("NurbsCurve", attr);
}

int X3DDefaultNodeHandler::endNurbsCurve() {
  return endUnhandled("NurbsCurve");
}

int X3DDefaultNodeHandler::startNurbsCurve2D(const X3DAttributes &attr) {
  return startUnhandled("NurbsCurve2D", attr);
}

int X3DDefaultNodeHandler::endNurbsCurve2D() {
  return endUnhandled("NurbsCurve2D");
}

int X3DDefaultNodeHandler::startNurbsOrientationInterpolator(const X3DAttributes &attr) {
  return startUnhandled("NurbsOrientationInterpolator", attr);
}

int X3DDefaultNodeHandler::endNurbsOrientationInterpolator() {
  return endUnhandled("NurbsOrientationInterpolator");
}

int X3DDefaultNodeHandler::startNurbsPatchSurface(const X3DAttributes &attr) {
  return startUnhandled("NurbsPatchSurface", attr);
}

int X3DDefaultNodeHandler::endNurbsPatchSurface() {
  return endUnhandled("NurbsPatchSurface");
}

int X3DDefaultNodeHandler::startNurbsPositionInterpolator(const X3DAttributes &attr) {
  return startUnhandled("NurbsPositionInterpolator", attr);
}

int X3DDefaultNodeHandler::endNurbsPositionInterpolator() {
  return endUnhandled("NurbsPositionInterpolator");
}

int X3DDefaultNodeHandler::startNurbsSet(const X3DAttributes &attr) {
  return startUnhandled("NurbsSet", attr);
}

int X3DDefaultNodeHandler::endNurbsSet() {
  return endUnhandled("NurbsSet");
}

int X3DDefaultNodeHandler::startNurbsSurfaceInterpolator(const X3DAttributes &attr) {
  return startUnhandled("NurbsSurfaceInterpolator", attr);
}

int X3DDefaultNodeHandler::endNurbsSurfaceInterpolator() {
  return endUnhandled("NurbsSurfaceInterpolator");
}

int X3DDefaultNodeHandler::startNurbsSweptSurface(const X3DAttributes &attr) {
  return startUnhandled("NurbsSweptSurface", attr);
}

int X3DDefaultNodeHandler::endNurbsSweptSurface() {
  return endUnhandled("NurbsSweptSurface");
}

int X3DDefaultNodeHandler::startNurbsSwungSurface(const X3DAttributes &attr) {
  return startUnhandled("NurbsSwungSurface", attr);
}

int X3DDefaultNodeHandler::endNurbsSwungSurface() {
  return endUnhandled("NurbsSwungSurface");
}

int X3DDefaultNodeHandler::startNurbsTextureCoordinate(const X3DAttributes &attr) {
  return startUnhandled("NurbsTextureCoordinate", attr);
}

int X3DDefaultNodeHandler::endNurbsTextureCoordinate() {
  return endUnhandled("NurbsTextureCoordinate");
}

int X3DDefaultNodeHandler::startNurbsTrimmedSurface(const X3DAttributes &attr) {
  return startUnhandled("NurbsTrimmedSurface", attr);
}

int X3DDefaultNodeHandler::endNurbsTrimmedSurface() {
  return endUnhandled("NurbsTrimmedSurface");
}

int X3DDefaultNodeHandler::startOrientationInterpolator(const X3DAttributes &attr) {
  return startUnhandled("OrientationInterpolator", attr);
}

int X3DDefaultNodeHandler::endOrientationInterpolator() {
  return endUnhandled("OrientationInterpolator");
}

int X3DDefaultNodeHandler::startPixelTexture(const X3DAttributes &attr) {
  return startUnhandled("PixelTexture", attr);
}

int X3DDefaultNodeHandler::endPixelTexture() {
  return endUnhandled("PixelTexture");
}

int X3DDefaultNodeHandler::startPlaneSensor(const X3DAttributes &attr) {
  return startUnhandled("PlaneSensor", attr);
}

int X3DDefaultNodeHandler::endPlaneSensor() {
  return endUnhandled("PlaneSensor");
}

int X3DDefaultNodeHandler::startPointLight(const X3DAttributes &attr) {
  return startUnhandled("PointLight", attr);
}

int X3DDefaultNodeHandler::endPointLight() {
  return endUnhandled("PointLight");
}

int X3DDefaultNodeHandler::startPolyline2D(const X3DAttributes &attr) {
  return startUnhandled("Polyline2D", attr);
}

int X3DDefaultNodeHandler::endPolyline2D() {
  return endUnhandled("Polyline2D");
}

int X3DDefaultNodeHandler::startPolypoint2D(const X3DAttributes &attr) {
  return startUnhandled("Polypoint2D", attr);
}

int X3DDefaultNodeHandler::endPolypoint2D() {
  return endUnhandled("Polypoint2D");
}

int X3DDefaultNodeHandler::startPositionInterpolator(const X3DAttributes &attr) {
  return startUnhandled("PositionInterpolator", attr);
}

int X3DDefaultNodeHandler::endPositionInterpolator() {
  return endUnhandled("PositionInterpolator");
}

int X3DDefaultNodeHandler::startPositionInterpolator2D(const X3DAttributes &attr) {
  return startUnhandled("PositionInterpolator2D", attr);
}

int X3DDefaultNodeHandler::endPositionInterpolator2D() {
  return endUnhandled("PositionInterpolator2D");
}

int X3DDefaultNodeHandler::startProtoBody(const X3DAttributes &attr) {
  return startUnhandled("ProtoBody", attr);
}

int X3DDefaultNodeHandler::endProtoBody() {
  return endUnhandled("ProtoBody");
}

int X3DDefaultNodeHandler::startProtoDeclare(const X3DAttributes &attr) {
  return startUnhandled("ProtoDeclare", attr);
}

int X3DDefaultNodeHandler::endProtoDeclare() {
  return endUnhandled("ProtoDeclare");
}

int X3DDefaultNodeHandler::startProtoInterface(const X3DAttributes &attr) {
  return startUnhandled("ProtoInterface", attr);
}

int X3DDefaultNodeHandler::endProtoInterface() {
  return endUnhandled("ProtoInterface");
}

int X3DDefaultNodeHandler::startProximitySensor(const X3DAttributes &attr) {
  return startUnhandled("ProximitySensor", attr);
}

int X3DDefaultNodeHandler::endProximitySensor() {
  return endUnhandled("ProximitySensor");
}

int X3DDefaultNodeHandler::startReceiverPdu(const X3DAttributes &attr) {
  return startUnhandled("ReceiverPdu", attr);
}

int X3DDefaultNodeHandler::endReceiverPdu() {
  return endUnhandled("ReceiverPdu");
}

int X3DDefaultNodeHandler::startRectangle2D(const X3DAttributes &attr) {
  return startUnhandled("Rectangle2D", attr);
}

int X3DDefaultNodeHandler::endRectangle2D() {
  return endUnhandled("Rectangle2D");
}

int X3DDefaultNodeHandler::startScalarInterpolator(const X3DAttributes &attr) {
  return startUnhandled("ScalarInterpolator", attr);
}

int X3DDefaultNodeHandler::endScalarInterpolator() {
  return endUnhandled("ScalarInterpolator");
}

int X3DDefaultNodeHandler::startScene(const X3DAttributes &attr) {
  return startUnhandled("Scene", attr);
}

int X3DDefaultNodeHandler::endScene() {
  return endUnhandled("Scene");
}

int X3DDefaultNodeHandler::startSignalPdu(const X3DAttributes &attr) {
  return startUnhandled("SignalPdu", attr);
}

int X3DDefaultNodeHandler::endSignalPdu() {
  return endUnhandled("SignalPdu");
}

int X3DDefaultNodeHandler::startSound(const X3DAttributes &attr) {
  return startUnhandled("Sound", attr);
}

int X3DDefaultNodeHandler::endSound() {
  return endUnhandled("Sound");
}

int X3DDefaultNodeHandler::startSphereSensor(const X3DAttributes &attr) {
  return startUnhandled("SphereSensor", attr);
}

int X3DDefaultNodeHandler::endSphereSensor() {
  return endUnhandled("SphereSensor");
}

int X3DDefaultNodeHandler::startSpotLight(const X3DAttributes &attr) {
  return startUnhandled("SpotLight", attr);
}

int X3DDefaultNodeHandler::endSpotLight() {
  return endUnhandled("SpotLight");
}

int X3DDefaultNodeHandler::startStringSensor(const X3DAttributes &attr) {
  return startUnhandled("StringSensor", attr);
}

int X3DDefaultNodeHandler::endStringSensor() {
  return endUnhandled("StringSensor");
}

int X3DDefaultNodeHandler::startText(const X3DAttributes &attr) {
  return startUnhandled("Text", attr);
}

int X3DDefaultNodeHandler::endText() {
  return endUnhandled("Text");
}

int X3DDefaultNodeHandler::startTextureBackground(const X3DAttributes &attr) {
  return startUnhandled("TextureBackground", attr);
}

int X3DDefaultNodeHandler::endTextureBackground() {
  return endUnhandled("TextureBackground");
}

int X3DDefaultNodeHandler::startTextureCoordinateGenerator(const X3DAttributes &attr) {
  return startUnhandled("TextureCoordinateGenerator", attr);
}

int X3DDefaultNodeHandler::endTextureCoordinateGenerator() {
  return endUnhandled("TextureCoordinateGenerator");
}

int X3DDefaultNodeHandler::startTimeSensor(const X3DAttributes &attr) {
  return startUnhandled("TimeSensor", attr);
}

int X3DDefaultNodeHandler::endTimeSensor() {
  return endUnhandled("TimeSensor");
}

int X3DDefaultNodeHandler::startTimeTrigger(const X3DAttributes &attr) {
  return startUnhandled("TimeTrigger", attr);
}

int X3DDefaultNodeHandler::endTimeTrigger() {
  return endUnhandled("TimeTrigger");
}

int X3DDefaultNodeHandler::startTouchSensor(const X3DAttributes &attr) {
  return startUnhandled("TouchSensor", attr);
}

int X3DDefaultNodeHandler::endTouchSensor() {
  return endUnhandled("TouchSensor");
}

int X3DDefaultNodeHandler::startTransmitterPdu(const X3DAttributes &attr) {
  return startUnhandled("TransmitterPdu", attr);
}

int X3DDefaultNodeHandler::endTransmitterPdu() {
  return endUnhandled("TransmitterPdu");
}

int X3DDefaultNodeHandler::startTriangleFanSet(const X3DAttributes &attr) {
  return startUnhandled("TriangleFanSet", attr);
}

int X3DDefaultNodeHandler::endTriangleFanSet() {
  return endUnhandled("TriangleFanSet");
}

int X3DDefaultNodeHandler::startTriangleSet(const X3DAttributes &attr) {
  return startUnhandled("TriangleSet", attr);
}

int X3DDefaultNodeHandler::endTriangleSet() {
  return endUnhandled("TriangleSet");
}

int X3DDefaultNodeHandler::startTriangleSet2D(const X3DAttributes &attr) {
  return startUnhandled("TriangleSet2D", attr);
}

int X3DDefaultNodeHandler::endTriangleSet2D() {
  return endUnhandled("TriangleSet2D");
}

int X3DDefaultNodeHandler::startTriangleStripSet(const X3DAttributes &attr) {
  return startUnhandled("TriangleStripSet", attr);
}

int X3DDefaultNodeHandler::endTriangleStripSet() {
  return endUnhandled("TriangleStripSet");
}

int X3DDefaultNodeHandler::startViewpoint(const X3DAttributes &attr) {
  return startUnhandled("Viewpoint", attr);
}

int X3DDefaultNodeHandler::endViewpoint() {
  return endUnhandled("Viewpoint");
}

int X3DDefaultNodeHandler::startVisibilitySensor(const X3DAttributes &attr) {
  return startUnhandled("VisibilitySensor", attr);
}

int X3DDefaultNodeHandler::endVisibilitySensor() {
  return endUnhandled("VisibilitySensor");
}

int X3DDefaultNodeHandler::startWorldInfo(const X3DAttributes &attr) {
  return startUnhandled("WorldInfo", attr);
}

int X3DDefaultNodeHandler::endWorldInfo() {
  return endUnhandled("WorldInfo");
}

int X3DDefaultNodeHandler::startX3D(const X3DAttributes &attr) {
  return startUnhandled("X3D", attr);
}

int X3DDefaultNodeHandler::endX3D() {
  return endUnhandled("X3D");
}

int X3DDefaultNodeHandler::startcomponent(const X3DAttributes &attr) {
  return startUnhandled("component", attr);
}

int X3DDefaultNodeHandler::endcomponent() {
  return endUnhandled("component");
}

int X3DDefaultNodeHandler::startconnect(const X3DAttributes &attr) {
  return startUnhandled("connect", attr);
}

int X3DDefaultNodeHandler::endconnect() {
  return endUnhandled("connect");
}

int X3DDefaultNodeHandler::startfield(const X3DAttributes &attr) {
  return startUnhandled("field", attr);
}

int X3DDefaultNodeHandler::endfield() {
  return endUnhandled("field");
}

int X3DDefaultNodeHandler::starthead(const X3DAttributes &attr) {
  return startUnhandled("head", attr);
}

int X3DDefaultNodeHandler::endhead() {
  return endUnhandled("head");
}

int X3DDefaultNodeHandler::starthumanoidBodyType(const X3DAttributes &attr) {
  return startUnhandled("humanoidBodyType", attr);
}

int X3DDefaultNodeHandler::endhumanoidBodyType() {
  return endUnhandled("humanoidBodyType");
}

int X3DDefaultNodeHandler::startmeta(const X3DAttributes &attr) {
  return startUnhandled("meta", attr);
}

int X3DDefaultNodeHandler::endmeta() {
  return endUnhandled("meta");
}

int X3DDefaultNodeHandler::startCADAssembly(const X3DAttributes &attr) {
  return startUnhandled("CADAssembly", attr);
}

int X3DDefaultNodeHandler::endCADAssembly() {
  return endUnhandled("CADAssembly");
}

int X3DDefaultNodeHandler::startCADFace(const X3DAttributes &attr) {
  return startUnhandled("CADFace", attr);
}

int X3DDefaultNodeHandler::endCADFace() {
  return endUnhandled("CADFace");
}

int X3DDefaultNodeHandler::startCADLayer(const X3DAttributes &attr) {
  return startUnhandled("CADLayer", attr);
}

int X3DDefaultNodeHandler::endCADLayer() {
  return endUnhandled("CADLayer");
}

int X3DDefaultNodeHandler::startCADPart(const X3DAttributes &attr) {
  return startUnhandled("CADPart", attr);
}

int X3DDefaultNodeHandler::endCADPart() {
  return endUnhandled("CADPart");
}

int X3DDefaultNodeHandler::startComposedCubeMapTexture(const X3DAttributes &attr) {
  return startUnhandled("ComposedCubeMapTexture", attr);
}

int X3DDefaultNodeHandler::endComposedCubeMapTexture() {
  return endUnhandled("ComposedCubeMapTexture");
}

int X3DDefaultNodeHandler::startComposedShader(const X3DAttributes &attr) {
  return startUnhandled("ComposedShader", attr);
}

int X3DDefaultNodeHandler::endComposedShader() {
  return endUnhandled("ComposedShader");
}

int X3DDefaultNodeHandler::startComposedTexture3D(const X3DAttributes &attr) {
  return startUnhandled("ComposedTexture3D", attr);
}

int X3DDefaultNodeHandler::endComposedTexture3D() {
  return endUnhandled("ComposedTexture3D");
}

int X3DDefaultNodeHandler::startFloatVertexAttribute(const X3DAttributes &attr) {
  return startUnhandled("FloatVertexAttribute", attr);
}

int X3DDefaultNodeHandler::endFloatVertexAttribute() {
  return endUnhandled("FloatVertexAttribute");
}

int X3DDefaultNodeHandler::startFogCoordinate(const X3DAttributes &attr) {
  return startUnhandled("FogCoordinate", attr);
}

int X3DDefaultNodeHandler::endFogCoordinate() {
  return endUnhandled("FogCoordinate");
}

int X3DDefaultNodeHandler::startGeneratedCubeMapTexture(const X3DAttributes &attr) {
  return startUnhandled("GeneratedCubeMapTexture", attr);
}

int X3DDefaultNodeHandler::endGeneratedCubeMapTexture() {
  return endUnhandled("GeneratedCubeMapTexture");
}

int X3DDefaultNodeHandler::startImageCubeMapTexture(const X3DAttributes &attr) {
  return startUnhandled("ImageCubeMapTexture", attr);
}

int X3DDefaultNodeHandler::endImageCubeMapTexture() {
  return endUnhandled("ImageCubeMapTexture");
}

int X3DDefaultNodeHandler::startImageTexture3D(const X3DAttributes &attr) {
  return startUnhandled("ImageTexture3D", attr);
}

int X3DDefaultNodeHandler::endImageTexture3D() {
  return endUnhandled("ImageTexture3D");
}

int X3DDefaultNodeHandler::startIndexedQuadSet(const X3DAttributes &attr) {
  return startUnhandled("IndexedQuadSet", attr);
}

int X3DDefaultNodeHandler::endIndexedQuadSet() {
  return endUnhandled("IndexedQuadSet");
}

int X3DDefaultNodeHandler::startLocalFog(const X3DAttributes &attr) {
  return startUnhandled("LocalFog", attr);
}

int X3DDefaultNodeHandler::endLocalFog() {
  return endUnhandled("LocalFog");
}

int X3DDefaultNodeHandler::startMatrix3VertexAttribute(const X3DAttributes &attr) {
  return startUnhandled("Matrix3VertexAttribute", attr);
}

int X3DDefaultNodeHandler::endMatrix3VertexAttribute() {
  return endUnhandled("Matrix3VertexAttribute");
}

int X3DDefaultNodeHandler::startMatrix4VertexAttribute(const X3DAttributes &attr) {
  return startUnhandled("Matrix4VertexAttribute", attr);
}

int X3DDefaultNodeHandler::endMatrix4VertexAttribute() {
  return endUnhandled("Matrix4VertexAttribute");
}

int X3DDefaultNodeHandler::startPackagedShader(const X3DAttributes &attr) {
  return startUnhandled("PackagedShader", attr);
}

int X3DDefaultNodeHandler::endPackagedShader() {
  return endUnhandled("PackagedShader");
}

int X3DDefaultNodeHandler::startPixelTexture3D(const X3DAttributes &attr) {
  return startUnhandled("PixelTexture3D", attr);
}

int X3DDefaultNodeHandler::endPixelTexture3D() {
  return endUnhandled("PixelTexture3D");
}

int X3DDefaultNodeHandler::startProgramShader(const X3DAttributes &attr) {
  return startUnhandled("ProgramShader", attr);
}

int X3DDefaultNodeHandler::endProgramShader() {
  return endUnhandled("ProgramShader");
}

int X3DDefaultNodeHandler::startQuadSet(const X3DAttributes &attr) {
  return startUnhandled("QuadSet", attr);
}

int X3DDefaultNodeHandler::endQuadSet() {
  return endUnhandled("QuadSet");
}

int X3DDefaultNodeHandler::startShaderPart(const X3DAttributes &attr) {
  return startUnhandled("ShaderPart", attr);
}

int X3DDefaultNodeHandler::endShaderPart() {
  return endUnhandled("ShaderPart");
}

int X3DDefaultNodeHandler::startShaderProgram(const X3DAttributes &attr) {
  return startUnhandled("ShaderProgram", attr);
}

int X3DDefaultNodeHandler::endShaderProgram() {
  return endUnhandled("ShaderProgram");
}

int X3DDefaultNodeHandler::startTextureCoordinate3D(const X3DAttributes &attr) {
  return startUnhandled("TextureCoordinate3D", attr);
}

int X3DDefaultNodeHandler::endTextureCoordinate3D() {
  return endUnhandled("TextureCoordinate3D");
}

int X3DDefaultNodeHandler::startTextureCoordinate4D(const X3DAttributes &attr) {
  return startUnhandled("TextureCoordinate4D", attr);
}

int X3DDefaultNodeHandler::endTextureCoordinate4D() {
  return endUnhandled("TextureCoordinate4D");
}

int X3DDefaultNodeHandler::startTextureTransform3D(const X3DAttributes &attr) {
  return startUnhandled("TextureTransform3D", attr);
}

int X3DDefaultNodeHandler::endTextureTransform3D() {
  return endUnhandled("TextureTransform3D");
}

int X3DDefaultNodeHandler::startTextureTransformMatrix3D(const X3DAttributes &attr) {
  return startUnhandled("TextureTransformMatrix3D", attr);
}

int X3DDefaultNodeHandler::endTextureTransformMatrix3D() {
  return endUnhandled("TextureTransformMatrix3D");
}

int X3DDefaultNodeHandler::startUnknown(int , const char* nodeName, const X3DAttributes &attr) {
  return startUnhandled(nodeName ? nodeName : "Unknown", attr);
}

int X3DDefaultNodeHandler::endUnknown(int , const char* nodeName) {
  return endUnhandled(nodeName ? nodeName : "Unknown");
}

int X3DDefaultNodeHandler::startUnhandled(const char* , const X3DAttributes &) {
  // do nothing in the default implementation
  return 1;
}

int X3DDefaultNodeHandler::endUnhandled(const char* ) {
  // do nothing in the default implementation
  return 1;
}

// END GENERATED
}

