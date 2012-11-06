#include "X3DLogNodeHandler.h"
#include <xiot/X3DAttributes.h>
#include <cassert>

X3DLogNodeHandler::X3DLogNodeHandler(std::string fileName)
{
	//fileName.erase(fileName.find_last_of("."));
	fileName.append(".log");

	fp = fopen(fileName.c_str(), "w+");

	assert(fp);

	iCounter = 1;
}

X3DLogNodeHandler::~X3DLogNodeHandler()
{
	fclose(fp);
}

std::string X3DLogNodeHandler::getAttributesAsString(const X3DAttributes &attr)
{
  std::string result;
  for (size_t i = 0; i < static_cast<int>(attr.getLength()); i++)
  {
    result.append(attr.getAttributeName(static_cast<int>(i)));
    result.append(" ");
  }
  return result;
}

void X3DLogNodeHandler::startDocument()
{
  fprintf(fp, "Event %4i - Start Document\n", iCounter++);
}

void X3DLogNodeHandler::endDocument()
{
  fprintf(fp, "Event %4i - End Document", iCounter++);
}

int X3DLogNodeHandler::startShape(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Shape with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endShape() {
  fprintf(fp, "Event %4i - End node Shape\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startAppearance(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Appearance with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endAppearance() {
  fprintf(fp, "Event %4i - End node Appearance\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMaterial(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Material with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMaterial() {
  fprintf(fp, "Event %4i - End node Material\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedFaceSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedFaceSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedFaceSet() {
  fprintf(fp, "Event %4i - End node IndexedFaceSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProtoInstance(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProtoInstance with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProtoInstance() {
  fprintf(fp, "Event %4i - End node ProtoInstance\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTransform(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Transform with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTransform() {
  fprintf(fp, "Event %4i - End node Transform\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startImageTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ImageTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endImageTexture() {
  fprintf(fp, "Event %4i - End node ImageTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureTransform(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureTransform with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureTransform() {
  fprintf(fp, "Event %4i - End node TextureTransform\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Coordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCoordinate() {
  fprintf(fp, "Event %4i - End node Coordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNormal(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Normal with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNormal() {
  fprintf(fp, "Event %4i - End node Normal\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startColor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Color with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endColor() {
  fprintf(fp, "Event %4i - End node Color\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startColorRGBA(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ColorRGBA with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endColorRGBA() {
  fprintf(fp, "Event %4i - End node ColorRGBA\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureCoordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureCoordinate() {
  fprintf(fp, "Event %4i - End node TextureCoordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startROUTE(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ROUTE with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endROUTE() {
  fprintf(fp, "Event %4i - End node ROUTE\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startfieldValue(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node fieldValue with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endfieldValue() {
  fprintf(fp, "Event %4i - End node fieldValue\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGroup(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Group with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGroup() {
  fprintf(fp, "Event %4i - End node Group\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startLOD(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node LOD with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endLOD() {
  fprintf(fp, "Event %4i - End node LOD\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSwitch(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Switch with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSwitch() {
  fprintf(fp, "Event %4i - End node Switch\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startScript(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Script with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endScript() {
  fprintf(fp, "Event %4i - End node Script\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedTriangleFanSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedTriangleFanSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedTriangleFanSet() {
  fprintf(fp, "Event %4i - End node IndexedTriangleFanSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedTriangleSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedTriangleSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedTriangleSet() {
  fprintf(fp, "Event %4i - End node IndexedTriangleSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedTriangleStripSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedTriangleStripSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedTriangleStripSet() {
  fprintf(fp, "Event %4i - End node IndexedTriangleStripSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMultiTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MultiTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMultiTexture() {
  fprintf(fp, "Event %4i - End node MultiTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMultiTextureCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MultiTextureCoordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMultiTextureCoordinate() {
  fprintf(fp, "Event %4i - End node MultiTextureCoordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMultiTextureTransform(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MultiTextureTransform with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMultiTextureTransform() {
  fprintf(fp, "Event %4i - End node MultiTextureTransform\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedLineSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedLineSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedLineSet() {
  fprintf(fp, "Event %4i - End node IndexedLineSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPointSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PointSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPointSet() {
  fprintf(fp, "Event %4i - End node PointSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startStaticGroup(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node StaticGroup with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endStaticGroup() {
  fprintf(fp, "Event %4i - End node StaticGroup\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSphere(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Sphere with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSphere() {
  fprintf(fp, "Event %4i - End node Sphere\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBox(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Box with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBox() {
  fprintf(fp, "Event %4i - End node Box\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCone(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Cone with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCone() {
  fprintf(fp, "Event %4i - End node Cone\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startAnchor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Anchor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endAnchor() {
  fprintf(fp, "Event %4i - End node Anchor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startArc2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Arc2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endArc2D() {
  fprintf(fp, "Event %4i - End node Arc2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startArcClose2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ArcClose2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endArcClose2D() {
  fprintf(fp, "Event %4i - End node ArcClose2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startAudioClip(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node AudioClip with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endAudioClip() {
  fprintf(fp, "Event %4i - End node AudioClip\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBackground(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Background with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBackground() {
  fprintf(fp, "Event %4i - End node Background\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBillboard(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Billboard with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBillboard() {
  fprintf(fp, "Event %4i - End node Billboard\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBooleanFilter(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node BooleanFilter with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBooleanFilter() {
  fprintf(fp, "Event %4i - End node BooleanFilter\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBooleanSequencer(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node BooleanSequencer with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBooleanSequencer() {
  fprintf(fp, "Event %4i - End node BooleanSequencer\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBooleanToggle(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node BooleanToggle with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBooleanToggle() {
  fprintf(fp, "Event %4i - End node BooleanToggle\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startBooleanTrigger(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node BooleanTrigger with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endBooleanTrigger() {
  fprintf(fp, "Event %4i - End node BooleanTrigger\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCircle2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Circle2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCircle2D() {
  fprintf(fp, "Event %4i - End node Circle2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCollision(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Collision with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCollision() {
  fprintf(fp, "Event %4i - End node Collision\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startColorInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ColorInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endColorInterpolator() {
  fprintf(fp, "Event %4i - End node ColorInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startContour2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Contour2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endContour2D() {
  fprintf(fp, "Event %4i - End node Contour2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startContourPolyline2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ContourPolyline2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endContourPolyline2D() {
  fprintf(fp, "Event %4i - End node ContourPolyline2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCoordinateDouble(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CoordinateDouble with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCoordinateDouble() {
  fprintf(fp, "Event %4i - End node CoordinateDouble\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCoordinateInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CoordinateInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCoordinateInterpolator() {
  fprintf(fp, "Event %4i - End node CoordinateInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCoordinateInterpolator2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CoordinateInterpolator2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCoordinateInterpolator2D() {
  fprintf(fp, "Event %4i - End node CoordinateInterpolator2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCylinder(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Cylinder with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCylinder() {
  fprintf(fp, "Event %4i - End node Cylinder\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCylinderSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CylinderSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCylinderSensor() {
  fprintf(fp, "Event %4i - End node CylinderSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startDirectionalLight(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node DirectionalLight with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endDirectionalLight() {
  fprintf(fp, "Event %4i - End node DirectionalLight\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startDisk2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Disk2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endDisk2D() {
  fprintf(fp, "Event %4i - End node Disk2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startEXPORT(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node EXPORT with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endEXPORT() {
  fprintf(fp, "Event %4i - End node EXPORT\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startElevationGrid(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ElevationGrid with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endElevationGrid() {
  fprintf(fp, "Event %4i - End node ElevationGrid\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startEspduTransform(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node EspduTransform with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endEspduTransform() {
  fprintf(fp, "Event %4i - End node EspduTransform\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startExternProtoDeclare(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ExternProtoDeclare with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endExternProtoDeclare() {
  fprintf(fp, "Event %4i - End node ExternProtoDeclare\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startExtrusion(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Extrusion with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endExtrusion() {
  fprintf(fp, "Event %4i - End node Extrusion\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startFillProperties(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node FillProperties with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endFillProperties() {
  fprintf(fp, "Event %4i - End node FillProperties\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startFog(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Fog with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endFog() {
  fprintf(fp, "Event %4i - End node Fog\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startFontStyle(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node FontStyle with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endFontStyle() {
  fprintf(fp, "Event %4i - End node FontStyle\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoCoordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoCoordinate() {
  fprintf(fp, "Event %4i - End node GeoCoordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoElevationGrid(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoElevationGrid with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoElevationGrid() {
  fprintf(fp, "Event %4i - End node GeoElevationGrid\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoLOD(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoLOD with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoLOD() {
  fprintf(fp, "Event %4i - End node GeoLOD\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoLocation(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoLocation with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoLocation() {
  fprintf(fp, "Event %4i - End node GeoLocation\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoMetadata(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoMetadata with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoMetadata() {
  fprintf(fp, "Event %4i - End node GeoMetadata\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoOrigin(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoOrigin with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoOrigin() {
  fprintf(fp, "Event %4i - End node GeoOrigin\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoPositionInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoPositionInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoPositionInterpolator() {
  fprintf(fp, "Event %4i - End node GeoPositionInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoTouchSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoTouchSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoTouchSensor() {
  fprintf(fp, "Event %4i - End node GeoTouchSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeoViewpoint(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeoViewpoint with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeoViewpoint() {
  fprintf(fp, "Event %4i - End node GeoViewpoint\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startHAnimDisplacer(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node HAnimDisplacer with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endHAnimDisplacer() {
  fprintf(fp, "Event %4i - End node HAnimDisplacer\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startHAnimHumanoid(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node HAnimHumanoid with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endHAnimHumanoid() {
  fprintf(fp, "Event %4i - End node HAnimHumanoid\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startHAnimJoint(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node HAnimJoint with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endHAnimJoint() {
  fprintf(fp, "Event %4i - End node HAnimJoint\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startHAnimSegment(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node HAnimSegment with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endHAnimSegment() {
  fprintf(fp, "Event %4i - End node HAnimSegment\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startHAnimSite(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node HAnimSite with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endHAnimSite() {
  fprintf(fp, "Event %4i - End node HAnimSite\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIMPORT(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IMPORT with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIMPORT() {
  fprintf(fp, "Event %4i - End node IMPORT\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIS(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IS with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIS() {
  fprintf(fp, "Event %4i - End node IS\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startInline(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Inline with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endInline() {
  fprintf(fp, "Event %4i - End node Inline\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIntegerSequencer(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IntegerSequencer with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIntegerSequencer() {
  fprintf(fp, "Event %4i - End node IntegerSequencer\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIntegerTrigger(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IntegerTrigger with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIntegerTrigger() {
  fprintf(fp, "Event %4i - End node IntegerTrigger\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startKeySensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node KeySensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endKeySensor() {
  fprintf(fp, "Event %4i - End node KeySensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startLineProperties(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node LineProperties with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endLineProperties() {
  fprintf(fp, "Event %4i - End node LineProperties\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startLineSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node LineSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endLineSet() {
  fprintf(fp, "Event %4i - End node LineSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startLoadSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node LoadSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endLoadSensor() {
  fprintf(fp, "Event %4i - End node LoadSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMetadataDouble(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MetadataDouble with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMetadataDouble() {
  fprintf(fp, "Event %4i - End node MetadataDouble\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMetadataFloat(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MetadataFloat with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMetadataFloat() {
  fprintf(fp, "Event %4i - End node MetadataFloat\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMetadataInteger(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MetadataInteger with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMetadataInteger() {
  fprintf(fp, "Event %4i - End node MetadataInteger\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMetadataSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MetadataSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMetadataSet() {
  fprintf(fp, "Event %4i - End node MetadataSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMetadataString(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MetadataString with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMetadataString() {
  fprintf(fp, "Event %4i - End node MetadataString\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMovieTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node MovieTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMovieTexture() {
  fprintf(fp, "Event %4i - End node MovieTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNavigationInfo(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NavigationInfo with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNavigationInfo() {
  fprintf(fp, "Event %4i - End node NavigationInfo\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNormalInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NormalInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNormalInterpolator() {
  fprintf(fp, "Event %4i - End node NormalInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsCurve(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsCurve with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsCurve() {
  fprintf(fp, "Event %4i - End node NurbsCurve\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsCurve2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsCurve2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsCurve2D() {
  fprintf(fp, "Event %4i - End node NurbsCurve2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsOrientationInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsOrientationInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsOrientationInterpolator() {
  fprintf(fp, "Event %4i - End node NurbsOrientationInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsPatchSurface(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsPatchSurface with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsPatchSurface() {
  fprintf(fp, "Event %4i - End node NurbsPatchSurface\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsPositionInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsPositionInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsPositionInterpolator() {
  fprintf(fp, "Event %4i - End node NurbsPositionInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsSet() {
  fprintf(fp, "Event %4i - End node NurbsSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsSurfaceInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsSurfaceInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsSurfaceInterpolator() {
  fprintf(fp, "Event %4i - End node NurbsSurfaceInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsSweptSurface(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsSweptSurface with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsSweptSurface() {
  fprintf(fp, "Event %4i - End node NurbsSweptSurface\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsSwungSurface(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsSwungSurface with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsSwungSurface() {
  fprintf(fp, "Event %4i - End node NurbsSwungSurface\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsTextureCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsTextureCoordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsTextureCoordinate() {
  fprintf(fp, "Event %4i - End node NurbsTextureCoordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startNurbsTrimmedSurface(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node NurbsTrimmedSurface with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endNurbsTrimmedSurface() {
  fprintf(fp, "Event %4i - End node NurbsTrimmedSurface\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startOrientationInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node OrientationInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endOrientationInterpolator() {
  fprintf(fp, "Event %4i - End node OrientationInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPixelTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PixelTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPixelTexture() {
  fprintf(fp, "Event %4i - End node PixelTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPlaneSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PlaneSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPlaneSensor() {
  fprintf(fp, "Event %4i - End node PlaneSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPointLight(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PointLight with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPointLight() {
  fprintf(fp, "Event %4i - End node PointLight\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPolyline2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Polyline2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPolyline2D() {
  fprintf(fp, "Event %4i - End node Polyline2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPolypoint2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Polypoint2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPolypoint2D() {
  fprintf(fp, "Event %4i - End node Polypoint2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPositionInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PositionInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPositionInterpolator() {
  fprintf(fp, "Event %4i - End node PositionInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPositionInterpolator2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PositionInterpolator2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPositionInterpolator2D() {
  fprintf(fp, "Event %4i - End node PositionInterpolator2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProtoBody(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProtoBody with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProtoBody() {
  fprintf(fp, "Event %4i - End node ProtoBody\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProtoDeclare(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProtoDeclare with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProtoDeclare() {
  fprintf(fp, "Event %4i - End node ProtoDeclare\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProtoInterface(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProtoInterface with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProtoInterface() {
  fprintf(fp, "Event %4i - End node ProtoInterface\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProximitySensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProximitySensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProximitySensor() {
  fprintf(fp, "Event %4i - End node ProximitySensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startReceiverPdu(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ReceiverPdu with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endReceiverPdu() {
  fprintf(fp, "Event %4i - End node ReceiverPdu\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startRectangle2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Rectangle2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endRectangle2D() {
  fprintf(fp, "Event %4i - End node Rectangle2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startScalarInterpolator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ScalarInterpolator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endScalarInterpolator() {
  fprintf(fp, "Event %4i - End node ScalarInterpolator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startScene(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Scene with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endScene() {
  fprintf(fp, "Event %4i - End node Scene\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSignalPdu(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node SignalPdu with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSignalPdu() {
  fprintf(fp, "Event %4i - End node SignalPdu\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSound(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Sound with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSound() {
  fprintf(fp, "Event %4i - End node Sound\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSphereSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node SphereSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSphereSensor() {
  fprintf(fp, "Event %4i - End node SphereSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startSpotLight(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node SpotLight with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endSpotLight() {
  fprintf(fp, "Event %4i - End node SpotLight\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startStringSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node StringSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endStringSensor() {
  fprintf(fp, "Event %4i - End node StringSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startText(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Text with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endText() {
  fprintf(fp, "Event %4i - End node Text\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureBackground(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureBackground with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureBackground() {
  fprintf(fp, "Event %4i - End node TextureBackground\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureCoordinateGenerator(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureCoordinateGenerator with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureCoordinateGenerator() {
  fprintf(fp, "Event %4i - End node TextureCoordinateGenerator\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTimeSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TimeSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTimeSensor() {
  fprintf(fp, "Event %4i - End node TimeSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTimeTrigger(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TimeTrigger with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTimeTrigger() {
  fprintf(fp, "Event %4i - End node TimeTrigger\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTouchSensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TouchSensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTouchSensor() {
  fprintf(fp, "Event %4i - End node TouchSensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTransmitterPdu(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TransmitterPdu with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTransmitterPdu() {
  fprintf(fp, "Event %4i - End node TransmitterPdu\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTriangleFanSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TriangleFanSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTriangleFanSet() {
  fprintf(fp, "Event %4i - End node TriangleFanSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTriangleSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TriangleSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTriangleSet() {
  fprintf(fp, "Event %4i - End node TriangleSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTriangleSet2D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TriangleSet2D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTriangleSet2D() {
  fprintf(fp, "Event %4i - End node TriangleSet2D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTriangleStripSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TriangleStripSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTriangleStripSet() {
  fprintf(fp, "Event %4i - End node TriangleStripSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startViewpoint(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Viewpoint with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endViewpoint() {
  fprintf(fp, "Event %4i - End node Viewpoint\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startVisibilitySensor(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node VisibilitySensor with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endVisibilitySensor() {
  fprintf(fp, "Event %4i - End node VisibilitySensor\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startWorldInfo(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node WorldInfo with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endWorldInfo() {
  fprintf(fp, "Event %4i - End node WorldInfo\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startX3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node X3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endX3D() {
  fprintf(fp, "Event %4i - End node X3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startcomponent(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node component with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endcomponent() {
  fprintf(fp, "Event %4i - End node component\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startconnect(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node connect with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endconnect() {
  fprintf(fp, "Event %4i - End node connect\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startfield(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node field with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endfield() {
  fprintf(fp, "Event %4i - End node field\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::starthead(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node head with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endhead() {
  fprintf(fp, "Event %4i - End node head\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::starthumanoidBodyType(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node humanoidBodyType with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endhumanoidBodyType() {
  fprintf(fp, "Event %4i - End node humanoidBodyType\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startmeta(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node meta with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endmeta() {
  fprintf(fp, "Event %4i - End node meta\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCADAssembly(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CADAssembly with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCADAssembly() {
  fprintf(fp, "Event %4i - End node CADAssembly\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCADFace(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CADFace with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCADFace() {
  fprintf(fp, "Event %4i - End node CADFace\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCADLayer(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CADLayer with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCADLayer() {
  fprintf(fp, "Event %4i - End node CADLayer\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startCADPart(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node CADPart with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endCADPart() {
  fprintf(fp, "Event %4i - End node CADPart\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startComposedCubeMapTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ComposedCubeMapTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endComposedCubeMapTexture() {
  fprintf(fp, "Event %4i - End node ComposedCubeMapTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startComposedShader(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ComposedShader with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endComposedShader() {
  fprintf(fp, "Event %4i - End node ComposedShader\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startComposedTexture3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ComposedTexture3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endComposedTexture3D() {
  fprintf(fp, "Event %4i - End node ComposedTexture3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startFloatVertexAttribute(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node FloatVertexAttribute with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endFloatVertexAttribute() {
  fprintf(fp, "Event %4i - End node FloatVertexAttribute\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startFogCoordinate(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node FogCoordinate with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endFogCoordinate() {
  fprintf(fp, "Event %4i - End node FogCoordinate\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startGeneratedCubeMapTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node GeneratedCubeMapTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endGeneratedCubeMapTexture() {
  fprintf(fp, "Event %4i - End node GeneratedCubeMapTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startImageCubeMapTexture(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ImageCubeMapTexture with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endImageCubeMapTexture() {
  fprintf(fp, "Event %4i - End node ImageCubeMapTexture\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startImageTexture3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ImageTexture3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endImageTexture3D() {
  fprintf(fp, "Event %4i - End node ImageTexture3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startIndexedQuadSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node IndexedQuadSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endIndexedQuadSet() {
  fprintf(fp, "Event %4i - End node IndexedQuadSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startLocalFog(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node LocalFog with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endLocalFog() {
  fprintf(fp, "Event %4i - End node LocalFog\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMatrix3VertexAttribute(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Matrix3VertexAttribute with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMatrix3VertexAttribute() {
  fprintf(fp, "Event %4i - End node Matrix3VertexAttribute\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startMatrix4VertexAttribute(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node Matrix4VertexAttribute with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endMatrix4VertexAttribute() {
  fprintf(fp, "Event %4i - End node Matrix4VertexAttribute\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPackagedShader(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PackagedShader with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPackagedShader() {
  fprintf(fp, "Event %4i - End node PackagedShader\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startPixelTexture3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node PixelTexture3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endPixelTexture3D() {
  fprintf(fp, "Event %4i - End node PixelTexture3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startProgramShader(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ProgramShader with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endProgramShader() {
  fprintf(fp, "Event %4i - End node ProgramShader\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startQuadSet(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node QuadSet with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endQuadSet() {
  fprintf(fp, "Event %4i - End node QuadSet\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startShaderPart(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ShaderPart with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endShaderPart() {
  fprintf(fp, "Event %4i - End node ShaderPart\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startShaderProgram(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node ShaderProgram with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endShaderProgram() {
  fprintf(fp, "Event %4i - End node ShaderProgram\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureCoordinate3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureCoordinate3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureCoordinate3D() {
  fprintf(fp, "Event %4i - End node TextureCoordinate3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureCoordinate4D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureCoordinate4D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureCoordinate4D() {
  fprintf(fp, "Event %4i - End node TextureCoordinate4D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureTransform3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureTransform3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureTransform3D() {
  fprintf(fp, "Event %4i - End node TextureTransform3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startTextureTransformMatrix3D(const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start node TextureTransformMatrix3D with %i attribute(s): %s\n", iCounter++, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endTextureTransformMatrix3D() {
  fprintf(fp, "Event %4i - End node TextureTransformMatrix3D\n", iCounter++);
  return 1;
}

int X3DLogNodeHandler::startUnknown(const char* nodeName, const X3DAttributes &attr) {
  fprintf(fp, "Event %4i - Start unknown node %s with %i attribute(s): %s\n", iCounter++, nodeName, static_cast<int>(attr.getLength()), getAttributesAsString(attr).c_str());
  return 1;
}

int X3DLogNodeHandler::endUnknown(int id, const char* nodeName) {
  fprintf(fp, "Event %4i - End unknown node %s.\n", iCounter++, nodeName);
  return 1;
}

