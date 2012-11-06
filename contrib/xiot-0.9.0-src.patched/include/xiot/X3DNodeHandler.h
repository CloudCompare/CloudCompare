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
#ifndef X3D_X3DNODEHANDLER_H
#define X3D_X3DNODEHANDLER_H

#include <xiot/XIOTConfig.h>
#include <string> 

namespace XIOT {

class X3DAttributes;

/** 
 * <b>X3DNodeHandler</b> is an interface for the different NodeHandler implementations used. It provides startXXX/endXXX
 * functions for each X3D node, as well as startDocument()/endDocument(), which are called at the import begin/end and 
 * startUnknown()/endUnknown() functions, that are called whenever an unknown node is getting parsed.
 *
 * The X3DDefaultNodeHandler delivers a default implementation of this interface which does nothing. It serves as the parent
 * class for all specific NodeHandler implementations, which only have to override the startXXX/endXXX functions of the nodes
 * that they want to handle.

 * @see X3DDefaultNodeHandler
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DNodeHandler
{
public:
  /// Destructor.
  virtual ~X3DNodeHandler() {};

  virtual void startDocument() = 0;
  virtual void endDocument() = 0;

  // START GENERATED
    /// Callbacks for Shape Nodes
  virtual int startShape(const X3DAttributes& attr) = 0;
  virtual int endShape() = 0;

  /// Callbacks for Appearance Nodes
  virtual int startAppearance(const X3DAttributes& attr) = 0;
  virtual int endAppearance() = 0;

  /// Callbacks for Material Nodes
  virtual int startMaterial(const X3DAttributes& attr) = 0;
  virtual int endMaterial() = 0;

  /// Callbacks for IndexedFaceSet Nodes
  virtual int startIndexedFaceSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedFaceSet() = 0;

  /// Callbacks for ProtoInstance Nodes
  virtual int startProtoInstance(const X3DAttributes& attr) = 0;
  virtual int endProtoInstance() = 0;

  /// Callbacks for Transform Nodes
  virtual int startTransform(const X3DAttributes& attr) = 0;
  virtual int endTransform() = 0;

  /// Callbacks for ImageTexture Nodes
  virtual int startImageTexture(const X3DAttributes& attr) = 0;
  virtual int endImageTexture() = 0;

  /// Callbacks for TextureTransform Nodes
  virtual int startTextureTransform(const X3DAttributes& attr) = 0;
  virtual int endTextureTransform() = 0;

  /// Callbacks for Coordinate Nodes
  virtual int startCoordinate(const X3DAttributes& attr) = 0;
  virtual int endCoordinate() = 0;

  /// Callbacks for Normal Nodes
  virtual int startNormal(const X3DAttributes& attr) = 0;
  virtual int endNormal() = 0;

  /// Callbacks for Color Nodes
  virtual int startColor(const X3DAttributes& attr) = 0;
  virtual int endColor() = 0;

  /// Callbacks for ColorRGBA Nodes
  virtual int startColorRGBA(const X3DAttributes& attr) = 0;
  virtual int endColorRGBA() = 0;

  /// Callbacks for TextureCoordinate Nodes
  virtual int startTextureCoordinate(const X3DAttributes& attr) = 0;
  virtual int endTextureCoordinate() = 0;

  /// Callbacks for ROUTE Nodes
  virtual int startROUTE(const X3DAttributes& attr) = 0;
  virtual int endROUTE() = 0;

  /// Callbacks for fieldValue Nodes
  virtual int startfieldValue(const X3DAttributes& attr) = 0;
  virtual int endfieldValue() = 0;

  /// Callbacks for Group Nodes
  virtual int startGroup(const X3DAttributes& attr) = 0;
  virtual int endGroup() = 0;

  /// Callbacks for LOD Nodes
  virtual int startLOD(const X3DAttributes& attr) = 0;
  virtual int endLOD() = 0;

  /// Callbacks for Switch Nodes
  virtual int startSwitch(const X3DAttributes& attr) = 0;
  virtual int endSwitch() = 0;

  /// Callbacks for Script Nodes
  virtual int startScript(const X3DAttributes& attr) = 0;
  virtual int endScript() = 0;

  /// Callbacks for IndexedTriangleFanSet Nodes
  virtual int startIndexedTriangleFanSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedTriangleFanSet() = 0;

  /// Callbacks for IndexedTriangleSet Nodes
  virtual int startIndexedTriangleSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedTriangleSet() = 0;

  /// Callbacks for IndexedTriangleStripSet Nodes
  virtual int startIndexedTriangleStripSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedTriangleStripSet() = 0;

  /// Callbacks for MultiTexture Nodes
  virtual int startMultiTexture(const X3DAttributes& attr) = 0;
  virtual int endMultiTexture() = 0;

  /// Callbacks for MultiTextureCoordinate Nodes
  virtual int startMultiTextureCoordinate(const X3DAttributes& attr) = 0;
  virtual int endMultiTextureCoordinate() = 0;

  /// Callbacks for MultiTextureTransform Nodes
  virtual int startMultiTextureTransform(const X3DAttributes& attr) = 0;
  virtual int endMultiTextureTransform() = 0;

  /// Callbacks for IndexedLineSet Nodes
  virtual int startIndexedLineSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedLineSet() = 0;

  /// Callbacks for PointSet Nodes
  virtual int startPointSet(const X3DAttributes& attr) = 0;
  virtual int endPointSet() = 0;

  /// Callbacks for StaticGroup Nodes
  virtual int startStaticGroup(const X3DAttributes& attr) = 0;
  virtual int endStaticGroup() = 0;

  /// Callbacks for Sphere Nodes
  virtual int startSphere(const X3DAttributes& attr) = 0;
  virtual int endSphere() = 0;

  /// Callbacks for Box Nodes
  virtual int startBox(const X3DAttributes& attr) = 0;
  virtual int endBox() = 0;

  /// Callbacks for Cone Nodes
  virtual int startCone(const X3DAttributes& attr) = 0;
  virtual int endCone() = 0;

  /// Callbacks for Anchor Nodes
  virtual int startAnchor(const X3DAttributes& attr) = 0;
  virtual int endAnchor() = 0;

  /// Callbacks for Arc2D Nodes
  virtual int startArc2D(const X3DAttributes& attr) = 0;
  virtual int endArc2D() = 0;

  /// Callbacks for ArcClose2D Nodes
  virtual int startArcClose2D(const X3DAttributes& attr) = 0;
  virtual int endArcClose2D() = 0;

  /// Callbacks for AudioClip Nodes
  virtual int startAudioClip(const X3DAttributes& attr) = 0;
  virtual int endAudioClip() = 0;

  /// Callbacks for Background Nodes
  virtual int startBackground(const X3DAttributes& attr) = 0;
  virtual int endBackground() = 0;

  /// Callbacks for Billboard Nodes
  virtual int startBillboard(const X3DAttributes& attr) = 0;
  virtual int endBillboard() = 0;

  /// Callbacks for BooleanFilter Nodes
  virtual int startBooleanFilter(const X3DAttributes& attr) = 0;
  virtual int endBooleanFilter() = 0;

  /// Callbacks for BooleanSequencer Nodes
  virtual int startBooleanSequencer(const X3DAttributes& attr) = 0;
  virtual int endBooleanSequencer() = 0;

  /// Callbacks for BooleanToggle Nodes
  virtual int startBooleanToggle(const X3DAttributes& attr) = 0;
  virtual int endBooleanToggle() = 0;

  /// Callbacks for BooleanTrigger Nodes
  virtual int startBooleanTrigger(const X3DAttributes& attr) = 0;
  virtual int endBooleanTrigger() = 0;

  /// Callbacks for Circle2D Nodes
  virtual int startCircle2D(const X3DAttributes& attr) = 0;
  virtual int endCircle2D() = 0;

  /// Callbacks for Collision Nodes
  virtual int startCollision(const X3DAttributes& attr) = 0;
  virtual int endCollision() = 0;

  /// Callbacks for ColorInterpolator Nodes
  virtual int startColorInterpolator(const X3DAttributes& attr) = 0;
  virtual int endColorInterpolator() = 0;

  /// Callbacks for Contour2D Nodes
  virtual int startContour2D(const X3DAttributes& attr) = 0;
  virtual int endContour2D() = 0;

  /// Callbacks for ContourPolyline2D Nodes
  virtual int startContourPolyline2D(const X3DAttributes& attr) = 0;
  virtual int endContourPolyline2D() = 0;

  /// Callbacks for CoordinateDouble Nodes
  virtual int startCoordinateDouble(const X3DAttributes& attr) = 0;
  virtual int endCoordinateDouble() = 0;

  /// Callbacks for CoordinateInterpolator Nodes
  virtual int startCoordinateInterpolator(const X3DAttributes& attr) = 0;
  virtual int endCoordinateInterpolator() = 0;

  /// Callbacks for CoordinateInterpolator2D Nodes
  virtual int startCoordinateInterpolator2D(const X3DAttributes& attr) = 0;
  virtual int endCoordinateInterpolator2D() = 0;

  /// Callbacks for Cylinder Nodes
  virtual int startCylinder(const X3DAttributes& attr) = 0;
  virtual int endCylinder() = 0;

  /// Callbacks for CylinderSensor Nodes
  virtual int startCylinderSensor(const X3DAttributes& attr) = 0;
  virtual int endCylinderSensor() = 0;

  /// Callbacks for DirectionalLight Nodes
  virtual int startDirectionalLight(const X3DAttributes& attr) = 0;
  virtual int endDirectionalLight() = 0;

  /// Callbacks for Disk2D Nodes
  virtual int startDisk2D(const X3DAttributes& attr) = 0;
  virtual int endDisk2D() = 0;

  /// Callbacks for EXPORT Nodes
  virtual int startEXPORT(const X3DAttributes& attr) = 0;
  virtual int endEXPORT() = 0;

  /// Callbacks for ElevationGrid Nodes
  virtual int startElevationGrid(const X3DAttributes& attr) = 0;
  virtual int endElevationGrid() = 0;

  /// Callbacks for EspduTransform Nodes
  virtual int startEspduTransform(const X3DAttributes& attr) = 0;
  virtual int endEspduTransform() = 0;

  /// Callbacks for ExternProtoDeclare Nodes
  virtual int startExternProtoDeclare(const X3DAttributes& attr) = 0;
  virtual int endExternProtoDeclare() = 0;

  /// Callbacks for Extrusion Nodes
  virtual int startExtrusion(const X3DAttributes& attr) = 0;
  virtual int endExtrusion() = 0;

  /// Callbacks for FillProperties Nodes
  virtual int startFillProperties(const X3DAttributes& attr) = 0;
  virtual int endFillProperties() = 0;

  /// Callbacks for Fog Nodes
  virtual int startFog(const X3DAttributes& attr) = 0;
  virtual int endFog() = 0;

  /// Callbacks for FontStyle Nodes
  virtual int startFontStyle(const X3DAttributes& attr) = 0;
  virtual int endFontStyle() = 0;

  /// Callbacks for GeoCoordinate Nodes
  virtual int startGeoCoordinate(const X3DAttributes& attr) = 0;
  virtual int endGeoCoordinate() = 0;

  /// Callbacks for GeoElevationGrid Nodes
  virtual int startGeoElevationGrid(const X3DAttributes& attr) = 0;
  virtual int endGeoElevationGrid() = 0;

  /// Callbacks for GeoLOD Nodes
  virtual int startGeoLOD(const X3DAttributes& attr) = 0;
  virtual int endGeoLOD() = 0;

  /// Callbacks for GeoLocation Nodes
  virtual int startGeoLocation(const X3DAttributes& attr) = 0;
  virtual int endGeoLocation() = 0;

  /// Callbacks for GeoMetadata Nodes
  virtual int startGeoMetadata(const X3DAttributes& attr) = 0;
  virtual int endGeoMetadata() = 0;

  /// Callbacks for GeoOrigin Nodes
  virtual int startGeoOrigin(const X3DAttributes& attr) = 0;
  virtual int endGeoOrigin() = 0;

  /// Callbacks for GeoPositionInterpolator Nodes
  virtual int startGeoPositionInterpolator(const X3DAttributes& attr) = 0;
  virtual int endGeoPositionInterpolator() = 0;

  /// Callbacks for GeoTouchSensor Nodes
  virtual int startGeoTouchSensor(const X3DAttributes& attr) = 0;
  virtual int endGeoTouchSensor() = 0;

  /// Callbacks for GeoViewpoint Nodes
  virtual int startGeoViewpoint(const X3DAttributes& attr) = 0;
  virtual int endGeoViewpoint() = 0;

  /// Callbacks for HAnimDisplacer Nodes
  virtual int startHAnimDisplacer(const X3DAttributes& attr) = 0;
  virtual int endHAnimDisplacer() = 0;

  /// Callbacks for HAnimHumanoid Nodes
  virtual int startHAnimHumanoid(const X3DAttributes& attr) = 0;
  virtual int endHAnimHumanoid() = 0;

  /// Callbacks for HAnimJoint Nodes
  virtual int startHAnimJoint(const X3DAttributes& attr) = 0;
  virtual int endHAnimJoint() = 0;

  /// Callbacks for HAnimSegment Nodes
  virtual int startHAnimSegment(const X3DAttributes& attr) = 0;
  virtual int endHAnimSegment() = 0;

  /// Callbacks for HAnimSite Nodes
  virtual int startHAnimSite(const X3DAttributes& attr) = 0;
  virtual int endHAnimSite() = 0;

  /// Callbacks for IMPORT Nodes
  virtual int startIMPORT(const X3DAttributes& attr) = 0;
  virtual int endIMPORT() = 0;

  /// Callbacks for IS Nodes
  virtual int startIS(const X3DAttributes& attr) = 0;
  virtual int endIS() = 0;

  /// Callbacks for Inline Nodes
  virtual int startInline(const X3DAttributes& attr) = 0;
  virtual int endInline() = 0;

  /// Callbacks for IntegerSequencer Nodes
  virtual int startIntegerSequencer(const X3DAttributes& attr) = 0;
  virtual int endIntegerSequencer() = 0;

  /// Callbacks for IntegerTrigger Nodes
  virtual int startIntegerTrigger(const X3DAttributes& attr) = 0;
  virtual int endIntegerTrigger() = 0;

  /// Callbacks for KeySensor Nodes
  virtual int startKeySensor(const X3DAttributes& attr) = 0;
  virtual int endKeySensor() = 0;

  /// Callbacks for LineProperties Nodes
  virtual int startLineProperties(const X3DAttributes& attr) = 0;
  virtual int endLineProperties() = 0;

  /// Callbacks for LineSet Nodes
  virtual int startLineSet(const X3DAttributes& attr) = 0;
  virtual int endLineSet() = 0;

  /// Callbacks for LoadSensor Nodes
  virtual int startLoadSensor(const X3DAttributes& attr) = 0;
  virtual int endLoadSensor() = 0;

  /// Callbacks for MetadataDouble Nodes
  virtual int startMetadataDouble(const X3DAttributes& attr) = 0;
  virtual int endMetadataDouble() = 0;

  /// Callbacks for MetadataFloat Nodes
  virtual int startMetadataFloat(const X3DAttributes& attr) = 0;
  virtual int endMetadataFloat() = 0;

  /// Callbacks for MetadataInteger Nodes
  virtual int startMetadataInteger(const X3DAttributes& attr) = 0;
  virtual int endMetadataInteger() = 0;

  /// Callbacks for MetadataSet Nodes
  virtual int startMetadataSet(const X3DAttributes& attr) = 0;
  virtual int endMetadataSet() = 0;

  /// Callbacks for MetadataString Nodes
  virtual int startMetadataString(const X3DAttributes& attr) = 0;
  virtual int endMetadataString() = 0;

  /// Callbacks for MovieTexture Nodes
  virtual int startMovieTexture(const X3DAttributes& attr) = 0;
  virtual int endMovieTexture() = 0;

  /// Callbacks for NavigationInfo Nodes
  virtual int startNavigationInfo(const X3DAttributes& attr) = 0;
  virtual int endNavigationInfo() = 0;

  /// Callbacks for NormalInterpolator Nodes
  virtual int startNormalInterpolator(const X3DAttributes& attr) = 0;
  virtual int endNormalInterpolator() = 0;

  /// Callbacks for NurbsCurve Nodes
  virtual int startNurbsCurve(const X3DAttributes& attr) = 0;
  virtual int endNurbsCurve() = 0;

  /// Callbacks for NurbsCurve2D Nodes
  virtual int startNurbsCurve2D(const X3DAttributes& attr) = 0;
  virtual int endNurbsCurve2D() = 0;

  /// Callbacks for NurbsOrientationInterpolator Nodes
  virtual int startNurbsOrientationInterpolator(const X3DAttributes& attr) = 0;
  virtual int endNurbsOrientationInterpolator() = 0;

  /// Callbacks for NurbsPatchSurface Nodes
  virtual int startNurbsPatchSurface(const X3DAttributes& attr) = 0;
  virtual int endNurbsPatchSurface() = 0;

  /// Callbacks for NurbsPositionInterpolator Nodes
  virtual int startNurbsPositionInterpolator(const X3DAttributes& attr) = 0;
  virtual int endNurbsPositionInterpolator() = 0;

  /// Callbacks for NurbsSet Nodes
  virtual int startNurbsSet(const X3DAttributes& attr) = 0;
  virtual int endNurbsSet() = 0;

  /// Callbacks for NurbsSurfaceInterpolator Nodes
  virtual int startNurbsSurfaceInterpolator(const X3DAttributes& attr) = 0;
  virtual int endNurbsSurfaceInterpolator() = 0;

  /// Callbacks for NurbsSweptSurface Nodes
  virtual int startNurbsSweptSurface(const X3DAttributes& attr) = 0;
  virtual int endNurbsSweptSurface() = 0;

  /// Callbacks for NurbsSwungSurface Nodes
  virtual int startNurbsSwungSurface(const X3DAttributes& attr) = 0;
  virtual int endNurbsSwungSurface() = 0;

  /// Callbacks for NurbsTextureCoordinate Nodes
  virtual int startNurbsTextureCoordinate(const X3DAttributes& attr) = 0;
  virtual int endNurbsTextureCoordinate() = 0;

  /// Callbacks for NurbsTrimmedSurface Nodes
  virtual int startNurbsTrimmedSurface(const X3DAttributes& attr) = 0;
  virtual int endNurbsTrimmedSurface() = 0;

  /// Callbacks for OrientationInterpolator Nodes
  virtual int startOrientationInterpolator(const X3DAttributes& attr) = 0;
  virtual int endOrientationInterpolator() = 0;

  /// Callbacks for PixelTexture Nodes
  virtual int startPixelTexture(const X3DAttributes& attr) = 0;
  virtual int endPixelTexture() = 0;

  /// Callbacks for PlaneSensor Nodes
  virtual int startPlaneSensor(const X3DAttributes& attr) = 0;
  virtual int endPlaneSensor() = 0;

  /// Callbacks for PointLight Nodes
  virtual int startPointLight(const X3DAttributes& attr) = 0;
  virtual int endPointLight() = 0;

  /// Callbacks for Polyline2D Nodes
  virtual int startPolyline2D(const X3DAttributes& attr) = 0;
  virtual int endPolyline2D() = 0;

  /// Callbacks for Polypoint2D Nodes
  virtual int startPolypoint2D(const X3DAttributes& attr) = 0;
  virtual int endPolypoint2D() = 0;

  /// Callbacks for PositionInterpolator Nodes
  virtual int startPositionInterpolator(const X3DAttributes& attr) = 0;
  virtual int endPositionInterpolator() = 0;

  /// Callbacks for PositionInterpolator2D Nodes
  virtual int startPositionInterpolator2D(const X3DAttributes& attr) = 0;
  virtual int endPositionInterpolator2D() = 0;

  /// Callbacks for ProtoBody Nodes
  virtual int startProtoBody(const X3DAttributes& attr) = 0;
  virtual int endProtoBody() = 0;

  /// Callbacks for ProtoDeclare Nodes
  virtual int startProtoDeclare(const X3DAttributes& attr) = 0;
  virtual int endProtoDeclare() = 0;

  /// Callbacks for ProtoInterface Nodes
  virtual int startProtoInterface(const X3DAttributes& attr) = 0;
  virtual int endProtoInterface() = 0;

  /// Callbacks for ProximitySensor Nodes
  virtual int startProximitySensor(const X3DAttributes& attr) = 0;
  virtual int endProximitySensor() = 0;

  /// Callbacks for ReceiverPdu Nodes
  virtual int startReceiverPdu(const X3DAttributes& attr) = 0;
  virtual int endReceiverPdu() = 0;

  /// Callbacks for Rectangle2D Nodes
  virtual int startRectangle2D(const X3DAttributes& attr) = 0;
  virtual int endRectangle2D() = 0;

  /// Callbacks for ScalarInterpolator Nodes
  virtual int startScalarInterpolator(const X3DAttributes& attr) = 0;
  virtual int endScalarInterpolator() = 0;

  /// Callbacks for Scene Nodes
  virtual int startScene(const X3DAttributes& attr) = 0;
  virtual int endScene() = 0;

  /// Callbacks for SignalPdu Nodes
  virtual int startSignalPdu(const X3DAttributes& attr) = 0;
  virtual int endSignalPdu() = 0;

  /// Callbacks for Sound Nodes
  virtual int startSound(const X3DAttributes& attr) = 0;
  virtual int endSound() = 0;

  /// Callbacks for SphereSensor Nodes
  virtual int startSphereSensor(const X3DAttributes& attr) = 0;
  virtual int endSphereSensor() = 0;

  /// Callbacks for SpotLight Nodes
  virtual int startSpotLight(const X3DAttributes& attr) = 0;
  virtual int endSpotLight() = 0;

  /// Callbacks for StringSensor Nodes
  virtual int startStringSensor(const X3DAttributes& attr) = 0;
  virtual int endStringSensor() = 0;

  /// Callbacks for Text Nodes
  virtual int startText(const X3DAttributes& attr) = 0;
  virtual int endText() = 0;

  /// Callbacks for TextureBackground Nodes
  virtual int startTextureBackground(const X3DAttributes& attr) = 0;
  virtual int endTextureBackground() = 0;

  /// Callbacks for TextureCoordinateGenerator Nodes
  virtual int startTextureCoordinateGenerator(const X3DAttributes& attr) = 0;
  virtual int endTextureCoordinateGenerator() = 0;

  /// Callbacks for TimeSensor Nodes
  virtual int startTimeSensor(const X3DAttributes& attr) = 0;
  virtual int endTimeSensor() = 0;

  /// Callbacks for TimeTrigger Nodes
  virtual int startTimeTrigger(const X3DAttributes& attr) = 0;
  virtual int endTimeTrigger() = 0;

  /// Callbacks for TouchSensor Nodes
  virtual int startTouchSensor(const X3DAttributes& attr) = 0;
  virtual int endTouchSensor() = 0;

  /// Callbacks for TransmitterPdu Nodes
  virtual int startTransmitterPdu(const X3DAttributes& attr) = 0;
  virtual int endTransmitterPdu() = 0;

  /// Callbacks for TriangleFanSet Nodes
  virtual int startTriangleFanSet(const X3DAttributes& attr) = 0;
  virtual int endTriangleFanSet() = 0;

  /// Callbacks for TriangleSet Nodes
  virtual int startTriangleSet(const X3DAttributes& attr) = 0;
  virtual int endTriangleSet() = 0;

  /// Callbacks for TriangleSet2D Nodes
  virtual int startTriangleSet2D(const X3DAttributes& attr) = 0;
  virtual int endTriangleSet2D() = 0;

  /// Callbacks for TriangleStripSet Nodes
  virtual int startTriangleStripSet(const X3DAttributes& attr) = 0;
  virtual int endTriangleStripSet() = 0;

  /// Callbacks for Viewpoint Nodes
  virtual int startViewpoint(const X3DAttributes& attr) = 0;
  virtual int endViewpoint() = 0;

  /// Callbacks for VisibilitySensor Nodes
  virtual int startVisibilitySensor(const X3DAttributes& attr) = 0;
  virtual int endVisibilitySensor() = 0;

  /// Callbacks for WorldInfo Nodes
  virtual int startWorldInfo(const X3DAttributes& attr) = 0;
  virtual int endWorldInfo() = 0;

  /// Callbacks for X3D Nodes
  virtual int startX3D(const X3DAttributes& attr) = 0;
  virtual int endX3D() = 0;

  /// Callbacks for component Nodes
  virtual int startcomponent(const X3DAttributes& attr) = 0;
  virtual int endcomponent() = 0;

  /// Callbacks for connect Nodes
  virtual int startconnect(const X3DAttributes& attr) = 0;
  virtual int endconnect() = 0;

  /// Callbacks for field Nodes
  virtual int startfield(const X3DAttributes& attr) = 0;
  virtual int endfield() = 0;

  /// Callbacks for head Nodes
  virtual int starthead(const X3DAttributes& attr) = 0;
  virtual int endhead() = 0;

  /// Callbacks for humanoidBodyType Nodes
  virtual int starthumanoidBodyType(const X3DAttributes& attr) = 0;
  virtual int endhumanoidBodyType() = 0;

  /// Callbacks for meta Nodes
  virtual int startmeta(const X3DAttributes& attr) = 0;
  virtual int endmeta() = 0;

  /// Callbacks for CADAssembly Nodes
  virtual int startCADAssembly(const X3DAttributes& attr) = 0;
  virtual int endCADAssembly() = 0;

  /// Callbacks for CADFace Nodes
  virtual int startCADFace(const X3DAttributes& attr) = 0;
  virtual int endCADFace() = 0;

  /// Callbacks for CADLayer Nodes
  virtual int startCADLayer(const X3DAttributes& attr) = 0;
  virtual int endCADLayer() = 0;

  /// Callbacks for CADPart Nodes
  virtual int startCADPart(const X3DAttributes& attr) = 0;
  virtual int endCADPart() = 0;

  /// Callbacks for ComposedCubeMapTexture Nodes
  virtual int startComposedCubeMapTexture(const X3DAttributes& attr) = 0;
  virtual int endComposedCubeMapTexture() = 0;

  /// Callbacks for ComposedShader Nodes
  virtual int startComposedShader(const X3DAttributes& attr) = 0;
  virtual int endComposedShader() = 0;

  /// Callbacks for ComposedTexture3D Nodes
  virtual int startComposedTexture3D(const X3DAttributes& attr) = 0;
  virtual int endComposedTexture3D() = 0;

  /// Callbacks for FloatVertexAttribute Nodes
  virtual int startFloatVertexAttribute(const X3DAttributes& attr) = 0;
  virtual int endFloatVertexAttribute() = 0;

  /// Callbacks for FogCoordinate Nodes
  virtual int startFogCoordinate(const X3DAttributes& attr) = 0;
  virtual int endFogCoordinate() = 0;

  /// Callbacks for GeneratedCubeMapTexture Nodes
  virtual int startGeneratedCubeMapTexture(const X3DAttributes& attr) = 0;
  virtual int endGeneratedCubeMapTexture() = 0;

  /// Callbacks for ImageCubeMapTexture Nodes
  virtual int startImageCubeMapTexture(const X3DAttributes& attr) = 0;
  virtual int endImageCubeMapTexture() = 0;

  /// Callbacks for ImageTexture3D Nodes
  virtual int startImageTexture3D(const X3DAttributes& attr) = 0;
  virtual int endImageTexture3D() = 0;

  /// Callbacks for IndexedQuadSet Nodes
  virtual int startIndexedQuadSet(const X3DAttributes& attr) = 0;
  virtual int endIndexedQuadSet() = 0;

  /// Callbacks for LocalFog Nodes
  virtual int startLocalFog(const X3DAttributes& attr) = 0;
  virtual int endLocalFog() = 0;

  /// Callbacks for Matrix3VertexAttribute Nodes
  virtual int startMatrix3VertexAttribute(const X3DAttributes& attr) = 0;
  virtual int endMatrix3VertexAttribute() = 0;

  /// Callbacks for Matrix4VertexAttribute Nodes
  virtual int startMatrix4VertexAttribute(const X3DAttributes& attr) = 0;
  virtual int endMatrix4VertexAttribute() = 0;

  /// Callbacks for PackagedShader Nodes
  virtual int startPackagedShader(const X3DAttributes& attr) = 0;
  virtual int endPackagedShader() = 0;

  /// Callbacks for PixelTexture3D Nodes
  virtual int startPixelTexture3D(const X3DAttributes& attr) = 0;
  virtual int endPixelTexture3D() = 0;

  /// Callbacks for ProgramShader Nodes
  virtual int startProgramShader(const X3DAttributes& attr) = 0;
  virtual int endProgramShader() = 0;

  /// Callbacks for QuadSet Nodes
  virtual int startQuadSet(const X3DAttributes& attr) = 0;
  virtual int endQuadSet() = 0;

  /// Callbacks for ShaderPart Nodes
  virtual int startShaderPart(const X3DAttributes& attr) = 0;
  virtual int endShaderPart() = 0;

  /// Callbacks for ShaderProgram Nodes
  virtual int startShaderProgram(const X3DAttributes& attr) = 0;
  virtual int endShaderProgram() = 0;

  /// Callbacks for TextureCoordinate3D Nodes
  virtual int startTextureCoordinate3D(const X3DAttributes& attr) = 0;
  virtual int endTextureCoordinate3D() = 0;

  /// Callbacks for TextureCoordinate4D Nodes
  virtual int startTextureCoordinate4D(const X3DAttributes& attr) = 0;
  virtual int endTextureCoordinate4D() = 0;

  /// Callbacks for TextureTransform3D Nodes
  virtual int startTextureTransform3D(const X3DAttributes& attr) = 0;
  virtual int endTextureTransform3D() = 0;

  /// Callbacks for TextureTransformMatrix3D Nodes
  virtual int startTextureTransformMatrix3D(const X3DAttributes& attr) = 0;
  virtual int endTextureTransformMatrix3D() = 0;

  virtual int startUnknown(int id, const char* nodeName, const X3DAttributes& attr) = 0;
  virtual int endUnknown(int id, const char* nodeName) = 0;
  // END GENERATED
};

} // namepace


#endif

