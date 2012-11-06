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
#ifndef X3D_X3DDEFAULTNODEHANDLER_H
#define X3D_X3DDEFAULTNODEHANDLER_H

#include <xiot/X3DNodeHandler.h>

namespace XIOT {

class X3DAttributes;

/** 
 * The <b>X3DDefaultNodeHandler</b> delivers a default implementation of the X3DNodeHandler interface, which does nothing.
 *
 * It serves as the parent class for all specific NodeHandler implementations, which only have to override the 
 * startXXX/endXXX functions of the nodes that they want to handle.
 *
 * Additionally, it provides a startUnhandled(const X3DAttributes &attr) function, that is called by all start callback
 * functions. In the default implementation, it does nothing, but it can be overriden in order to deal with the unhandled
 * node events in a specific way (e.g. throw an exception).
 *
 * @see X3DNodeHandler
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DDefaultNodeHandler : public X3DNodeHandler
{
public:
  /// Constructor.
  X3DDefaultNodeHandler();
  /// Destructor.
  virtual ~X3DDefaultNodeHandler();

  /// The default implementation does nothing
  virtual void startDocument();
  /// The default implementation does nothing
  virtual void endDocument();

  /// Called by all start callback functions
  /// The default Implementation does nothing
  virtual int startUnhandled(const char* nodeName, const X3DAttributes &attr);

  /// Called by all end callback functions
  /// The default Implementation does nothing
  virtual int endUnhandled(const char* nodeName);
  


  // START GENERATED
  virtual int startShape(const X3DAttributes &attr);
  virtual int endShape();

  virtual int startAppearance(const X3DAttributes &attr);
  virtual int endAppearance();

  virtual int startMaterial(const X3DAttributes &attr);
  virtual int endMaterial();

  virtual int startIndexedFaceSet(const X3DAttributes &attr);
  virtual int endIndexedFaceSet();

  virtual int startProtoInstance(const X3DAttributes &attr);
  virtual int endProtoInstance();

  virtual int startTransform(const X3DAttributes &attr);
  virtual int endTransform();

  virtual int startImageTexture(const X3DAttributes &attr);
  virtual int endImageTexture();

  virtual int startTextureTransform(const X3DAttributes &attr);
  virtual int endTextureTransform();

  virtual int startCoordinate(const X3DAttributes &attr);
  virtual int endCoordinate();

  virtual int startNormal(const X3DAttributes &attr);
  virtual int endNormal();

  virtual int startColor(const X3DAttributes &attr);
  virtual int endColor();

  virtual int startColorRGBA(const X3DAttributes &attr);
  virtual int endColorRGBA();

  virtual int startTextureCoordinate(const X3DAttributes &attr);
  virtual int endTextureCoordinate();

  virtual int startROUTE(const X3DAttributes &attr);
  virtual int endROUTE();

  virtual int startfieldValue(const X3DAttributes &attr);
  virtual int endfieldValue();

  virtual int startGroup(const X3DAttributes &attr);
  virtual int endGroup();

  virtual int startLOD(const X3DAttributes &attr);
  virtual int endLOD();

  virtual int startSwitch(const X3DAttributes &attr);
  virtual int endSwitch();

  virtual int startScript(const X3DAttributes &attr);
  virtual int endScript();

  virtual int startIndexedTriangleFanSet(const X3DAttributes &attr);
  virtual int endIndexedTriangleFanSet();

  virtual int startIndexedTriangleSet(const X3DAttributes &attr);
  virtual int endIndexedTriangleSet();

  virtual int startIndexedTriangleStripSet(const X3DAttributes &attr);
  virtual int endIndexedTriangleStripSet();

  virtual int startMultiTexture(const X3DAttributes &attr);
  virtual int endMultiTexture();

  virtual int startMultiTextureCoordinate(const X3DAttributes &attr);
  virtual int endMultiTextureCoordinate();

  virtual int startMultiTextureTransform(const X3DAttributes &attr);
  virtual int endMultiTextureTransform();

  virtual int startIndexedLineSet(const X3DAttributes &attr);
  virtual int endIndexedLineSet();

  virtual int startPointSet(const X3DAttributes &attr);
  virtual int endPointSet();

  virtual int startStaticGroup(const X3DAttributes &attr);
  virtual int endStaticGroup();

  virtual int startSphere(const X3DAttributes &attr);
  virtual int endSphere();

  virtual int startBox(const X3DAttributes &attr);
  virtual int endBox();

  virtual int startCone(const X3DAttributes &attr);
  virtual int endCone();

  virtual int startAnchor(const X3DAttributes &attr);
  virtual int endAnchor();

  virtual int startArc2D(const X3DAttributes &attr);
  virtual int endArc2D();

  virtual int startArcClose2D(const X3DAttributes &attr);
  virtual int endArcClose2D();

  virtual int startAudioClip(const X3DAttributes &attr);
  virtual int endAudioClip();

  virtual int startBackground(const X3DAttributes &attr);
  virtual int endBackground();

  virtual int startBillboard(const X3DAttributes &attr);
  virtual int endBillboard();

  virtual int startBooleanFilter(const X3DAttributes &attr);
  virtual int endBooleanFilter();

  virtual int startBooleanSequencer(const X3DAttributes &attr);
  virtual int endBooleanSequencer();

  virtual int startBooleanToggle(const X3DAttributes &attr);
  virtual int endBooleanToggle();

  virtual int startBooleanTrigger(const X3DAttributes &attr);
  virtual int endBooleanTrigger();

  virtual int startCircle2D(const X3DAttributes &attr);
  virtual int endCircle2D();

  virtual int startCollision(const X3DAttributes &attr);
  virtual int endCollision();

  virtual int startColorInterpolator(const X3DAttributes &attr);
  virtual int endColorInterpolator();

  virtual int startContour2D(const X3DAttributes &attr);
  virtual int endContour2D();

  virtual int startContourPolyline2D(const X3DAttributes &attr);
  virtual int endContourPolyline2D();

  virtual int startCoordinateDouble(const X3DAttributes &attr);
  virtual int endCoordinateDouble();

  virtual int startCoordinateInterpolator(const X3DAttributes &attr);
  virtual int endCoordinateInterpolator();

  virtual int startCoordinateInterpolator2D(const X3DAttributes &attr);
  virtual int endCoordinateInterpolator2D();

  virtual int startCylinder(const X3DAttributes &attr);
  virtual int endCylinder();

  virtual int startCylinderSensor(const X3DAttributes &attr);
  virtual int endCylinderSensor();

  virtual int startDirectionalLight(const X3DAttributes &attr);
  virtual int endDirectionalLight();

  virtual int startDisk2D(const X3DAttributes &attr);
  virtual int endDisk2D();

  virtual int startEXPORT(const X3DAttributes &attr);
  virtual int endEXPORT();

  virtual int startElevationGrid(const X3DAttributes &attr);
  virtual int endElevationGrid();

  virtual int startEspduTransform(const X3DAttributes &attr);
  virtual int endEspduTransform();

  virtual int startExternProtoDeclare(const X3DAttributes &attr);
  virtual int endExternProtoDeclare();

  virtual int startExtrusion(const X3DAttributes &attr);
  virtual int endExtrusion();

  virtual int startFillProperties(const X3DAttributes &attr);
  virtual int endFillProperties();

  virtual int startFog(const X3DAttributes &attr);
  virtual int endFog();

  virtual int startFontStyle(const X3DAttributes &attr);
  virtual int endFontStyle();

  virtual int startGeoCoordinate(const X3DAttributes &attr);
  virtual int endGeoCoordinate();

  virtual int startGeoElevationGrid(const X3DAttributes &attr);
  virtual int endGeoElevationGrid();

  virtual int startGeoLOD(const X3DAttributes &attr);
  virtual int endGeoLOD();

  virtual int startGeoLocation(const X3DAttributes &attr);
  virtual int endGeoLocation();

  virtual int startGeoMetadata(const X3DAttributes &attr);
  virtual int endGeoMetadata();

  virtual int startGeoOrigin(const X3DAttributes &attr);
  virtual int endGeoOrigin();

  virtual int startGeoPositionInterpolator(const X3DAttributes &attr);
  virtual int endGeoPositionInterpolator();

  virtual int startGeoTouchSensor(const X3DAttributes &attr);
  virtual int endGeoTouchSensor();

  virtual int startGeoViewpoint(const X3DAttributes &attr);
  virtual int endGeoViewpoint();

  virtual int startHAnimDisplacer(const X3DAttributes &attr);
  virtual int endHAnimDisplacer();

  virtual int startHAnimHumanoid(const X3DAttributes &attr);
  virtual int endHAnimHumanoid();

  virtual int startHAnimJoint(const X3DAttributes &attr);
  virtual int endHAnimJoint();

  virtual int startHAnimSegment(const X3DAttributes &attr);
  virtual int endHAnimSegment();

  virtual int startHAnimSite(const X3DAttributes &attr);
  virtual int endHAnimSite();

  virtual int startIMPORT(const X3DAttributes &attr);
  virtual int endIMPORT();

  virtual int startIS(const X3DAttributes &attr);
  virtual int endIS();

  virtual int startInline(const X3DAttributes &attr);
  virtual int endInline();

  virtual int startIntegerSequencer(const X3DAttributes &attr);
  virtual int endIntegerSequencer();

  virtual int startIntegerTrigger(const X3DAttributes &attr);
  virtual int endIntegerTrigger();

  virtual int startKeySensor(const X3DAttributes &attr);
  virtual int endKeySensor();

  virtual int startLineProperties(const X3DAttributes &attr);
  virtual int endLineProperties();

  virtual int startLineSet(const X3DAttributes &attr);
  virtual int endLineSet();

  virtual int startLoadSensor(const X3DAttributes &attr);
  virtual int endLoadSensor();

  virtual int startMetadataDouble(const X3DAttributes &attr);
  virtual int endMetadataDouble();

  virtual int startMetadataFloat(const X3DAttributes &attr);
  virtual int endMetadataFloat();

  virtual int startMetadataInteger(const X3DAttributes &attr);
  virtual int endMetadataInteger();

  virtual int startMetadataSet(const X3DAttributes &attr);
  virtual int endMetadataSet();

  virtual int startMetadataString(const X3DAttributes &attr);
  virtual int endMetadataString();

  virtual int startMovieTexture(const X3DAttributes &attr);
  virtual int endMovieTexture();

  virtual int startNavigationInfo(const X3DAttributes &attr);
  virtual int endNavigationInfo();

  virtual int startNormalInterpolator(const X3DAttributes &attr);
  virtual int endNormalInterpolator();

  virtual int startNurbsCurve(const X3DAttributes &attr);
  virtual int endNurbsCurve();

  virtual int startNurbsCurve2D(const X3DAttributes &attr);
  virtual int endNurbsCurve2D();

  virtual int startNurbsOrientationInterpolator(const X3DAttributes &attr);
  virtual int endNurbsOrientationInterpolator();

  virtual int startNurbsPatchSurface(const X3DAttributes &attr);
  virtual int endNurbsPatchSurface();

  virtual int startNurbsPositionInterpolator(const X3DAttributes &attr);
  virtual int endNurbsPositionInterpolator();

  virtual int startNurbsSet(const X3DAttributes &attr);
  virtual int endNurbsSet();

  virtual int startNurbsSurfaceInterpolator(const X3DAttributes &attr);
  virtual int endNurbsSurfaceInterpolator();

  virtual int startNurbsSweptSurface(const X3DAttributes &attr);
  virtual int endNurbsSweptSurface();

  virtual int startNurbsSwungSurface(const X3DAttributes &attr);
  virtual int endNurbsSwungSurface();

  virtual int startNurbsTextureCoordinate(const X3DAttributes &attr);
  virtual int endNurbsTextureCoordinate();

  virtual int startNurbsTrimmedSurface(const X3DAttributes &attr);
  virtual int endNurbsTrimmedSurface();

  virtual int startOrientationInterpolator(const X3DAttributes &attr);
  virtual int endOrientationInterpolator();

  virtual int startPixelTexture(const X3DAttributes &attr);
  virtual int endPixelTexture();

  virtual int startPlaneSensor(const X3DAttributes &attr);
  virtual int endPlaneSensor();

  virtual int startPointLight(const X3DAttributes &attr);
  virtual int endPointLight();

  virtual int startPolyline2D(const X3DAttributes &attr);
  virtual int endPolyline2D();

  virtual int startPolypoint2D(const X3DAttributes &attr);
  virtual int endPolypoint2D();

  virtual int startPositionInterpolator(const X3DAttributes &attr);
  virtual int endPositionInterpolator();

  virtual int startPositionInterpolator2D(const X3DAttributes &attr);
  virtual int endPositionInterpolator2D();

  virtual int startProtoBody(const X3DAttributes &attr);
  virtual int endProtoBody();

  virtual int startProtoDeclare(const X3DAttributes &attr);
  virtual int endProtoDeclare();

  virtual int startProtoInterface(const X3DAttributes &attr);
  virtual int endProtoInterface();

  virtual int startProximitySensor(const X3DAttributes &attr);
  virtual int endProximitySensor();

  virtual int startReceiverPdu(const X3DAttributes &attr);
  virtual int endReceiverPdu();

  virtual int startRectangle2D(const X3DAttributes &attr);
  virtual int endRectangle2D();

  virtual int startScalarInterpolator(const X3DAttributes &attr);
  virtual int endScalarInterpolator();

  virtual int startScene(const X3DAttributes &attr);
  virtual int endScene();

  virtual int startSignalPdu(const X3DAttributes &attr);
  virtual int endSignalPdu();

  virtual int startSound(const X3DAttributes &attr);
  virtual int endSound();

  virtual int startSphereSensor(const X3DAttributes &attr);
  virtual int endSphereSensor();

  virtual int startSpotLight(const X3DAttributes &attr);
  virtual int endSpotLight();

  virtual int startStringSensor(const X3DAttributes &attr);
  virtual int endStringSensor();

  virtual int startText(const X3DAttributes &attr);
  virtual int endText();

  virtual int startTextureBackground(const X3DAttributes &attr);
  virtual int endTextureBackground();

  virtual int startTextureCoordinateGenerator(const X3DAttributes &attr);
  virtual int endTextureCoordinateGenerator();

  virtual int startTimeSensor(const X3DAttributes &attr);
  virtual int endTimeSensor();

  virtual int startTimeTrigger(const X3DAttributes &attr);
  virtual int endTimeTrigger();

  virtual int startTouchSensor(const X3DAttributes &attr);
  virtual int endTouchSensor();

  virtual int startTransmitterPdu(const X3DAttributes &attr);
  virtual int endTransmitterPdu();

  virtual int startTriangleFanSet(const X3DAttributes &attr);
  virtual int endTriangleFanSet();

  virtual int startTriangleSet(const X3DAttributes &attr);
  virtual int endTriangleSet();

  virtual int startTriangleSet2D(const X3DAttributes &attr);
  virtual int endTriangleSet2D();

  virtual int startTriangleStripSet(const X3DAttributes &attr);
  virtual int endTriangleStripSet();

  virtual int startViewpoint(const X3DAttributes &attr);
  virtual int endViewpoint();

  virtual int startVisibilitySensor(const X3DAttributes &attr);
  virtual int endVisibilitySensor();

  virtual int startWorldInfo(const X3DAttributes &attr);
  virtual int endWorldInfo();

  virtual int startX3D(const X3DAttributes &attr);
  virtual int endX3D();

  virtual int startcomponent(const X3DAttributes &attr);
  virtual int endcomponent();

  virtual int startconnect(const X3DAttributes &attr);
  virtual int endconnect();

  virtual int startfield(const X3DAttributes &attr);
  virtual int endfield();

  virtual int starthead(const X3DAttributes &attr);
  virtual int endhead();

  virtual int starthumanoidBodyType(const X3DAttributes &attr);
  virtual int endhumanoidBodyType();

  virtual int startmeta(const X3DAttributes &attr);
  virtual int endmeta();

  virtual int startCADAssembly(const X3DAttributes &attr);
  virtual int endCADAssembly();

  virtual int startCADFace(const X3DAttributes &attr);
  virtual int endCADFace();

  virtual int startCADLayer(const X3DAttributes &attr);
  virtual int endCADLayer();

  virtual int startCADPart(const X3DAttributes &attr);
  virtual int endCADPart();

  virtual int startComposedCubeMapTexture(const X3DAttributes &attr);
  virtual int endComposedCubeMapTexture();

  virtual int startComposedShader(const X3DAttributes &attr);
  virtual int endComposedShader();

  virtual int startComposedTexture3D(const X3DAttributes &attr);
  virtual int endComposedTexture3D();

  virtual int startFloatVertexAttribute(const X3DAttributes &attr);
  virtual int endFloatVertexAttribute();

  virtual int startFogCoordinate(const X3DAttributes &attr);
  virtual int endFogCoordinate();

  virtual int startGeneratedCubeMapTexture(const X3DAttributes &attr);
  virtual int endGeneratedCubeMapTexture();

  virtual int startImageCubeMapTexture(const X3DAttributes &attr);
  virtual int endImageCubeMapTexture();

  virtual int startImageTexture3D(const X3DAttributes &attr);
  virtual int endImageTexture3D();

  virtual int startIndexedQuadSet(const X3DAttributes &attr);
  virtual int endIndexedQuadSet();

  virtual int startLocalFog(const X3DAttributes &attr);
  virtual int endLocalFog();

  virtual int startMatrix3VertexAttribute(const X3DAttributes &attr);
  virtual int endMatrix3VertexAttribute();

  virtual int startMatrix4VertexAttribute(const X3DAttributes &attr);
  virtual int endMatrix4VertexAttribute();

  virtual int startPackagedShader(const X3DAttributes &attr);
  virtual int endPackagedShader();

  virtual int startPixelTexture3D(const X3DAttributes &attr);
  virtual int endPixelTexture3D();

  virtual int startProgramShader(const X3DAttributes &attr);
  virtual int endProgramShader();

  virtual int startQuadSet(const X3DAttributes &attr);
  virtual int endQuadSet();

  virtual int startShaderPart(const X3DAttributes &attr);
  virtual int endShaderPart();

  virtual int startShaderProgram(const X3DAttributes &attr);
  virtual int endShaderProgram();

  virtual int startTextureCoordinate3D(const X3DAttributes &attr);
  virtual int endTextureCoordinate3D();

  virtual int startTextureCoordinate4D(const X3DAttributes &attr);
  virtual int endTextureCoordinate4D();

  virtual int startTextureTransform3D(const X3DAttributes &attr);
  virtual int endTextureTransform3D();

  virtual int startTextureTransformMatrix3D(const X3DAttributes &attr);
  virtual int endTextureTransformMatrix3D();

  virtual int startUnknown(int id, const char* nodeName, const X3DAttributes &attr);
  virtual int endUnknown(int id, const char* nodeName);
  // END GENERATED

  
};

} // namepace


#endif

