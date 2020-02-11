#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_CLASSES_HPP

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScripting                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>

#include <ccGLWindow.h>
#include <ccShader.h>
#include <ccGlFilter.h>
#include <ccGLWidget.h>
#include <ccGuiParameters.h>


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_glWindow
			{
				
				ModulePtr bs_ccGLWindow(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccGLWindow>(), "ccGLWindow");
					m->add(constructor<ccGLWindow(QSurfaceFormat*, ccGLWindowParent*, bool)>(), "ccGLWindow");

					m->add(fun(&ccGLWindow::PAN_ONLY), "PAN_ONLY");
					m->add(fun(&ccGLWindow::TRANSFORM_CAMERA), "TRANSFORM_CAMERA");
					m->add(fun(&ccGLWindow::TRANSFORM_ENTITIES), "TRANSFORM_ENTITIES");
#ifdef CC_GL_WINDOW_USE_QWINDOW
					m->add(fun(&ccGLWindow::parentWidget), "parentWidget");
					m->add(fun(&ccGLWindow::setParentWidget), "setParentWidget");
					m->add(fun(&ccGLWindow::font), "font");
					m->add(fun(&ccGLWindow::setWindowTitle), "setWindowTitle");
					m->add(fun(&ccGLWindow::windowTitle), "windowTitle");
#endif
					
					m->add(fun(&ccGLWindow::setSceneDB), "setSceneDB");
					m->add(fun(&ccGLWindow::getSceneDB), "getSceneDB");
					m->add(fun(static_cast<void(ccGLWindow::*)(int,int,const QString&, const QFont&)>(&ccGLWindow::renderText)), "renderText");
					m->add(fun(static_cast<void(ccGLWindow::*)(double, double, double, const QString&, const QFont&)>(&ccGLWindow::renderText)), "renderText");
					m->add(fun([](ccGLWindow* obj, int a, int b, const QString& c) {obj->renderText(a, b, c); }), "renderText");
					m->add(fun([](ccGLWindow* obj, double a, double b, double c, const QString& d) {obj->renderText(a, b, c, d); }), "renderText");
					m->add(fun(&ccGLWindow::toBeRefreshed), "toBeRefreshed");
					m->add(fun(&ccGLWindow::refresh), "refresh");
					m->add(fun([](ccGLWindow* obj) {obj->refresh(); }), "refresh");
					m->add(fun(&ccGLWindow::invalidateViewport), "invalidateViewport");
					m->add(fun(&ccGLWindow::deprecate3DLayer), "deprecate3DLayer");
					m->add(fun(&ccGLWindow::display3DLabel), "display3DLabel");
					m->add(fun([](ccGLWindow* obj, QString a, int b, int c, unsigned char d, float e, const unsigned char* f) {obj->displayText(a, b, c, d, e, f); }), "displayText");
					m->add(fun([](ccGLWindow* obj, QString a, int b, int c, unsigned char d, float e) {obj->displayText(a, b, c, d, e); }), "displayText");
					m->add(fun([](ccGLWindow* obj, QString a, int b, int c, unsigned char d) {obj->displayText(a, b, c, d); }), "displayText");
					m->add(fun([](ccGLWindow* obj, QString a, int b, int c) {obj->displayText(a, b, c); }), "displayText");
					m->add(fun(&ccGLWindow::displayText), "displayText");
					m->add(fun([](ccGLWindow* obj, const QString& a, const CCVector3& b, const unsigned char* c) {obj->display3DLabel(a, b, c); }), "display3DLabel");
					m->add(fun([](ccGLWindow* obj, const QString& a, const CCVector3& b) {obj->display3DLabel(a, b); }), "display3DLabel");
					m->add(fun(&ccGLWindow::getTextDisplayFont), "getTextDisplayFont");
					m->add(fun(&ccGLWindow::getLabelDisplayFont), "getLabelDisplayFont");
					m->add(fun(&ccGLWindow::getViewportParameters), "getViewportParameters");
					m->add(fun(&ccGLWindow::toCenteredGLCoordinates), "toCenteredGLCoordinates");
					m->add(fun(&ccGLWindow::toCornerGLCoordinates), "toCornerGLCoordinates");
					m->add(fun(&ccGLWindow::setupProjectiveViewport), "setupProjectiveViewport");
					m->add(fun([](ccGLWindow* obj, const ccGLMatrixd& a, float b, float c, bool d) {obj->setupProjectiveViewport(a, b, c, d); }), "setupProjectiveViewport");
					m->add(fun([](ccGLWindow* obj, const ccGLMatrixd& a, float b, float c) {obj->setupProjectiveViewport(a, b, c); }), "setupProjectiveViewport");
					m->add(fun([](ccGLWindow* obj, const ccGLMatrixd& a, float b) {obj->setupProjectiveViewport(a, b); }), "setupProjectiveViewport");
					m->add(fun([](ccGLWindow* obj, const ccGLMatrixd& a) {obj->setupProjectiveViewport(a); }), "setupProjectiveViewport");
					m->add(fun(&ccGLWindow::asWidget), "asWidget");
					m->add(fun(&ccGLWindow::getScreenSize), "getScreenSize");
					m->add(fun(&ccGLWindow::getGLCameraParameters), "getGLCameraParameters");
					m->add(fun(&ccGLWindow::displayNewMessage), "displayNewMessage");
					m->add(fun([](ccGLWindow* obj, const QString& a, ccGLWindow::MessagePosition b, bool c, int d) {obj->displayNewMessage(a, b, c, d); }), "displayNewMessage");
					m->add(fun([](ccGLWindow* obj, const QString& a, ccGLWindow::MessagePosition b, bool c) {obj->displayNewMessage(a, b, c); }), "displayNewMessage");
					m->add(fun([](ccGLWindow* obj, const QString& a, ccGLWindow::MessagePosition b) {obj->displayNewMessage(a, b); }), "displayNewMessage");
					m->add(fun(&ccGLWindow::setSunLight), "setSunLight");
					m->add(fun(&ccGLWindow::toggleSunLight), "toggleSunLight");
					m->add(fun(&ccGLWindow::sunLightEnabled), "sunLightEnabled");
					m->add(fun(&ccGLWindow::setCustomLight), "setCustomLight");
					m->add(fun(&ccGLWindow::toggleCustomLight), "toggleCustomLight");
					m->add(fun(&ccGLWindow::customLightEnabled), "customLightEnabled");
					m->add(fun(&ccGLWindow::setZoom), "setZoom");
					m->add(fun(&ccGLWindow::updateZoom), "updateZoom");
					m->add(fun(&ccGLWindow::setPivotVisibility), "setPivotVisibility");
					m->add(fun(&ccGLWindow::showPivotSymbol), "showPivotSymbol");
					m->add(fun(&ccGLWindow::setPixelSize), "setPixelSize");
					m->add(fun(&ccGLWindow::setPivotPoint), "setPivotPoint");
					m->add(fun([](ccGLWindow* obj, const CCVector3d& a, bool b) {obj->setPivotPoint(a, b); }), "setPivotPoint");
					m->add(fun([](ccGLWindow* obj, const CCVector3d& a) {obj->setPivotPoint(a); }), "setPivotPoint");
					m->add(fun(&ccGLWindow::setCameraPos), "setCameraPos");
					m->add(fun(&ccGLWindow::moveCamera), "moveCamera");
					m->add(fun(&ccGLWindow::setPerspectiveState), "setPerspectiveState");
					m->add(fun(&ccGLWindow::togglePerspective), "togglePerspective");
					m->add(fun(&ccGLWindow::getPerspectiveState), "getPerspectiveState");
					m->add(fun(&ccGLWindow::objectPerspectiveEnabled), "objectPerspectiveEnabled");
					m->add(fun(&ccGLWindow::viewerPerspectiveEnabled), "viewerPerspectiveEnabled");
					m->add(fun(&ccGLWindow::setBubbleViewMode), "setBubbleViewMode");
					m->add(fun(&ccGLWindow::bubbleViewModeEnabled), "bubbleViewModeEnabled");
					m->add(fun(&ccGLWindow::setBubbleViewFov), "setBubbleViewFov");
					m->add(fun(&ccGLWindow::updateConstellationCenterAndZoom), "updateConstellationCenterAndZoom");
					m->add(fun([](ccGLWindow* obj) {obj->updateConstellationCenterAndZoom(); }), "updateConstellationCenterAndZoom");
					m->add(fun(&ccGLWindow::getVisibleObjectsBB), "getVisibleObjectsBB");
					m->add(fun(&ccGLWindow::rotateBaseViewMat), "rotateBaseViewMat");
					m->add(fun(&ccGLWindow::getBaseViewMat), "getBaseViewMat");
					m->add(fun(&ccGLWindow::setBaseViewMat), "setBaseViewMat");
					m->add(fun(&ccGLWindow::setView), "setView");
					m->add(fun(&ccGLWindow::setCustomView), "setCustomView");
					m->add(fun(&ccGLWindow::setInteractionMode), "setInteractionMode");
					m->add(fun(&ccGLWindow::getInteractionMode), "getInteractionMode");
					m->add(fun(&ccGLWindow::setPickingMode), "setPickingMode");
					m->add(fun(&ccGLWindow::getPickingMode), "getPickingMode");
					m->add(fun(&ccGLWindow::lockPickingMode), "lockPickingMode");
					m->add(fun(&ccGLWindow::isPickingModeLocked), "isPickingModeLocked");
					m->add(fun(&ccGLWindow::setUnclosable), "setUnclosable");
					m->add(fun(&ccGLWindow::getContext), "getContext");
					m->add(fun(&ccGLWindow::setPointSize), "setPointSize");
					m->add(fun(&ccGLWindow::setLineWidth), "setLineWidth");
					m->add(fun(&ccGLWindow::getFontPointSize), "getFontPointSize");
					m->add(fun(&ccGLWindow::getLabelFontPointSize), "getLabelFontPointSize");
					m->add(fun(&ccGLWindow::getOwnDB), "getOwnDB");
					m->add(fun(&ccGLWindow::addToOwnDB), "addToOwnDB");
					m->add(fun([](ccGLWindow* obj, ccHObject* a) {obj->addToOwnDB(a); }), "addToOwnDB");
					m->add(fun(&ccGLWindow::removeFromOwnDB), "removeFromOwnDB");
					m->add(fun(&ccGLWindow::setViewportParameters), "setViewportParameters");
					m->add(fun(&ccGLWindow::setFov), "setFov");
					m->add(fun(&ccGLWindow::getFov), "getFov");
					m->add(fun(&ccGLWindow::setAspectRatio), "setAspectRatio");
					m->add(fun(&ccGLWindow::setZNearCoef), "setZNearCoef");
					m->add(fun(&ccGLWindow::invalidateVisualization), "invalidateVisualization");
					m->add(fun(&ccGLWindow::renderToImage), "renderToImage");
					m->add(fun([](ccGLWindow* obj, float a, bool b, bool c) {obj->renderToImage(a, b, c); }), "renderToImage");
					m->add(fun([](ccGLWindow* obj, float a, bool b) {obj->renderToImage(a, b); }), "renderToImage");
					m->add(fun([](ccGLWindow* obj, float a) {obj->renderToImage(a); }), "renderToImage");
					m->add(fun([](ccGLWindow* obj) {obj->renderToImage(); }), "renderToImage");
					m->add(fun(&ccGLWindow::renderToFile), "renderToFile");
					m->add(fun([](ccGLWindow* obj, QString a, float b, bool c) {obj->renderToFile(a, b, c); }), "renderToFile");
					m->add(fun([](ccGLWindow* obj, QString a, float b) {obj->renderToFile(a, b); }), "renderToFile");
					m->add(fun([](ccGLWindow* obj, QString a) {obj->renderToFile(a); }), "renderToFile");
					m->add(fun(&ccGLWindow::setShaderPath), "setShaderPath");
					m->add(fun(&ccGLWindow::setShader), "setShader");
					m->add(fun(&ccGLWindow::setGlFilter), "setGlFilter");
					m->add(fun(&ccGLWindow::getGlFilter), "getGlFilter");
					m->add(fun(&ccGLWindow::areShadersEnabled), "areShadersEnabled");
					m->add(fun(&ccGLWindow::areGLFiltersEnabled), "areGLFiltersEnabled");
					m->add(fun(&ccGLWindow::computeActualPixelSize), "computeActualPixelSize");
					m->add(fun(&ccGLWindow::computePerspectiveZoom), "computePerspectiveZoom");
					m->add(fun(&ccGLWindow::hasColorRampShader), "hasColorRampShader");
					m->add(fun(&ccGLWindow::isRectangularPickingAllowed), "isRectangularPickingAllowed");
					m->add(fun(&ccGLWindow::setRectangularPickingAllowed), "setRectangularPickingAllowed");
					m->add(fun(&ccGLWindow::getCurrentViewDir), "getCurrentViewDir");
					m->add(fun(&ccGLWindow::getCurrentUpDir), "getCurrentUpDir");
					m->add(fun(&ccGLWindow::getDisplayParameters), "getDisplayParameters");
					m->add(fun(&ccGLWindow::setDisplayParameters), "setDisplayParameters");
					m->add(fun(&ccGLWindow::hasOverridenDisplayParameters), "hasOverridenDisplayParameters");
					m->add(fun(&ccGLWindow::setPickingRadius), "setPickingRadius");
					m->add(fun(&ccGLWindow::getPickingRadius), "getPickingRadius");
					m->add(fun(&ccGLWindow::displayOverlayEntities), "displayOverlayEntities");
					m->add(fun(&ccGLWindow::overlayEntitiesAreDisplayed), "overlayEntitiesAreDisplayed");
					m->add(fun(&ccGLWindow::backprojectPointOnTriangle), "backprojectPointOnTriangle");
					m->add(fun(&ccGLWindow::getUniqueID), "getUniqueID");
					m->add(fun(&ccGLWindow::qtWidth), "qtWidth");
					m->add(fun(&ccGLWindow::qtHeight), "qtHeight");
					m->add(fun(&ccGLWindow::qtSize), "qtSize");
					m->add(fun(&ccGLWindow::glWidth), "glWidth");
					m->add(fun(&ccGLWindow::glHeight), "glHeight");
					m->add(fun(&ccGLWindow::glSize), "glSize");
					m->add(fun(&ccGLWindow::isLODEnabled), "isLODEnabled");
					m->add(fun(&ccGLWindow::setLODEnabled), "setLODEnabled");
					m->add(fun(&ccGLWindow::toggleExclusiveFullScreen), "toggleExclusiveFullScreen");
					m->add(fun(&ccGLWindow::exclusiveFullScreen), "exclusiveFullScreen");
					m->add(fun(&ccGLWindow::enableDebugTrace), "enableDebugTrace");
					m->add(fun(&ccGLWindow::toggleDebugTrace), "toggleDebugTrace");
					m->add(user_type<ccGLWindow::StereoParams>(), "StereoParams");
						m->add(constructor<ccGLWindow::StereoParams()>(), "StereoParams");
						chaiscript::utility::add_class<ccGLWindow::StereoParams::GlassType>(*m,
							"GlassType",
							{
								{ ccGLWindow::StereoParams::GlassType::RED_BLUE, "RED_BLUE" },
								{ ccGLWindow::StereoParams::GlassType::BLUE_RED, "BLUE_RED" },
								{ ccGLWindow::StereoParams::GlassType::RED_CYAN, "RED_CYAN" },
								{ ccGLWindow::StereoParams::GlassType::CYAN_RED, "CYAN_RED" },
								{ ccGLWindow::StereoParams::GlassType::NVIDIA_VISION, "NVIDIA_VISION" },
								{ ccGLWindow::StereoParams::GlassType::OCULUS, "OCULUS" },
								{ ccGLWindow::StereoParams::GlassType::GENERIC_STEREO_DISPLAY, "GENERIC_STEREO_DISPLAY" }
							}
						);
						m->add(fun(&ccGLWindow::StereoParams::isAnaglyph), "");
						m->add(fun(&ccGLWindow::StereoParams::screenWidth_mm), "screenWidth_mm");
						m->add(fun(&ccGLWindow::StereoParams::screenDistance_mm), "screenDistance_mm");
						m->add(fun(&ccGLWindow::StereoParams::eyeSeparation_mm), "eyeSeparation_mm");
						m->add(fun(&ccGLWindow::StereoParams::stereoStrength), "stereoStrength");
						m->add(fun(&ccGLWindow::StereoParams::glassType), "glassType");
					m->add(fun(&ccGLWindow::enableStereoMode), "enableStereoMode");
					m->add(fun(&ccGLWindow::disableStereoMode), "disableStereoMode");
					m->add(fun(&ccGLWindow::stereoModeIsEnabled), "stereoModeIsEnabled");
					m->add(fun(&ccGLWindow::getStereoParams), "getStereoParams");
					m->add(fun(&ccGLWindow::showCursorCoordinates), "showCursorCoordinates");
					m->add(fun(&ccGLWindow::cursorCoordinatesShown), "cursorCoordinatesShown");
					m->add(fun(&ccGLWindow::setAutoPickPivotAtCenter), "setAutoPickPivotAtCenter");
					m->add(fun(&ccGLWindow::autoPickPivotAtCenter), "autoPickPivotAtCenter");
					m->add(fun(&ccGLWindow::lockRotationAxis), "lockRotationAxis");
					m->add(fun(&ccGLWindow::zoomGlobal), "zoomGlobal");
					m->add(fun(&ccGLWindow::redraw), "redraw");
					m->add(fun(&ccGLWindow::onWheelEvent), "onWheelEvent");
					m->add(fun(&ccGLWindow::startFrameRateTest), "startFrameRateTest");
					m->add(fun(&ccGLWindow::requestUpdate), "requestUpdate");
#ifdef CC_GL_WINDOW_USE_QWINDOW
					//! For compatibility with the QOpenGLWidget version
					m->add(fun(&ccGLWindow::update), "update");
#endif
					//SIGNALS
					m->add(fun(&ccGLWindow::entitySelectionChanged), "entitySelectionChanged");
					m->add(fun(&ccGLWindow::entitiesSelectionChanged), "entitiesSelectionChanged");
					m->add(fun(&ccGLWindow::itemPicked), "itemPicked");
					m->add(fun(&ccGLWindow::itemPickedFast), "itemPickedFast");
					m->add(fun(&ccGLWindow::fastPickingFinished), "fastPickingFinished");
					m->add(fun(&ccGLWindow::viewMatRotated), "viewMatRotated");
					m->add(fun(&ccGLWindow::cameraDisplaced), "cameraDisplaced");
					m->add(fun(&ccGLWindow::mouseWheelRotated), "mouseWheelRotated");
					m->add(fun(&ccGLWindow::perspectiveStateChanged), "perspectiveStateChanged");
					m->add(fun(&ccGLWindow::baseViewMatChanged), "baseViewMatChanged");
					m->add(fun(&ccGLWindow::pixelSizeChanged), "pixelSizeChanged");
					m->add(fun(&ccGLWindow::fovChanged), "fovChanged");
					m->add(fun(&ccGLWindow::zNearCoefChanged), "zNearCoefChanged");
					m->add(fun(&ccGLWindow::pivotPointChanged), "pivotPointChanged");
					m->add(fun(&ccGLWindow::cameraPosChanged), "cameraPosChanged");
					m->add(fun(&ccGLWindow::translation), "translation");
					m->add(fun(&ccGLWindow::rotation), "rotation");
					m->add(fun(&ccGLWindow::leftButtonClicked), "leftButtonClicked");
					m->add(fun(&ccGLWindow::rightButtonClicked), "rightButtonClicked");
					m->add(fun(&ccGLWindow::mouseMoved), "mouseMoved");
					m->add(fun(&ccGLWindow::buttonReleased), "buttonReleased");
					m->add(fun(&ccGLWindow::drawing3D), "drawing3D");
					m->add(fun(&ccGLWindow::filesDropped), "filesDropped");
					m->add(fun(&ccGLWindow::newLabel), "newLabel");
					m->add(fun(&ccGLWindow::exclusiveFullScreenToggled), "exclusiveFullScreenToggled");
					m->add(fun(&ccGLWindow::middleButtonClicked), "middleButtonClicked");

					m->add(chaiscript::base_class<ccGenericGLDisplay, ccGLWindow>());
#ifdef CC_GL_WINDOW_USE_QWINDOW
					m->add(chaiscript::base_class<QWindow, ccGLWindow>());
#else
					m->add(chaiscript::base_class<QOpenGLWidget, ccGLWindow>());
					m->add(chaiscript::base_class<QWidget, ccGLWindow>());
#endif
					m->add(chaiscript::base_class<QObject, ccGLWindow>());
					return m;
				}

				ModulePtr bs_ccGLWidget(ModulePtr m = std::make_shared<Module>())
				{
#ifdef CC_GL_WINDOW_USE_QWINDOW
					m->add(user_type<ccGLWidget>()(), "ccGLWidget");
					m->add(constructor<ccGLWidget(ccGLWindow*, QWidget*)>(), "ccGLWidget");

					m->add(fun(&ccGLWidget::associatedWindow), "associatedWindow");
					m->add(fun(&ccGLWidget::FromWidget), "FromWidget");
					m->add(fun(&ccGLWidget::setAssociatedWindow), "setAssociatedWindow");

					m->add(chaiscript::base_class<QWidget, ccGLWidget>());
					m->add(chaiscript::base_class<QObject, ccGLWidget>());
#endif
					m->add(fun(&CreateGLWindow), "CreateGLWindow");
					m->add(fun([](ccGLWindow*& a, QWidget*& b, bool c) {CreateGLWindow(a, b, c); }), "CreateGLWindow");
					m->add(fun([](ccGLWindow*& a, QWidget*& b) {CreateGLWindow(a, b); }), "CreateGLWindow");
					m->add(fun(&GLWindowFromWidget), "GLWindowFromWidget");

					return m;
				}

				ModulePtr bs_ccGui(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccGui::ParamStruct>(), "ParamStruct");
					m->add(constructor<ccGui::ParamStruct()>(), "ParamStruct");
					m->add(fun(&ccGui::ParamStruct::lightDiffuseColor), "lightDiffuseColor");
					m->add(fun(&ccGui::ParamStruct::lightAmbientColor), "lightAmbientColor");
					m->add(fun(&ccGui::ParamStruct::lightSpecularColor), "lightSpecularColor");
					m->add(fun(&ccGui::ParamStruct::lightDoubleSided), "lightDoubleSided");
					m->add(fun(&ccGui::ParamStruct::meshFrontDiff), "meshFrontDiff");
					m->add(fun(&ccGui::ParamStruct::meshBackDiff), "meshBackDiff");
					m->add(fun(&ccGui::ParamStruct::meshSpecular), "meshSpecular");
					m->add(fun(&ccGui::ParamStruct::textDefaultCol), "textDefaultCol");
					m->add(fun(&ccGui::ParamStruct::pointsDefaultCol), "pointsDefaultCol");
					m->add(fun(&ccGui::ParamStruct::backgroundCol), "backgroundCol");
					m->add(fun(&ccGui::ParamStruct::labelBackgroundCol), "labelBackgroundCol");
					m->add(fun(&ccGui::ParamStruct::labelMarkerCol), "labelMarkerCol");
					m->add(fun(&ccGui::ParamStruct::bbDefaultCol), "bbDefaultCol");
					m->add(fun(&ccGui::ParamStruct::drawBackgroundGradient), "drawBackgroundGradient");
					m->add(fun(&ccGui::ParamStruct::decimateMeshOnMove), "decimateMeshOnMove");
					m->add(fun(&ccGui::ParamStruct::minLoDMeshSize), "minLoDMeshSize");
					m->add(fun(&ccGui::ParamStruct::decimateCloudOnMove), "decimateCloudOnMove");
					m->add(fun(&ccGui::ParamStruct::minLoDCloudSize), "minLoDCloudSize");
					m->add(fun(&ccGui::ParamStruct::displayCross), "displayCross");
					m->add(fun(&ccGui::ParamStruct::useVBOs), "useVBOs");
					m->add(fun(&ccGui::ParamStruct::labelMarkerSize), "labelMarkerSize");
					m->add(fun(&ccGui::ParamStruct::colorScaleShowHistogram), "colorScaleShowHistogram");
					m->add(fun(&ccGui::ParamStruct::colorScaleUseShader), "colorScaleUseShader");
					m->add(fun(&ccGui::ParamStruct::colorScaleShaderSupported), "colorScaleShaderSupported");
					m->add(fun(&ccGui::ParamStruct::colorScaleRampWidth), "colorScaleRampWidth");
					m->add(fun(&ccGui::ParamStruct::defaultFontSize), "defaultFontSize");
					m->add(fun(&ccGui::ParamStruct::labelFontSize), "labelFontSize");
					m->add(fun(&ccGui::ParamStruct::displayedNumPrecision), "displayedNumPrecision");
					m->add(fun(&ccGui::ParamStruct::labelOpacity), "labelOpacity");
					m->add(fun(&ccGui::ParamStruct::zoomSpeed), "zoomSpeed");
					chaiscript::utility::add_class<ccGui::ParamStruct::ComputeOctreeForPicking>(*m,
						"ComputeOctreeForPicking",
						{
							{ ccGui::ParamStruct::ComputeOctreeForPicking::ALWAYS, "ALWAYS" },
							{ ccGui::ParamStruct::ComputeOctreeForPicking::ASK_USER, "ASK_USER" },
							{ ccGui::ParamStruct::ComputeOctreeForPicking::NEVER, "NEVER" }
						}
					);
					m->add(fun(&ccGui::ParamStruct::autoComputeOctree), "autoComputeOctree");
					m->add(fun(&ccGui::ParamStruct::drawRoundedPoints), "drawRoundedPoints");
					m->add(fun(&ccGui::ParamStruct::reset), "reset");
					m->add(fun(&ccGui::ParamStruct::fromPersistentSettings), "fromPersistentSettings");
					m->add(fun(&ccGui::ParamStruct::toPersistentSettings), "toPersistentSettings");
					m->add(fun(&ccGui::ParamStruct::isInPersistentSettings), "isInPersistentSettings");
					m->add(fun(&ccGui::Parameters), "Parameters");
					m->add(fun(&ccGui::Set), "SetParameters");
					m->add(fun(&ccGui::ReleaseInstance), "ccGui_ReleaseInstance");

					return m;
				}


				ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
				{
					bs_ccGLWindow(m);
					bs_ccGLWidget(m);
					bs_ccGui(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_CLASSES_HPP