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

#include "ccXRayRenderer.h"

#include "ccGLUtils.h"
#include "ccGLWindowInterface.h"
#include "ccGLWindowSignalEmitter.h"

#include <algorithm>
#include <ccFrameBufferObject.h>
#include <ccGlFilter.h>
#include <ccPointCloud.h>
#include <ccShader.h>
#include <ccXRayFilterSupport.h>
#include <cmath>

namespace
{
	// Match the normal traversal: disabled parents hide their descendants, invisible
	// containers do not. This is a CPU flag check, with no extra drawing or allocation.
	bool HasVisibleXRayCloud(const ccHObject* object, const ccGenericGLDisplay* display)
	{
		if (!object || !object->isEnabled())
			return false;
		if (object->isVisible() && object->getDisplay() == display
		    && object->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			const auto* cloud = static_cast<const ccPointCloud*>(object);
			if (cloud->isXRayEnabled() && cloud->size() != 0)
				return true;
		}
		for (unsigned i = 0; i < object->getChildrenNumber(); ++i)
		{
			if (HasVisibleXRayCloud(object->getChild(i), display))
				return true;
		}
		return false;
	}
} // namespace

ccXRayRenderer::ccXRayRenderer(ccGLWindowInterface& window)
    : QObject(window.signalEmitter())
    , m_window(window)
{
}

ccXRayRenderer::~ccXRayRenderer()
{
	// Destroy/Reset release GL objects while the owning context is current.
	assert(!hasResources());
}

ccXRayRenderer* ccXRayRenderer::Find(const ccGLWindowInterface& window)
{
	return window.signalEmitter()->findChild<ccXRayRenderer*>(QString(), Qt::FindDirectChildrenOnly);
}

ccXRayRenderer* ccXRayRenderer::Prepare(ccGLWindowInterface& window)
{
	auto*       renderer  = Find(window);
	const auto* support   = dynamic_cast<const ccXRayFilterSupport*>(window.m_activeGLFilter);
	const bool  requested = !(renderer && renderer->m_xRayUnavailable)
	                       && (!support || support->supportsXRay())
	                       && (HasVisibleXRayCloud(window.m_globalDBRoot, &window) || HasVisibleXRayCloud(window.m_winDBRoot, &window));
	if (!requested)
	{
		if (renderer)
			renderer->release();
		return nullptr;
	}
	if (!renderer)
		renderer = new ccXRayRenderer(window);
	renderer->m_settings = ccXRaySettings::Parameters(&window);
	GLint viewport[4];
	window.functions()->glGetIntegerv(GL_VIEWPORT, viewport);
	return renderer->init(viewport[2], viewport[3]) ? renderer : nullptr;
}

void ccXRayRenderer::Reset(ccGLWindowInterface& window)
{
	if (auto* renderer = Find(window))
	{
		renderer->release();
		renderer->m_xRayUnavailable = false;
	}
}

void ccXRayRenderer::Destroy(ccGLWindowInterface& window)
{
	if (auto* renderer = Find(window))
	{
		renderer->release();
		delete renderer;
	}
}

void ccXRayRenderer::restoreScene()
{
	if (m_xRaySceneValid && m_window.m_currentLODState.level > 0 && !m_window.m_stereoModeEnabled)
		drawTexture(false);
}

ccColor::Rgba ccXRayRenderer::TextColor(const ccGLWindowInterface& window)
{
	const auto& parameters = window.getDisplayParameters();
	const auto* renderer   = Find(window);
	if (!renderer || !renderer->m_xRayFbo)
		return parameters.textDefaultCol;
	const auto& background = renderer->background();
	auto        luminance  = [](const ccColor::Rgb& color)
	{
		auto linear = [](unsigned char value)
		{
			const double s = value / 255.0;
			return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4);
		};
		return 0.2126 * linear(color.r) + 0.7152 * linear(color.g) + 0.0722 * linear(color.b);
	};
	const double top      = luminance(background.color);
	const double bottom   = background.gradient ? luminance(background.secondColor) : top;
	auto         contrast = [&](double text)
	{
		auto ratio = [text](double backdrop)
		{ return (std::max(text, backdrop) + 0.05) / (std::min(text, backdrop) + 0.05); };
		return std::min(ratio(top), ratio(bottom));
	};
	if (contrast(luminance(parameters.textDefaultCol)) >= 3.0)
		return parameters.textDefaultCol;
	return contrast(0.0) >= contrast(1.0) ? ccColor::black : ccColor::white;
}

void ccXRayRenderer::release()
{
	// The accumulation buffer borrows the opaque scene's depth attachment.
	delete m_xRayFbo;
	m_xRayFbo = nullptr;
	delete m_xRaySceneFbo;
	m_xRaySceneFbo = nullptr;
	delete m_xRayShader;
	m_xRayShader     = nullptr;
	m_xRaySceneValid = false;
}

bool ccXRayRenderer::init(int w, int h)
{
	if (m_xRayUnavailable || w <= 0 || h <= 0)
		return false;
	auto*                glFunc      = m_window.functions();
	ccFrameBufferObject* previousFbo = m_window.m_activeFbo;
	glFunc->glPushAttrib(GL_TEXTURE_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
	glFunc->glActiveTexture(GL_TEXTURE0);
	bool success = true;
	if (!m_xRayShader)
	{
		m_xRayShader = new ccShader();
		success      = m_xRayShader->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                                        "#version 120\n"
		                                                     "void main() { gl_Position = ftransform(); gl_TexCoord[0] = gl_MultiTexCoord0; }\n")
		          && m_xRayShader->addShaderFromSourceCode(QOpenGLShader::Fragment,
		                                                   "#version 120\n"
		                                                   "uniform sampler2D uColor;\n"
		                                                   "uniform sampler2D uDepth;\n"
		                                                   "uniform bool uComposite;\n"
		                                                   "void main() {\n"
		                                                   "  vec4 color = texture2D(uColor, gl_TexCoord[0].xy);\n"
		                                                   "  if (uComposite) {\n"
		                                                   "    gl_FragColor = vec4(color.rgb, 1.0);\n"
		                                                   "    gl_FragDepth = gl_FragCoord.z;\n"
		                                                   "  } else {\n"
		                                                   "    gl_FragColor = color;\n"
		                                                   "    gl_FragDepth = texture2D(uDepth, gl_TexCoord[0].xy).r;\n"
		                                                   "  }\n"
		                                                   "}\n")
		          && m_xRayShader->link();
	}
	if (success && (!m_xRayFbo || m_xRayFbo->width() != static_cast<unsigned>(w) || m_xRayFbo->height() != static_cast<unsigned>(h)))
	{
		delete m_xRayFbo;
		m_xRayFbo = new ccFrameBufferObject();
		delete m_xRaySceneFbo;
		m_xRaySceneFbo   = new ccFrameBufferObject();
		m_xRaySceneValid = false;
		// Preserve the opaque image/depth between progressive LOD updates. The
		// original destination may be a window, a stereo eye or an export FBO.
		success = m_xRaySceneFbo->init(w, h)
		          && m_xRaySceneFbo->initColor(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE)
		          && m_xRaySceneFbo->initDepth()
		          && m_xRayFbo->init(w, h)
		          && m_xRayFbo->initColor(GL_RGBA32F, GL_RGBA, GL_FLOAT)
		          && m_xRayFbo->attachDepth(m_xRaySceneFbo->getDepthTexture());
	}
	m_window.bindFBO(previousFbo);
	glFunc->glPopAttrib();
	if (!success)
	{
		ccLog::Warning("[X-ray] Floating-point rendering is unavailable; using the normal point display.");
		release();
		m_xRayUnavailable = true;
	}
	return success;
}

void ccXRayRenderer::drawTexture(bool composite)
{
	auto* glFunc          = m_window.functions();
	GLint previousProgram = 0;
	glFunc->glGetIntegerv(GL_CURRENT_PROGRAM, &previousProgram);
	glFunc->glPushAttrib(GL_ALL_ATTRIB_BITS);
	glFunc->glActiveTexture(GL_TEXTURE0);
	glFunc->glDisable(GL_LIGHTING);
	glFunc->glDisable(GL_ALPHA_TEST);
	glFunc->glDisable(GL_CULL_FACE);
	glFunc->glDisable(GL_STENCIL_TEST);
	glFunc->glDisable(GL_SCISSOR_TEST);
	glFunc->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	GLint clipPlaneCount = 0;
	glFunc->glGetIntegerv(GL_MAX_CLIP_PLANES, &clipPlaneCount);
	for (GLint plane = 0; plane < clipPlaneCount; ++plane)
		glFunc->glDisable(GL_CLIP_PLANE0 + plane);
	if (composite)
	{
		glFunc->glDisable(GL_DEPTH_TEST);
		glFunc->glDepthMask(GL_FALSE);
		glFunc->glEnable(GL_BLEND);
		glFunc->glBlendEquation(GL_FUNC_ADD);
		if (m_settings.reverse)
			glFunc->glBlendFuncSeparate(GL_ZERO, GL_SRC_COLOR, GL_ZERO, GL_ONE);
		else
			glFunc->glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_COLOR, GL_ZERO, GL_ONE);
	}
	else
	{
		glFunc->glEnable(GL_DEPTH_TEST);
		glFunc->glDepthFunc(GL_ALWAYS);
		glFunc->glDepthMask(GL_TRUE);
		glFunc->glDisable(GL_BLEND);
	}
	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glPushMatrix();
	glFunc->glLoadIdentity();
	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	glFunc->glLoadIdentity();
	m_xRayShader->bind();
	m_xRayShader->setUniformValue("uColor", 0);
	m_xRayShader->setUniformValue("uDepth", 1);
	m_xRayShader->setUniformValue("uComposite", composite);
	glFunc->glActiveTexture(GL_TEXTURE1);
	glFunc->glBindTexture(GL_TEXTURE_2D, m_xRaySceneFbo->getDepthTexture());
	glFunc->glActiveTexture(GL_TEXTURE0);
	ccGLUtils::DisplayTexture2DPosition(composite ? m_xRayFbo->getColorTexture() : m_xRaySceneFbo->getColorTexture(), -1, -1, 2, 2);
	m_xRayShader->release();
	glFunc->glPopMatrix();
	glFunc->glMatrixMode(GL_PROJECTION);
	glFunc->glPopMatrix();
	glFunc->glPopAttrib();
	glFunc->glUseProgram(previousProgram);
}

void ccXRayRenderer::draw(CC_DRAW_CONTEXT& context)
{
	assert(m_xRayFbo && m_xRayShader);
	auto*                glFunc          = m_window.functions();
	ccFrameBufferObject* previousFbo     = m_window.m_activeFbo;
	GLint                previousProgram = 0;
	GLint                viewport[4];
	GLint                drawBuffer = 0;
	glFunc->glGetIntegerv(GL_CURRENT_PROGRAM, &previousProgram);
	glFunc->glGetIntegerv(GL_VIEWPORT, viewport);
	glFunc->glGetIntegerv(GL_DRAW_BUFFER, &drawBuffer);
	glFunc->glPushAttrib(GL_ALL_ATTRIB_BITS);
	glFunc->glActiveTexture(GL_TEXTURE0);
	glFunc->glReadBuffer(drawBuffer);
	// Copy on the GPU, including the viewport offset for side-by-side stereo.
	glFunc->glBindTexture(GL_TEXTURE_2D, m_xRaySceneFbo->getColorTexture());
	glFunc->glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, viewport[0], viewport[1], viewport[2], viewport[3]);
	glFunc->glBindTexture(GL_TEXTURE_2D, m_xRaySceneFbo->getDepthTexture());
	glFunc->glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, viewport[0], viewport[1], viewport[2], viewport[3]);
	glFunc->glBindTexture(GL_TEXTURE_2D, 0);
	m_xRaySceneValid = true;
	m_window.bindFBO(m_xRayFbo);
	glFunc->glViewport(0, 0, viewport[2], viewport[3]);
	glFunc->glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glFunc->glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glFunc->glDisable(GL_SCISSOR_TEST);
	glFunc->glEnable(GL_DEPTH_TEST);
	glFunc->glDepthMask(GL_FALSE);
	glFunc->glDisable(GL_LIGHTING);
	glFunc->glDisable(GL_ALPHA_TEST);
	const float neutral = m_settings.reverse ? 1.0f : 0.0f;
	glFunc->glClearColor(neutral, neutral, neutral, 0);
	glFunc->glClear(GL_COLOR_BUFFER_BIT);
	glFunc->glEnable(GL_BLEND);
	glFunc->glBlendEquation(GL_FUNC_ADD);
	// Screen accumulates on black; Multiply accumulates on white. The point
	// shader includes exposure and alpha in the corresponding blend factor.
	if (m_settings.reverse)
		glFunc->glBlendFuncSeparate(GL_ZERO, GL_SRC_COLOR, GL_ZERO, GL_ONE);
	else
		glFunc->glBlendFuncSeparate(GL_ONE, GL_ONE_MINUS_SRC_COLOR, GL_ZERO, GL_ONE);

	// X-ray is recomputed against the latest opaque depth at every LOD update.
	// It uses a compensated stride while moving and full density at rest, without
	// modifying the ordinary clouds' progressive LOD state.
	CC_DRAW_CONTEXT xRayContext = context;
	xRayContext.drawingFlags |= CC_DRAW_XRAY_PASS;
	xRayContext.drawingFlags &= ~CC_LOD_ACTIVATED;
	xRayContext.currentLODLevel     = 0;
	xRayContext.decimateCloudOnMove = context.decimateCloudOnMove && m_window.m_mouseMoved && !m_window.m_captureMode.enabled;
	auto drawClouds                 = [&]()
	{
		if (m_window.m_globalDBRoot)
			m_window.m_globalDBRoot->draw(xRayContext);
		if (m_window.m_winDBRoot)
			m_window.m_winDBRoot->draw(xRayContext);
	};
	drawClouds();
	m_window.bindFBO(previousFbo);
	glFunc->glPopAttrib();
	glFunc->glUseProgram(previousProgram);
	drawTexture(true);

	// Populate the final scene depth only AFTER accumulation. Picking, automatic
	// pivot placement and depth filters retain a surface depth for X-ray clouds,
	// while X-ray points never occlude each other during their color pass.
	glFunc->glPushAttrib(GL_ALL_ATTRIB_BITS);
	glFunc->glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glFunc->glEnable(GL_DEPTH_TEST);
	glFunc->glDepthMask(GL_TRUE);
	glFunc->glDisable(GL_BLEND);
	xRayContext.drawingFlags &= ~CC_DRAW_XRAY_ENABLED;
	drawClouds();
	glFunc->glPopAttrib();
	glFunc->glUseProgram(previousProgram);
}
