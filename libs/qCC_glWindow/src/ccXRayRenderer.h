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

#pragma once

#include "ccXRaySettings.h"

#include <QObject>

class ccGLWindowInterface;
class ccFrameBufferObject;
class ccShader;
struct ccGLDrawContext;

//! Private, lazily created renderer owned by the view's existing signal emitter.
//! GL resources are released explicitly before the view loses its current context.
class CCGLWINDOW_LIB_API ccXRayRenderer : public QObject
{
	Q_OBJECT

  public:
	static ccXRayRenderer* Find(const ccGLWindowInterface& window);
	//! Requires the view's GL context to be current. Returns null for ordinary rendering.
	static ccXRayRenderer* Prepare(ccGLWindowInterface& window);
	//! Release GL resources before context teardown, retaining per-view CPU state.
	static void Reset(ccGLWindowInterface& window);
	//! Release resources and detach the helper before the view itself is destroyed.
	static void          Destroy(ccGLWindowInterface& window);
	static ccColor::Rgba TextColor(const ccGLWindowInterface& window);

	bool hasResources() const
	{
		return m_xRayFbo || m_xRaySceneFbo || m_xRayShader;
	}
	const ccXRaySettings& settings() const
	{
		return m_settings;
	}
	const ccXRaySettings::Background& background() const
	{
		return m_settings.reverse ? m_settings.reverseBackground : m_settings.background;
	}
	void restoreScene();
	void draw(ccGLDrawContext& context);

  private:
	explicit ccXRayRenderer(ccGLWindowInterface& window);
	~ccXRayRenderer() override;
	bool init(int width, int height);
	void release();
	void drawTexture(bool composite);

	ccGLWindowInterface& m_window;
	ccXRaySettings       m_settings;
	ccFrameBufferObject* m_xRayFbo         = nullptr;
	ccFrameBufferObject* m_xRaySceneFbo    = nullptr;
	ccShader*            m_xRayShader      = nullptr;
	bool                 m_xRayUnavailable = false;
	bool                 m_xRaySceneValid  = false;
};
