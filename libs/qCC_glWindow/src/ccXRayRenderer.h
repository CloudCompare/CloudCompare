#pragma once

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

#include "ccXRaySettings.h"

// Qt
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
	//! Returns the existing renderer for the given view, or nullptr if none is active.
	static ccXRayRenderer* Find(const ccGLWindowInterface& window);

	//! Requires the view's GL context to be current. Returns null for ordinary rendering.
	static ccXRayRenderer* Prepare(ccGLWindowInterface& window);

	//! Release GL resources before context teardown, retaining per-view CPU state.
	static void Reset(ccGLWindowInterface& window);

	//! Release resources and detach the helper before the view itself is destroyed.
	static void Destroy(ccGLWindowInterface& window);

	//! Returns the text color that contrasts with the view's background, or the default if no renderer is active.
	static ccColor::Rgba TextColor(const ccGLWindowInterface& window);

	//! Returns true if the renderer has allocated GL resources for the given view.
	bool hasResources() const
	{
		return m_xRayFbo || m_xRaySceneFbo || m_xRayShader;
	}

	//! Returns the current X-ray settings for the given view.
	const ccXRaySettings& settings() const
	{
		return m_settings;
	}

	//! Returns the current background for the given view, taking the reverse flag into account.
	const ccXRaySettings::Background& background() const
	{
		return m_settings.reverse ? m_settings.reverseBackground : m_settings.background;
	}

	//! Restores the scene to its original state after X-ray rendering, if possible.
	void restoreScene();
	//! Draws the X-ray effect for the given context.
	void draw(ccGLDrawContext& context);

  protected: // methods
	//! Constructor
	explicit ccXRayRenderer(ccGLWindowInterface& window);
	//! Destructor
	~ccXRayRenderer() override;

	//! Initializes GL resources for the given viewport size. Returns false if floating-point rendering is unavailable.
	bool init(int width, int height);
	//! Releases GL resources, but retains per-view CPU state.
	void release();
	//! Draws the X-ray effect for the given context, optionally compositing with the original scene.
	void drawTexture(bool composite);
	//! Draws the clouds in the current scene, using the current context.
	void drawClouds(ccGLDrawContext& context);

  protected: // members
	ccGLWindowInterface& m_window;
	ccXRaySettings       m_settings;
	ccFrameBufferObject* m_xRayFbo         = nullptr;
	ccFrameBufferObject* m_xRaySceneFbo    = nullptr;
	ccShader*            m_xRayShader      = nullptr;
	bool                 m_xRayUnavailable = false;
	bool                 m_xRaySceneValid  = false;
};
