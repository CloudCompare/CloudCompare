//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

//qCC_db
#include <ccIncludeGL.h>
#include <ccLog.h>

//CCLib
#include <CCGeom.h>

//CCFbo
#include <ccFrameBufferObject.h>

//OVRlib
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>
#include <Extras/OVR_Math.h>

//system
#include <vector>

//Oculus SDK 'session'
struct OculusHMD
{
	OculusHMD()
		: session(nullptr)
		, fbo(nullptr)
		, hasTextureSet(false)
		, lastOVRPos(0, 0, 0)
		, hasLastOVRPos(false)
	{
		textureSize.w = textureSize.h = 0;
	}

	~OculusHMD()
	{
		stop(true);
	}

	void setSesion(ovrSession s)
	{
		if (session && session != s)
		{
			//auto-stop
			stop(false);
		}

		session = s;
		if (session)
		{
			ovrHmdDesc hmdDesc    = ovr_GetHmdDesc(session);
			eyeRenderDesc[0]      = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
			eyeRenderDesc[1]      = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);
			hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
			hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;
		}
	}

	bool initTextureSet(QOpenGLContext* context)
	{
		if (!session || !context)
		{
			assert(false);
			return false;
		}

		ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
		ovrSizei recommendedTex0Size = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], 1.0f);
		ovrSizei recommendedTex1Size = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], 1.0f);

		//determine the rendering FOV and allocate the required ovrSwapTextureSet (see https://developer.oculus.com/documentation/pcsdk/latest/concepts/dg-render/)
		ovrSizei bufferSize;
		{
			bufferSize.w  = recommendedTex0Size.w + recommendedTex1Size.w;
			bufferSize.h = std::max(recommendedTex0Size.h, recommendedTex1Size.h);
		}

		if (	!hasTextureSet
			||	!fbo
			||	textureSize.w != bufferSize.w
			||	textureSize.h != bufferSize.h )
		{
			destroyTextureSet();

			ovrTextureSwapChainDesc desc = {};
			desc.Type = ovrTexture_2D;
			desc.ArraySize = 1;
			desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
			desc.Width = bufferSize.w;
			desc.Height = bufferSize.h;
			desc.MipLevels = 1;
			desc.SampleCount = 1;
			desc.StaticImage = ovrFalse;
			if (!ovr_CreateTextureSwapChainGL(session,
				&desc,
				&textureSwapChain) == ovrSuccess)
			{
				return false;
			}
			hasTextureSet = true;

			assert(!fbo);
			fbo = new ccFrameBufferObject;
			if (	!fbo->init(	static_cast<unsigned>(bufferSize.w),
								static_cast<unsigned>(bufferSize.h) )
				//||	!fbo->initDepth()
				)
			{
				destroyTextureSet();
				return false;
			}

			QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
			//we create a depth texture for each color texture
			assert(depthTextures.empty());

			int textureCount = 0;
			ovr_GetTextureSwapChainLength(session, textureSwapChain, &textureCount);			depthTextures.resize(textureCount, 0);
			for (int i = 0; i < textureCount; ++i)
			{
				//set the color texture
				{
					unsigned int texId;
					ovr_GetTextureSwapChainBufferGL(session, textureSwapChain, 0, &texId);					glFunc->glBindTexture(GL_TEXTURE_2D, texId);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR/*GL_LINEAR_MIPMAP_LINEAR*/);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					if (context->hasExtension(QByteArrayLiteral("GLE_EXT_texture_filter_anisotropic")))
					{
						glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 1);
					}
					glFunc->glBindTexture(GL_TEXTURE_2D, 0);
				}

				//create the depth texture
				{
					glFunc->glPushAttrib(GL_ENABLE_BIT);
					glFunc->glEnable(GL_TEXTURE_2D);

					GLuint texID = 0;
					glFunc->glGenTextures(1, &texID);
					glFunc->glBindTexture(GL_TEXTURE_2D, texID);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
					glFunc->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
					glFunc->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
					glFunc->glBindTexture(GL_TEXTURE_2D, 0);

					glFunc->glPopAttrib();

					depthTextures[i] = texID;
				}
			}

			//DGM: doesn't work :(
			//fbo->initDepth();

			textureSize = bufferSize;
		}

		// Initialize our single full screen Fov layer.
		layer.Header.Type      = ovrLayerType_EyeFov;
		layer.Header.Flags     = ovrLayerFlag_TextureOriginAtBottomLeft;
		layer.ColorTexture[0]  = textureSwapChain;
		layer.ColorTexture[1]  = textureSwapChain;
		layer.Fov[0]           = eyeRenderDesc[0].Fov;
		layer.Fov[1]           = eyeRenderDesc[1].Fov;
		layer.Viewport[0].Pos.x = 0;
		layer.Viewport[0].Pos.y = 0;
		layer.Viewport[0].Size  = recommendedTex0Size;
		layer.Viewport[1].Pos.x = recommendedTex0Size.w;
		layer.Viewport[1].Pos.y = 0;
		layer.Viewport[1].Size  = recommendedTex1Size;

		return true;
	}

	bool initMirrorTexture(int w, int h, QOpenGLExtension_ARB_framebuffer_object& glExt)
	{
		if (!session)
		{
			assert(false);
			return false;
		}

		//mirrorTexture
		if (!mirror.texture)
		{
			ovrMirrorTextureDesc desc;
			memset(&desc, 0, sizeof(desc));
			desc.Width = w;
			desc.Height = h;
			desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

			// Create mirror texture and an FBO used to copy mirror texture to back buffer
			ovrResult result = ovr_CreateMirrorTextureGL(session, &desc, &mirror.texture);
			if (OVR_SUCCESS(result))
			{
				// Configure the mirror read buffer
				ovr_GetMirrorTextureBufferGL(session, mirror.texture, &mirror.textureID);
				mirror.size = QSize(w, h);

				// And the FBO
				glExt.glGenFramebuffers(1, &mirror.fbo);
				glExt.glBindFramebuffer(GL_READ_FRAMEBUFFER, mirror.fbo);
				glExt.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirror.textureID, 0);
				glExt.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
				glExt.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

			}
			else
			{
				ccLog::Warning("[Oculus] Failed to create the mirror texture");
				return false;
			}
		}

		return true;
	}

	void releaseMirrorTexture(QOpenGLExtension_ARB_framebuffer_object& glExt)
	{
		if (mirror.fbo)
		{
			glExt.glDeleteFramebuffers(1, &mirror.fbo);
			mirror.fbo = 0;
		}
		
		if (mirror.texture)
		{
			ovr_DestroyMirrorTexture(session, mirror.texture);
			mirror.texture = nullptr;
		}
	}

	void stop(bool autoShutdown = true)
	{
		if (session)
		{ 
			//destroy the textures (if any)
			destroyTextureSet();

			//then destroy the session
			ovr_Destroy(session);
			session = 0;
		}

		if (autoShutdown)
		{
			ovr_Shutdown();
		}
	}

	//! Destroy the textures (if any)
	void destroyTextureSet(QOpenGLFunctions_2_1* glFunc = 0)
	{
		if (fbo)
		{
			delete fbo;
			fbo = 0;
		}

		if (hasTextureSet)
		{
			ovr_DestroyTextureSwapChain(session, textureSwapChain);
			hasTextureSet = false;
		}

		if (!depthTextures.empty())
		{
			QOpenGLContext* context = QOpenGLContext::currentContext();
			if (context)
			{
				QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
				for (size_t i = 0; i < depthTextures.size(); ++i)
				{
					glFunc->glDeleteTextures(1, &(depthTextures[i]));
				}
			}
			depthTextures.clear();
		}

		textureSize.w = textureSize.h = 0;
	}

	//! Session handle
	ovrSession session;

	//! Dedicated FBO
	ccFrameBufferObject* fbo;

	//! Mirror texture
	struct Mirror
	{
		//! texture
		ovrMirrorTexture texture = nullptr;
		//! texture ID
		GLuint textureID = 0;
		//! texture size
		QSize size;
		//! FBO
		GLuint fbo = 0;

	};
	
	//! Mirror
	Mirror mirror;

	//! Color texture(s)
	ovrTextureSwapChain textureSwapChain;
	//! Whether color texture(s) are available
	bool hasTextureSet;
	//! Depth texture(s)
	std::vector<GLuint> depthTextures;
	//! Texture(s) size
	ovrSizei textureSize;

	//stereo pair rendering parameters
	ovrEyeRenderDesc eyeRenderDesc[2];
	ovrVector3f      hmdToEyeViewOffset[2];
	ovrLayerEyeFov   layer;

	//! Last sensor position
	CCVector3d lastOVRPos;
	//! Whether a position has been already recorded or not
	bool hasLastOVRPos;
};

static ccGLMatrixd FromOVRMat(const OVR::Matrix4f& ovrMat)
{
	ccGLMatrixd ccMat;
	double* data = ccMat.data();
	data[0] = ovrMat.M[0][0]; data[4] = ovrMat.M[0][1]; data[ 8] = ovrMat.M[0][2]; data[12] = ovrMat.M[0][3];
	data[1] = ovrMat.M[1][0]; data[5] = ovrMat.M[1][1];	data[9] = ovrMat.M[1][2]; data[13] = ovrMat.M[1][3];
	data[2] = ovrMat.M[2][0]; data[6] = ovrMat.M[2][1];	data[10] = ovrMat.M[2][2]; data[14] = ovrMat.M[2][3];
	data[3] = ovrMat.M[3][0]; data[7] = ovrMat.M[3][1];	data[11] = ovrMat.M[3][2]; data[15] = ovrMat.M[3][3];

	return ccMat;
}

static OVR::Matrix4f ToOVRMat(const ccGLMatrixd& ccMat)
{
	const double* M = ccMat.data();
	return OVR::Matrix4f(M[0], M[4], M[8], M[12],
		M[1], M[5], M[9], M[13],
		M[2], M[6], M[10], M[14],
		M[3], M[7], M[11], M[15]);
}

static void OVR_CDECL LogCallback(uintptr_t /*userData*/, int level, const char* message)
{
	switch (level)
	{
	case ovrLogLevel_Debug:
		ccLog::PrintDebug(QString("[oculus] ") + message);
		break;
	case ovrLogLevel_Info:
		ccLog::Print(QString("[oculus] ") + message);
		break;
	case ovrLogLevel_Error:
		ccLog::Warning(QString("[oculus] ") + message);
		break;
	default:
		assert(false);
		break;
	}
}
