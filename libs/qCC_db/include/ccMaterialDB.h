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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Always first
// #include "ccIncludeGL.h"
#include "ccLog.h"

// Qt
#include <QFileInfo>
#include <QFileSystemWatcher>
#include <QImage>
#include <QOpenGLTexture>

//! Smart texture database
class ccMaterialDB : public QObject
{
	Q_OBJECT

  public:
	//! Default constructor
	ccMaterialDB()
	    : m_initialized(false)
	{
	}

	//! Initializes the database (connects the file watcher)
	void init()
	{
		if (!m_initialized)
		{
			connect(&m_watcher, &QFileSystemWatcher::fileChanged, this, &ccMaterialDB::onFileChanged);
			m_initialized = true;
		}
	}

	//! Callback called when a file has changed (deleted, renamed or updated)
	void onFileChanged(const QString& filename)
	{
		if (!m_textures.contains(filename))
		{
			assert(false);
			m_watcher.removePath(filename);
			return;
		}

		if (QFileInfo(filename).exists()) // make sure the image still exists
		{
			ccLog::Warning(tr("File '%1' has been updated").arg(filename));
			QImage image;
			if (image.load(filename))
			{
				// update the texture
				m_textures[filename].image = image;
				openGLTextures.remove(filename);
			}
			else
			{
				ccLog::Warning(tr("Failed to load the new version of the file"));
			}
		}
		else
		{
			ccLog::Warning(tr("File '%1' has been deleted or renamed").arg(filename));
		}
	}

	//! Returns whether the database contains a texture associated with the given filename.
	inline bool hasTexture(const QString& filename) const
	{
		return m_textures.contains(filename);
	}

	//!	Returns the texture associated with the given filename (if any).
	inline QImage getTexture(const QString& filename) const
	{
		return m_textures.contains(filename) ? m_textures[filename].image : QImage();
	}

	//! Add a texture associated with the given filename to the database.
	void addTexture(const QString& filename, const QImage& image)
	{
		if (!m_initialized)
			init();

		if (m_textures.contains(filename))
		{
			++m_textures[filename].counter;
		}
		else
		{
			m_textures[filename].image   = image;
			m_textures[filename].counter = 1;
			m_watcher.addPath(filename);
		}
	}

	//!	Increase the texture counter associated with the given filename.
	void increaseTextureCounter(const QString& filename)
	{
		if (m_textures.contains(filename))
		{
			assert(m_textures[filename].counter >= 1);
			++m_textures[filename].counter;
		}
		else
		{
			assert(false);
		}
	}

	//!	Decrease the texture counter associated with the given filename.
	void releaseTexture(const QString& filename)
	{
		if (m_textures.contains(filename))
		{
			if (m_textures[filename].counter > 1)
			{
				--m_textures[filename].counter;
			}
			else
			{
				removeTexture(filename);
			}
		}
	}

	//!	Remove the texture associated with the given filename from the database.
	void removeTexture(const QString& filename)
	{
		m_textures.remove(filename);
		m_watcher.removePath(filename);

		assert(QOpenGLContext::currentContext());
		openGLTextures.remove(filename);
	}

	//! Add an OpenGL texture associated with the given filename to the database.
	void addOpenGLTexture(const QString& filename, QSharedPointer<QOpenGLTexture> texture)
	{
		openGLTextures[filename] = texture;
	}

	//! Remove the OpenGL texture associated with the given filename from the database.
	void removeOpenGLTexture(const QString& filename)
	{
		openGLTextures.remove(filename);
	}

	//!	Returns the OpenGL texture associated with the given filename (if any).
	QSharedPointer<QOpenGLTexture> getOpenGLTexture(const QString& filename) const
	{
		return openGLTextures.contains(filename) ? openGLTextures[filename] : nullptr;
	}

	//!	Releases all OpenGL textures associated with the database.
	void releaseAllOpenGLTextures()
	{
		openGLTextures.clear();
	}

  protected:
	QMap<QString, QSharedPointer<QOpenGLTexture>> openGLTextures; //!< OpenGL textures associated with the database

	//! Texture information
	struct TextureInfo
	{
		QImage   image;       //!< Texture image
		unsigned counter = 0; //!< Texture counter (number of times the texture is used)
	};

	bool                       m_initialized; //!< Whether the database has been initialized (file watcher connected)
	QFileSystemWatcher         m_watcher;     //!< File system watcher to monitor texture files for changes
	QMap<QString, TextureInfo> m_textures;    //!< Texture information associated with the database
};
