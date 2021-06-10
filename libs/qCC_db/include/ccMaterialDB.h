//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
//#include "ccIncludeGL.h"
#include "ccLog.h"

//Qt
#include <QFileSystemWatcher>
#include <QFileInfo>
#include <QImage>
#include <QOpenGLTexture>

//! Smart texture database
class ccMaterialDB : public QObject
{
	Q_OBJECT

public:

	ccMaterialDB()
		: m_initialized(false)
	{}

	void init()
	{
		if (!m_initialized)
		{
			connect(&m_watcher, &QFileSystemWatcher::fileChanged, this, &ccMaterialDB::onFileChanged);
			m_initialized = true;
		}
	}

	void onFileChanged(const QString& filename)
	{
		if (!m_textures.contains(filename))
		{
			assert(false);
			m_watcher.removePath(filename);
			return;
		}
		
		if (QFileInfo(filename).exists()) //make sure the image still exists
		{
			ccLog::Warning(tr("File '%1' has been updated").arg(filename));
			QImage image;
			if (image.load(filename))
			{
				//update the texture
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

	inline bool hasTexture(const QString& filename) const
	{
		return m_textures.contains(filename);
	}

	inline QImage getTexture(const QString& filename) const
	{
		return m_textures.contains(filename) ? m_textures[filename].image : QImage();
	}

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
			m_textures[filename].image = image;
			m_textures[filename].counter = 1;
			m_watcher.addPath(filename);
		}
	}

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

	void removeTexture(const QString& filename)
	{
		m_textures.remove(filename);
		m_watcher.removePath(filename);

		assert(QOpenGLContext::currentContext());
		openGLTextures.remove(filename);
	}

	QMap<QString, QSharedPointer<QOpenGLTexture> > openGLTextures;

protected:

	struct TextureInfo
	{
		QImage image;
		unsigned counter = 0;
	};

	bool m_initialized;
	QFileSystemWatcher m_watcher;
	QMap<QString, TextureInfo> m_textures;
};
