//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: PuzzlerPlugin                      #
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
//#                             COPYRIGHT: DGM                             #
//#                                                                        #
//##########################################################################

#include <QtGui>
#include <QInputDialog>
#include <QMainWindow>
#include <QFileDialog>
#include <QProgressDialog>

#include <ccImage.h>
#include <ccHObjectCaster.h>

#include "qPuzzler.h"

PuzzlerPlugin::PuzzlerPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/puzzler/info.json" )
	, m_action( nullptr )
{
}

void PuzzlerPlugin::onNewSelection( const ccHObject::Container& selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}

	m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::IMAGE));
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> PuzzlerPlugin::getActions()
{
	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &PuzzlerPlugin::doAction );
	}

	return { m_action };
}

struct Nipple
{
	int dx = 0;
	int dy = 0;
	bool outside = false;

	enum Side { Top, Bottom, Left, Right, Undefined };
	Side side = Undefined;

	bool isValid() const { return side != Undefined; }

	void randomize(Side s, int radius)
	{
		outside = ((rand() & 1) == 1); //randomize the fact that the nipple is pointing inside or outside
		int extShift = static_cast<int>((-0.5 + (rand() / static_cast<double>(RAND_MAX))) * 0.75 * radius);
		int lateralShift = static_cast<int>((-0.5 + (rand() / static_cast<double>(RAND_MAX))) * (2 * radius));
		side = s;

		switch (s)
		{
		case Top:
			dx = lateralShift;
			dy = outside ? extShift : -extShift;
			break;
		case Bottom:
			dx = lateralShift;
			dy = outside ? -extShift : extShift;
			break;
		case Left:
			dx = outside ? -extShift : extShift;
			dy = lateralShift;
			break;
		case Right:
			dx = outside ? extShift : -extShift;
			dy = lateralShift;
			break;
		case Undefined:
			assert(false);
			dx = dy = 0;
		}
	}

	Nipple opposite(int radius) const
	{
		Nipple op;
		op.outside = !outside;

		switch (side)
		{
		case Top:
			op.dx = dx;
			op.dy = dy;
			op.side = Bottom;
			break;
		case Bottom:
			op.dx = dx;
			op.dy = dy;
			op.side = Top;
			break;
		case Left:
			op.dx = dx;
			op.dy = dy;
			op.side = Right;
			break;
		case Right:
			op.dx = dx;
			op.dy = dy;
			op.side = Left;
			break;
		}
		return op;
	}
};

struct Tile
{
	Nipple leftNipple, rightNipple, topNipple, bottomNipple;

	static bool InsideDisc(int x, int y, int cx, int cy, int radius)
	{
		int dx = cx - x;
		int dy = cy - y;
		int dr2 = dx * dx + dy * dy;
		return dr2 <= radius * radius;
	}

	bool inTile(int x, int y, int x0, int y0, int x1, int y1, int nippleRadius) const
	{
		if (x < x0)
		{
			//outside left nipple
			assert(leftNipple.isValid() && leftNipple.outside);
			return InsideDisc(x, y, x0 + leftNipple.dx, (y0 + y1) / 2 + leftNipple.dy, nippleRadius);
		}
		else if (x > x1)
		{
			//outside right nipple
			assert(rightNipple.isValid() && rightNipple.outside);
			return InsideDisc(x, y, x1 + rightNipple.dx, (y0 + y1) / 2 + rightNipple.dy, nippleRadius);
		}
		if (y < y0)
		{
			//outside bottom nipple
			assert(bottomNipple.isValid() && bottomNipple.outside);
			return InsideDisc(x, y, (x0 + x1) / 2 + bottomNipple.dx, y0 + bottomNipple.dy, nippleRadius);
		}
		else if (y > y1)
		{
			//outside top nipple
			assert(topNipple.isValid() && topNipple.outside);
			return InsideDisc(x, y, (x0 + x1) / 2 + topNipple.dx, y1 + topNipple.dy, nippleRadius);
		}
		//we are inside the tile

		if (leftNipple.isValid() && !leftNipple.outside && InsideDisc(x, y, x0 + leftNipple.dx, (y0 + y1) / 2 + leftNipple.dy, nippleRadius - 1))
		{
			//inside left nipple
			return false;
		}
		if (rightNipple.isValid() && !rightNipple.outside && InsideDisc(x, y, x1 + rightNipple.dx, (y0 + y1) / 2 + rightNipple.dy, nippleRadius - 1))
		{
			//inside right nipple
			return false;
		}
		if (bottomNipple.isValid() && !bottomNipple.outside && InsideDisc(x, y, (x0 + x1) / 2 + bottomNipple.dx, y0 + bottomNipple.dy, nippleRadius - 1))
		{
			//inside bottom nipple
			return false;
		}
		if (topNipple.isValid() && !topNipple.outside && InsideDisc(x, y, (x0 + x1) / 2 + topNipple.dx, y1 + topNipple.dy, nippleRadius - 1))
		{
			//inside top nipple
			return false;
		}

		return true;
	}
};

void PuzzlerPlugin::doAction()
{	
	if ( m_app == nullptr )
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT( false );
		
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	//process images only
	if (selectedEntities.size() != 1 || !selectedEntities.front()->isA(CC_TYPES::IMAGE))
	{
		assert(false);
		return;
	}

	ccImage* image = ccHObjectCaster::ToImage(selectedEntities.front());
	if (image == nullptr || image->getW() == 0 || image->getH() == 0)
	{
		ccLog::Error("Invalid image");
		assert(false);
		return;
	}

	const QImage& qImage = image->data();
	ccLog::Print(QString("Image size: (%1 x %2)").arg(qImage.width()).arg(qImage.height()));

	static int s_tileCount = 40;
	bool ok = false;
	int tileCount = QInputDialog::getInt(m_app->getMainWindow(), "Tile count", "Count", s_tileCount, 2, 1000, 10, &ok);

	if (!ok)
	{
		//Cancelled by user
		return;
	}
	s_tileCount = tileCount;
	ccLog::Print(QString("Tile count: %1").arg(tileCount));

	//let's estimate the tile dimensions
	int nW = 0, nH = 0;
	int tileWidth_pix = 0;
	{
		int surface_pix = qImage.width() * qImage.height();
		double tileSurface_pix = surface_pix / static_cast<double>(tileCount);
		tileWidth_pix = static_cast<int>(sqrt(tileSurface_pix));
		nW = std::max(4, static_cast<int>(std::round(qImage.width() / tileWidth_pix)));
		nH = std::max(4, static_cast<int>(std::round(qImage.height() / tileWidth_pix)));
	}
	int tileW = qImage.width() / nW;
	int tileH = qImage.height() / nH;

	std::vector<Tile> tiles;
	try
	{
		tiles.resize(nW * nH);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memoery");
		return;
	}
	int nippleRadius = std::max(tileWidth_pix / 6, 1);
	for (int i = 0; i < nW; ++i)
	{
		for (int j = 0; j < nH; ++j)
		{
			Tile& tile = tiles[j * nW + i];
			if (i + 1 != nW)
			{
				tile.rightNipple.randomize(Nipple::Right, nippleRadius);
				tiles[j * nW + i + 1].leftNipple = tile.rightNipple.opposite(nippleRadius);
			}
			if (j + 1 != nH)
			{
				tile.topNipple.randomize(Nipple::Top, nippleRadius);
				tiles[(j + 1) * nW + i].bottomNipple = tile.topNipple.opposite(nippleRadius);
			}
		}
	}

	static QString s_outputDir;
	if (s_outputDir.isEmpty())
	{
		s_outputDir = QCoreApplication::applicationDirPath();
	}
	QString outputDir = QFileDialog::getExistingDirectory(getMainAppInterface()->getMainWindow(), "Output folder", s_outputDir);
	if (!outputDir.isEmpty())
	{
		s_outputDir = outputDir;
	}
#ifndef DEBUG_TILING
	else
	{
		//nothing to do?
		return;
	}
#else
	QImage testImage = qImage;
#endif

	QProgressDialog pDlg("Export tiles", "Cancel", 0, static_cast<int>(tiles.size()), getMainAppInterface()->getMainWindow());
	pDlg.show();
	QCoreApplication::processEvents();

	bool scrambleSuffix = true;
	int progressCounter = 0;
	for (int i = 0; i < nW; ++i)
	{
		int dxStart = i * tileW;
		int dxStop = (i + 1 == nW ? qImage.width() - 1 : dxStart + tileW - 1);

		for (int j = 0; j < nH; ++j)
		{
			int tileIndex = j * nW + i;
			Tile& tile = tiles[tileIndex];

#ifdef DEBUG_TILING
			QRgb tileColor = qRgb(64 + rand() % 192, 64 + rand() % 192, 64 + rand() % 192);
#endif

			int dyStart = j * tileH;
			int dyStop = (j + 1 == nH ? qImage.height() - 1 : dyStart + tileH - 1);

			int minX = dxStart;
			int minY = dyStart;
			int maxX = dxStop;
			int maxY = dyStop;

			if (tile.leftNipple.isValid() && tile.leftNipple.outside)
				minX = std::min(minX, minX + tile.leftNipple.dx - nippleRadius);
			if (tile.rightNipple.isValid() && tile.rightNipple.outside)
				maxX = std::max(maxX, maxX + tile.rightNipple.dx + nippleRadius);
			if (tile.bottomNipple.isValid() && tile.bottomNipple.outside)
				minY = std::min(minY, minY + tile.bottomNipple.dy - nippleRadius);
			if (tile.topNipple.isValid() && tile.topNipple.outside)
				maxY = std::max(maxY, maxY + tile.topNipple.dy + nippleRadius);

			QImage tileImage(QSize(maxX - minX + 1, maxY - minY + 1), QImage::Format::Format_ARGB32);
			tileImage.fill(qRgba(0, 0, 0, 0));

			for (int x = minX; x <= maxX; ++x)
			{
				if (x < 0 || x >= qImage.width())
				{
					//outbounds image
					assert(false);
					continue;
				}
				for (int y = minY; y <= maxY; ++y)
				{
					if (y < 0 || y >= qImage.height())
					{
						//outbounds image
						assert(false);
						continue;
					}

					bool inTile = tile.inTile(x, y, dxStart, dyStart, dxStop, dyStop, nippleRadius);
					if (inTile)
					{
#ifdef DEBUG_TILING
						testImage.setPixel(x, y, tileColor);
#endif
						tileImage.setPixel(x - minX, y - minY, qImage.pixel(x, y));
					}
				}
			}

			if (!outputDir.isEmpty())
			{
				QString suffix;
				if (scrambleSuffix)
				{
					suffix = QUuid::createUuid().toString();
				}
				else
				{
					suffix = QString("%1_%2").arg(i).arg(j);
				}
				tileImage.save(outputDir + "/tile_" + suffix + ".png");
			}

			if (pDlg.wasCanceled())
			{
				//aborted
				i = nW;
				break;
			}
			pDlg.setValue(++progressCounter);
		}
	}

#ifdef DEBUG_TILING
	ccImage* testImageCC = new ccImage(testImage, image->getName() + ".puzzle");
	getMainAppInterface()->addToDB(testImageCC);
#endif
}
