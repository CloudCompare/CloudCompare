// CloudCompare - GPL-2.0-or-later
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSettings>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QToolBox>
#include <QToolButton>
#include <QtTest>
#include <ccDisplaySettingsDlg.h>
#include <ccEDLFilter.h>
#include <ccFrameBufferObject.h>
#include <ccGLWindow.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <memory>

class XRayTestWindow : public ccGLWindow
{
  public:
	XRayTestWindow()
	    : ccGLWindow(nullptr, nullptr, true)
	{
	}
	bool hasXRayResources() const
	{
		return m_xRayFbo || m_xRaySceneFbo || m_xRayShader;
	}
	ccColor::Rgba overlayColor() const
	{
		return effectiveTextColor();
	}

	// Exercise successive updates of the same framebuffer, as progressive LOD
	// does. Export alone always starts a fresh frame and cannot catch stale depth
	// or repeated compositing of the preceding frame's X-ray color.
	QImage renderLODFrame(unsigned level)
	{
		doMakeCurrent();
		if (!m_fbo && !initFBO(glWidth(), glHeight()))
			return {};
		auto* previousFbo = m_activeFbo;
		bindFBO(m_fbo);
		auto*           gl = functions();
		CC_DRAW_CONTEXT context;
		getContext(context);
		RenderingParams parameters;
		parameters.useFBO      = true;
		parameters.draw3DCross = false;
		if (level == 0)
		{
			gl->glDepthMask(GL_TRUE);
			drawBackground(context, parameters);
		}
		m_currentLODState.level      = level;
		m_currentLODState.inProgress = level != 0;
		draw3D(context, parameters);
		QImage result(glWidth(), glHeight(), QImage::Format_RGBA8888);
		result.fill(Qt::magenta);
		bindFBO(m_fbo);
		gl->glReadBuffer(GL_COLOR_ATTACHMENT0);
		gl->glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
		gl->glReadPixels(0, 0, glWidth(), glHeight(), GL_RGBA, GL_UNSIGNED_BYTE, result.bits());
		const GLenum error = gl->glGetError();
		if (error != GL_NO_ERROR)
			qWarning() << "LOD readback error:" << error;
		bindFBO(previousFbo);
		m_currentLODState = LODState();
		return result.mirrored();
	}
};

class TestXRayDisplay : public QObject
{
	Q_OBJECT
  private Q_SLOTS:
	void settingsAndPreview()
	{
		const auto         original = ccGui::Parameters();
		ccGui::ParamStruct defaults;
		ccGui::Set(defaults);
		ccDisplaySettingsDlg dialog(nullptr);
		dialog.setAttribute(Qt::WA_DontShowOnScreen);
		auto* reverse         = dialog.findChild<QCheckBox*>("xRayReverseCheckBox");
		auto* screenGradient  = dialog.findChild<QCheckBox*>("xRayGradientCheckBox");
		auto* reverseGradient = dialog.findChild<QCheckBox*>("xRayReverseGradientCheckBox");
		auto* screenSecond    = dialog.findChild<QToolButton*>("xRaySecondColorButton");
		auto* reverseSecond   = dialog.findChild<QToolButton*>("xRayReverseSecondColorButton");
		auto* exposure        = dialog.findChild<QDoubleSpinBox*>("xRayExposureSpinBox");
		auto* toolbox         = dialog.findChild<QToolBox*>("toolBox");
		QVERIFY(reverse && screenGradient && reverseGradient && screenSecond && reverseSecond && exposure && toolbox);
		QVERIFY(!screenSecond->isEnabled() && !reverseSecond->isEnabled());
		screenGradient->setChecked(true);
		QVERIFY(screenSecond->isEnabled() && !reverseSecond->isEnabled());
		reverseGradient->setChecked(true);
		QVERIFY(reverseSecond->isEnabled());
		reverse->setChecked(true);
		exposure->setValue(-2.0);
		QVERIFY(ccGui::Parameters().xRayReverse);
		QVERIFY(ccGui::Parameters().xRayBackground.gradient);
		QVERIFY(ccGui::Parameters().xRayReverseBackground.gradient);
		QCOMPARE(ccGui::Parameters().xRayExposure, -2.0);
		QVERIFY(ccGui::Parameters().backgroundCol == defaults.backgroundCol);
		toolbox->setCurrentIndex(toolbox->count() - 1);
		dialog.show();
		QTest::qWait(30);
		QVERIFY(dialog.grab().save("xray-settings.png"));
		dialog.reject(); // includes Escape and window-close path
		QVERIFY(!ccGui::Parameters().xRayReverse);
		QVERIFY(ccGui::Parameters().xRayBackground == defaults.xRayBackground);
		QVERIFY(ccGui::Parameters().xRayReverseBackground == defaults.xRayReverseBackground);
		QCOMPARE(ccGui::Parameters().xRayExposure, defaults.xRayExposure);
		ccGui::Set(original);

		// Write settings in an isolated directory, never into the user's profile.
		QTemporaryDir dir;
		QVERIFY(dir.isValid());
		const auto organization = QCoreApplication::organizationName();
		const auto application  = QCoreApplication::applicationName();
		const auto format       = QSettings::defaultFormat();
		QSettings::setDefaultFormat(QSettings::IniFormat);
		QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, dir.path());
		QCoreApplication::setOrganizationName("XRaySettingsTest");
		QCoreApplication::setApplicationName("RoundTrip");
		defaults.xRayReverse           = true;
		defaults.xRayBackground        = {ccColor::Rgb(10, 20, 30), ccColor::Rgb(40, 50, 60), true};
		defaults.xRayReverseBackground = {ccColor::Rgb(210, 220, 230), ccColor::Rgb(140, 150, 160), false};
		defaults.toPersistentSettings();
		ccGui::ParamStruct loaded;
		loaded.fromPersistentSettings();
		QCoreApplication::setOrganizationName(organization);
		QCoreApplication::setApplicationName(application);
		QSettings::setDefaultFormat(format);
		QVERIFY(loaded.xRayReverse);
		QVERIFY(loaded.xRayBackground == defaults.xRayBackground);
		QVERIFY(loaded.xRayReverseBackground == defaults.xRayReverseBackground);
	}

	void persistenceAndClone()
	{
		ccPointCloud cloud;
		QVERIFY(!cloud.isXRayEnabled());
		QVERIFY(cloud.reserve(1));
		cloud.addPoint(CCVector3(0, 0, 0));
		cloud.setXRayEnabled(true);
		std::unique_ptr<ccPointCloud> clone(cloud.cloneThis());
		QVERIFY(clone && clone->isXRayEnabled());
		clone->setXRayEnabled(false);
		QVERIFY(cloud.isXRayEnabled());
		QVERIFY(!clone->isXRayEnabled());

		QTemporaryFile file;
		QVERIFY(file.open());
		const short version = cloud.minimumFileVersion();
		QVERIFY(cloud.toFile(file, version));
		QVERIFY(file.seek(0));
		QCOMPARE(ccObject::ReadClassIDFromFile(file, version), CC_TYPES::POINT_CLOUD);
		int                                  flags = (sizeof(PointCoordinateType) == 8 ? ccSerializableObject::DF_POINT_COORDS_64_BITS : 0)
		                                             | (sizeof(ScalarType) == 4 ? ccSerializableObject::DF_SCALAR_VAL_32_BITS : 0);
		ccSerializableObject::LoadingContext context(version, flags);
		ccPointCloud                         loaded;
		QVERIFY(loaded.fromFile(file, context));
		QVERIFY(loaded.isXRayEnabled());
		QCOMPARE(loaded.size(), cloud.size());
	}

	void rendering()
	{
		ccGLWindowInterface::SetShaderPath(QStringLiteral(TEST_SHADER_PATH));
		ccHObject      scene("Test scene");
		XRayTestWindow window;
		window.setAttribute(Qt::WA_DontShowOnScreen);
		window.resize(256, 256);
		window.show();
		QTest::qWait(100);
		QVERIFY2(window.isValid(), "A working OpenGL context is required for rendering regression tests");
		window.doMakeCurrent();

		ccGui::ParamStruct parameters;
		QCOMPARE(parameters.xRayExposure, -3.0);
		parameters.backgroundCol          = ccColor::black;
		parameters.drawBackgroundGradient = false;
		parameters.displayCross           = false;
		parameters.decimateCloudOnMove    = false;
		parameters.xRayExposure           = 2;
		window.setDisplayParameters(parameters, true);
		window.setSunLight(false);
		window.setCustomLight(false);
		window.setPivotVisibility(ccGLWindowInterface::PIVOT_HIDE);

		auto makeCloud = [&](const ccColor::Rgb& color, float z)
		{
			auto* cloud = new ccPointCloud();
			cloud->reserve(289);
			for (int y = -8; y <= 8; ++y)
				for (int x = -8; x <= 8; ++x)
					cloud->addPoint(CCVector3(x / 8.0f, y / 8.0f, z));
			cloud->setColor(color);
			cloud->showColors(true);
			cloud->setVisible(true);
			cloud->setPointSize(7);
			cloud->setDisplay(&window);
			scene.addChild(cloud);
			return cloud;
		};
		auto* xray   = makeCloud(ccColor::red, 0.0f);
		auto* opaque = makeCloud(ccColor::green, -0.5f);
		QVERIFY(xray->reserveTheNormsTable());
		for (unsigned i = 0; i < xray->size(); ++i)
			xray->addNorm(CCVector3(1, 0, 0));
		xray->showNormals(true);
		window.setSceneDB(&scene);
		window.setView(CC_TOP_VIEW, false);
		window.zoomGlobal();
		auto capture = [&]()
		{
			window.invalidateViewport();
			window.redraw();
			window.doMakeCurrent();
			return window.renderToImage(1, false, false, true);
		};
		auto center = [](const QImage& image)
		{ return image.pixelColor(image.width() / 2, image.height() / 2); };

		const QImage baseline = capture();
		QVERIFY(!baseline.isNull());
		QVERIFY(!window.hasXRayResources());
		QVERIFY(center(baseline).red() > 200);
		xray->setXRayEnabled(true);
		const QImage front = capture();
		QVERIFY(window.hasXRayResources());
		QVERIFY(center(front).red() > 0);
		QVERIFY(center(front).green() > 200); // opaque surface remains visible beneath X-ray
		QVERIFY(front != baseline);
		scene.swapChildren(0, 1);
		QCOMPARE(capture(), front); // ordinary geometry occludes independently of tree order

		// Point overlaps must follow Screen per color channel, without a final
		// brightness compression. Measure each layer separately against black.
		opaque->setVisible(false);
		const QColor firstLayer  = center(capture());
		auto*        secondCloud = makeCloud(ccColor::Rgb(128, 64, 192), 0.0f);
		secondCloud->setXRayEnabled(true);
		xray->setVisible(false);
		const QColor secondLayer = center(capture());
		xray->setVisible(true);
		const QImage screened = capture();
		const QColor combined = center(screened);
		for (int channel = 0; channel < 3; ++channel)
		{
			const int    a        = channel == 0 ? firstLayer.red() : channel == 1 ? firstLayer.green()
			                                                                       : firstLayer.blue();
			const int    b        = channel == 0 ? secondLayer.red() : channel == 1 ? secondLayer.green()
			                                                                        : secondLayer.blue();
			const int    actual   = channel == 0 ? combined.red() : channel == 1 ? combined.green()
			                                                                     : combined.blue();
			const double expected = a + b - a * b / 255.0;
			QVERIFY2(std::abs(actual - expected) <= 2.0, "Overlapping points must obey the Screen blend equation");
		}
		QVERIFY(combined.red() > firstLayer.red());
		scene.swapChildren(1, 2);
		QCOMPARE(capture(), screened);
		scene.swapChildren(1, 2);
		secondCloud->setColor(ccColor::black);
		QCOMPARE(center(capture()), firstLayer); // black contributes nothing
		secondCloud->setEnabled(false);
		opaque->setVisible(true);
		QCOMPARE(capture(), front);

		// Repeated progressive updates must not accumulate the displayed image.
		const QImage firstLOD = window.renderLODFrame(0);
		const QImage nextLOD  = window.renderLODFrame(1);
		if (nextLOD != firstLOD)
		{
			firstLOD.save("xray-lod-first.png");
			nextLOD.save("xray-lod-next.png");
			qDebug() << "LOD center:" << center(firstLOD) << center(nextLOD);
		}
		QCOMPARE(nextLOD, firstLOD);
		const QImage secondLOD = window.renderLODFrame(2);
		if (secondLOD != firstLOD)
		{
			firstLOD.save("xray-lod-first.png");
			secondLOD.save("xray-lod-second.png");
			qDebug() << "LOD center:" << center(firstLOD) << center(secondLOD);
		}
		QCOMPARE(secondLOD, firstLOD);
		// A newly drawn opaque surface behind X-ray must pass the restored opaque
		// depth test (the final picking depth includes the foreground X-ray points).
		ccGLMatrix nearerOpaque;
		nearerOpaque.setTranslation(CCVector3(0, 0, 0.25f));
		opaque->setGLTransformation(nearerOpaque);
		opaque->setColor(ccColor::blue);
		const QImage refined = window.renderLODFrame(3);
		QCOMPARE(refined, window.renderLODFrame(0));
		QVERIFY(center(refined).blue() > 200);
		opaque->resetGLTransformation();
		opaque->setColor(ccColor::green);

		// Move opaque points in front, keeping the same screen footprint.
		ccGLMatrix transform;
		transform.setTranslation(CCVector3(0, 0, 1));
		opaque->setGLTransformation(transform);
		const QImage occluded = capture();
		QVERIFY(center(occluded).green() > 200);
		QCOMPARE(center(occluded).red(), 0);
		opaque->resetGLTransformation();

		// Meshes obey the same occlusion rule.
		opaque->setVisible(false);
		auto* vertices = new ccPointCloud();
		QVERIFY(vertices->reserve(4));
		vertices->addPoint(CCVector3(-1, -1, 0.5f));
		vertices->addPoint(CCVector3(1, -1, 0.5f));
		vertices->addPoint(CCVector3(1, 1, 0.5f));
		vertices->addPoint(CCVector3(-1, 1, 0.5f));
		vertices->setColor(ccColor::green);
		auto* mesh = new ccMesh(vertices);
		QVERIFY(mesh->reserve(2));
		mesh->addTriangle(0, 1, 2);
		mesh->addTriangle(0, 2, 3);
		mesh->addChild(vertices);
		vertices->setEnabled(false);
		mesh->showColors(true);
		mesh->setVisible(true);
		mesh->setDisplay(&window);
		scene.addChild(mesh);
		const QImage meshOccluded = capture();
		QCOMPARE(center(meshOccluded).red(), 0);
		QVERIFY(center(meshOccluded).green() > 200);
		mesh->setEnabled(false);
		opaque->setVisible(true);

		// Enabling scene lights cannot change X-ray colors.
		window.setSunLight(true);
		window.setCustomLight(true);
		QCOMPARE(capture(), front);
		window.setSunLight(false);
		window.setCustomLight(false);

		// Changing the cloud property must enlarge its point footprints without
		// fading them. Also check the inherited window size and explicit override.
		auto redCoverage = [](const QImage& image)
		{
			int count = 0;
			for (int y = 0; y < image.height(); ++y)
				for (int x = 0; x < image.width(); ++x)
					count += image.pixelColor(x, y).red() > 0 ? 1 : 0;
			return count;
		};
		auto peakRed = [](const QImage& image)
		{
			int peak = 0;
			for (int y = 0; y < image.height(); ++y)
				for (int x = 0; x < image.width(); ++x)
					peak = std::max(peak, image.pixelColor(x, y).red());
			return peak;
		};
		xray->setPointSize(1);
		const QImage smallPoints = capture();
		xray->setPointSize(7);
		QCOMPARE(capture(), front);
		QVERIFY(redCoverage(smallPoints) > 0);
		QVERIFY(redCoverage(front) > redCoverage(smallPoints) * 20);
		QCOMPARE(peakRed(front), peakRed(smallPoints));
		window.setPointSize(3);
		QCOMPARE(capture(), front); // explicit cloud size takes precedence
		xray->setPointSize(0);
		const QImage inheritedPoints = capture();
		QVERIFY(redCoverage(inheritedPoints) > redCoverage(smallPoints));
		QVERIFY(redCoverage(inheritedPoints) < redCoverage(front));
		QCOMPARE(peakRed(inheritedPoints), peakRed(smallPoints));
		window.setPointSize(1);
		xray->setPointSize(7);

		parameters.xRayExposure = 1;
		window.setDisplayParameters(parameters, true);
		const QImage dimmer = capture();
		QVERIFY(center(dimmer).red() < center(front).red());
		parameters.xRayExposure = 2;
		window.setDisplayParameters(parameters, true);

		// EDL remains active, suspends X-ray, and leaves the per-cloud preference intact.
		window.setGlFilter(new ccEDLFilter());
		QVERIFY(window.getGlFilter());
		const QImage edl = capture();
		QVERIFY(!edl.isNull());
		QVERIFY(!window.hasXRayResources());
		QVERIFY(xray->isXRayEnabled());
		xray->setXRayEnabled(false);
		QCOMPARE(capture(), edl);
		xray->setXRayEnabled(true);
		window.setGlFilter(nullptr);
		QCOMPARE(capture(), front);

		// Toggling off frees the extra buffers and restores the exact original image.
		xray->setXRayEnabled(false);
		QCOMPARE(capture(), baseline);
		QVERIFY(!window.hasXRayResources());

		// Reverse uses original hues with Multiply, including between clouds.
		parameters.xRayReverse = true;
		window.setDisplayParameters(parameters, true);
		QCOMPARE(capture(), baseline); // no X-ray cloud: no background override
		xray->setXRayEnabled(true);
		opaque->setVisible(false);
		const QImage reverseRed = capture();
		QVERIFY(window.overlayColor() == ccColor::black);
		const QColor redLayer = center(reverseRed);
		QCOMPARE(reverseRed.pixelColor(2, 2), QColor(Qt::white));
		QCOMPARE(redLayer.red(), 255);
		QVERIFY(redLayer.green() < 255 && redLayer.blue() < 255); // red, never cyan
		secondCloud->setEnabled(true);
		secondCloud->setColor(ccColor::Rgb(128, 64, 192));
		xray->setVisible(false);
		const QColor multiplySecond = center(capture());
		xray->setVisible(true);
		const QImage multiplied     = capture();
		const QColor multiplyResult = center(multiplied);
		for (int channel = 0; channel < 3; ++channel)
		{
			const int a      = channel == 0 ? redLayer.red() : channel == 1 ? redLayer.green()
			                                                                : redLayer.blue();
			const int b      = channel == 0 ? multiplySecond.red() : channel == 1 ? multiplySecond.green()
			                                                                      : multiplySecond.blue();
			const int actual = channel == 0 ? multiplyResult.red() : channel == 1 ? multiplyResult.green()
			                                                                      : multiplyResult.blue();
			QVERIFY(std::abs(actual - a * b / 255.0) <= 2.0);
		}
		scene.swapChildren(1, 2);
		QCOMPARE(capture(), multiplied);
		scene.swapChildren(1, 2);
		secondCloud->setColor(ccColor::white);
		QCOMPARE(capture(), reverseRed); // white is Multiply's neutral color
		secondCloud->setEnabled(false);
		const QImage reverseLOD = window.renderLODFrame(0);
		QCOMPARE(window.renderLODFrame(1), reverseLOD);
		parameters.xRayExposure = 3;
		window.setDisplayParameters(parameters, true);
		QVERIFY(center(capture()).green() < redLayer.green());
		parameters.xRayExposure          = 2;
		parameters.xRayReverseBackground = {ccColor::Rgb(240, 220, 200), ccColor::Rgb(100, 120, 140), true};
		window.setDisplayParameters(parameters, true);
		const QImage reverseGradientImage = capture();
		QVERIFY(reverseGradientImage.pixelColor(2, 2).red() > 230);
		QVERIFY(reverseGradientImage.pixelColor(2, reverseGradientImage.height() - 3).red() < 110);
		parameters.xRayReverseBackground.gradient = false;
		window.setDisplayParameters(parameters, true);
		const QImage reverseFlat = capture();
		QCOMPARE(reverseFlat.pixelColor(2, 2), QColor(240, 220, 200));
		QCOMPARE(reverseFlat.pixelColor(2, reverseFlat.height() - 3), QColor(240, 220, 200));
		parameters.xRayReverse    = false;
		parameters.xRayBackground = {ccColor::Rgb(10, 20, 30), ccColor::Rgb(50, 60, 70), true};
		window.setDisplayParameters(parameters, true);
		const QImage screenGradientImage = capture();
		QVERIFY(screenGradientImage.pixelColor(2, 2).red() < 15);
		QVERIFY(screenGradientImage.pixelColor(2, screenGradientImage.height() - 3).red() > 45);
		parameters.xRayReverse = true;
		window.setDisplayParameters(parameters, true);
		QCOMPARE(capture(), reverseFlat); // independent profiles survive mode switches
		opaque->setVisible(true);
		opaque->setGLTransformation(transform);
		QCOMPARE(center(capture()), QColor(Qt::green)); // opaque foreground still occludes
		opaque->resetGLTransformation();
		window.setGlFilter(new ccEDLFilter());
		const QImage reverseEDL = capture();
		xray->setXRayEnabled(false);
		QCOMPARE(capture(), reverseEDL);
		window.setGlFilter(nullptr);
		QCOMPARE(capture(), baseline);
		QVERIFY(window.overlayColor() == parameters.textDefaultCol);
		window.setSceneDB(nullptr);
	}
};

QTEST_MAIN(TestXRayDisplay)
#include "TestXRayDisplay.moc"
