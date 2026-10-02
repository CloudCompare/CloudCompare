#pragma once

// Qt
#include <QObject>
#include <QtTest/QtTest>

// Saves the depth maps of several sensors with DepthMapFileFilter
class TestDepthMapFilter : public QObject
{
	Q_OBJECT
  private slots:
	void initTestCase();
	void multipleSensorsAreSavedNextToTheRequestedFile();
};
