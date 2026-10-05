#pragma once

// Qt
#include <QObject>
#include <QtTest/QtTest>

// Saves a small generated cloud (or mesh) with each filter, loads it back
// and checks that the data survived (not the file layout)
class TestFileRoundTrip : public QObject
{
	Q_OBJECT
  private slots:
	void initTestCase();
	void roundTrip_data();
	void roundTrip();
	void meshRoundTrip_data();
	void meshRoundTrip();
};
