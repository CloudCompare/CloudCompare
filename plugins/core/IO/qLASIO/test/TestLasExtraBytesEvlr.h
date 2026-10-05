#pragma once

#include <QObject>
#include <QtTest/QtTest>

// Reads the "Extra Bytes" descriptor from a VLR, from an EVLR, or from both (VLR first)
class TestLasExtraBytesEvlr : public QObject
{
	Q_OBJECT
  private slots:
	void initTestCase();
	void evlrOnly();
	void vlrOnly();
	void bothPresent();
	void truncatedEvlr();
	void fullLengthName();
	void writeEvlr_data();
	void writeEvlr();
};
