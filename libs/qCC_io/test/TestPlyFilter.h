#pragma once

#include <QObject>
#include <QtTest/QtTest>

class TestPlyFilter : public QObject
{
	Q_OBJECT
  private slots:
	void testOutOfRangeTextureIndexIsSanitized() const;
};
