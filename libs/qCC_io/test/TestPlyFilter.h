#ifndef CC_TEST_PLYFILTER_HEADER
#define CC_TEST_PLYFILTER_HEADER

#include <QObject>
#include <QtTest/QtTest>

class TestPlyFilter : public QObject
{
	Q_OBJECT
  private slots:
	void testOutOfRangeTextureIndexIsSanitized() const;
};

#endif // CC_TEST_PLYFILTER_HEADER
