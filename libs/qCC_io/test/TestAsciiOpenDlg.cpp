// qCC_io
#include <AsciiOpenDlg.h>

// Qt
#include <QComboBox>
#include <QList>
#include <QObject>
#include <QSpinBox>
#include <QTableWidget>
#include <QTest>
#include <QTextStream>
#include <QToolButton>

//! Returns the column types of the dialog's current open sequence
static QList<int> GetColumnTypes(const AsciiOpenDlg& dlg)
{
	QList<int> types;
	for (const AsciiOpenDlg::SequenceItem& item : dlg.getOpenSequence())
	{
		types.append(item.type);
	}
	return types;
}

// Changes the column types in the ASCII open dialog as a user would, refreshes
// the table, and checks which column types are kept
class TestAsciiOpenDlg : public QObject
{
	Q_OBJECT
  private slots:

	void keepColumnTypes_data()
	{
		QTest::addColumn<QString>("content");
		QTest::addColumn<int>("skipBefore");       // lines to skip before the user changes the columns
		QTest::addColumn<QList<int>>("userTypes"); // one value per column, -1 = not changed by the user
		QTest::addColumn<bool>("resetFirst");      // whether the 'reset' button is clicked after the user changes
		QTest::addColumn<int>("skipAfter");        // lines to skip after, or -1 to toggle the 'comma as decimal' option instead
		QTest::addColumn<QList<int>>("expectedTypes");

		const QString neh = "north,east,height\n10.5,20.5,1.5\n11.5,21.5,2.5\n12.5,22.5,3.5\n13.5,23.5,4.5\n";

		QTest::newRow("header appears") << neh << 0 << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, -1} << false << 1
		                                << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, ASCII_OPEN_DLG_Z};
		QTest::newRow("header names") << QString("z,y,x\n1,2,3\n4,5,6\n7,8,9\n") << 0 << QList<int>{-1, ASCII_OPEN_DLG_Scalar, -1} << false << 1
		                              << QList<int>{ASCII_OPEN_DLG_Z, ASCII_OPEN_DLG_Scalar, ASCII_OPEN_DLG_X};
		QTest::newRow("separator change") << QString("1 2 3\n4 5 6\n7 8 9\n") << 0 << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, -1} << false << -1
		                                  << QList<int>{ASCII_OPEN_DLG_X, ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_Z};
		QTest::newRow("skip lines change") << neh << 1 << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, -1} << false << 2
		                                   << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, ASCII_OPEN_DLG_Z};
		QTest::newRow("reset") << neh << 0 << QList<int>{ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_X, -1} << true << 1
		                       << QList<int>{ASCII_OPEN_DLG_X, ASCII_OPEN_DLG_Y, ASCII_OPEN_DLG_Z};
	}

	void keepColumnTypes()
	{
		QFETCH(QString, content);
		QFETCH(int, skipBefore);
		QFETCH(QList<int>, userTypes);
		QFETCH(bool, resetFirst);
		QFETCH(int, skipAfter);
		QFETCH(QList<int>, expectedTypes);

		QTextStream  stream(&content);
		AsciiOpenDlg dlg;
		dlg.setInput("test.csv", &stream); // returns false when no previous context can be applied: not an error

		QSpinBox* skipLinesSpinBox = dlg.findChild<QSpinBox*>("spinBoxSkipLines");
		QVERIFY(skipLinesSpinBox);
		skipLinesSpinBox->setValue(skipBefore);

		QTableWidget* table = dlg.findChild<QTableWidget*>("tableWidget");
		QVERIFY(table);
		QCOMPARE(table->columnCount(), userTypes.size());
		for (int i = 0; i < userTypes.size(); ++i)
		{
			if (userTypes[i] >= 0)
			{
				QComboBox* combo = qobject_cast<QComboBox*>(table->cellWidget(0, i));
				QVERIFY(combo);
				combo->setCurrentIndex(userTypes[i]); // same signal as a user choice
			}
		}

		if (resetFirst)
		{
			QToolButton* resetButton = dlg.findChild<QToolButton*>("resetColumnsToolButton");
			QVERIFY(resetButton);
			resetButton->click();
		}

		if (skipAfter >= 0)
		{
			skipLinesSpinBox->setValue(skipAfter);
		}
		else
		{
			dlg.commaDecimalCheckBoxToggled(false);
		}

		QCOMPARE(GetColumnTypes(dlg), expectedTypes);
	}
};

QTEST_MAIN(TestAsciiOpenDlg)
#include "TestAsciiOpenDlg.moc"
