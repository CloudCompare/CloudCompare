// qCC_io
#include <AsciiOpenDlg.h>

// Qt
#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QList>
#include <QMessageBox>
#include <QObject>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTest>
#include <QTextStream>
#include <QTimer>
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

//! Returns the content of an ASCII file with the given number of columns (3 lines of numbers)
static QString CreateWideContent(int columnCount)
{
	QString content;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < columnCount; ++j)
		{
			content += (j == 0 ? "" : " ") + QString::number(1000.0 * (i + 1) + j + 0.5);
		}
		content += '\n';
	}
	return content;
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

	// Opens a file with many columns: the table has every column
	void wideColumns_data()
	{
		QTest::addColumn<int>("columnCount");

		QTest::newRow("512 columns") << 512;
		QTest::newRow("600 columns") << 600;
	}

	void wideColumns()
	{
		QFETCH(int, columnCount);

		QString      content = CreateWideContent(columnCount);
		QTextStream  stream(&content);
		AsciiOpenDlg dlg;
		dlg.setInput("test.txt", &stream);

		QTableWidget* table = dlg.findChild<QTableWidget*>("tableWidget");
		QVERIFY(table);
		QCOMPARE(table->columnCount(), columnCount);
		QCOMPARE(dlg.getColumnsCount(), static_cast<unsigned>(columnCount));

		QLabel* truncatedLabel = dlg.findChild<QLabel*>("columnsTruncatedLabel");
		QVERIFY(truncatedLabel);
		QVERIFY(truncatedLabel->isHidden()); // the dialog itself is not shown, so isVisible() would always be false
	}

	// Opens a file with many columns, with the confirmation enabled (as the GUI does): the user is asked once
	// before more than 512 columns are created, and 'No' keeps 512 columns
	void confirmWideFile_data()
	{
		QTest::addColumn<int>("columnCount");
		QTest::addColumn<bool>("confirm");
		QTest::addColumn<int>("answer"); // QMessageBox::Yes or QMessageBox::No
		QTest::addColumn<int>("expectedQuestions");
		QTest::addColumn<int>("expectedColumns");

		QTest::newRow("No") << 600 << true << int(QMessageBox::No) << 1 << 512;
		QTest::newRow("Yes") << 600 << true << int(QMessageBox::Yes) << 1 << 600;
		QTest::newRow("512 columns") << 512 << true << int(QMessageBox::Yes) << 0 << 512;
		QTest::newRow("513 columns") << 513 << true << int(QMessageBox::Yes) << 1 << 513;
		QTest::newRow("no confirmation") << 600 << false << int(QMessageBox::No) << 0 << 600;
	}

	void confirmWideFile()
	{
		QFETCH(int, columnCount);
		QFETCH(bool, confirm);
		QFETCH(int, answer);
		QFETCH(int, expectedQuestions);
		QFETCH(int, expectedColumns);

		// answers the question box (it is modal, so the timer runs in its event loop)
		int    questionCount = 0;
		QTimer answerTimer;
		answerTimer.setInterval(10);
		QObject::connect(&answerTimer,
		                 &QTimer::timeout,
		                 [&]()
		                 {
			                 for (QWidget* widget : QApplication::topLevelWidgets())
			                 {
				                 QMessageBox* box = qobject_cast<QMessageBox*>(widget);
				                 if (box && box->isVisible())
				                 {
					                 ++questionCount;
					                 box->button(static_cast<QMessageBox::StandardButton>(answer))->click();
				                 }
			                 }
		                 });
		answerTimer.start();

		QString      content = CreateWideContent(columnCount);
		QTextStream  stream(&content);
		AsciiOpenDlg dlg;
		dlg.setInput("test.txt", &stream, confirm);

		QTableWidget* table = dlg.findChild<QTableWidget*>("tableWidget");
		QVERIFY(table);
		QCOMPARE(table->columnCount(), expectedColumns);

		QLabel* truncatedLabel = dlg.findChild<QLabel*>("columnsTruncatedLabel");
		QVERIFY(truncatedLabel);
		QCOMPARE(truncatedLabel->isHidden(), expectedColumns == columnCount);

		// refreshing the table does not ask again
		dlg.commaDecimalCheckBoxToggled(false);
		QCOMPARE(table->columnCount(), expectedColumns);
		QCOMPARE(questionCount, expectedQuestions);
	}
};

QTEST_MAIN(TestAsciiOpenDlg)
#include "TestAsciiOpenDlg.moc"
