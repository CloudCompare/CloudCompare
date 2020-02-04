//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScriptingPlugin                #
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
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################


#include <QtWidgets>
#include <QPlainTextEdit>


#include "codeeditor.h"
#include "chaiScriptSyntaxHighlighter.h"

CodeEditor::CodeEditor(QWidget* parent) : QPlainTextEdit(parent)
{
	lineNumberArea = new LineNumberArea(this);

	connect(this, &QPlainTextEdit::blockCountChanged, this, &CodeEditor::updateLineNumberAreaWidth);
	connect(this, &QPlainTextEdit::updateRequest, this, &CodeEditor::updateLineNumberArea);
	connect(this, &QPlainTextEdit::cursorPositionChanged, this, &CodeEditor::startAllHighlighting);

	updateLineNumberAreaWidth(0);
	highlightCurrentLine();
	setAttribute(Qt::WA_DeleteOnClose);
	isUntitled = true;
	setupEditor();
	installEventFilter(this);
}

CodeEditor::~CodeEditor()
{

}

bool CodeEditor::eventFilter(QObject* target, QEvent* event)
{

	if (target == this && event->type() == QEvent::Wheel)
	{
		QWheelEvent* wheel = static_cast<QWheelEvent*>(event);
		if (wheel->modifiers() == Qt::ControlModifier)
		{
			if (wheel->delta() > 0)
			{
				zoomIn(1);
			}
			else
			{
				zoomOut(1);
			}
			return true;
		}
	}
	return QPlainTextEdit::eventFilter(target, event);
}

void CodeEditor::setupEditor()
{
	QFont font;
	font.setFamily("Courier");
	font.setFixedPitch(true);
	font.setPointSize(20);

	setFont(font);
	/*QPalette p = palette();
	p.setColor(QPalette::Base, Qt::black);
	p.setColor(QPalette::Text, Qt::white);
	setPalette(p);*/
	setLineWrapMode(QPlainTextEdit::NoWrap);
	highlighter = new ChaiScriptSyntaxHighlighter(document());

}


void CodeEditor::startAllHighlighting()
{
	QList<QTextEdit::ExtraSelection> selections;
	setExtraSelections(selections);
	highlightCurrentLine();
	matchPairedChars();
}


int CodeEditor::lineNumberAreaWidth()
{
	int digits = 1;
	int max = qMax(1, blockCount());
	while (max >= 10) {
		max /= 10;
		++digits;
	}

	int space = 3 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;

	return space;
}



void CodeEditor::updateLineNumberAreaWidth(int /* newBlockCount */)
{
	setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}



void CodeEditor::updateLineNumberArea(const QRect& rect, int dy)
{
	if (dy)
		lineNumberArea->scroll(0, dy);
	else
		lineNumberArea->update(0, rect.y(), lineNumberArea->width(), rect.height());

	if (rect.contains(viewport()->rect()))
		updateLineNumberAreaWidth(0);
}



void CodeEditor::resizeEvent(QResizeEvent* e)
{
	QPlainTextEdit::resizeEvent(e);

	QRect cr = contentsRect();
	lineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}



void CodeEditor::highlightCurrentLine()
{
	QList<QTextEdit::ExtraSelection> selections = extraSelections();

	if (!isReadOnly()) {
		QTextEdit::ExtraSelection selection;

		QColor lineColor = QColor(Qt::yellow).lighter(160);

		selection.format.setBackground(lineColor);
		selection.format.setProperty(QTextFormat::FullWidthSelection, true);
		selection.cursor = textCursor();
		selection.cursor.clearSelection();
		selections.append(selection);
	}

	setExtraSelections(selections);
}



void CodeEditor::lineNumberAreaPaintEvent(QPaintEvent* event)
{
	QPainter painter(lineNumberArea);
	painter.fillRect(event->rect(), Qt::lightGray);


	QTextBlock block = firstVisibleBlock();
	int blockNumber = block.blockNumber();
	int top = (int)blockBoundingGeometry(block).translated(contentOffset()).top();
	int bottom = top + (int)blockBoundingRect(block).height();

	while (block.isValid() && top <= event->rect().bottom()) {
		if (block.isVisible() && bottom >= event->rect().top()) {
			QString number = QString::number(blockNumber + 1);
			painter.setPen(Qt::black);
			painter.drawText(0, top, lineNumberArea->width(), fontMetrics().height(),
				Qt::AlignRight, number);
		}

		block = block.next();
		top = bottom;
		bottom = top + (int)blockBoundingRect(block).height();
		++blockNumber;
	}
}

void CodeEditor::newFile()
{
	static int sequenceNumber = 1;

	isUntitled = true;
	curFile = tr("Script%1.chai").arg(sequenceNumber++);
	setWindowTitle(curFile + "[*]");

	connect(document(), &QTextDocument::contentsChanged,
		this, &CodeEditor::documentWasModified);
}

bool CodeEditor::loadFile(const QString& fileName)
{
	QFile file(fileName);
	if (!file.open(QFile::ReadOnly | QFile::Text)) {
		QMessageBox::warning(this, tr("MDI"),
			tr("Cannot read file %1:\n%2.")
			.arg(fileName)
			.arg(file.errorString()));
		return false;
	}

	QTextStream in(&file);
	QApplication::setOverrideCursor(Qt::WaitCursor);
	setPlainText(in.readAll());
	QApplication::restoreOverrideCursor();

	setCurrentFile(fileName);

	connect(document(), &QTextDocument::contentsChanged,
		this, &CodeEditor::documentWasModified);

	return true;
}

bool CodeEditor::save()
{
	if (isUntitled) {
		return saveAs();
	}
	else {
		return saveFile(curFile);
	}
}

bool CodeEditor::saveAs()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save As"),
		curFile);
	if (fileName.isEmpty())
		return false;

	return saveFile(fileName);
}

bool CodeEditor::saveFile(const QString& fileName)
{
	QFile file(fileName);
	if (!file.open(QFile::WriteOnly | QFile::Text)) {
		QMessageBox::warning(this, tr("MDI"),
			tr("Cannot write file %1:\n%2.")
			.arg(QDir::toNativeSeparators(fileName), file.errorString()));
		return false;
	}

	QTextStream out(&file);
	QApplication::setOverrideCursor(Qt::WaitCursor);
	out << toPlainText();
	QApplication::restoreOverrideCursor();

	setCurrentFile(fileName);
	return true;
}


QString CodeEditor::userFriendlyCurrentFile()
{
	return strippedName(curFile);
}

void CodeEditor::closeEvent(QCloseEvent* event)
{
	if (maybeSave()) {
		event->accept();
	}
	else {
		event->ignore();
	}
}

void CodeEditor::documentWasModified()
{
	setWindowModified(document()->isModified());

}

bool CodeEditor::maybeSave()
{
	if (!document()->isModified())
		return true;
	const QMessageBox::StandardButton ret
		= QMessageBox::warning(this, tr("MDI"),
			tr("'%1' has been modified.\n"
				"Do you want to save your changes?")
			.arg(userFriendlyCurrentFile()),
			QMessageBox::Save | QMessageBox::Discard
			| QMessageBox::Cancel);
	switch (ret) {
	case QMessageBox::Save:
		return save();
	case QMessageBox::Cancel:
		return false;
	default:
		break;
	}
	return true;
}

void CodeEditor::setCurrentFile(const QString& fileName)
{
	curFile = QFileInfo(fileName).canonicalFilePath();
	isUntitled = false;
	document()->setModified(false);
	setWindowModified(false);
	setWindowTitle(userFriendlyCurrentFile() + "[*]");
}

QString CodeEditor::strippedName(const QString& fullFileName)
{
	return QFileInfo(fullFileName).fileName();
}


void CodeEditor::matchPairedChars()
{
	if (!matchPairedChars('(', ')'))
	{
		if (!matchPairedChars('[', ']'))
		{
			if (!matchPairedChars('{', '}'))
			{
			}
		}
	}
}


bool CodeEditor::matchPairedChars(char lhs, char rhs)
{
	bool match = false;


	TextBlockData* data = static_cast<TextBlockData*>(textCursor().block().userData());

	if (data) {
		QVector<ParenthesisInfo*> infos = data->parentheses();

		int pos = textCursor().block().position();
		for (int i = 0; i < infos.size(); ++i) {
			ParenthesisInfo* info = infos.at(i);

			int curPos = textCursor().position() - textCursor().block().position();
			if (info->position == curPos - 1 && info->character == lhs) {
				if (matchLeftPairedChars(textCursor().block(), i + 1, 0, lhs, rhs))
				{
					createPairedCharsSelection(pos + info->position);
					match = true;
				}
			}
			else if (info->position == curPos && info->character == rhs) {
				if (matchRightPairedChars(textCursor().block(), i - 1, 0, lhs, rhs))
				{
					createPairedCharsSelection(pos + info->position);
					match = true;
				}
			}
		}
	}
	return match;
}

bool CodeEditor::matchLeftPairedChars(QTextBlock currentBlock, int i, int numLeftPairedChars, char lhs, char rhs)
{
	TextBlockData* data = static_cast<TextBlockData*>(currentBlock.userData());
	QVector<ParenthesisInfo*> infos = data->parentheses();

	int docPos = currentBlock.position();
	for (; i < infos.size(); ++i) {
		ParenthesisInfo* info = infos.at(i);

		if (info->character == lhs) {
			++numLeftPairedChars;
			continue;
		}

		if (info->character == rhs && numLeftPairedChars == 0) {
			createPairedCharsSelection(docPos + info->position);
			return true;
		}
		else if (info->character == rhs)
		{
			--numLeftPairedChars;
		}
	}

	currentBlock = currentBlock.next();
	if (currentBlock.isValid())
		return matchLeftPairedChars(currentBlock, 0, numLeftPairedChars, lhs, rhs);

	return false;
}

bool CodeEditor::matchRightPairedChars(QTextBlock currentBlock, int i, int numRightPairedChars, char lhs, char rhs)
{
	TextBlockData* data = static_cast<TextBlockData*>(currentBlock.userData());
	QVector<ParenthesisInfo*> parentheses = data->parentheses();

	int docPos = currentBlock.position();
	for (; i > -1 && parentheses.size() > 0; --i) {
		ParenthesisInfo* info = parentheses.at(i);
		if (info->character == rhs) {
			++numRightPairedChars;
			continue;
		}
		if (info->character == lhs && numRightPairedChars == 0) {
			createPairedCharsSelection(docPos + info->position);
			return true;
		}
		else if (info->character == lhs)
		{
			--numRightPairedChars;
		}
	}

	currentBlock = currentBlock.previous();
	if (currentBlock.isValid())
		return matchRightPairedChars(currentBlock, 0, numRightPairedChars, lhs, rhs);

	return false;
}

void CodeEditor::createPairedCharsSelection(int pos)
{
	QList<QTextEdit::ExtraSelection> selections = extraSelections();

	QTextEdit::ExtraSelection selection;
	QTextCharFormat format = selection.format;
	format.setBackground(Qt::green);
	selection.format = format;

	QTextCursor cursor = textCursor();
	cursor.setPosition(pos);
	cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor);
	selection.cursor = cursor;

	selections.append(selection);

	setExtraSelections(selections);
}


void CodeEditor::comment()
{
	int lineCount = getSelectedLineCount();
	QTextCursor cursor = textCursor();

	for (int i = 0; i < lineCount; i++)
	{
		cursor.movePosition(QTextCursor::MoveOperation::StartOfLine);
		cursor.insertText("//");
		cursor.movePosition(QTextCursor::MoveOperation::Up);
	}
}

void CodeEditor::uncomment()
{
	int lineCount = getSelectedLineCount();
	QTextCursor cursor = textCursor();

	for (int i = 0; i < lineCount; i++)
	{
		cursor.movePosition(QTextCursor::MoveOperation::StartOfLine);
		QString line = cursor.block().text();
		if (line.startsWith("//"))
		{
			cursor.deleteChar();
			cursor.deleteChar();
		}
		cursor.movePosition(QTextCursor::MoveOperation::Up);
	}
}

void CodeEditor::indentMore()
{
	int lineCount = getSelectedLineCount();
	QTextCursor cursor = textCursor();

	for (int i = 0; i < lineCount; i++)
	{
		cursor.movePosition(QTextCursor::MoveOperation::StartOfLine);
		cursor.insertText("\t");
		cursor.movePosition(QTextCursor::MoveOperation::Up);
	}

}

void CodeEditor::indentLess()
{
	int lineCount = getSelectedLineCount();
	QTextCursor cursor = textCursor();

	for (int i = 0; i < lineCount; i++)
	{
		cursor.movePosition(QTextCursor::MoveOperation::StartOfLine);
		QString line = cursor.block().text();
		if (line.startsWith("\t"))
		{
			cursor.deleteChar();
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				if (line.startsWith(" "))
				{
					cursor.deleteChar();
				}
				else
				{
					break;
				}
				line = cursor.block().text();
			}
		}
		cursor.movePosition(QTextCursor::MoveOperation::Up);
	}
}



int CodeEditor::getSelectedLineCount()
{
	QTextCursor cursor = textCursor();
	if (cursor.hasSelection())
	{
		cursor.setPosition(cursor.selectionStart());
		int temp = cursor.blockNumber();
		cursor = textCursor();
		cursor.setPosition(cursor.selectionEnd());
		int diff = cursor.blockNumber() - temp;
		return diff + 1;
	}
	else 
	{
		return 1;
	}
}
