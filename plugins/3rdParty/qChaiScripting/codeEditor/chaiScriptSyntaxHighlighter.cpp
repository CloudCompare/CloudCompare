/*

This is a C++ port of the following PyQt example
http://diotavelli.net/PyQtWiki/Python%20syntax%20highlighting
C++ port by Frankie Simon (www.kickdrive.de, www.fuh-edv.de)

The following free software license applies for this file ("X11 license"):

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.

ChaiScriptSyntaxHighlighter.cpp

   Created on: 21.10.2016
	   Author: klemens.morgenstern
 */

#include "chaiScriptSyntaxHighlighter.h"


TextBlockData::TextBlockData()
{
	// Nothing to do
}

QVector<ParenthesisInfo*> TextBlockData::parentheses()
{
	return m_parentheses;
}


void TextBlockData::insert(ParenthesisInfo* info)
{
	int i = 0;
	while (i < m_parentheses.size() &&
		info->position > m_parentheses.at(i)->position)
		++i;

	m_parentheses.insert(i, info);
}



ChaiScriptSyntaxHighlighter::ChaiScriptSyntaxHighlighter(QTextDocument* parent)
	: QSyntaxHighlighter(parent)
{
	keywords = QStringList() << "attr" << "auto" << "break" <<
		"else" << "for" << "fun" << "if" << "try" << "catch" << "while" << "var"
		<< "true" << "false" << "def" << "this" << "return"
		;

	operators = QStringList() << "=" <<
		// Comparison
		"==" << "!=" << "<" << "<=" << ">" << ">=" <<
		// Arithmetic
		"\\+" << "-" << "\\*" << "/" << "//" << "%" << "\\*\\*" <<
		// In-place
		"\\+=" << "-=" << "\\*=" << "/=" << "%=" <<
		// Bitwise
		"\\^" << "\\|" << "&" << "~" << ">>" << "<<";

	braces = QStringList() << "\\{" << "\\}";  
	brackets = QStringList() << "\\[" << "\\]";
	parenthesis = QStringList() << "\\(" << "\\)";
	
	basicStyles.insert("keyword", getTextCharFormat(QColor("blue")));
	basicStyles.insert("operator", getTextCharFormat(QColor("red")));
	basicStyles.insert("brace", getTextCharFormat(QColor("darkGray")));
	basicStyles.insert("bracket", getTextCharFormat(QColor(104,151,187)));
	basicStyles.insert("parenthesis", getTextCharFormat(QColor(204, 120, 50)));
	basicStyles.insert("defclass", getTextCharFormat(QColor("black"), "bold"));
	basicStyles.insert("string", getTextCharFormat(QColor("magenta")));
	basicStyles.insert("comment", getTextCharFormat(QColor("darkGreen"), "italic"));
	basicStyles.insert("numbers", getTextCharFormat(QColor("brown")));

	initializeRules();
}

void ChaiScriptSyntaxHighlighter::initializeRules()
{
	for (auto& currKeyword : keywords)
	{
		rules.append(ChaiScriptRule(QString("\\b%1\\b").arg(currKeyword), 0, basicStyles.value("keyword")));
	}
	for (auto& currOperator : operators)
	{
		rules.append(ChaiScriptRule(QString("%1").arg(currOperator), 0, basicStyles.value("operator")));
	}
	for (auto& currBrace : braces)
	{
		rules.append(ChaiScriptRule(QString("%1").arg(currBrace), 0, basicStyles.value("brace")));
	}
	for (auto& currBracket : brackets)
	{
		rules.append(ChaiScriptRule(QString("%1").arg(currBracket), 0, basicStyles.value("bracket")));
	}
	for (auto& currParenthesis : parenthesis)
	{
		rules.append(ChaiScriptRule(QString("%1").arg(currParenthesis), 0, basicStyles.value("parenthesis")));
	}

	// Double-quoted string, possibly containing escape sequences
	// FF: originally in python : r'"[^"\\]*(\\.[^"\\]*)*"'
	rules.append(ChaiScriptRule("\"[^\"\\\\]*(\\\\.[^\"\\\\]*)*\"", 0, basicStyles.value("string")));
	// Single-quoted string, possibly containing escape sequences
	// FF: originally in python : r"'[^'\\]*(\\.[^'\\]*)*'"
	rules.append(ChaiScriptRule("'[^'\\\\]*(\\\\.[^'\\\\]*)*'", 0, basicStyles.value("string")));

	// 'def' followed by an identifier
	// FF: originally: r'\bdef\b\s*(\w+)'
	rules.append(ChaiScriptRule("\\bdef\\b\\s*(\\w+)", 1, basicStyles.value("defclass")));
	//  'class' followed by an identifier
	// FF: originally: r'\bclass\b\s*(\w+)'
	rules.append(ChaiScriptRule("\\bclass\\b\\s*(\\w+)", 1, basicStyles.value("defclass")));

	// From '#' until a newline
	// FF: originally: r'#[^\\n]*'
	rules.append(ChaiScriptRule("//[^\\n]*", 0, basicStyles.value("comment")));

	// Numeric literals
	rules.append(ChaiScriptRule("\\b[+-]?[0-9]+[lL]?\\b", 0, basicStyles.value("numbers"))); // r'\b[+-]?[0-9]+[lL]?\b'
	rules.append(ChaiScriptRule("\\b[+-]?0[xX][0-9A-Fa-f]+[lL]?\\b", 0, basicStyles.value("numbers"))); // r'\b[+-]?0[xX][0-9A-Fa-f]+[lL]?\b'
	rules.append(ChaiScriptRule("\\b[+-]?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\\b", 0, basicStyles.value("numbers"))); // r'\b[+-]?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?\b'
}

void ChaiScriptSyntaxHighlighter::markSearchedForChar(const QString& text, TextBlockData* data, char srchChar)
{
	int charPos = text.indexOf(srchChar);
	while (charPos != -1) {
		ParenthesisInfo* info = new ParenthesisInfo;
		info->character = srchChar;
		info->position = charPos;

		data->insert(info);
		charPos = text.indexOf(srchChar, charPos + 1);
	}
}

void ChaiScriptSyntaxHighlighter::highlightBlock(const QString& text)
{
	TextBlockData* data = new TextBlockData;
	markSearchedForChar(text, data, '(');
	markSearchedForChar(text, data, ')');
	markSearchedForChar(text, data, '{');
	markSearchedForChar(text, data, '}');
	markSearchedForChar(text, data, '[');
	markSearchedForChar(text, data, ']');
	

	for (ChaiScriptRule& currRule : rules)
	{
		int idx = currRule.pattern.indexIn(text, 0);
		while (idx >= 0)
		{
			// Get index of Nth match
			idx = currRule.pattern.pos(currRule.nth);
			int length = currRule.pattern.cap(currRule.nth).length();
			setFormat(idx, length, currRule.format);
			idx = currRule.pattern.indexIn(text, idx + length);
		}
	}
	setCurrentBlockUserData(data);
	setCurrentBlockState(0);
}

bool ChaiScriptSyntaxHighlighter::matchMultiline(const QString& text, const QRegExp& delimiter, const int inState, const QTextCharFormat& style)
{
	int start = -1;
	int add = -1;
	int end = -1;
	int length = 0;

	// If inside triple-single quotes, start at 0
	if (previousBlockState() == inState)
	{
		start = 0;
		add = 0;
	}
	// Otherwise, look for the delimiter on this line
	else {
		start = delimiter.indexIn(text);
		// Move past this match
		add = delimiter.matchedLength();
	}

	// As long as there's a delimiter match on this line...
	while (start >= 0)
	{
		// Look for the ending delimiter
		end = delimiter.indexIn(text, start + add);
		// Ending delimiter on this line?
		if (end >= add) {
			length = end - start + add + delimiter.matchedLength();
			setCurrentBlockState(0);
		}
		// No; multi-line string
		else {
			setCurrentBlockState(inState);
			length = text.length() - start + add;
		}
		// Apply formatting and look for next
		setFormat(start, length, style);
		start = delimiter.indexIn(text, start + length);
	}
	// Return True if still inside a multi-line string, False otherwise
	if (currentBlockState() == inState)
		return true;
	else
		return false;
}

const QTextCharFormat ChaiScriptSyntaxHighlighter::getTextCharFormat(const QString& colorName, const QString& style)
{
	QTextCharFormat charFormat;
	QColor color(colorName);
	charFormat.setForeground(color);
	if (style.contains("bold", Qt::CaseInsensitive))
		charFormat.setFontWeight(QFont::Bold);
	if (style.contains("italic", Qt::CaseInsensitive))
		charFormat.setFontItalic(true);
	return charFormat;
}

const QTextCharFormat ChaiScriptSyntaxHighlighter::getTextCharFormat(QColor color, const QString& style)
{
	QTextCharFormat charFormat;
	charFormat.setForeground(color);
	if (style.contains("bold", Qt::CaseInsensitive))
		charFormat.setFontWeight(QFont::Bold);
	if (style.contains("italic", Qt::CaseInsensitive))
		charFormat.setFontItalic(true);
	return charFormat;
}

