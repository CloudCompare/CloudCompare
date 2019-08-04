#include "Bitmap.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "Grid.h"
#include <algorithm>
#include <GfxTL/VectorXD.h>
#include <MiscLib/Performance.h>
using namespace MiscLib;
extern MiscLib::performance_t totalTime_components;

void DilateSquare(const MiscLib::Vector< char > &bitmap,
	size_t uextent, size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *dilated)
{
	// first pixel is special
	(*dilated)[0] = bitmap[0] || bitmap[1] ||
		bitmap[uextent] || bitmap[uextent + 1];
	if(vwrap)
		(*dilated)[0] = (*dilated)[0] ||
			bitmap[(vextent - 1) * uextent] ||
			bitmap[(vextent - 1) * uextent + 1];
	if(uwrap)
		(*dilated)[0] = (*dilated)[0] || bitmap[uextent - 1]
			|| bitmap[2 * uextent - 1];
	if(vwrap && uwrap)
		(*dilated)[0] = (*dilated)[0] ||
			bitmap[(vextent - 1) * uextent + uextent - 1];
	// first row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[i] = bitmap[i - 1] || bitmap[i] || bitmap[i + 1] ||
				bitmap[uextent + i - 1] || bitmap[uextent + i] ||
				bitmap[uextent + i + 1];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[i] = bitmap[i - 1] || bitmap[i] || bitmap[i + 1] ||
				bitmap[uextent + i - 1] || bitmap[uextent + i] ||
				bitmap[uextent + i + 1] ||
				bitmap[(vextent - 1) * uextent + i - 1] ||
				bitmap[(vextent - 1) * uextent + i] ||
				bitmap[(vextent - 1) * uextent + i + 1];
		}
	}
	// last pixel of first row is special
	(*dilated)[uextent - 1] = bitmap[uextent - 1] || bitmap[uextent - 2] ||
		bitmap[2 * uextent - 1] || bitmap[2 * uextent - 2];
	if(vwrap)
		(*dilated)[uextent - 1] = (*dilated)[uextent - 1] ||
			bitmap[vextent * uextent - 1] ||
			bitmap[vextent * uextent - 2];
	if(uwrap)
		(*dilated)[uextent - 1] = (*dilated)[uextent - 1] ||
			bitmap[uextent] || bitmap[0];
	if(uwrap && vwrap)
		(*dilated)[uextent - 1] = (*dilated)[uextent - 1] ||
			bitmap[(vextent - 1) * uextent];
	size_t row = 0, prevRow, nextRow = uextent;
	for(size_t j = 1; j < vextent - 1; ++j)
	{
		prevRow = row;
		row = nextRow;
		nextRow = row + uextent;
		// first pixel in row is special
		(*dilated)[row] = bitmap[prevRow] || bitmap[prevRow + 1] ||
			bitmap[row] || bitmap[row + 1] || bitmap[nextRow] ||
			bitmap[nextRow + 1];
		if(uwrap)
			(*dilated)[row] = (*dilated)[row] || bitmap[row - 1] ||
				bitmap[nextRow - 1] || bitmap[nextRow + uextent - 1];
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[row + i] = bitmap[prevRow + i - 1] ||
				bitmap[prevRow + i ] || bitmap[prevRow + i + 1] ||
				bitmap[row + i - 1] || bitmap[row + i] ||
				bitmap[row + i + 1] || bitmap[nextRow + i - 1] ||
				bitmap[nextRow + i] || bitmap[nextRow + i + 1];
		}
		// last pixel in row is special
		(*dilated)[row + uextent - 1] = bitmap[prevRow + uextent - 2] ||
			bitmap[prevRow + uextent - 1] ||
			bitmap[row + uextent - 2] || bitmap[row + uextent - 1] ||
			bitmap[nextRow + uextent - 2] || bitmap[nextRow + uextent - 1];
		if(uwrap)
			(*dilated)[row + uextent - 1] = (*dilated)[row + uextent - 1] ||
				bitmap[prevRow] || bitmap[row] || bitmap[nextRow];
	}
	// first pixel of last row is special
	(*dilated)[(vextent - 1) * uextent] = bitmap[(vextent - 1) * uextent] ||
		bitmap[(vextent - 1) * uextent + 1] ||
		bitmap[(vextent - 2) * uextent] || bitmap[(vextent - 2) * uextent + 1];
	if(vwrap)
		(*dilated)[(vextent - 1) * uextent] =
			(*dilated)[(vextent - 1) * uextent] || bitmap[0] || bitmap[1];
	if(uwrap)
		(*dilated)[(vextent - 1) * uextent] =
			(*dilated)[(vextent - 1) * uextent] ||
			bitmap[vextent * uextent - 1] ||
			bitmap[(vextent - 1) * uextent - 1];
	if(vwrap && uwrap)
		(*dilated)[(vextent - 1) * uextent] =
			(*dilated)[(vextent - 1) * uextent] || bitmap[uextent - 1];
	// last row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] ||
				bitmap[(vextent - 1) * uextent + i - 1] ||
				bitmap[(vextent - 1) * uextent + i + 1] ||
				bitmap[(vextent - 2) * uextent + i] ||
				bitmap[(vextent - 2) * uextent + i - 1] ||
				bitmap[(vextent - 2) * uextent + i + 1];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] ||
				bitmap[(vextent - 1) * uextent + i - 1] ||
				bitmap[(vextent - 1) * uextent + i + 1] ||
				bitmap[(vextent - 2) * uextent + i] ||
				bitmap[(vextent - 2) * uextent + i - 1] ||
				bitmap[(vextent - 2) * uextent + i + 1] ||
				bitmap[i - 1] || bitmap[i] || bitmap[i + 1];
		}
	}
	// last pixel
	(*dilated)[bitmap.size() - 1] = bitmap[bitmap.size() - 1] ||
		bitmap[bitmap.size() - 2] || bitmap[bitmap.size() - uextent - 1] ||
		bitmap[bitmap.size() - uextent - 2];
	if(vwrap)
		(*dilated)[bitmap.size() - 1] = (*dilated)[bitmap.size() - 1] ||
			bitmap[uextent - 1] || bitmap[uextent - 2];
	if(uwrap)
		(*dilated)[bitmap.size() - 1] = (*dilated)[bitmap.size() - 1] ||
			bitmap[bitmap.size() - uextent] ||
			bitmap[bitmap.size() - 2 * uextent];
	if(uwrap && vwrap)
		(*dilated)[bitmap.size() - 1] = (*dilated)[bitmap.size() - 1] ||
			bitmap[0];
}

void DilateCross(const MiscLib::Vector< char > &bitmap,
	size_t uextent, size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *dilated)
{
	// first pixel is special
	(*dilated)[0] = bitmap[0] || bitmap[1] ||
		bitmap[uextent];
	if(vwrap)
		(*dilated)[0] = (*dilated)[0] ||
			bitmap[(vextent - 1) * uextent];
	if(uwrap)
		(*dilated)[0] = (*dilated)[0] || bitmap[uextent - 1];
	// first row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[i] = bitmap[i - 1] || bitmap[i] || bitmap[i + 1] ||
				bitmap[uextent + i];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[i] = bitmap[i - 1] || bitmap[i] || bitmap[i + 1] ||
				bitmap[uextent + i] || bitmap[(vextent - 1) * uextent + i];
		}
	}
	// last pixel of first row is special
	(*dilated)[uextent - 1] = bitmap[uextent - 1] || bitmap[uextent - 2] ||
		bitmap[2 * uextent - 1];
	if(vwrap)
		(*dilated)[uextent - 1] = (*dilated)[uextent - 1] ||
			bitmap[vextent * uextent - 1];
	if(uwrap)
		(*dilated)[uextent - 1] = (*dilated)[uextent - 1] || bitmap[0];
	size_t row = 0, prevRow, nextRow = uextent;
	for(size_t j = 1; j < vextent - 1; ++j)
	{
		prevRow = row;
		row = nextRow;
		nextRow = row + uextent;
		// first pixel in row is special
		(*dilated)[row] = bitmap[prevRow] ||
			bitmap[row] || bitmap[row + 1] || bitmap[nextRow];
		if(uwrap)
			(*dilated)[row] = (*dilated)[row] || bitmap[nextRow - 1];
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[row + i] = bitmap[prevRow + i ] ||
				bitmap[row + i - 1] || bitmap[row + i] ||
				bitmap[row + i + 1] || bitmap[nextRow + i];
		}
		// last pixel in row is special
		(*dilated)[row + uextent - 1] = bitmap[prevRow + uextent - 1] ||
			bitmap[row + uextent - 2] || bitmap[row + uextent - 1] ||
			bitmap[nextRow + uextent - 1];
		if(uwrap)
			(*dilated)[row + uextent - 1] = (*dilated)[row + uextent - 1] ||
				bitmap[row];
	}
	// first pixel of last row is special
	(*dilated)[(vextent - 1) * uextent] = bitmap[(vextent - 1) * uextent] ||
		bitmap[(vextent - 1) * uextent + 1] ||
		bitmap[(vextent - 2) * uextent];
	if(vwrap)
		(*dilated)[(vextent - 1) * uextent] =
			(*dilated)[(vextent - 1) * uextent] || bitmap[0];
	if(uwrap)
		(*dilated)[(vextent - 1) * uextent] =
			(*dilated)[(vextent - 1) * uextent] ||
			bitmap[vextent * uextent - 1];
	// last row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] ||
				bitmap[(vextent - 1) * uextent + i - 1] ||
				bitmap[(vextent - 1) * uextent + i + 1] ||
				bitmap[(vextent - 2) * uextent + i];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*dilated)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] ||
				bitmap[(vextent - 1) * uextent + i - 1] ||
				bitmap[(vextent - 1) * uextent + i + 1] ||
				bitmap[(vextent - 2) * uextent + i] ||
				bitmap[i];
		}
	}
	// last pixel
	(*dilated)[bitmap.size() - 1] = bitmap[bitmap.size() - 1] ||
		bitmap[bitmap.size() - 2] || bitmap[bitmap.size() - uextent - 1];
	if(vwrap)
		(*dilated)[bitmap.size() - 1] = (*dilated)[bitmap.size() - 1] ||
			bitmap[uextent - 1];
	if(uwrap)
		(*dilated)[bitmap.size() - 1] = (*dilated)[bitmap.size() - 1] ||
			bitmap[bitmap.size() - uextent];
}

void ErodeSquare(const MiscLib::Vector< char > &bitmap,
	size_t uextent, size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *eroded)
{
	// first pixel is special
	(*eroded)[0] = /*bitmap[0] && bitmap[1] &&
		bitmap[uextent] && bitmap[uextent + 1]*/ false;
	//if(vwrap)
	//	(*eroded)[0] = (*eroded)[0] &&
	//		bitmap[(vextent - 1) * uextent] &&
	//		bitmap[(vextent - 1) * uextent + 1];
	//if(uwrap)
	//	(*eroded)[0] = (*eroded)[0] && bitmap[uextent - 1]
	//		&& bitmap[2 * uextent - 1];
	if(vwrap && uwrap)
	{
		(*eroded)[0] = bitmap[0]
			&& bitmap[1]
			&& bitmap[uextent]
			&& bitmap[uextent + 1]
			&& bitmap[(vextent - 1) * uextent]
			&& bitmap[(vextent - 1) * uextent + 1]
			&& bitmap[uextent - 1]
			&& bitmap[2 * uextent - 1]
			&& bitmap[(vextent - 1) * uextent + uextent - 1];
		//(*eroded)[0] = (*eroded)[0] &&
		//	bitmap[(vextent - 1) * uextent + uextent - 1];
	}
	// first row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[i] = /*bitmap[i - 1] && bitmap[i] && bitmap[i + 1] &&
				bitmap[uextent + i - 1] && bitmap[uextent + i] &&
				bitmap[uextent + i + 1];*/ false;
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[i] = bitmap[i - 1] && bitmap[i] && bitmap[i + 1] &&
				bitmap[uextent + i - 1] && bitmap[uextent + i] &&
				bitmap[uextent + i + 1] &&
				bitmap[(vextent - 1) * uextent + i - 1] &&
				bitmap[(vextent - 1) * uextent + i] &&
				bitmap[(vextent - 1) * uextent + i + 1];
		}
	}
	// last pixel of first row is special
	(*eroded)[uextent - 1] = /*bitmap[uextent - 1] && bitmap[uextent - 2] &&
		bitmap[2 * uextent - 1] && bitmap[2 * uextent - 2]*/ false;
	//if(vwrap)
	//	(*eroded)[uextent - 1] = (*eroded)[uextent - 1] &&
	//		bitmap[vextent * uextent - 1] &&
	//		bitmap[vextent * uextent - 2];
	//if(uwrap)
	//	(*eroded)[uextent - 1] = (*eroded)[uextent - 1] &&
	//		bitmap[uextent] && bitmap[0];
	if(uwrap && vwrap)
	{
		(*eroded)[uextent - 1] = bitmap[uextent - 1]
			&& bitmap[uextent - 2]
			&& bitmap[2 * uextent - 1]
			&& bitmap[2 * uextent - 2]
			&& bitmap[vextent * uextent - 1]
			&& bitmap[vextent * uextent - 2]
			&& bitmap[uextent]
			&& bitmap[0]
			&& bitmap[(vextent - 1) * uextent];
	}
	size_t row = 0, prevRow, nextRow = uextent;
	for(size_t j = 1; j < vextent - 1; ++j)
	{
		prevRow = row;
		row = nextRow;
		nextRow = row + uextent;
		// first pixel in row is special
		(*eroded)[row] = /*bitmap[prevRow] && bitmap[prevRow + 1] &&
			bitmap[row] && bitmap[row + 1] && bitmap[nextRow] &&
			bitmap[nextRow + 1]*/ false;
		if(uwrap)
		{
			(*eroded)[row] = bitmap[prevRow]
				&& bitmap[prevRow + 1]
				&& bitmap[row]
				&& bitmap[row + 1]
				&& bitmap[nextRow]
				&& bitmap[nextRow + 1]
				&& bitmap[row - 1]
				&& bitmap[nextRow - 1]
				&& bitmap[nextRow + uextent - 1];
		}
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[row + i] = bitmap[prevRow + i - 1] &&
				bitmap[prevRow + i ] && bitmap[prevRow + i + 1] &&
				bitmap[row + i - 1] && bitmap[row + i] &&
				bitmap[row + i + 1] && bitmap[nextRow + i - 1] &&
				bitmap[nextRow + i] && bitmap[nextRow + i + 1];
		}
		// last pixel in row is special
		(*eroded)[row + uextent - 1] = /*bitmap[prevRow + uextent - 2] &&
			bitmap[prevRow + uextent - 1] &&
			bitmap[row + uextent - 2] && bitmap[row + uextent - 1] &&
			bitmap[nextRow + uextent - 2] && bitmap[nextRow + uextent - 1]*/ false;
		if(uwrap)
		{
			(*eroded)[row + uextent - 1] = bitmap[prevRow + uextent - 2]
				&& bitmap[prevRow + uextent - 1]
				&& bitmap[row + uextent - 2]
				&& bitmap[row + uextent - 1]
				&& bitmap[nextRow + uextent - 2]
				&& bitmap[nextRow + uextent - 1]
				&& bitmap[prevRow]
				&& bitmap[row]
				&& bitmap[nextRow];
		}
	}
	// first pixel of last row is special
	(*eroded)[(vextent - 1) * uextent] = /*bitmap[(vextent - 1) * uextent] &&
		bitmap[(vextent - 1) * uextent + 1] &&
		bitmap[(vextent - 2) * uextent] && bitmap[(vextent - 2) * uextent + 1]*/ false;
	//if(vwrap)
	//	(*eroded)[(vextent - 1) * uextent] =
	//		(*eroded)[(vextent - 1) * uextent] && bitmap[0] && bitmap[1];
	//if(uwrap)
	//	(*eroded)[(vextent - 1) * uextent] =
	//		(*eroded)[(vextent - 1) * uextent] &&
	//		bitmap[vextent * uextent - 1] &&
	//		bitmap[(vextent - 1) * uextent - 1];
	if(vwrap && uwrap)
	{
		(*eroded)[(vextent - 1) * uextent] = bitmap[(vextent - 1) * uextent]
			&& bitmap[(vextent - 1) * uextent + 1]
			&& bitmap[(vextent - 2) * uextent]
			&& bitmap[(vextent - 2) * uextent + 1]
			&& bitmap[0]
			&& bitmap[1]
			&& bitmap[vextent * uextent - 1]
			&& bitmap[(vextent - 1) * uextent - 1]
			&& bitmap[uextent - 1];
	}
	// last row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[(vextent - 1) * uextent + i] = false
				/*bitmap[(vextent - 1) * uextent + i] &&
				bitmap[(vextent - 1) * uextent + i - 1] &&
				bitmap[(vextent - 1) * uextent + i + 1] &&
				bitmap[(vextent - 2) * uextent + i] &&
				bitmap[(vextent - 2) * uextent + i - 1] &&
				bitmap[(vextent - 2) * uextent + i + 1]*/;
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] &&
				bitmap[(vextent - 1) * uextent + i - 1] &&
				bitmap[(vextent - 1) * uextent + i + 1] &&
				bitmap[(vextent - 2) * uextent + i] &&
				bitmap[(vextent - 2) * uextent + i - 1] &&
				bitmap[(vextent - 2) * uextent + i + 1] &&
				bitmap[i - 1] && bitmap[i] && bitmap[i + 1];
		}
	}
	// last pixel
	(*eroded)[bitmap.size() - 1] = false/*bitmap[bitmap.size() - 1] &&
		bitmap[bitmap.size() - 2] && bitmap[bitmap.size() - uextent - 1] &&
		bitmap[bitmap.size() - uextent - 2]*/;
	//if(vwrap)
	//	(*eroded)[bitmap.size() - 1] = (*eroded)[bitmap.size() - 1] &&
	//		bitmap[uextent - 1] && bitmap[uextent - 2];
	//if(uwrap)
	//	(*eroded)[bitmap.size() - 1] = (*eroded)[bitmap.size() - 1] &&
	//		bitmap[bitmap.size() - uextent] &&
	//		bitmap[bitmap.size() - 2 * uextent];
	if(uwrap && vwrap)
	{
		(*eroded)[bitmap.size() - 1] = bitmap[bitmap.size() - 1]
			&& bitmap[bitmap.size() - 2]
			&& bitmap[bitmap.size() - uextent - 1]
			&& bitmap[bitmap.size() - uextent - 2]
			&& bitmap[uextent - 1]
			&& bitmap[uextent - 2]
			&& bitmap[bitmap.size() - uextent]
			&& bitmap[bitmap.size() - 2 * uextent]
			&& bitmap[0];
	}
}

void ErodeCross(const MiscLib::Vector< char > &bitmap,
	size_t uextent, size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< char > *eroded)
{
	// first pixel is special
	(*eroded)[0] = bitmap[0] && bitmap[1] &&
		bitmap[uextent];
	if(vwrap)
		(*eroded)[0] = (*eroded)[0] &&
			bitmap[(vextent - 1) * uextent];
	if(uwrap)
		(*eroded)[0] = (*eroded)[0] && bitmap[uextent - 1];
	// first row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[i] = bitmap[i - 1] && bitmap[i] && bitmap[i + 1] &&
				bitmap[uextent + i];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[i] = bitmap[i - 1] && bitmap[i] && bitmap[i + 1] &&
				bitmap[uextent + i] && bitmap[(vextent - 1) * uextent + i];
		}
	}
	// last pixel of first row is special
	(*eroded)[uextent - 1] = bitmap[uextent - 1] && bitmap[uextent - 2] &&
		bitmap[2 * uextent - 1];
	if(vwrap)
		(*eroded)[uextent - 1] = (*eroded)[uextent - 1] &&
			bitmap[vextent * uextent - 1];
	if(uwrap)
		(*eroded)[uextent - 1] = (*eroded)[uextent - 1] && bitmap[0];
	size_t row = 0, prevRow, nextRow = uextent;
	for(size_t j = 1; j < vextent - 1; ++j)
	{
		prevRow = row;
		row = nextRow;
		nextRow = row + uextent;
		// first pixel in row is special
		(*eroded)[row] = bitmap[prevRow] &&
			bitmap[row] && bitmap[row + 1] && bitmap[nextRow];
		if(uwrap)
			(*eroded)[row] = (*eroded)[row] && bitmap[nextRow - 1];
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[row + i] = bitmap[prevRow + i ] &&
				bitmap[row + i - 1] && bitmap[row + i] &&
				bitmap[row + i + 1] && bitmap[nextRow + i];
		}
		// last pixel in row is special
		(*eroded)[row + uextent - 1] = bitmap[prevRow + uextent - 1] &&
			bitmap[row + uextent - 2] && bitmap[row + uextent - 1] &&
			bitmap[nextRow + uextent - 1];
		if(uwrap)
			(*eroded)[row + uextent - 1] = (*eroded)[row + uextent - 1] &&
				bitmap[row];
	}
	// first pixel of last row is special
	(*eroded)[(vextent - 1) * uextent] = bitmap[(vextent - 1) * uextent] &&
		bitmap[(vextent - 1) * uextent + 1] &&
		bitmap[(vextent - 2) * uextent];
	if(vwrap)
		(*eroded)[(vextent - 1) * uextent] =
			(*eroded)[(vextent - 1) * uextent] && bitmap[0];
	if(uwrap)
		(*eroded)[(vextent - 1) * uextent] =
			(*eroded)[(vextent - 1) * uextent] &&
			bitmap[vextent * uextent - 1];
	// last row is special
	if(!vwrap)
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] &&
				bitmap[(vextent - 1) * uextent + i - 1] &&
				bitmap[(vextent - 1) * uextent + i + 1] &&
				bitmap[(vextent - 2) * uextent + i];
		}
	}
	else
	{
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			(*eroded)[(vextent - 1) * uextent + i] =
				bitmap[(vextent - 1) * uextent + i] &&
				bitmap[(vextent - 1) * uextent + i - 1] &&
				bitmap[(vextent - 1) * uextent + i + 1] &&
				bitmap[(vextent - 2) * uextent + i] &&
				bitmap[i];
		}
	}
	// last pixel
	(*eroded)[bitmap.size() - 1] = bitmap[bitmap.size() - 1] &&
		bitmap[bitmap.size() - 2] && bitmap[bitmap.size() - uextent - 1];
	if(vwrap)
		(*eroded)[bitmap.size() - 1] = (*eroded)[bitmap.size() - 1] &&
			bitmap[uextent - 1];
	if(uwrap)
		(*eroded)[bitmap.size() - 1] = (*eroded)[bitmap.size() - 1] &&
			bitmap[bitmap.size() - uextent];
}

void PreWrappedComponents(const MiscLib::Vector< char > &bitmap, size_t uextent,
	size_t vextent, MiscLib::Vector< int > *preWrappedComponentsImg,
	MiscLib::Vector< int > *relabelComponentsImg,
	const MiscLib::Vector< std::pair< int, size_t > > &inLabels,
	MiscLib::Vector< std::pair< int, size_t > > *labels)
{
	MiscLib::Vector< std::pair< int, size_t > > tempLabels(inLabels);
	tempLabels.reserve(bitmap.size() / 2 + 1); // this is the maximum of possible tempLabels
	if(!tempLabels.size())
		tempLabels.push_back(std::make_pair(0, size_t(0)));
	int curLabel = static_cast<int>(tempLabels.size()) - 1;
	size_t prevRow, row = 0, nextRow = uextent;
	for(size_t j = 1; j < vextent - 1; ++j)
	{
		prevRow = row;
		row = nextRow;
		nextRow = row + uextent;
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			if(!bitmap[row + i])
			{
				(*preWrappedComponentsImg)[row + i] = 0;
				++tempLabels[0].second;
				continue;
			}
			// get neighborhood
			int n[8];
			n[0] = (*preWrappedComponentsImg)[prevRow + i - 1];
			n[1] = (*preWrappedComponentsImg)[prevRow + i];
			n[2] = (*preWrappedComponentsImg)[prevRow + i + 1];
			n[3] = (*preWrappedComponentsImg)[row + i - 1];
			n[4] = (*preWrappedComponentsImg)[row + i + 1];
			n[5] = (*preWrappedComponentsImg)[nextRow + i - 1];
			n[6] = (*preWrappedComponentsImg)[nextRow + i];
			n[7] = (*preWrappedComponentsImg)[nextRow + i + 1];
			(*preWrappedComponentsImg)[row + i] = Label(n, 8,
				&curLabel, &tempLabels);
		}
	}

	// reduce the tempLabels
	for(int i = static_cast<int>(tempLabels.size()) - 1; i > 0; --i)
		tempLabels[i].first = ReduceLabel(i, tempLabels);
	MiscLib::Vector< int > condensed(tempLabels.size());
	labels->clear();
	labels->reserve(condensed.size());
	int count = 0;
    for(size_t i = 0; i < tempLabels.size(); ++i)
		if(i == (size_t)tempLabels[i].first)
		{
			labels->push_back(std::make_pair(count, tempLabels[i].second));
			condensed[i] = count;
			++count;
		}
		else
			(*labels)[condensed[tempLabels[i].first]].second
				+= tempLabels[i].second;
	// set new component ids
	for(size_t i = 0; i < preWrappedComponentsImg->size(); ++i)
		(*preWrappedComponentsImg)[i] =
			condensed[tempLabels[(*preWrappedComponentsImg)[i]].first];
	for(size_t i = 0; i < relabelComponentsImg->size(); ++i)
		(*relabelComponentsImg)[i] =
			condensed[tempLabels[(*relabelComponentsImg)[i]].first];
}

void Components(const MiscLib::Vector< char > &bitmap,
	size_t uextent, size_t vextent, bool uwrap, bool vwrap,
	MiscLib::Vector< int > *componentsImg,
	MiscLib::Vector< std::pair< int, size_t > > *labels)
{
	componentsImg->resize(uextent * vextent);
	MiscLib::Vector< std::pair< int, size_t > > tempLabels;
	tempLabels.reserve(componentsImg->size() / 2 + 1); // this is the maximum of possible tempLabels
	tempLabels.push_back(std::make_pair(0, size_t(0)));
	// use an eight neighborhood
	// first row is special
	// first pixel is special
	// wrapping does not make sense in the first row: no pixels have been
	// assigned components yet
	int curLabel = 0;
	if(bitmap[0])
	{
		(*componentsImg)[0] = ++curLabel;
		tempLabels.push_back(std::make_pair(curLabel, size_t(1)));
	}
	else
	{
		(*componentsImg)[0] = 0;
		++tempLabels[0].second;
	}
	// handle first row
	for(size_t i = 1; i < uextent; ++i)
	{
		// in the first row only the previous pixel has to be considered
		if(bitmap[i])
		{
			if((*componentsImg)[i - 1])
			{
				(*componentsImg)[i] = (*componentsImg)[i - 1];
				++tempLabels[(*componentsImg)[i]].second;
			}
			else
			{
				(*componentsImg)[i] = ++curLabel;
				tempLabels.push_back(std::make_pair(curLabel, size_t(1)));
			}
		}
		else
		{
			(*componentsImg)[i] = 0;
			++tempLabels[0].second;
		}
	}
	size_t prevRow, row = 0;
	size_t jend = vwrap? vextent - 1 : vextent;
	for(size_t j = 1; j < jend; ++j)
	{
		prevRow = row;
		row = prevRow + uextent;
		// first pixel in row gets different treatment
		if(bitmap[row])
		{
			if((*componentsImg)[prevRow]) // pixel above in component?
			{
				(*componentsImg)[row] = (*componentsImg)[prevRow];
				++tempLabels[(*componentsImg)[row]].second;
			}
			else
			{
				int n[2];
				n[0] = (*componentsImg)[prevRow + 1];
				if(uwrap)
					n[1] = (*componentsImg)[prevRow + uextent - 1];
				(*componentsImg)[row] = Label(n, uwrap? 2 : 1, &curLabel,
					&tempLabels);
			}
		}
		else
		{
			(*componentsImg)[row] = 0;
			++tempLabels[0].second;
		}
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			if(!bitmap[row + i])
			{
				(*componentsImg)[row + i] = 0;
				++tempLabels[0].second;
				continue;
			}
			int n[4];
			n[0] = (*componentsImg)[row + i - 1];
			n[1] = (*componentsImg)[prevRow + i - 1];
			n[2] = (*componentsImg)[prevRow + i];
			n[3] = (*componentsImg)[prevRow + i + 1];
			(*componentsImg)[row + i] = Label(n, 4, &curLabel, &tempLabels);
		}
		// last pixel in the row
		if(!bitmap[row + uextent - 1])
		{
			(*componentsImg)[row + uextent - 1] = 0;
			++tempLabels[0].second;
			continue;
		}
		int n[5];
		n[0] = (*componentsImg)[row + uextent - 2];
		n[1] = (*componentsImg)[prevRow + uextent - 2];
		n[2] = (*componentsImg)[prevRow + uextent - 1];
		if(uwrap)
		{
			n[3] = (*componentsImg)[prevRow];
			n[4] = (*componentsImg)[row];
		}
		(*componentsImg)[row + uextent - 1] = Label(n, uwrap? 5 : 3, &curLabel,
			&tempLabels);
	}
	// last row
	if(vwrap) // in case of vwrapping the last row is computed with almost full
		// neighborhood
	{
		prevRow = (vextent - 2) * uextent;
		row = (vextent - 1) * uextent;
		// first pixel
		if(bitmap[row])
		{
			int n[6];
			n[0] = (*componentsImg)[prevRow];
			n[1] = (*componentsImg)[prevRow + 1];
			n[2] = (*componentsImg)[0];
			n[3] = (*componentsImg)[1];
			if(uwrap)
			{
				n[4] = (*componentsImg)[prevRow + uextent - 1];
				n[5] = (*componentsImg)[uextent - 1];
			}
			(*componentsImg)[row] = Label(n, uwrap? 6 : 4, &curLabel,
				&tempLabels);
		}
		else
		{
			(*componentsImg)[row] = 0;
			++tempLabels[0].second;
		}
		for(size_t i = 1; i < uextent - 1; ++i)
		{
			if(!bitmap[row + i])
			{
				(*componentsImg)[row + i] = 0;
				++tempLabels[0].second;
				continue;
			}
			int n[7];
			n[0] = (*componentsImg)[row + i - 1];
			n[1] = (*componentsImg)[prevRow + i - 1];
			n[2] = (*componentsImg)[prevRow + i];
			n[3] = (*componentsImg)[prevRow + i + 1];
			n[4] = (*componentsImg)[i - 1];
			n[5] = (*componentsImg)[i];
			n[6] = (*componentsImg)[i + 1];
			(*componentsImg)[row + i] = Label(n, 7, &curLabel, &tempLabels);
		}
		// last pixel
		if(!bitmap[row + uextent - 1])
		{
			(*componentsImg)[row + uextent - 1] = 0;
			++tempLabels[0].second;
		}
		else
		{
			int n[8];
			n[0] = (*componentsImg)[row + uextent - 2];
			n[1] = (*componentsImg)[prevRow + uextent - 2];
			n[2] = (*componentsImg)[prevRow + uextent - 1];
			n[3] = (*componentsImg)[uextent - 2];
			n[4] = (*componentsImg)[uextent - 1];
			if(uwrap)
			{
				n[5] = (*componentsImg)[prevRow];
				n[6] = (*componentsImg)[row];
				n[7] = (*componentsImg)[0];
			}
			(*componentsImg)[row + uextent - 1] = Label(n, uwrap? 8 : 5,
				&curLabel, &tempLabels);
		}
	}
	// reduce the tempLabels
	for(int i = static_cast<int>(tempLabels.size()) - 1; i > 0; --i)
		tempLabels[i].first = ReduceLabel(i, tempLabels);
	MiscLib::Vector< int > condensed(tempLabels.size());
	labels->clear();
	labels->reserve(condensed.size());
	int count = 0;
    for(size_t i = 0; i < tempLabels.size(); ++i)
		if(i == (size_t)tempLabels[i].first)
		{
			labels->push_back(std::make_pair(count, tempLabels[i].second));
			condensed[i] = count;
			++count;
		}
		else
			(*labels)[condensed[tempLabels[i].first]].second
				+= tempLabels[i].second;
	// set new component ids
	for(size_t i = 0; i < componentsImg->size(); ++i)
		(*componentsImg)[i] =
			condensed[tempLabels[(*componentsImg)[i]].first];
}

int Label(int n[], int size, int *curLabel,
	MiscLib::Vector< std::pair< int, size_t > > *labels)
{
	// check if components are set
	int count = 0, found;
	for(int i = 0; i < size; ++i)
	{
		if(n[i])
		{
			++count;
			found = n[i];
		}
	}
	if(!count)
	{
		++(*curLabel);
		labels->push_back(std::make_pair(*curLabel, size_t(1)));
		return *curLabel;
	}
	else if(count == 1)
	{
		++(*labels)[found].second;
		return found;
	}
	else
	{
		++(*labels)[found].second;
		// associate the remaining labels
		for(int i = 0; i < size; ++i)
		{
			if(n[i] && n[i] != found)
				AssociateLabel(n[i], found, labels);
		}
		return found;
	}
}

void AssociateLabel(int a, int b,
	MiscLib::Vector< std::pair< int, size_t > > *labels)
{
	if(a > b)
	{
		AssociateLabel(b, a, labels);
		return;
	}
	if((a == b) || ((*labels)[b].first == a))
		return;
	if((*labels)[b].first == b)
		(*labels)[b].first = a;
	else
	{
		AssociateLabel((*labels)[b].first, a, labels);
		if((*labels)[b].first > a)
			(*labels)[b].first = a;
	}
}

int ReduceLabel(int a,
	const MiscLib::Vector< std::pair< int, size_t > > &labels)
{
	if(labels[a].first == a)
		return labels[a].first;
	return ReduceLabel(labels[a].first, labels);
}

bool IsEdge(const MiscLib::Vector< int > &componentImg, size_t uextent,
	size_t vextent, int label, bool uwrap, bool vwrap,
	int startx, int starty, int dirx, int diry,
	size_t *targetx, size_t *targety)
{
	if(dirx > 0)
	{
		if((size_t)startx == uextent)
			return false;
		if((size_t)starty == vextent)
			return false;
		*targetx = (uwrap && (size_t)startx == uextent - 1)? 0 : startx + 1;
		*targety = starty;
		return componentImg[starty * uextent + startx] == label
			&& ((starty > 0 && componentImg[(starty - 1) * uextent + startx] != label)
				|| (!vwrap && starty == 0)
				|| (vwrap && starty == 0 && componentImg[(vextent - 1) * uextent + startx] != label));
	}
	if(dirx < 0)
	{
		if(!uwrap && startx == 0)
			return false;
		*targetx = (startx == 0)? uextent - 1 : startx - 1;
		*targety = starty;
		return (!vwrap && (size_t)starty == vextent && componentImg[(starty - 1) * uextent + (*targetx)] == label)
			|| ((size_t)starty != vextent && (componentImg[starty * uextent + (*targetx)] != label
			&& ((starty > 0 && componentImg[(starty - 1) * uextent + (*targetx)] == label)
				|| (vwrap && starty == 0 && componentImg[(vextent - 1) * uextent + (*targetx)] == label))));
	}
	if(diry > 0)
	{
		if((size_t)starty == vextent)
			return false;
		*targetx = startx;
		*targety = (vwrap && (size_t)starty == vextent - 1)? 0 : starty + 1;
		return (!uwrap && (size_t)startx == uextent && componentImg[starty * uextent + (startx - 1)] == label)
			|| ((size_t)startx != uextent && (componentImg[starty * uextent + startx] != label
			&& (((size_t)startx > 0 && componentImg[starty * uextent + (startx - 1)] == label)
				|| (uwrap && startx == 0 && componentImg[starty * uextent + (uextent - 1)] == label))));
	}
	if(diry < 0)
	{
		if(!vwrap && starty == 0)
			return false;
		if((size_t)startx == uextent)
			return false;
		*targetx = startx;
		*targety = (starty == 0)? vextent - 1 : starty - 1;
		return componentImg[(*targety) * uextent + startx] == label
			&& ((startx > 0 && componentImg[(*targety) * uextent + (startx - 1)] != label)
				|| (!uwrap && startx == 0)
				|| (uwrap && startx == 0 && componentImg[(*targety) * uextent + (uextent - 1)] != label));
	}
	return false;
}

// finds the loops around a connected component as polygons
void ComponentLoops(const MiscLib::Vector< int > &componentImg, size_t uextent,
	size_t vextent, int label, bool uwrap, bool vwrap,
	MiscLib::Vector< MiscLib::Vector< GfxTL::VectorXD< 2, size_t > > > *polys)
{
	typedef GfxTL::VectorXD< 2, size_t > Vec2;
	// find first point of component
	size_t firsti = 0;
	int x, y, prevx, prevy;
	// the corners of our pixels will be the vertices of our polygons
	// (x, y) is the upper left corner of the pixel y * uextent + x
	HashGrid< bool, 4 > edges;
	size_t edgesExtent[] = { uextent + 1, vextent + 1, 3, 3 };
	edges.Extent(edgesExtent);
	bool prevPixelWasWhite = true;
	do
	{
		// find the first edge in the polygon
		// edges are oriented so that the "black" pixels are on the right
		// black pixels are pixels == label
		for(; firsti < componentImg.size(); ++firsti)
		{
			if(prevPixelWasWhite && componentImg[firsti] == label)
			{
				prevPixelWasWhite = false;
				x = static_cast<int>(firsti % uextent);
				y = static_cast<int>(firsti / uextent);
				break;
			}
			prevPixelWasWhite = componentImg[firsti] != label;
		}
		if(firsti >= componentImg.size()) // unable to find a pixel -> good bye
		{
			// if there is a uwrap, then the last row could be an outer loop
			// this outer loop could be missed of all pixels in the last
			// row are black
			// to find that out we spawn another trial at the first
			// pixel in the last row (if it is black)
			// if the loop has already been detected, than this
			// edge should already be in edges
			if(!uwrap)
				break;
			if(componentImg[(vextent - 1) * uextent] == label)
			{
				x = 0;
				y = static_cast<int>(vextent) - 1;
			}
		}
		MiscLib::Vector< Vec2 > poly;
		// we initialize the path with an oriented edge
		// since the black pixel is on the right the edge goes from
		// bottom to top, i.e. from (x, y + 1) to (x, y)
		if((x > 0 && (size_t)y < vextent - 1)
			|| (!uwrap && !vwrap) || (vwrap && !uwrap && y == 0))
		{
			// on the left of pixel
			// check if edge was visited already
			unsigned int edgeIndex[] = { static_cast<unsigned>(x), static_cast<unsigned>(y), 1u, 2u };
			if(edges.find(edgeIndex))
				continue;
			prevx = 0;
			prevy = 1;
		}
		else if(uwrap && !vwrap && x == 0 && (size_t)y != vextent - 1)
		{
			size_t dx, dy;
			if(!IsEdge(componentImg, uextent, vextent, label, uwrap, vwrap,
				x, y, 1, 0, &dx, &dy))
				continue;
			// check if edge was visited already
			unsigned int edgeIndex[] = { static_cast<unsigned>(x + 1), static_cast<unsigned>(y), 0, 1 };
			if(edges.find(edgeIndex))
				continue;
			// on top of pixel
			prevx = -1;
			prevy = 0;
			++x;
		}
		else if(uwrap && !vwrap && x == 0 && (size_t)y == vextent - 1)
		{
			size_t dx, dy;
			if(!IsEdge(componentImg, uextent, vextent, label, uwrap, vwrap,
				x + 1, y + 1, -1, 0, &dx, &dy))
				continue;
			// on bottom of pixel
			// check if edge was visited already
			unsigned int edgeIndex[] = { static_cast<unsigned>(x + 1), static_cast<unsigned>(y + 1), 0, 1 };
			if(edges.find(edgeIndex))
				continue;
			prevx = -1;
			prevy = 0;
			++y;
		}
		else if(!uwrap && vwrap && (size_t)x == uextent - 1)
		{
			// on right of pixel
			size_t dx, dy;
			if(!IsEdge(componentImg, uextent, vextent, label, uwrap, vwrap,
				x + 1, y + 1, 0, -1, &dx, &dy))
				continue;
			// on bottom of pixel
			// check if edge was visited already
			unsigned int edgeIndex[] = { static_cast<unsigned>(x + 1), static_cast<unsigned>(y + 1), 1, 0 };
			if(edges.find(edgeIndex))
				continue;
			prevx = 0;
			prevy = 1;
			++y;
		}
		else
			continue; // we are unable to start a loop at this position
		poly.push_back(Vec2(x + prevx, y + prevy));
		edges[x][y][prevx + 1][prevy + 1] = true;
		do
		{
			poly.push_back(Vec2(x, y));
			// check the four neighbors of (x, y) from left to right
			// starting from where we came from
			// we take the first edge that we encounter
			size_t nextx, nexty;
			size_t checkEdge;
			for(checkEdge = 0; checkEdge < 3; ++checkEdge)
			{
				std::swap(prevx, prevy);
				prevx *= -1;
				if(IsEdge(componentImg, uextent, vextent, label, uwrap, vwrap,
					x, y, prevx, prevy, &nextx, &nexty))
					break;
			}
			if(checkEdge > 3)
				return;
			x = static_cast<int>(nextx);
			y = static_cast<int>(nexty);
			prevx = -prevx;
			prevy = -prevy;
			edges[x][y][prevx + 1][prevy + 1] = true;
		}
		while(poly[0] != Vec2(x, y));
		polys->push_back(poly);
	}
	while(firsti < componentImg.size());
#ifdef _DEBUG
	static int fname_int = 0;
	std::ostringstream fn;
	fn << "ComponentLoopsInput" << fname_int << ".txt";
	std::ofstream file;
	file.open(fn.str().c_str(), std::ios::out);
	for(size_t j = 0; j < vextent; ++j)
	{
		for(size_t i = 0; i < uextent; ++i)
			file /*<< std::setw(3)*/ << componentImg[j * uextent + i]/* << " "*/;
		file << std::endl;
	}
	file.close();
	MiscLib::Vector< int > loopsImg((uextent + 1) * (vextent + 1), 0);
	std::ostringstream fn2;
	fn2 << "ComponentLoopsOutput" << fname_int++ << ".txt";
	for(size_t i = 0; i < polys->size(); ++i)
		for(size_t j = 0; j < (*polys)[i].size(); ++j)
			loopsImg[(*polys)[i][j][1] * (uextent + 1) + (*polys)[i][j][0]] = i + 1;
	file.open(fn2.str().c_str(), std::ios::out);
	for(size_t j = 0; j < vextent + 1; ++j)
	{
		for(size_t i = 0; i < uextent + 1; ++i)
			file /*<< std::setw(3)*/ << loopsImg[j * (uextent + 1) + i]/* << " "*/;
		file << std::endl;
	}
	file.close();
#endif
}
