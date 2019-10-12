#ifndef BLOCK_DBASE_IO_HEADER
#define BLOCK_DBASE_IO_HEADER

#if defined( libBlockDBaseIO_EXPORTS )
#  define BLOCKDB_IO_LIB_API __declspec(dllexport)
#else
#  define BLOCKDB_IO_LIB_API __declspec(dllimport)
#endif

//system
#include <vector>
#include <string>

class BLOCKDB_IO_LIB_API BlockDBasiIO
{
public:
	BlockDBasiIO();
private:
	
};

bool connectPgsDB(const std::string &connInfo);


#endif //BLOCK_DBASE_IO_HEADER
