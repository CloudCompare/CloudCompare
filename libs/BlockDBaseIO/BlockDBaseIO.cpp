#include "BlockDBaseIO.h"

#include "libpq-fe.h"

BlockDBasiIO::BlockDBasiIO()
{

}

bool connectPgsDB(const std::string &connInfo)
{

	PGconn* Conn = PQconnectdb(connInfo.c_str());
	if (PQstatus(Conn) == CONNECTION_OK) {


	}
	return true;
}
