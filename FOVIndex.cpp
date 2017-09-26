//#include "stdafx.h"
#include "Rtree.h"
#include <stdio.h>

FILE* fp, *fplog;
int countdataid;
int ResultObjCnt;
int NodeAccessCnt;

int main()
{
	fplog = fopen("log.txt", "r+");
	infoadd(); //-----build OR-tree indexing

	//CircleRangeQuery();
	fclose(fplog);
	return 0;
}
