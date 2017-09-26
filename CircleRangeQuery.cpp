
#include "Rtree.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>

using namespace std;

extern FILE* fp;
extern int countdataid;
extern int ResultObjCnt;
extern int NodeAccessCnt;


void CircleRangeQuery()
{
	REALTYPE QueryPoint_lat = 0.0;
	REALTYPE QueryPoint_lng = 0.0;
	REALTYPE QueryRadius = 0.0;  //50 meters
	FILE *fpquery = fopen("queries.txt", "r+");
	FILE *fpoutput = fopen("output.txt", "r+");
	countdataid = 0;
	int TotalObjNum = 10000;
	int MaxResultObjCnt = -1;
	int MinResultObjCnt = TotalObjNum; //1M
	int TotalResultObjCnt = 0;
	int MaxNodeAccessCnt = -1;
	int MinNodeAccessCnt = TotalObjNum;
	int TotalNodeAccessCnt = 0;
	double MaxQueryTime = 0.0;
	double MinQueryTime = DBL_MAX;

	clock_t tStart = clock();
	clock_t tStarti = clock();

	RTREEROOT root; 
	root.root_node = (RTREENODE*)malloc(sizeof(RTREENODE));
	InitNode(root.root_node);

	fp = fopen("IR2tree.index", "rb+");
	fseek(fp, 0L, SEEK_END);
	long leng = ftell(fp);


	while (!feof(fpquery))
	{
		tStarti = clock();
		ResultObjCnt = 0;
		NodeAccessCnt = 0;
		fscanf(fpquery, "%f, %f, %f\n", &QueryPoint_lat, &QueryPoint_lng, &QueryRadius);

		if (leng == 0)
		{
			printf("Sorry, the index is empty!\n");
		}//end if leng==0
		else
		{
			FindRoot(root.root_node);
			RTREEBRANCH* RootBranch = (RTREEBRANCH*)malloc(sizeof(RTREEBRANCH));
			InitBranch(RootBranch);
			RootBranch->childid = root.root_node->nodeid;
			RootBranch->mbr = RTreeNodeCover(root.root_node);
			RootBranch->angle = MinimumBoundingAngle(root.root_node);
			RootBranch->R = MinimumBoundingR(root.root_node);

			stack<RTREEBRANCH*> RangeQueryStack;
			RangeQueryStack.push(RootBranch);
			RTREENODE node;
			while (!RangeQueryStack.empty())
			{
				RTREEBRANCH* branchp = RangeQueryStack.top();
				RangeQueryStack.pop();
				fseek(fp, (branchp->childid - 1)*sizeof(RTREENODE), 0);
				fread(&node, sizeof(RTREENODE), 1, fp);
				NodeAccessCnt++;

				int flag = NodeOverlapCheck(branchp, QueryPoint_lat, QueryPoint_lng, QueryRadius);
				if (flag == 1) //partial overlap, then expand node (node must be an index node)
				{
					if (node.level>0) //non-leaf node
					{
						//countdataid++;
						for (int i = 0; i<node.count; i++)
						{
							RTREEBRANCH* childbranchp = (RTREEBRANCH*)malloc(sizeof(RTREEBRANCH));
							CopyBranch(childbranchp, &(node.branch[i]));
							RangeQueryStack.push(childbranchp);
						}
					}
					else //leaf node
					{
						for (int i = 0; i<node.count; i++)
						{
							int ObjOverlapFlag = ObjectOverlapCheck(node.branch[i].mbr.bound[1], 
							                                        node.branch[i].mbr.bound[0], 
																	node.branch[i].angle.min, 
																	node.branch[i].angle.max, 
																	node.branch[i].R.min, 
																	QueryPoint_lat, 
																	QueryPoint_lng, 
																	QueryRadius);
							if (ObjOverlapFlag == 1)//overlap, otherwise ignore it.
							{
								ResultObjCnt++;
							}
						}
					}
				}
				else if (flag == 2) 
				{//total overlap, then report all the objects in node to be results
					ReportResults(&node);
				}
				//else countdataid++;
				//else no overlap, prune node
				free(branchp);
			}//end while
			//printf("%d\n", countdataid);
		}//end else

		TotalResultObjCnt += ResultObjCnt;
		if (MaxResultObjCnt < ResultObjCnt) MaxResultObjCnt = ResultObjCnt;
		if (MinResultObjCnt > ResultObjCnt) MinResultObjCnt = ResultObjCnt;
		TotalNodeAccessCnt += NodeAccessCnt;
		if (MaxNodeAccessCnt < NodeAccessCnt) MaxNodeAccessCnt = NodeAccessCnt;
		if (MinNodeAccessCnt > NodeAccessCnt) MinNodeAccessCnt = NodeAccessCnt;
		if (MaxQueryTime < (double)(clock() - tStarti)) 
			MaxQueryTime = (double)(clock() - tStarti);
		if (MinQueryTime > (double)(clock() - tStarti)) 
			MinQueryTime = (double)(clock() - tStarti);

	}
	fclose(fp);
	//fprintf(fpoutput, "")
	fprintf(fpoutput, "Result object #: Average %d, Maximum %d, Minimum %d\n", 
	        TotalResultObjCnt / TotalObjNum, MaxResultObjCnt, MinResultObjCnt);
	fprintf(fpoutput, "Accessed node #: Average %d, Maximum %d, Minimum %d\n", 
	        TotalNodeAccessCnt / TotalObjNum, MaxNodeAccessCnt, MinNodeAccessCnt);
	fprintf(fpoutput, "Query time: Average %.6fs, Maximum %.6f, Minimum %.6f\n", 
	        (double)(clock() - tStart) / CLOCKS_PER_SEC /TotalObjNum, 
			MaxQueryTime, MinQueryTime);
	fclose(fpoutput);
	fclose(fpquery);
}




REALTYPE MaxDist(RTREEMBR* mbrp, REALTYPE qlat, REALTYPE qlng)
{
	REALTYPE px, py;
	if (qlat > (mbrp->bound[1] + mbrp->bound[3]) / 2) py = mbrp->bound[1];
	else py = mbrp->bound[3];
	if (qlng > (mbrp->bound[0] + mbrp->bound[2]) / 2) px = mbrp->bound[0];
	else px = mbrp->bound[2];
	return EarthDistance(py, px, qlat, qlng);
}



REALTYPE MinDist(RTREEMBR* mbrp, REALTYPE qlat, REALTYPE qlng)
{
	if (qlat < mbrp->bound[3] && 
	    qlat > mbrp->bound[1] && 
		qlng < mbrp->bound[2] && 
		qlng > mbrp->bound[0]) return 0;
	REALTYPE px, py;
	if (qlat > (mbrp->bound[1] + mbrp->bound[3]) / 2) py = mbrp->bound[3];
	else py = mbrp->bound[1];
	if (qlng > (mbrp->bound[0] + mbrp->bound[2]) / 2) px = mbrp->bound[2];
	else px = mbrp->bound[0];
	return EarthDistance(py, px, qlat, qlng);
}



//the return value is in [0, 2PI]
REALTYPE CalculateAngleClockwiseVector(REALTYPE v1x, 
                                       REALTYPE v1y, 
									   REALTYPE v2x, 
									   REALTYPE v2y) 
{
	//angle is in [0, PI) x1*y1+x2*y2
	REALTYPE angle = acos(v1x*v2x + v1y*v2y);  
	/** 
	 * Cross multipleply x1*y2+x2*y1 = |a|*|b|*sin<a,b> 
	 * right hand rule, counter clockwise 
	 */
	REALTYPE cross = v1x*v2y + v2x*v1y;  
	if (cross>0) return 2 * 3.1415926 - angle;
	else return angle;
}


void CoverTwoAnglesVector(ONED_RANGE pqs, ONED_RANGE pqe, 
                          ONED_RANGE * angle, ONED_RANGE& CoverAngleVS, 
						  ONED_RANGE& CoverAngleVE)
{
	REALTYPE AngleS1E2 = CalculateAngleClockwiseVector(pqs.min, pqs.max, 
	                                                   sin(angle->max), 
													   cos(angle->max));
	REALTYPE AngleS2E1 = CalculateAngleClockwiseVector(sin(angle->min), 
	                                                   cos(angle->min), 
													   pqe.min, pqe.max);
	if (AngleS1E2 > AngleS2E1)
	{
		CoverAngleVS.min = pqs.min; CoverAngleVS.max = pqs.max;
		CoverAngleVE.min = sin(angle->max); CoverAngleVE.max = cos(angle->max);
	}
	else
	{
		CoverAngleVS.min = sin(angle->min); CoverAngleVE.max = cos(angle->min);
		CoverAngleVE.min = pqe.min; CoverAngleVE.max = pqe.max;
	}
}



void CalculateMaxMinAngle(ONED_RANGE* angle, RTREEMBR* mbrp, 
                          REALTYPE qlat, REALTYPE qlng, 
						  REALTYPE &minangle, REALTYPE &maxangle)
{
	ONED_RANGE pqs, pqe; //two vectors
	if (qlng>mbrp->bound[2])
	{
		if (qlat>mbrp->bound[3]) 
		{ 
			pqs.min = mbrp->bound[2] - qlng; 
			pqs.max = mbrp->bound[1] - qlat; 
			pqe.min = mbrp->bound[0] - qlng; 
			pqe.max = mbrp->bound[3]; 
		}
		else if (qlat<mbrp->bound[1]) 
		{ 
			pqs.min = mbrp->bound[2] - qlng; 
			pqs.max = mbrp->bound[3] - qlat; 
			pqe.min = mbrp->bound[0] - qlng; 
			pqe.max = mbrp->bound[1]; 
		}
		else 
		{ 
			pqs.min = mbrp->bound[2] - qlng; 
			pqs.max = mbrp->bound[1] - qlat; 
			pqe.min = mbrp->bound[2] - qlng; 
			pqe.max = mbrp->bound[3]; 
		}
	}
	else if (qlng < mbrp->bound[2])
	{
		if (qlat>mbrp->bound[3]) 
		{ 
			pqs.min = mbrp->bound[2] - qlng; 
			pqs.max = mbrp->bound[3] - qlat; 
			pqe.min = mbrp->bound[0] - qlng; 
			pqe.max = mbrp->bound[1]; 
		}
		else if (qlat<mbrp->bound[1]) 
		{ 
			pqs.min = mbrp->bound[0] - qlng; 
			pqs.max = mbrp->bound[3] - qlat; 
			pqe.min = mbrp->bound[2] - qlng; 
			pqe.max = mbrp->bound[1]; 
		}
		else 
		{ 
			pqs.min = mbrp->bound[0] - qlng; 
			pqs.max = mbrp->bound[3] - qlat; 
			pqe.min = mbrp->bound[0] - qlng; 
			pqe.max = mbrp->bound[1]; 
		}
	}
	else
	{
		if (qlat>mbrp->bound[3]) 
		{ 
			pqs.min = mbrp->bound[2] - qlng; 
			pqs.max = mbrp->bound[3] - qlat; 
			pqe.min = mbrp->bound[0] - qlng; 
			pqe.max = mbrp->bound[3]; 
		}
		else if (qlat<mbrp->bound[1]) 
		{ 
			pqs.min = mbrp->bound[0] - qlng; 
			pqs.max = mbrp->bound[1] - qlat; 
			pqe.min = mbrp->bound[2] - qlng; 
			pqe.max = mbrp->bound[1]; 
		}
		else 
		{ 
			minangle = 0; 
			maxangle = 2 * PI; 
			return; 
		}
	}
	ONED_RANGE CoverAngleVS, CoverAngleVE;
	CoverTwoAnglesVector(pqs, pqe, angle, CoverAngleVS, CoverAngleVE);
	REALTYPE CoverAngle = CalculateAngleClockwiseVector(CoverAngleVS.min, 
	                                                    CoverAngleVS.max, 
														CoverAngleVE.min, 
														CoverAngleVE.max);
	REALTYPE MBRpqAngle = CalculateAngleClockwiseVector(pqs.min, pqs.max, 
	                                                    pqe.min, pqe.max);
	REALTYPE MBAAngle = CalculateAngleClockwiseVector(sin(angle->min), 
	                                                  cos(angle->min), 
													  sin(angle->max), 
													  cos(angle->max));
	minangle = CoverAngle - MBRpqAngle - MBAAngle;
	if (minangle < 0) minangle = 0;
	maxangle = CoverAngle;
}



/**
 * Check whether MBR mbrp is contained in the query circle 
 */
bool MBRContainCircle(RTREEMBR* mbrp, REALTYPE qlat, REALTYPE qlng, REALTYPE qradius) 
{
	if (EarthDistance(mbrp->bound[3], mbrp->bound[0], qlat, qlng) < qradius &&
		EarthDistance(mbrp->bound[3], mbrp->bound[2], qlat, qlng) < qradius &&
		EarthDistance(mbrp->bound[1], mbrp->bound[0], qlat, qlng) < qradius &&
		EarthDistance(mbrp->bound[1], mbrp->bound[2], qlat, qlng) < qradius) return true;
	else return false;
}



/**
 * Check nodep whether overlap with the circle query (qpoint, qradius)
 * If patial overlap, then expand nodep; 
 * if total overlap, then report nodep to be reults; 
 * if no overlap, then prune nodep.
 */

int NodeOverlapCheck(RTREEBRANCH* branchp, REALTYPE qlat, REALTYPE qlng, REALTYPE qradius)
{
	//------check whether can be reported as results.
	if (MBRContainCircle(&(branchp->mbr), qlat, qlng, qradius)) return 2; //total overlap
	REALTYPE maxdist = MaxDist(&(branchp->mbr), qlat, qlng);
	REALTYPE mindist = MinDist(&(branchp->mbr), qlat, qlng);
	REALTYPE maxangle = 0, minangle = 0;
	CalculateMaxMinAngle(&(branchp->angle), &(branchp->mbr), qlat, qlng, minangle, maxangle);
	if (branchp->R.min / 1000.0 <= qradius + branchp->R.max / 1000.0 && 
	    maxangle <= asin(qradius / maxdist) && 
		branchp->R.min / 1000.0 >= maxdist*cos(minangle) - 
		                           sqrt(qradius*qradius - 
								   maxdist*maxdist*sin(maxangle)*sin(maxangle)))
		return 2;

	//-----check whether can be pruned
	//if(Disjoint(&(branchp->mbr), qlat, qlng, qradius)) return 3; //disjoint
	if (mindist > branchp->R.max / 1000.0 + qradius) return 3; //disjoint
	if (mindist>0 && minangle > asin(qradius / mindist)) return 3;
	if (branchp->R.max / 1000.0 <= mindist * cos(maxangle) - 
	                               sqrt(qradius * qradius - 
								        mindist * 
										mindist * 
										sin(minangle) * 
										sin(minangle))) return 3;

	return 1; //parital overlap
}



/** 
 * Calculate the earth distance between two points degree(-90,90) (-180, 180)
 */
REALTYPE EarthDistance(REALTYPE plat, REALTYPE plng, REALTYPE qlat, REALTYPE qlng) 
{
	REALTYPE distpq = 6371 * 2 * asin(sqrt(pow(sin((plat - qlat)*PI / 180 / 2), 2) + 
	                  cos(plat*PI / 180) * 
					  cos(qlat*PI / 180) *
					  pow(sin((plng - qlng)*PI / 180 / 2), 2)));
	return distpq; //kilometers
}



int ObjectOverlapCheck(REALTYPE plat, REALTYPE plng, 
                       REALTYPE pthetas, REALTYPE pthetae, 
					   REALTYPE pR, REALTYPE qlat, REALTYPE qlng, 
					   REALTYPE qradius)
{
	REALTYPE distpq = EarthDistance(plat, plng, qlat, qlng);
	if (distpq <= qradius) return 1;
	REALTYPE BetaS = acos((plat - plat)*sin(pthetas) + (plng - plng)*cos(pthetas));
	REALTYPE BetaE = acos((plat - plat)*sin(pthetae) + (plng - plng)*cos(pthetae));
	if (distpq*sin(BetaS) <= qradius && distpq*sin(BetaE) <= qradius) //
	{
		if (pR >= distpq*cos(BetaS - sqrt(qradius*qradius - 
		                                  distpq*sin(BetaS)*distpq*sin(BetaS)) && 
		    pR >= distpq*cos(BetaE - sqrt(qradius*qradius - 
			                              distpq*sin(BetaE)*distpq*sin(BetaE)))))
			return 1;
	}
	return 0;
}




void ReportResults(RTREENODE* node)
{
	if (node->level>0) //non-leaf node
	{
		for (int i = 0; i<node->count; i++)
		{
			RTREENODE childnode;
			fseek(fp, (node->branch[i].childid - 1)*sizeof(RTREENODE), 0);
			fread(&childnode, sizeof(RTREENODE), 1, fp);
			NodeAccessCnt++;
			ReportResults(&childnode);
		}
	}
	else //leaf node
	{
		for (int i = 0; i<node->count; i++)
		{	
			ResultObjCnt++;
		}
	}
}