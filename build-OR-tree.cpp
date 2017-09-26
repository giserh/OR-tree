// infoadd.cpp : implementation file
//

//#include "stdafx.h"

#include "Rtree.h"
#include <stdio.h>
#include <string>
#include <assert.h>
#include  <math.h>
#include <limits>
#include <stdlib.h>


using namespace std;


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


extern FILE* fp;
extern FILE* fplog;
extern int countdataid;


/**
 * Copy branch b1 from branch b2
 */
void CopyBranch(pRTREEBRANCH b1,pRTREEBRANCH b2)
{
	b1->childid=b2->childid;
	b1->m_data=b2->m_data;
	b1->angle.min = b2->angle.min;
	b1->angle.max = b2->angle.max;
	b1->R.min = b2->R.min;
	b1->R.max = b2->R.max;
	for(int j=0;j<SIDES_NUMB;j++)
	{
		b1->mbr.bound[j]=b2->mbr.bound[j];
	}
}

/**
 * Copy node n1 from node n2
 */
void Copy(pRTREENODE n1,pRTREENODE n2)
{
	n1->taken=n2->taken;
	n1->nodeid=n2->nodeid;
	n1->parentid=n2->parentid;
	n1->level=n2->level;
	n1->count=n2->count;
	for(int i=0;i<MAXCARD;i++)
		CopyBranch(&n1->branch[i],&n2->branch[i]);
}


void InitBranch(pRTREEBRANCH p)
{
	p->childid=0;
	p->m_data=0;
	p->angle.min = 0; p->angle.max = 0;
	p->R.min = 0; p->R.max = 0;
	for(int i=0;i<SIDES_NUMB;i++) p->mbr.bound[i]=0;
}

void InitNode(pRTREENODE p)
{
	p->taken=0;
	p->nodeid=0;
	p->parentid=-1;
	p->count=0;
	p->level=-1;
	for(int j=0;j<MAXCARD;j++)
	{
		InitBranch(&(p->branch[j]));
	}
}

void EmptyNode(pRTREENODE p)
{
	int node=p->nodeid;
	InitNode(p);
	fseek(fp,(node-1)*sizeof(RTREENODE),0);
	fwrite(p,sizeof(RTREENODE),1,fp);
	fflush(fp);
}

/**
 * Initialize a RTREEPARTITION structure.
 */
static void _RTreeInitPart( RTREEPARTITION *p, int maxrects, int minfill)
{
 int i;
 assert(p);

 p->count[0] = 0;
 p->count[1] = 0;
 for(int j=0;j<SIDES_NUMB;j++)
	{
		p->cover[0].bound[j]=0;
		p->cover[1].bound[j]=0;
	}
 p->AlloverScore[0] = p->AlloverScore[1] = (REALTYPE)0;
 p->total = maxrects;
 p->minfill = minfill;
 for (i=0; i<maxrects; i++)
 {
  p->taken[i] = 0;
  p->partition[i] = -1;
 }
}


/**
 * Combine two rectangles, make one that includes both.
 */
 RTREEMBR RTreeCombineRect( RTREEMBR *rc1, RTREEMBR *rc2 )
{
 int i, j;
 RTREEMBR new_rect;

 if (INVALID_RECT(rc1))
  return *rc2;

 if (INVALID_RECT(rc2))
  return *rc1;

 for (i = 0; i < DIMS_NUMB; i++)
 {
  new_rect.bound[i] = MIN(rc1->bound[i], rc2->bound[i]);
  j = i + DIMS_NUMB;
  new_rect.bound[j] = MAX(rc1->bound[j], rc2->bound[j]);
 } 
 return new_rect;
}



 /**
 * Find the smallest rectangle that includes all 
 * rectangles in branches of a node.
 */
 RTREEMBR RTreeNodeCover( RTREENODE *node )
{
 int i, first_time=1;
 RTREEMBR mbr;
 assert(node);

 for(i=0;i<SIDES_NUMB;i++) mbr.bound[i]=0;

 for (i = 0; i < MAXCARD; i++)
 {
  if (node->branch[i].childid || node->branch[i].m_data)
  {
   if (first_time)
   {
    mbr = node->branch[i].mbr;
    first_time = 0;
   }
   else
    mbr = RTreeCombineRect(&mbr, &(node->branch[i].mbr));
  }
 }
 return mbr;
}





/**
 * Calculate the n-dimensional volume of the bounding sphere of a rectangle.
 * The exact volume of the bounding sphere for the given RTREEMBR.
 */
 REALTYPE RTreeRectSphericalVolume( pRTREEMBR mbr )
{
 int i;
 REALTYPE sum_of_squares=0, radius;

 if (INVALID_RECT(mbr))
  return (REALTYPE) 0;
 
 for (i=0; i<DIMS_NUMB; i++) {
  REALTYPE half_extent = (mbr->bound[i+DIMS_NUMB] - mbr->bound[i]) / 2;
  sum_of_squares += half_extent * half_extent;
 }
 radius = (REALTYPE)sqrt(sum_of_squares);
 return (REALTYPE)(pow(radius, DIMS_NUMB) * UnitSphereVolume);
}



/**
 * Load branch buffer with branches from full node plus the extra branch.
 */
static void _RTreeGetBranches(HRTREEROOT root, RTREENODE *node, RTREEBRANCH *br)
{
 int i;

 assert(node && br);
 
 /* load the branch buffer */
 for (i=0; i<MAXKIDS(node); i++)
 {
  /* n should have every entry full */
  assert(node->branch[i].childid || node->branch[i].m_data); 
  CopyBranch(&(root->BranchBuf[i]),&(node->branch[i]));
 }

 CopyBranch(&(root->BranchBuf[MAXKIDS(node)]), br);
 root->BranchCount = MAXKIDS(node) + 1;

 /* calculate mbr containing all in the set */
 root->CoverSplit = root->BranchBuf[0].mbr;

 for (i=1; i<MAXKIDS(node)+1; i++)
 {
  root->CoverSplit = RTreeCombineRect(&root->CoverSplit, &root->BranchBuf[i].mbr);
 }

 root->CoverSplitArea = RTreeRectSphericalVolume(&root->CoverSplit);
 EmptyNode(node);
}

 

 

/**
 * Put a branch in one of the groups.
 */
static void _RTreeClassify(HRTREEROOT root, int i, int group, RTREEPARTITION *p)
{
 assert(p);
 fprintf(fplog, "p->taken[%d]=%d\n", i, p->taken[i]);
 fflush(fplog);

 p->partition[i] = group;
 p->taken[i] = TRUE;

 /*Calculate the allover MBR*/
 if (p->count[group] == 0)
 {
	 p->cover[group] = root->BranchBuf[i].mbr;
	 p->coverAngle[group] = root->BranchBuf[i].angle;
	 p->coverR[group] = root->BranchBuf[i].R;
 }
 else
 {
	  p->cover[group] = RTreeCombineRect(&root->BranchBuf[i].mbr, &p->cover[group]);
	  p->coverAngle[group] = CoverTwoAngles(root->BranchBuf[i].angle, p->coverAngle[group]);
	  p->coverR[group].min = MIN(root->BranchBuf[i].R.min, p->coverR[group].min);
	  p->coverR[group].max = MAX(root->BranchBuf[i].R.max, p->coverR[group].max);
 }

 RTREEBRANCH tmp_branch_group;
 InitBranch(&tmp_branch_group); 
 tmp_branch_group.angle.min = p->coverAngle[0].min; 
 tmp_branch_group.angle.max = p->coverAngle[0].max;
 tmp_branch_group.R.min = p->coverR[0].min; tmp_branch_group.R.max = p->coverR[0].max;
 for(int j=0;j<SIDES_NUMB;j++)
 {
	 tmp_branch_group.mbr.bound[j] = p->cover[0].bound[j];
 }
 p->AlloverScore[group] = CalculateAlloverScore(&tmp_branch_group);
 p->count[group]++;
}



/**
 * Pick two rects from set to be the first elements of the two groups.
 * Pick the two that waste the most area if covered by a single rectangle.
 */
static void _RTreePickSeeds(HRTREEROOT root, RTREEPARTITION *p)
{
 int i, j, seed0=0, seed1=0;
 REALTYPE worst, waste;


 
 //worst = 0 - DBL_MAX;
 worst = 0 - INFINITY;

 //Calculate the branch pair (A, B) with the maximum waste{area(A+B)-area(A)-area(B)}
 for (i=0; i<p->total-1; i++)
 {
  for (j=i+1; j<p->total; j++)
  {
 //  RTREEMBR one_rect;
  // one_rect = RTreeCombineRect(&root->BranchBuf[i].mbr, &root->BranchBuf[j].mbr);
  // wasteMBRp = RTreeRectSphericalVolume(&one_rect) - area[i] - area[j];

   //-----------angle------------//
   /**
   ONED_RANGE one_angle = CoverTwoAngles(root->BranchBuf[i].angle, root->BranchBuf[j].angle);
   REALTYPE angleCover = ( acos(sin(one_angle.min)*sin(one_angle.max) + 
                           cos(one_angle.min)*cos(one_angle.max)) ) * 
						   MAX(root->BranchBuf[i].R.max, root->BranchBuf[j].R.max);
   wasteMBA = pow(angleCover - angle[i] - angle[j], 2);
   */
   
   //----------R---------------//
   /**
    REALTYPE RangeRCover = MAX(root->BranchBuf[i].R.max, root->BranchBuf[j].R.max) 
	                       - MIN(root->BranchBuf[i].R.min, root->BranchBuf[j].R.min);
	wasteRangeR = pow(RangeRCover - rangeR[i] - rangeR[j], 2);
	waste = wasteMBRp + wasteMBA +wasteRangeR;
	*/

	waste = CalculateAlloverScoreWaste(&(root->BranchBuf[i]), &(root->BranchBuf[j]));

   if (waste > worst)
   {
    worst = waste;
    seed0 = i;
    seed1 = j;
   }
  }
 }
 _RTreeClassify(root, seed0, 0, p);
 _RTreeClassify(root, seed1, 1, p);
}



/**
 * Method #0 for choosing a partition:
 * As the seeds for the two groups, pick the two rects that would waste the
 * most area if covered by a single rectangle, i.e. evidently the worst pair
 * to have in the same group.
 * Of the remaining, one at a time is chosen to be put in one of the two groups.
 * The one chosen is the one with the greatest difference in area expansion
 * depending on which group - the mbr most strongly attracted to one group
 * and repelled from the other.
 * If one group gets too full (more would force other group to violate min
 * fill requirement) then other group gets the rest.
 * These last are the ones that can go in either group most easily.
 */
static void _RTreeMethodZero(HRTREEROOT root, RTREEPARTITION *p, int minfill )
{
 int i;
 REALTYPE biggestDiff;
 int group, chosen=0, betterGroup=0;
 assert(p);
	RTREEBRANCH pbranch_0, pbranch_1;
	InitBranch(&pbranch_0);
	InitBranch(&pbranch_1);
	pbranch_0.angle.min = p->coverAngle[0].min; pbranch_0.angle.max = p->coverAngle[0].max;
	pbranch_1.angle.min = p->coverAngle[1].min; pbranch_1.angle.max = p->coverAngle[1].max;
	for(int mbrj=0;mbrj<SIDES_NUMB;mbrj++)
	{
		pbranch_0.mbr.bound[mbrj]=p->cover[0].bound[mbrj];
		pbranch_1.mbr.bound[mbrj]=p->cover[1].bound[mbrj];
	}
	pbranch_0.R.min = p->coverR[0].min; pbranch_0.R.max = p->coverR[0].max;
	pbranch_1.R.min = p->coverR[1].min; pbranch_1.R.max = p->coverR[1].max;

 _RTreeInitPart(p, root->BranchCount, minfill);
 _RTreePickSeeds(root, p);

 while (p->count[0] + p->count[1] < p->total && 
  p->count[0] < p->total - p->minfill && 
  p->count[1] < p->total - p->minfill)
 {
  biggestDiff = (REALTYPE)-1;
  for (i=0; i<p->total; i++)
  {
   if (!p->taken[i])
   {
    REALTYPE growth0, growth1, diff;
	growth0 = CalculateAlloverScoreWaste(&(root->BranchBuf[i]), &pbranch_0);
	growth1 = CalculateAlloverScoreWaste(&(root->BranchBuf[i]), &pbranch_1);
    diff = growth1 - growth0;
    if (diff >= 0)
     group = 0;
    else
    {
     group = 1;
     diff = -diff;
    }
    if (diff > biggestDiff)
    {
     biggestDiff = diff;
     chosen = i;
     betterGroup = group;
    }
    else if (diff==biggestDiff && p->count[group]<p->count[betterGroup])
    { /**
       * Consider two factors: 1) area waste; 
       * 2) the number of branches of the two groups (try to be balance).
	   */
     chosen = i;
     betterGroup = group;
    }
   }//end if taken
  }//end for
  _RTreeClassify(root, chosen, betterGroup, p);
 }//end while

 /* if one group too full, put remaining rects in the other */
 if (p->count[0] + p->count[1] < p->total)
 {
  if (p->count[0] >= p->total - p->minfill)
   group = 1;
  else
   group = 0;
  
  for (i=0; i<p->total; i++)
  {
   if (!p->taken[i])
    _RTreeClassify(root, i, group, p);
  }
 }
 
 fprintf(fplog, "assert(%d +%d  == %d)\n", p->count[0], p->count[1], p->total);
 fflush(fplog);
 
 assert(p->count[0] + p->count[1] == p->total);
 assert(p->count[0] >= p->minfill && p->count[1] >= p->minfill);
 //assert(1 + 1 == 2);
 //assert(1 + 1 == 3);
}



/**
 * Copy branches from the buffer into two nodes according to the partition.
 */
static void _RTreeLoadNodes(HRTREEROOT root, RTREENODE *n, RTREENODE *q, RTREEPARTITION *p)
{
 int i;
 assert(n && q && p);

 for (i=0; i<p->total; i++)
 {
  assert(p->partition[i] == 0 || p->partition[i] == 1);
  if (p->partition[i] == 0)
   AddBranch(root, &root->BranchBuf[i], n, NULL);
  else if (p->partition[i] == 1)
   AddBranch(root, &root->BranchBuf[i], q, NULL);
 }
}




/**
 * Split a node.
 * Divides the nodes branches and the extra one between two nodes.
 * Old node is one of the new ones, and one really new one is created.
 * Tries more than one method for choosing a partition, uses best result.
 */
void SplitNode(HRTREEROOT root, RTREENODE *node, RTREEBRANCH *br, RTREENODE **new_node)
{
 RTREEPARTITION *p;
 int level,oldrootid;
 assert(node && br);
 
 /* load all the branches into a buffer, initialize old node */
 level = node->level;
 oldrootid=node->parentid;
 _RTreeGetBranches(root, node, br);


 /* find partition */
 p = &(root->Partitions[0]);

 /* Note: can&apos;t use MINFILL(n) below since node was cleared by GetBranches() */
 _RTreeMethodZero(root, p, (level>0 ? MINNODEFILL : MINLEAFFILL));


 /* put branches from buffer into 2 nodes according to chosen partition */
 int i,ftaken=2;
 long filelen;
 node->parentid=oldrootid;//////
 fseek(fp,0L,SEEK_END);
 filelen=ftell(fp);
 for(i=0;i<(int)filelen;i=i+sizeof(RTREENODE))
 {
	 fseek(fp,i,0);
	 fread(&ftaken,sizeof(int),1,fp);//
	 if(ftaken==0) break;
 }
 node->nodeid=i/sizeof(RTREENODE)+1;
 node->taken=1;
 node->level=level;
	 
 *new_node = (RTREENODE*)malloc(sizeof(RTREENODE));
 InitNode(*new_node);
 (*new_node)->level = level;
 (*new_node)->parentid=oldrootid;

 for(;i<(int)filelen;i=i+sizeof(RTREENODE))
 {
	 fseek(fp,i,0);
	 fread(&ftaken,sizeof(int),1,fp);//
	 if(ftaken==0 && (i/sizeof(RTREENODE)+1)!=(unsigned int)(node->nodeid)) break;
 }
 (*new_node)->nodeid=i/sizeof(RTREENODE)+1;
 (*new_node)->taken=1;


 _RTreeLoadNodes(root, node, *new_node, p);

 RTREENODE child;
 for(i=0;i<node->count;i++)
 {
	 if(node->branch[i].childid)
	 {
		 fseek(fp,(node->branch[i].childid-1)*sizeof(RTREENODE),0);
	     fread(&child,sizeof(RTREENODE),1,fp);
	     child.parentid=node->nodeid;
	     fseek(fp,(child.nodeid-1)*sizeof(RTREENODE),0);
	     fwrite(&child,sizeof(RTREENODE),1,fp);
	     fflush(fp);
	 }
 }

  for(i=0;i<(*new_node)->count;i++)
 {
	 if((*new_node)->branch[i].childid)
	 {
		 fseek(fp,((*new_node)->branch[i].childid-1)*sizeof(RTREENODE),0);
	     fread(&child,sizeof(RTREENODE),1,fp);
	     child.parentid=(*new_node)->nodeid;
	     fseek(fp,(child.nodeid-1)*sizeof(RTREENODE),0);
	     fwrite(&child,sizeof(RTREENODE),1,fp);
	     fflush(fp);
	 }
 }
 assert(node->count+(*new_node)->count == p->total);
}


/**
 * Add a branch to a node.  Split the node if necessary.
 * Returns 0 if node not split.  Old node updated.
 * Returns 1 if node split, sets *new_node to address of new node.
 * Old node updated, becomes one of two.
 */
 int AddBranch(HRTREEROOT root, RTREEBRANCH *br, RTREENODE *node, RTREENODE **new_node)
{
 int i;
 assert(br && node);
 
 if (node->count < MAXKIDS(node))  /* split won&apos;t be necessary */
 {
  for (i = 0; i < MAXKIDS(node); i++)  /* find empty branch */
  {
   if (node->branch[i].childid == 0 && node->branch[i].m_data==0)
   {
    CopyBranch(&(node->branch[i]),br);
    node->count++;  
	fseek(fp,(node->nodeid-1)*sizeof(RTREENODE),0);
	fwrite(node,sizeof(RTREENODE),1,fp);
	fflush(fp);
    break;
   }
  }

  return 0;
 }
 
 assert(new_node);
 SplitNode(root, node, br, new_node);
 return 1;
}

 REALTYPE CalculateAlloverScoreWaste(RTREEBRANCH *branch1, RTREEBRANCH *branch2)
 {
	  RTREEMBR cover_rect = RTreeCombineRect(&(branch1->mbr), &(branch2->mbr));
	  REALTYPE AreaWaste = RTreeRectSphericalVolume(&cover_rect) - 
	                       RTreeRectSphericalVolume(&branch1->mbr) - 
						   RTreeRectSphericalVolume(&branch2->mbr);
	  ONED_RANGE cover_angle = CoverTwoAngles(branch1->angle, branch2->angle);
	  REALTYPE CoverAngleWaste = pow(CalculateAngleClockwise(cover_angle.min, 
	                                                         cover_angle.max)*
															 MAX(branch1->R.max, 
															     branch2->R.max), 2);
	  CoverAngleWaste = CoverAngleWaste - pow(CalculateAngleClockwise(branch1->angle.min, 
	                                                                  branch1->angle.max) * 
																	  branch1->R.max, 2);
	  CoverAngleWaste = CoverAngleWaste - pow(CalculateAngleClockwise(branch2->angle.min, 
	                                                                  branch2->angle.max) * 
																	  branch2->R.max, 2);
	  REALTYPE RWaste = pow(MAX(branch1->R.max, branch2->R.max) - 
	                        MIN(branch1->R.min,branch2->R.min), 2);
	  RWaste = RWaste - pow(branch1->R.max - branch1->R.min, 2);
	  RWaste = RWaste - pow(branch2->R.max - branch2->R.min, 2);
	  return AreaWaste+CoverAngleWaste+RWaste;
 }

  REALTYPE CalculateAlloverScore(RTREEBRANCH *branch)
 {
	 REALTYPE AreaMBRp = RTreeRectSphericalVolume(&(branch->mbr));
	 REALTYPE AngleDiff = pow(acos(sin(branch->R.min)*sin(branch->R.max) + 
	                          cos(branch->R.min)*cos(branch->R.max))*branch->R.max, 2);
	  REALTYPE RDiff = pow(branch->R.max-branch->R.min, 2);
	  return AreaMBRp+AngleDiff+RDiff;
 }


/**
 * Pick a branch.  Pick the one that will need the smallest increase
 * in area to accomodate the new rectangle.  This will result in the
 * least total area for the covering rectangles in the current node.
 * In case of a tie, pick the one which was smaller before, to get
 * the best resolution when searching.
 */
 int RTreePickBranch( RTREEBRANCH *branch, RTREENODE *node)
{
 //RTREEMBR *r;
 int i, first_time = 1;
 REALTYPE increase, bestIncr=(REALTYPE)-1, alloverscore, bestScore=0;
 int best=0;
 //RTREEMBR tmp_rect;
 assert(branch && node);

 for (i=0; i<MAXKIDS(node); i++)
 {
  if (node->branch[i].childid)
  {
	  alloverscore = CalculateAlloverScore(&(node->branch[i]));
	   increase = CalculateAlloverScoreWaste(branch, &(node->branch[i]));
	   if (increase < bestIncr || first_time)
	   {
		best = i;
		bestScore = alloverscore;
		bestIncr = increase;
		first_time = 0;
	   }
	   else if (increase == bestIncr && alloverscore < bestScore)
	   {
		best = i;
		bestScore = alloverscore;
		bestIncr = increase;
	   }
  }
 }
 return best;
}


/**
 * Inserts a new data rectangle into the index structure.
 * Recursively descends tree, propagates splits back up.
 * Returns 0 if node was not split.  Old node updated.
 * If node was split, returns 1 and sets the pointer pointed to by
 * new_node to point to the new node.  Old node updated to become one of two.
 * The level argument specifies the number of steps up from the leaf
 * level to insert; e.g. a data rectangle goes in at level = 0.
 */
 int _RTreeInsertRect(HRTREEROOT root, RTREEBRANCH *pbranch,  
                      RTREENODE *node, RTREENODE **new_node, int level)
{
 int i;
 RTREEBRANCH b;
 InitBranch(&b);
 RTREENODE *n2;

 //assert(pbranch->mbr && node && new_node);
 assert(level >= 0 && level <= node->level);//because node begin from the root
 fprintf(fplog, "level=%d\n", level);
 fflush(fplog);

 /* Still above level for insertion, go down tree recursively */
 if (node->level > level)
 {
  i = RTreePickBranch(pbranch, node);
  RTREENODE child;
  InitNode(&child);
  fseek(fp,(node->branch[i].childid-1)*sizeof(RTREENODE),0);
  fread(&child,sizeof(RTREENODE),1,fp);

  if (!_RTreeInsertRect(root, pbranch, &child, &n2, level))
  {
   /* child was not split */
   node->branch[i].mbr = RTreeCombineRect(&(pbranch->mbr), &(node->branch[i].mbr));
   //-----------angle------------//
   node->branch[i].angle = CoverTwoAngles(node->branch[i].angle, pbranch->angle);
   //----------R---------------//
   node->branch[i].R.min = MIN(pbranch->R.min, node->branch[i].R.min);
   node->branch[i].R.max = MAX(pbranch->R.max, node->branch[i].R.max);


   if(node->branch[i].childid==0 && node->branch[i].m_data==0)
   {
	   node->branch[i].childid=pbranch->childid;
       node->branch[i].m_data=pbranch->m_data;
   }

   fseek(fp,(node->nodeid-1)*sizeof(RTREENODE),0);
   fwrite(node,sizeof(RTREENODE),1,fp);
   fflush(fp);
   return 0;
  }
  
  /* child was split */
  node->branch[i].mbr = RTreeNodeCover(&child);
  node->branch[i].angle = MinimumBoundingAngle(&child);
  node->branch[i].R = MinimumBoundingR(&child);
  node->branch[i].childid=child.nodeid;

  b.childid = n2->nodeid;
  b.angle = MinimumBoundingAngle(n2);
  b.R = MinimumBoundingR(n2);
  b.mbr = RTreeNodeCover(n2);
  
  return AddBranch(root, &b, node, new_node);
 } 
 else if (node->level == level) 
 {/* Have reached level for insertion. Add mbr, split if necessary */
  CopyBranch(&b,pbranch);

/*#pragma warning(push) // C4312 
#pragma warning( disable : 4312 )
  b.child = ( RTREENODE *) tid;
#pragma warning(pop)*/
  
  /* child field of leaves contains tid of data record */

  return AddBranch(root, &b, node, new_node);
 }
 
 /* Not supposed to happen */
 assert (FALSE);
 return 0;
}





void FindRoot(RTREENODE* rootp)
{
	fseek(fp,0L,SEEK_END);
	long leng=ftell(fp);
	int rootnodeid,fuqiid=1,ftaken;
	for(int i=0;i<(int)leng;i=i+sizeof(RTREENODE))
	  {
		  fseek(fp,i,0);
		  fread(&ftaken,sizeof(ftaken),1,fp);
		  fread(&rootnodeid,sizeof(rootnodeid),1,fp);
		  fread(&fuqiid,sizeof(fuqiid),1,fp);
		  if(fuqiid==0 && ftaken!=0) 
		  {  
			  fseek(fp,(rootnodeid-1)*sizeof(RTREENODE),0);
			  fread(rootp,sizeof(RTREENODE),1,fp);
			  break;
		  }//end if
	  }//end for
}


/**
 * The return value is in [0, 2PI]
 */
REALTYPE CalculateAngleClockwise(REALTYPE thetaS, REALTYPE thetaE) 
{
	//angle is in [0, PI) x1*y1+x2*y2
	REALTYPE angle = acos(sin(thetaS)*sin(thetaE) + cos(thetaS)*cos(thetaE));  
	/**
	 * Cross multipleply x1*y2+x2*y1 = |a|*|b|*sin<a,b> 
	 * right hand rule, counter clockwise 
	 */
	REALTYPE cross = sin(thetaS)*cos(thetaE) + cos(thetaS)*sin(thetaE);  
	if(cross>0) return 2*3.1415926-angle; 
	else return angle;
}

ONED_RANGE CoverTwoAngles(ONED_RANGE angle1, ONED_RANGE angle2)
{
	ONED_RANGE CoverAngle = {0,0};
	REALTYPE AngleS1E2 = CalculateAngleClockwise(angle1.min, angle2.max);
   REALTYPE AngleS2E1 = CalculateAngleClockwise(angle2.min, angle1.max);
   if(AngleS1E2 > AngleS2E1) 
   {
	   CoverAngle.min = angle1.min;
	   CoverAngle.max = angle2.max;
   }
   else
   {
	   CoverAngle.min = angle2.min;
	   CoverAngle.max = angle1.max;
   }
   return CoverAngle;
}

ONED_RANGE MinimumBoundingAngle(pRTREENODE node)
{
	ONED_RANGE angle = {0,0};
	REALTYPE MinDeltaAngle_FindS = INFINITY; //Defined in limit
	REALTYPE MaxDeltaAngle_FindE = 0;

	angle.min = node->branch[0].angle.min;
	for(int i=0; i<node->count; i++)
	{
		angle.max = node->branch[i].angle.max;
		MaxDeltaAngle_FindE = CalculateAngleClockwise(node->branch[i].angle.min, angle.max); 
		for(int j=0; j<node->count; j++)
		{
			REALTYPE tempangle = CalculateAngleClockwise(node->branch[i].angle.min, 
			                                             node->branch[j].angle.max); 
			if(tempangle > MaxDeltaAngle_FindE) 
			{
				MaxDeltaAngle_FindE = tempangle;
				angle.max = node->branch[j].angle.max;
			}
		}
		if(MinDeltaAngle_FindS < MaxDeltaAngle_FindE)
		{
			MinDeltaAngle_FindS = MaxDeltaAngle_FindE;
			angle.min = node->branch[i].angle.min;
		}
	}
	return angle;
}

ONED_RANGE MinimumBoundingR(pRTREENODE node)
{
	ONED_RANGE R = {INFINITY, 0 };
	for(int i=0; i<node->count; i++)
	{
		if(R.min > node->branch[i].R.min)  R.min = node->branch[i].R.min;
		if(R.max < node->branch[i].R.max) R.max=node->branch[i].R.max;
	}
	return R;
}


void infoadd() 
{
	// TODO: Add extra validation here
    //UpdateData(TRUE);

	//-----------Get data from the database MySQL-------------//
	/*mysql::MySQL_Driver *driver;
	Connection *conn;
	Statement *state;
	ResultSet *result;*
	driver = sql::mysql::get_mysql_driver_instance(); //initionalize driver
	conn = driver->connect("128.125.163.130:3306", "root", "pc4passlr"); //create connection
	state = conn->createStatement();
	state->execute("use PBS");
	result = state->executeQuery("select Plat, Plng, theta_x, R, alpha from metadata_tbl");*/

	countdataid=0; //initionalize zero

	/*
	char videoname[100] ={0};
	int fovnum=0;
	double lat =0.0;
	double lng = 0.0;
	double theta_x = 0.0;
	double R = 0.0;
	int alpha = 0;
	char timestamp[20] = {0};*///for clean metadata: metadata.txt
	

	char videoname[100] = { 0 };
	int fovnum = 0;
	REALTYPE lat = 0.0;
	REALTYPE lng = 0.0;
	REALTYPE theta_x = 0.0;
	REALTYPE R = 0.0;
	int alpha = 0;
	char timestamp1[20] = { 0 }, timestamp2[20] = { 0 };

	int fileID;
	REALTYPE Px, Py, prevX, prevY, speed;
	int prevDir;



	

	FILE *fpdataset = fopen("SlowSpeed_Dataset.txt", "r+");
	if(fopen==NULL) 
	{
		printf("failed open metadata.txt file\n");
		return;
	}
	else
	{
		while(!feof(fpdataset))
		{
			fscanf(fpdataset, "%s %d %d %f %f %f %f %f %f %f %f %d %f %d %s %s\n", 
			       videoname, &fileID, &fovnum, &lat, &lng, &Px, &Py, &prevX, &prevY, 
				   &speed, &theta_x, &prevDir, &R, &alpha, timestamp1, timestamp2);

			fprintf(fplog, "%s %d %d %f %f %f %f %f %f %f %f %d %f %d %s %s\n", 
			        videoname, fileID, fovnum, lat, lng, Px, Py, prevX, prevY, 
					speed, theta_x, prevDir, R, alpha, timestamp1, timestamp2);
			fflush(fplog);

			  countdataid++;
			  RTREEBRANCH branch;
			  InitBranch(&branch);
			  branch.m_data=countdataid;
			  /*branch.angle.min = fmod(result->getDouble("theta_x") - 
			                       result->getDouble("alpha")/2 + 360, 360);
			  //branch.angle.max = fmod(result->getDouble("theta_x") + 
			                       result->getDouble("alpha")/2+ 360, 360);
			  //branch.R.min =  result->getDouble("R");
			  //branch.R.max = result->getDouble("R");*/
			  
			  branch.angle.min = (fmod(theta_x - alpha/2 + 360, 360))*3.1415926/180;
			  branch.angle.max = (fmod(theta_x + alpha/2 + 360, 360))*3.1415926/180;
			  /*if(CalculateAngleClockwise(branch.angle.min, branch.angle.max)!=alpha)
			  {
				   swap(branch.angle.min, branch.angle.max);
			  }*/

			  branch.R.min =  R;
			  branch.R.max = R;
			  for(int j=0;j<=DIMS_NUMB;j=j+DIMS_NUMB)
			  {
				  //branch.mbr.bound[j]=result->getDouble("Plng"); //xmin, xmax
				  //branch.mbr.bound[j+1]=result->getDouble("Plat"); //ymin, ymax
				   branch.mbr.bound[j]=lng; //xmin, xmax
				  branch.mbr.bound[j+1]=lat; //ymin, ymax
			  }

			  //int i,j;
			  RTREEROOT root; //root为根结点
			  root.root_node=(RTREENODE*)malloc(sizeof(RTREENODE));
			  InitNode(root.root_node);
		  	  
			  fp=fopen("IR2tree.index","rb+");
			  int flagroot=0;
			  fseek(fp,0L,SEEK_END);
			  long leng=ftell(fp);
			  if(leng==0) 
			  {   
				  flagroot=1;
				  root.root_node->taken=1;
				  root.root_node->nodeid=1; 
				  root.root_node->parentid=0;
				  root.root_node->level=0;
				  root.root_node->count=1;
				  CopyBranch(&(root.root_node->branch[0]), &branch);
				 fseek(fp,0L,0); 
				 fwrite(root.root_node,sizeof(RTREENODE),1,fp);
				 fflush(fp);
			  }//end if leng==0
			 else
			  {
				  FindRoot(root.root_node);
				  
				  RTREENODE *newroot;
				  RTREENODE *newnode;
				  RTREEBRANCH b;
				  InitBranch(&b);

				 /* root split */
				 if (_RTreeInsertRect(&root, &branch, root.root_node, &newnode, 0))  
				 { //The returned two nodes have been written in the file.
					 int ftaken;
					 /* grow a new root, & tree taller */
					 newroot=(RTREENODE*)malloc(sizeof(RTREENODE)); 
					 InitNode(newroot);
					 newroot->taken=1;
					 newroot->parentid=0; 
					 newroot->level = root.root_node->level + 1;
         
					 fseek(fp,0L,SEEK_END);
					 leng=ftell(fp);
					 int i;
					 for(i=0;i<(int)leng;i=i+sizeof(RTREENODE))
					 {
						 fseek(fp,i,0);
						 fread(&ftaken,sizeof(int),1,fp);
						 if(ftaken==0) break;
					 }
					 newroot->nodeid=i/sizeof(RTREENODE)+1;
					 b.mbr = RTreeNodeCover(root.root_node);
					 b.childid = root.root_node->nodeid;
					 b.angle = MinimumBoundingAngle(root.root_node);
					 b.R = MinimumBoundingR(root.root_node);
					 fseek(fp,0L,SEEK_END);
					 AddBranch(&root, &b, newroot, NULL);
					 fseek(fp,0L,SEEK_END);

					 b.mbr = RTreeNodeCover(newnode);
					 b.childid = newnode->nodeid;
					 b.angle = MinimumBoundingAngle(newnode);
					 b.R = MinimumBoundingR(newnode);
					 AddBranch(&root, &b, newroot, NULL);
					 fseek(fp,0L,SEEK_END);

					 root.root_node->parentid=newroot->nodeid;
					 newnode->parentid=newroot->nodeid;
					 fseek(fp,((root.root_node->nodeid)-1)*sizeof(RTREENODE),0);
					 fwrite(root.root_node,sizeof(RTREENODE),1,fp);
					 fflush(fp);
					 fseek(fp,0L,SEEK_END);

					 fseek(fp,((newnode->nodeid)-1)*sizeof(RTREENODE),0);
					 fwrite(newnode,sizeof(RTREENODE),1,fp);
					 fflush(fp);
					 fseek(fp,0L,SEEK_END);

					 Copy(root.root_node , newroot);
					 delete newroot;
				 }//END IF

			 }//end else if leng==0
			 fclose(fp);
			 delete root.root_node;
		} //end while result->next()
	}//end else fopen metadata.txt 
	fclose(fpdataset);
}



