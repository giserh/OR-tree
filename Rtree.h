#ifndef  IR2TREE_H_INCLUDED
#define  IR2TREE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#ifndef  TRUE
#define  TRUE  1
#endif

#ifndef  FALSE
#define  FALSE  0
#endif

#define K 3
#define  METHODS  1
#define  PAGE_SIZE    4096 //4KB
#define  DIMS_NUMB    2    // number of dimensions
#define  SIDES_NUMB   2*DIMS_NUMB //min max
#define  PI 3.1415926

/* max branching factor of a node */
#define MAXCARD (int)((PAGE_SIZE-(5*sizeof(int))) / sizeof(RTREEBRANCH))

//#ifndef MAXCARD
//  #define MAXCARD 2 
//#endif

#ifndef INVALD_RECT
  #define INVALID_RECT(x) ((x)->bound[0] > (x)->bound[DIMS_NUMB])
#endif
#ifndef UnitSphereVolume
  #define UnitSphereVolume UnitSphereVolumes[DIMS_NUMB]
#endif

/**
 * Precomputed volumes of the unit spheres for the first few dimensions 
 */
const double UnitSphereVolumes[] = {
 0.000000,  /* dimension   0 */
 2.000000,  /* dimension   1 */
 3.141593,  /* dimension   2 */
 4.188790,  /* dimension   3 */
 4.934802,  /* dimension   4 */
 5.263789,  /* dimension   5 */
 5.167713,  /* dimension   6 */
 4.724766,  /* dimension   7 */
 4.058712,  /* dimension   8 */
 3.298509,  /* dimension   9 */
 2.550164,  /* dimension  10 */
 1.884104,  /* dimension  11 */
 1.335263,  /* dimension  12 */
 0.910629,  /* dimension  13 */
 0.599265,  /* dimension  14 */
 0.381443,  /* dimension  15 */
 0.235331,  /* dimension  16 */
 0.140981,  /* dimension  17 */
 0.082146,  /* dimension  18 */
 0.046622,  /* dimension  19 */
 0.025807,  /* dimension  20 */
};


typedef float REALTYPE;



/*
* If passed to a tree search, this callback function will be called
* with the ID of each data mbr that overlaps the search mbr
* plus whatever user specific pointer was passed to the search.
* It can terminate the search early by returning 0 in which case
* the search will return the number of hits found up to that point.
*/


typedef struct _RTREEMBR //MBR
{
 REALTYPE bound[SIDES_NUMB]; /* xmin,ymin,...,xmax,ymax,... */
}RTREEMBR,*pRTREEMBR;


/* One dimensional range type */
typedef struct _ONED_RANGE 
{
	REALTYPE min;
	REALTYPE max;
}ONED_RANGE, *pONED_RANGE;


/* Definition of a tree branch */
typedef struct _RTREEBRANCH
{
	RTREEMBR mbr;
	ONED_RANGE angle;//the angle with respect to the north. 
	ONED_RANGE R;    //The maximum visiable distance of FOV.
    int childid;     // Child ID of a non-leaf node 
	int m_data;      // Id of leaf node.
}RTREEBRANCH,*pRTREEBRANCH;

//objnum ---> the total number of objects contained in the subtree of the node.

//childid ---> (for non-leaf node) pointer of child nodes; 
//             (for leaf node) the length of vectors of all the child objects of the node.

//m_data ---> (for non-leaf node) pointer of child objects; 
//            (for leaf node) 0.



/* Definition of an R-tree node */
typedef struct _RTREENODE
{
 int taken;   //Mark whehther the node in the file is used or not: 1 (taken), 0 (not taken)
 int nodeid;  //Node ID
 int parentid;//Parent Node ID
 int count;   //The number branches
 int level;   /* 0 is leaf, others positive */
 RTREEBRANCH  branch[MAXCARD];
}RTREENODE,*pRTREENODE; 


/**
 * Used for node spit. 
 */
typedef struct _RTREEPARTITION
{
 int   partition[MAXCARD+1]; 
 int   total; //Number of branches.
 int   minfill; //Minimum filling area
 int   taken[MAXCARD+1]; //Mark whether each branch is grouped or not.
 int   count[2]; //The number branches in each two groups.
 RTREEMBR cover[2]; //The allover MBRs of the two groups.
 ONED_RANGE coverAngle[2];
 ONED_RANGE coverR[2];
 REALTYPE AlloverScore[2]; //The allover scores of the two groups.
} RTREEPARTITION;



typedef struct _RTREEROOT
{
 RTREENODE*  root_node;
 RTREEBRANCH  BranchBuf[MAXCARD+1];
 int    BranchCount;
 RTREEMBR  CoverSplit;
 REALTYPE  CoverSplitArea;
 RTREEPARTITION Partitions[METHODS];
} RTREEROOT, * HRTREEROOT;



//---------------------for query processing----------------------//
typedef struct _IR2TREEPRINODE
{
	int nodeid;
	REALTYPE distance;
	struct _IR2TREEPRINODE *next;
} IR2TREEPRINODE,*IR2TREEPRINODEp;

typedef struct
{
	IR2TREEPRINODEp front;
	IR2TREEPRINODEp rear;
	IR2TREEPRINODEp minpoint;
} IR2TREEPRIQUEUE,*IR2TREEPRIQUEUEp;


typedef struct _IR2TREELISTNODE
{
	int nodeid;
	struct _IR2TREELISTNODE *next;
}IR2TREELISTNODE,*IR2TREELISTNODEp;



//-------------------------------dataset----------------------------//
/*typedef struct
{
	int FOVid; 
	REALTYPE lat;
	REALTYPE lon;
	ONED_RANGE angle;
	REALTYPE R;
}DATASET,*DATASETp;*/





#ifndef MIN
 #define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
 #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef NODECARD
  #define NODECARD MAXCARD
#endif
#ifndef LEAFCARD
  #define LEAFCARD MAXCARD
#endif

/* balance criteria for node splitting */
/* NOTE: can be changed if needed. */
#ifndef MINNODEFILL
  #define MINNODEFILL (NODECARD / 2)
#endif

#ifndef MINLEAFFILL
  #define MINLEAFFILL (LEAFCARD / 2)
#endif

#ifndef MAXKIDS
  #define MAXKIDS(n) ((n)->level > 0 ? NODECARD : LEAFCARD)
#endif

#ifndef MINFILL
  #define MINFILL(n) ((n)->level > 0 ? MINNODEFILL : MINLEAFFILL)
#endif


/**
* Calculate the 2-dimensional area of a rectangle
*/
REALTYPE RTreeRectArea( RTREEMBR *mbr );

/**
* Calculate the n-dimensional volume of a rectangle
*/
REALTYPE RTreeRectVolume( RTREEMBR *mbr );


/**
* Calculate the n-dimensional volume of the bounding sphere of a rectangle
* The exact volume of the bounding sphere for the given RTREEMBR.
*/
REALTYPE RTreeRectSphericalVolume( RTREEMBR *mbr );



/**
* Combine two rectangles, make one that includes both.
*/
RTREEMBR RTreeCombineRect( RTREEMBR *rc1, RTREEMBR *rc2 );


/**
* Find the smallest rectangle that includes all rectangles in branches of a node.
*/
RTREEMBR RTreeNodeCover( pRTREENODE node );


/**
* Pick a branch.  Pick the one that will need the smallest increase
* in area to accomodate the new rectangle.  This will result in the
* least total area for the covering rectangles in the current node.
* In case of a tie, pick the one which was smaller before, to get
* the best resolution when searching.
*/
int RTreePickBranch( RTREEMBR *mbr, pRTREENODE node);


/**
* Add a branch to a node.  Split the node if necessary.
* Returns 0 if node not split.  Old node updated.
* Returns 1 if node split, sets *new_node to address of new node.
* Old node updated, becomes one of two.
*/
int AddBranch(HRTREEROOT root, RTREEBRANCH *br, 
              pRTREENODE node, pRTREENODE *new_node);


/** 
* Insert a data rectangle into an index structure.
* RTreeInsertRect provides for splitting the root;
* returns 1 if root was split, 0 if it was not.
* The level argument specifies the number of steps up from the leaf
* level to insert; e.g. a data rectangle goes in at level = 0.
* _RTreeInsertRect does the recursion.
*/
int RTreeInsert(HRTREEROOT root, RTREEMBR *data_mbr, void* data_id, int level);


void FindRoot(RTREENODE* rootp);

void InitBranch(pRTREEBRANCH p);

void InitNode(pRTREENODE p);

void CopyBranch(pRTREEBRANCH b1,pRTREEBRANCH b2);

int _RTreeInsertRect(HRTREEROOT root, RTREEBRANCH *pbranch,  
                     RTREENODE *node, RTREENODE **new_node, int level);

REALTYPE CalculateAngleClockwise(REALTYPE thetaS, REALTYPE thetaE);

ONED_RANGE MinimumBoundingAngle(pRTREENODE node);

ONED_RANGE MinimumBoundingR(pRTREENODE node);

void Copy(pRTREENODE n1,pRTREENODE n2);

void FindRoot(RTREENODE* rootp);


REALTYPE CalculateAlloverScoreWaste(RTREEBRANCH *branch1, RTREEBRANCH *branch2);

REALTYPE CalculateAlloverScore(RTREEBRANCH *branch);

ONED_RANGE CoverTwoAngles(ONED_RANGE angle1, ONED_RANGE angle2);

ONED_RANGE MinimumBoundingAngle(pRTREENODE node);

ONED_RANGE MinimumBoundingR(pRTREENODE node);


//---------------query processing-----------------//
REALTYPE MaxDist(RTREEMBR* mbrp, REALTYPE qlat, REALTYPE qlng);

REALTYPE MinDist(RTREEMBR* mbrp, REALTYPE qlat, REALTYPE qlng);

REALTYPE CalculateAngleClockwiseVector(REALTYPE v1x, 
                                       REALTYPE v1y, 
									   REALTYPE v2x, 
									   REALTYPE v2y);

void CoverTwoAnglesVector(ONED_RANGE pqs, ONED_RANGE pge, 
                          ONED_RANGE angle, ONED_RANGE &CoverAngleVS, 
						  ONED_RANGE &CoverAngleVE);

void CalculateMaxMinAngle(ONED_RANGE* angle, RTREEMBR* mbrp, 
                          REALTYPE qlat, REALTYPE qlng, 
						  REALTYPE &minangle, REALTYPE &maxangle);

bool MBRContainCircle(RTREEMBR* mbrp, REALTYPE qlat, 
                      REALTYPE qlng, REALTYPE qradius);

int NodeOverlapCheck(RTREEBRANCH* branchp, REALTYPE qlat, 
                     REALTYPE qlng, REALTYPE qradius);

int ObjectOverlapCheck(REALTYPE plat, REALTYPE plng, 
                       REALTYPE pthetas, REALTYPE pthetae, 
					   REALTYPE pR, REALTYPE qlat, REALTYPE qlng, 
					   REALTYPE qradius);

void ReportResults(RTREENODE* node);


void infoadd();

void CircleRangeQuery();

#ifdef __cplusplus
}
#endif

#endif /* IR2TREE_H_INCLUDED */