// Copyright (c) RVBUST, Inc - All rights reserved.
#include <string>
#include "LKH.h"
#include "Genetic.h"

namespace GtspSolver
{
namespace LKH
{

// derived from BestKOptMove.cpp
GainType bkoptm_BestG2;

// derived from BuildKDTree.cpp
Node **bkdt_KDTree;
int bkdt_cutoff;

// derived from CreateQuadrantCandidateSet.cpp
Node **cqcs_KDTree;
Candidate *cqcs_CandidateSet;
double *cqcs_XMin, *cqcs_XMax, *cqcs_YMin, *cqcs_YMax, *cqcs_ZMin, *cqcs_ZMax;
int cqcs_Candidates, cqcs_Radius;
int cqcs_Level = 0;

// derived from ERXT.cpp
Node *erxt_FirstFree;
int erxt_Tabu;

// derived from Gain23.cpp
Node *gain23_s1 = nullptr; ///< the global variable is used in Gain23.cpp
short gain23_OldReversed = 0;

// derived from GreedyTour.cpp
int grdt_EdgesInFragments = 0;
GainType grdt_Cost = 0;

// derived from Heap.cpp
int heap_HeapCount;
int heap_HeapCapacity;

// derived from PatchCycles.cpp
int pc_CurrentCycle;
int pc_Patchwork = 0;
int pc_RecLevel = 0;

// derived from Sequence.cpp
Node *seq_tp1;


// derived from SolveKarpSubproblems.cpp
Node **sksp_KDTree;
GainType sksp_GlobalBestCost, sksp_OldGlobalBestCost;
int sksp_CurrentSubproblem, sksp_Subproblems;

// derived from SolveRoheSubproblems.cpp
int srsp_Size;
Node **srsp_KDTree;

// derived from Statistics.cpp
int stat_TrialsMin, stat_TrialsMax, stat_TrialSum, stat_Successes;
GainType stat_CostMin, stat_CostMax, stat_CostSum;
double stat_TimeMin, stat_TimeMax, stat_TimeSum;

// declaration for global variables
/*****************************DGV**************************/

int AscentCandidates; /* Number of candidate edges to be associated
                        with each node during the ascent */
int BackboneTrials; /* Number of backbone trials in each run */
int Backtracking; /* Specifies whether backtracking is used for
                    the first move in a sequence of moves */
GainType BestCost; /* Cost of the tour in BestTour */
int *BestTour; /* Table containing best tour found */
GainType BetterCost; /* Cost of the tour stored in BetterTour */
int *BetterTour; /* Table containing the currently best tour
                   in a run */
int CacheMask; /* Mask for indexing the cache */
int *CacheVal; /* Table of cached distances */
int *CacheSig; /* Table of the signatures of cached
                 distances */
int CandidateFiles; /* Number of CANDIDATE_FILEs */
int *CostMatrix; /* Cost matrix */
int Dimension; /* Number of nodes in the problem */
int DimensionSaved; /* Saved value of Dimension */
double Excess; /* Maximum alpha-value allowed for any
                 candidate edge is set to Excess times the
                 absolute value of the lower bound of a
                 solution tour */
int ExtraCandidates; /* Number of extra neighbors to be added to
                       the candidate set of each node */
Node *FirstActive, *LastActive; /* First and last node in the list
                                  of "active" nodes */
Node *FirstNode; /* First node in the list of nodes */
Segment *FirstSegment; /* A pointer to the first segment in the cyclic
                         list of segments */
SSegment *FirstSSegment; /* A pointer to the first super segment in
                           the cyclic list of segments */
int Gain23Used; /* Specifies whether Gain23 is used */
int GainCriterionUsed; /* Specifies whether L&K's gain criterion is
                         used */
int GroupSize; /* Desired initial size of each segment */
int SGroupSize; /* Desired initial size of each super segment */
int Groups; /* Current number of segments */
int SGroups; /* Current number of super segments */
unsigned Hash; /* Hash value corresponding to the current tour */
Node **Heap; /* Heap used for computing minimum spanning
               trees */
HashTable *HTable; /* Hash table used for storing tours */
int InitialPeriod; /* Length of the first period in the ascent */
int InitialStepSize; /* Initial step size used in the ascent */
double InitialTourFraction; /* Fraction of the initial tour to be
                              constructed by INITIAL_TOUR_FILE edges */
char *LastLine; /* Last input line */
double LowerBound; /* Lower bound found by the ascent */
int Kicks; /* Specifies the number of K-swap-kicks */
int KickType; /* Specifies K for a K-swap-kick */
int M; /* The M-value is used when solving an ATSP-
         instance by transforming it to a STSP-instance */
int MaxBreadth; /* The maximum number of candidate edges
                  considered at each level of the search for
                  a move */
int MaxCandidates; /* Maximum number of candidate edges to be
                     associated with each node */
int MaxMatrixDimension; /* Maximum dimension for an explicit cost matrix */
int MaxSwaps; /* Maximum number of swaps made during the
                search for a move */
int MaxTrials; /* Maximum number of trials in each run */
int MergeTourFiles; /* Number of MERGE_TOUR_FILEs */
int MoveType; /* Specifies the sequantial move type to be used
                in local search. A value K >= 2 signifies
                that a k-opt moves are tried for k <= K */
Node *NodeSet; /* Array of all nodes */
int Norm; /* Measure of a 1-tree's discrepancy from a tour */
int NonsequentialMoveType; /* Specifies the nonsequential move type to
                             be used in local search. A value
                             L >= 4 signifies that nonsequential
                             l-opt moves are tried for l <= L */
GainType Optimum; /* Known optimal tour length.
                    If StopAtOptimum is 1, a run will be
                    terminated as soon as a tour length
                    becomes equal this value */
int PatchingA; /* Specifies the maximum number of alternating
                 cycles to be used for patching disjunct cycles */
int PatchingC; /* Specifies the maximum number of disjoint cycles to be
                 patched (by one or more alternating cycles) */
int Precision; /* Internal precision in the representation of
                 transformed distances */
int PredSucCostAvailable; /* PredCost and SucCost are available */
unsigned *Rand; /* Table of random values */
int RestrictedSearch; /* Specifies whether the choice of the first
                        edge to be broken is restricted */
short Reversed; /* Boolean used to indicate whether a tour has
                  been reversed */
int Run; /* Current run number */
int Runs; /* Total number of runs */
unsigned Seed; /* Initial seed for random number generation */
int StopAtOptimum; /* Specifies whether a run will be terminated if
                     the tour length becomes equal to Optimum */
int Subgradient; /* Specifies whether the Pi-values should be
                   determined by subgradient optimization */
int SubproblemSize; /* Number of nodes in a subproblem */
int SubsequentMoveType; /* Specifies the move type to be used for all
                          moves following the first move in a sequence
                          of moves. The value K >= 2 signifies that a
                          K-opt move is to be used */
int SubsequentPatching; /* Species whether patching is used for
                          subsequent moves */
SwapRecord *SwapStack; /* Stack of SwapRecords */
int Swaps; /* Number of swaps made during a tentative move */
double TimeLimit; /* The time limit in seconds for each run */
int TraceLevel; /* Specifies the level of detail of the output
                  given during the solution process.
                  The value 0 signifies a minimum amount of
                  output. The higher the value is the more
                  information is given */
int Trial; /* Ordinal number of the current trial */

/* The following variables are read by the functions ReadParameters and
   ReadProblem: */

char *ParameterFileName, *ProblemFileName, *PiFileName, *TourFileName,
    *OutputTourFileName, *InputTourFileName, **CandidateFileName,
    *InitialTourFileName, *SubproblemTourFileName, **MergeTourFileName;
char *Name, *Type, *EdgeWeightType, *EdgeWeightFormat, *EdgeDataFormat,
    *NodeCoordType, *DisplayDataType;
int CandidateSetSymmetric, CandidateSetType, CoordType, DelaunayPartitioning,
    DelaunayPure, ExtraCandidateSetSymmetric, ExtraCandidateSetType,
    InitialTourAlgorithm, KarpPartitioning, KCenterPartitioning,
    KMeansPartitioning, MoorePartitioning, PatchingAExtended,
    PatchingARestricted, PatchingCExtended, PatchingCRestricted, ProblemType,
    RohePartitioning, SierpinskiPartitioning, SubproblemBorders,
    SubproblemsCompressed, WeightType, WeightFormat;

FILE *ParameterFile, *ProblemFile, *PiFile, *InputTourFile, *TourFile,
    *InitialTourFile, *SubproblemTourFile, **MergeTourFile;
CostFunction Distance, D, C, c;
MoveFunction BestMove, BacktrackMove, BestSubsequentMove;

/**********************************END*******************************/


// global variables from header file Genetic.h
/***********************DGV****************************/

int MaxPopulationSize; /* The maximum size of the population */
int PopulationSize; /* The current size of the population */

CrossoverFunction Crossover;

int **Population; /* Array of individuals (solution tours) */
GainType *Fitness; /* The fitness (tour cost) of each individual */

/**********************END**********************************/

// global variables from header file "Sequence.h"
/*******************DGV**************************/

Node **t; /* The sequence of nodes to be used in a move */
Node **T; /* The currently best t's */
Node **tSaved; /* For saving t when using the BacktrackKOptMove function */
int *p; /* The permutation corresponding to the sequence in which
           the t's occur on the tour */
int *q; /* The inverse permutation of p */
int *incl; /* Array: incl[i] == j, if (t[i], t[j]) is an inclusion edge */
int *cycle; /* Array: cycle[i] is cycle number of t[i] */
GainType *G; /* For storing the G-values in the BestKOptMove function */
int K; /* The value K for the current K-opt move */

/**********************END************************/


} // namespace LKH
} // namespace GtspSolver
