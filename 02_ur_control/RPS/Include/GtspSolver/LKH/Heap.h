#ifndef _HEAP_H
#define _HEAP_H

/*
 * This header specifies the interface for the use of heaps.
 */

#include "LKH.h"

namespace GtspSolver
{
namespace LKH
{

void MakeHeap(int Size);
void HeapInsert(Node *N);
void HeapDelete(Node *N);
Node *HeapDeleteMin(void);
void HeapLazyInsert(Node *N);
void Heapify(void);
void HeapSiftUp(Node *N);
void HeapSiftDown(Node *N);

} // namespace LKH
} // namespace GtspSolver


#endif
