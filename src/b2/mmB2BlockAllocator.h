/*
-----------------------------------------------------------------------------
MIT License

Copyright (c) 2019 Erin Catto
Copyright (c) 2023-2023 mm_longcheng@icloud.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#ifndef __mmB2BlockAllocator_h__
#define __mmB2BlockAllocator_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"

#include "b2/mmB2Prefix.h"

enum
{
    b2_blockSizeCount = 14,
};

struct b2Block;
struct b2Chunk;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
struct b2BlockAllocator
{
    struct b2Chunk* m_chunks;
    int32 m_chunkCount;
    int32 m_chunkSpace;

    struct b2Block* m_freeLists[b2_blockSizeCount];
};

B2_API
void
b2BlockAllocatorInit(
    struct b2BlockAllocator* p);

B2_API
void
b2BlockAllocatorDestroy(
    struct b2BlockAllocator* p);

/// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
B2_API
void*
b2BlockAllocatorAllocate(
    struct b2BlockAllocator* p,
    int32 size);

/// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
B2_API
void
b2BlockAllocatorFree(
    struct b2BlockAllocator* p,
    void* m, 
    int32 size);

B2_API
void
b2BlockAllocatorClear(
    struct b2BlockAllocator* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2BlockAllocator_h__
