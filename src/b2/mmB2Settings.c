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

#include "mmB2Settings.h"

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

/// Implement this function to use your own memory allocator.
B2_API 
void* 
b2Alloc(
    int32 size)
{
    return malloc(size);
}

/// If you implement b2Alloc, you should also implement this function.
B2_API 
void 
b2Free(
    void* mem)
{
    free(mem);
}

/// Implement this to use your own logging.
B2_API 
void 
b2Log(
    const char* string, 
    ...)
{
    //static const char* tag = "b2";
    //static const int lvl = MM_LOG_INFO;
    //struct mmLogger* p = &gLogger;
    va_list ap;
    va_start(ap, string);
    //mmLogger_MsgVPrintf(p, lvl, tag, string, ap);
    vprintf(string, ap);
    va_end(ap);
}

