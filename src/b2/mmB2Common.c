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

#include "mmB2Common.h"

#include <stdio.h>
#include <assert.h>
#include <stdarg.h>

FILE* b2_dumpFile = NULL;

B2_API
void
b2OpenDump(
    const char* fileName)
{
    b2Assert(b2_dumpFile == NULL);
    b2_dumpFile = fopen(fileName, "w");
}

B2_API
void
b2Dump(
    const char* string, 
    ...)
{
    if (b2_dumpFile == NULL)
    {
        return;
    }
    else
    {
        va_list ap;
        va_start(ap, string);
        vfprintf(b2_dumpFile, string, ap);
        va_end(ap);
    }
}

B2_API
void
b2CloseDump(void)
{
    fclose(b2_dumpFile);
    b2_dumpFile = NULL;
}

B2_API const struct b2Version b2_version = { 2, 4, 1 };

