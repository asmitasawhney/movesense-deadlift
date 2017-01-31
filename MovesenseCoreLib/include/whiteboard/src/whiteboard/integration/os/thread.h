#pragma once
// Copyright (c) Suunto Oy 2015. All rights reserved.

#include "whiteboard/integration/port.h"

/** Thread priorities */
enum WbThreadPriority
{
    WB_THREAD_PRIORITY_LOW,
    WB_THREAD_PRIORITY_NORMAL,
    WB_THREAD_PRIORITY_HIGH
};

/** Type of thread entry point */
typedef void (*WbThreadEntryPoint)(void* pUserData);

struct WbThreadInfo; // Forward declaration hides the implementation
typedef WbThreadInfo* WbThreadHandle;
#define WB_INVALID_THREAD NULL

/** Creates a new thread.
*
* @param threadName Name of the thread
* @param maxStackDepth Maximum depth of the stack
* @param priority Thread priority
* @param threadEntryPoint Entry point of the thread
* @param pUserData User specified pointer that should be passed to thread routine
* @return Handle to new thread
*/
WB_API WbThreadHandle WbThreadCreate(
    const char* threadName, uint16 maxStackDepth, WbThreadPriority priority, WbThreadEntryPoint threadEntryPoint, void* pUserData);

/** Attaches existing external thread to whiteboard threading system
*
* @return Handle to the thread
*/
WB_API WbThreadHandle WbThreadAttachExternal();

/** Deletes a thread
*
* @param threadHandle Handle of the thread to delete
*/
WB_API void WbThreadDelete(WbThreadHandle threadHandle);

/** Gets handle of current thread
*
* @return Handle of current thread
*/
WB_API WbThreadHandle WbThreadGetCurrent();

/** Gets name of the given thread
*
* @param threadHandle Handle of the thread
* @return name string or NULL if not available
*/
WB_API const char* WbThreadGetName(WbThreadHandle threadHandle);

/** Checks whether the thread is terminating
*
* @return A value indicating whether the thread should terminate
*/
WB_API bool WbThreadIsCurrentThreadTerminating();

/** Puts the calling thread for given number of milliseconds
*
* @param milliseconds Number of milliseconds to sleep
*/
WB_API void WbThreadSleepMs(size_t milliseconds);

/** Yields thread execution.
*/
WB_API void WbThreadYield();
