#pragma once
// Copyright (c) Suunto Oy 2015. All rights reserved.

#include "whiteboard/integration/port.h"

typedef uint32 WbEventFlagMask;

struct WbEventGroup; // Forward declaration hides the implementation
typedef WbEventGroup* WbEventGroupHandle;
#define WB_INVALID_EVENTGROUP NULL

/** Creates a new event group
*
* @param numberOfFlags Number of event flags to allocate
* @return New event group instance
*/
WB_API WbEventGroupHandle WbEventGroupCreate(size_t numberOfFlags);

/** Deletes a event group
*
* @param group Event group that should be deleted
*/
WB_API void WbEventGroupDelete(WbEventGroupHandle group);

/** Waits for one or all flags. Flags are cleared on exit.
*
* @param group Event group instance
* @param waitForAllFlags A value indicating whether all flags should be set before returning
* @param timeoutMs Timeout of wait in milliseconds
* @return Mask of flags that were set or zero if timeout elapsed
*/
WB_API WbEventFlagMask WbEventGroupWait(WbEventGroupHandle group, bool waitAll, size_t timeoutMs);

/** Clears flags
*
* @param group Event group instance
* @param flags Mask of flags that should be cleared
* @param isIsr A value indicating whether this function is called from interrupt service routine
*/
WB_API void WbEventGroupClearFlags(WbEventGroupHandle group, WbEventFlagMask flags, bool isIsr);

/** Sets flags
*
* @param group Event group instance
* @param flags Mask of flags that should be set
* @param isIsr A value indicating whether this function is called from interrupt service routine
*/
WB_API void WbEventGroupSetFlags(WbEventGroupHandle group, WbEventFlagMask flags, bool isIsr);