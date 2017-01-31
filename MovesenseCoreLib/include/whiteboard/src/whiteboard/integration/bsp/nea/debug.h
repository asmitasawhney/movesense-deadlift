#pragma once
// Copyright (c) Suunto Oy 2015. All rights reserved.

#include "whiteboard/integration/port.h"

#include "common/core/dbgassert.h"
#include "common/core/debug.h"

#define WB_DEBUGLOG DEBUGLOG

#if !defined(NDEBUG) || defined(WB_ASSERT_IN_RELEASE)
  #define WB_ASSERT ASSERT
#else
  #define WB_ASSERT(cond) ((void)0) 
#endif

#define WB_DEBUG_ASSERT DEBUG_ASSERT
#define WB_VERIFY VERIFY

#define WbDebugSetThreadName debugSetThreadName
