/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/

// Copyright (c) Suunto Oy 2014 - 2016. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(LogEntries) == 8, SizeOfStructure_LogEntries_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(LogEntries) == 4, AlignmentOfStructure_LogEntries_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(LogEntry) == 24, SizeOfStructure_LogEntry_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(LogEntry) == 8, AlignmentOfStructure_LogEntry_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<LogEntries> LogEntries::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<LogEntries> LogEntries::cleaner;)
const whiteboard::StructureValueSerializer<LogEntry> LogEntry::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<LogEntry> LogEntry::cleaner;)

} // namespace WB_RES
