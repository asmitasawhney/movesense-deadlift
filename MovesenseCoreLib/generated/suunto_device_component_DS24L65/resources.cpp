/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/

// Copyright (c) Suunto Oy 2014 - 2016. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(CommandsObject) == 8, SizeOfStructure_CommandsObject_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(CommandsObject) == 4, AlignmentOfStructure_CommandsObject_IsNotWhatExpected);

WB_STATIC_VERIFY(sizeof(EEprom64ByteArray) == 8, SizeOfStructure_EEprom64ByteArray_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(EEprom64ByteArray) == 4, AlignmentOfStructure_EEprom64ByteArray_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<CommandsObject> CommandsObject::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<CommandsObject> CommandsObject::cleaner;)
const whiteboard::StructureValueSerializer<EEprom64ByteArray> EEprom64ByteArray::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<EEprom64ByteArray> EEprom64ByteArray::cleaner;)

} // namespace WB_RES
