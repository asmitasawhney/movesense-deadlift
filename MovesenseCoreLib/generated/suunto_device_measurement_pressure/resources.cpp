/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/

// Copyright (c) Suunto Oy 2014 - 2016. All rights reserved.

#include "resources.h"

namespace WB_RES {

WB_STATIC_VERIFY(sizeof(PressureValue) == 12, SizeOfStructure_PressureValue_IsNotWhatExpected);
WB_STATIC_VERIFY(WB_TYPE_ALIGNMENT(PressureValue) == 4, AlignmentOfStructure_PressureValue_IsNotWhatExpected);


const whiteboard::StructureValueSerializer<PressureValue> PressureValue::serializer;
WB_WHEN_STRUCTURE_CLEANING_NEEDED(const whiteboard::StructureValueCleaner<PressureValue> PressureValue::cleaner;)

} // namespace WB_RES
