#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/

// Copyright (c) Suunto Oy 2014 - 2016. All rights reserved.

#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"

#include "whiteboard/builtinTypes/Array.h"
#include "whiteboard/builtinTypes/ByteStream.h"
#include "whiteboard/builtinTypes/Date.h"
#include "whiteboard/builtinTypes/DateTime.h"
#include "whiteboard/builtinTypes/Optional.h"
#include "whiteboard/builtinTypes/Structures.h"
#include "whiteboard/builtinTypes/Time.h"
#include "whiteboard/builtinTypes/Timestamp.h"
#include "whiteboard/builtinTypes/TypedEnum.h"
#include "whiteboard/builtinTypes/Vector2D.h"
#include "whiteboard/builtinTypes/Vector3D.h"
#include "whiteboard/builtinTypes/WrapperFor32BitPointer.h"

#define WB_EXECUTION_CONTEXT_INSTANTION_REF(id)						static_cast<whiteboard::ExecutionContextId>(id)
#define WB_RESOURCE_VALUE(whiteboardId, localResourceId, executionContextId) \
	static_cast<whiteboard::ResourceId::Value>( \
		(static_cast<uint32>(localResourceId) << 16) | \
		(static_cast<uint32>(whiteboardId) << 8) | \
		(static_cast<uint32>(executionContextId) << 4) | \
		(static_cast<uint32>(whiteboard::ID_INVALID_RESOURCE_INSTANCE)))

#define WB_CALLER_CONTEXT										whiteboard::ID_INVALID_EXECUTION_CONTEXT


/** Helper function for force linking of the library
* (Visual C will ignore whole library if none of the symbols are referenced)
*/
void pullInWbResources();

#include "../wb-resources/resources.h"
#include "../suunto_shared/resources.h"

#define WB_EXEC_CTX_PRIMARYSERVICES              WB_EXECUTION_CONTEXT_INSTANTION_REF(0)
#define WB_EXEC_CTX_APPLICATION                  WB_EXECUTION_CONTEXT_INSTANTION_REF(1)
#define WB_EXEC_CTX_MEAS                         WB_EXECUTION_CONTEXT_INSTANTION_REF(2)

namespace WB_RES {

WB_STRUCT_PACK_BEGIN()

struct SystemModeValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 6912;

	enum Type
	{
		UNKNOWN = 0U,
		FULLPOWEROFF = 1U,
		SYSTEMFAILURE = 2U,
		POWEROFF = 3U,
		WAITFORCHARGE = 4U,
		APPLICATION = 5U,
		FACTORYINTERACTIVE = 10U,
		FACTORYEXTERNAL = 11U,
		UPDATEBOOTLOADER = 12U,
		UPDATEPERIPHERALS = 13U,
		UPDATEAPPLICATION = 14U,
		FILESYNCHRONIZATION = 15U
	};
};
typedef whiteboard::TypedEnum<SystemModeValues, SystemModeValues::Type, uint8> SystemMode;

struct WB_STRUCT_PACKED SystemModeStatus;

struct WB_STRUCT_PACKED SystemModeStatus
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 6913;
	static const whiteboard::StructureValueSerializer<SystemModeStatus> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<SystemModeStatus> cleaner;)

	WB_ALIGN(1) SystemMode current;
	WB_ALIGN(1) whiteboard::Optional< SystemMode > next;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(current)
			.visit(next);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct DEVICE;

struct DEVICE_SYSTEM;

struct DEVICE_SYSTEM_MODE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 6912, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 6912;

	struct GET
	{
		typedef SystemModeStatus Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct PUT
	{
		struct Parameters
		{
			struct NEWSTATE
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef SystemMode Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Device/System/Mode */
		class ParameterListRef
		{
		private:
			/** Prevent use of default constructor */
			ParameterListRef() DELETED;

			/** Prevent use of copy constructor */
			ParameterListRef(const ParameterListRef&) DELETED;

			/** Prevent use of assignment operator */
			const ParameterListRef& operator=(const ParameterListRef&) DELETED;

		public:
			/** Constructor that initializes this class from existing parameter list
			*
			* @param rParameterList Reference to parameter list that contains untyped parameters
			*/
			inline ParameterListRef(const whiteboard::ParameterList& rParameterList)
				: mrParameterList(rParameterList)
			{
			}

			/** Gets NEWSTATE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::NEWSTATE::ConstReferenceType getNewState() const
			{
				return mrParameterList[Parameters::NEWSTATE::Index].convertTo<Parameters::NEWSTATE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};


} // namespace LOCAL

} // namespace WB_RES