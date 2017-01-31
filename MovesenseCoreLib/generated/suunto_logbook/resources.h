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

struct WB_STRUCT_PACKED LogEntry;
struct WB_STRUCT_PACKED LogEntries;

struct WB_STRUCT_PACKED LogEntry
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 9216;
	static const whiteboard::StructureValueSerializer<LogEntry> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<LogEntry> cleaner;)

	WB_ALIGN(4) uint32 id;
	WB_ALIGN(4) uint32 modificationTimestamp;
	WB_ALIGN(8) whiteboard::Optional< uint64 > size;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(id)
			.visit(modificationTimestamp)
			.visit(size);
	}
};

struct WB_STRUCT_PACKED LogEntries
{
	// Structure type identification and serialization
	typedef int Structure;
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 9217;
	static const whiteboard::StructureValueSerializer<LogEntries> serializer;
	WB_WHEN_STRUCTURE_CLEANING_NEEDED(static const whiteboard::StructureValueCleaner<LogEntries> cleaner;)

	WB_ALIGN(4) whiteboard::Array< LogEntry > elements;

	inline void visit(whiteboard::IStructureVisitor& rVisitor)
	{
		rVisitor
			.visit(elements);
	}
};

WB_STRUCT_PACK_END()

namespace LOCAL
{

struct ROOT;

struct LOGBOOK;

struct LOGBOOK_ENTRIES
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9216, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9216;

	struct GET
	{
		typedef LogEntries Response_HTTP_CODE_CONTINUE_Type;
		typedef LogEntries Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			struct STARTAFTERID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/Entries */
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

			/** Checks whether optional parameter STARTAFTERID has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasStartAfterId() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::STARTAFTERID::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::STARTAFTERID::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets STARTAFTERID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::STARTAFTERID::ConstReferenceType getStartAfterId() const
			{
				return mrParameterList[Parameters::STARTAFTERID::Index].convertTo<Parameters::STARTAFTERID::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct LOGBOOK_ISFULL
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9217, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9217;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef bool NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct LOGBOOK_ISOPEN
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9218, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9218;

	struct GET
	{
		typedef bool Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef bool Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef bool NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct LOGBOOK_LOG;

struct LOGBOOK_LOG_LOGINDEX;

struct LOGBOOK_LOG_LOGINDEX_FLAGS
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9219, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9219;

	struct GET
	{
		typedef uint16 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			struct LOGINDEX
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/Log/{LogIndex}/Flags */
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

			/** Gets LOGINDEX parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOGINDEX::ConstReferenceType getLogIndex() const
			{
				return mrParameterList[Parameters::LOGINDEX::Index].convertTo<Parameters::LOGINDEX::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};

	struct PUT
	{
		struct Parameters
		{
			struct LOGINDEX
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			struct FLAGS
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 2;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/Log/{LogIndex}/Flags */
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

			/** Gets LOGINDEX parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOGINDEX::ConstReferenceType getLogIndex() const
			{
				return mrParameterList[Parameters::LOGINDEX::Index].convertTo<Parameters::LOGINDEX::ConstReferenceType>();
			}

			/** Gets FLAGS parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::FLAGS::ConstReferenceType getFlags() const
			{
				return mrParameterList[Parameters::FLAGS::Index].convertTo<Parameters::FLAGS::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct LOGBOOK_LOG_LOGINDEX_REMOVE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9220, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9220;

	struct PUT
	{
		struct Parameters
		{
			struct LOGINDEX
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/Log/{LogIndex}/Remove */
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

			/** Gets LOGINDEX parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOGINDEX::ConstReferenceType getLogIndex() const
			{
				return mrParameterList[Parameters::LOGINDEX::Index].convertTo<Parameters::LOGINDEX::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct LOGBOOK_LOGGING
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9221, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9221;

	struct GET
	{
		typedef uint16 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef uint16 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct LOGBOOK_UNSYNCHRONISEDLOGS
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9222, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9222;

	struct GET
	{
		typedef uint16 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct SUBSCRIBE
	{
		typedef uint16 Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};

	struct EVENT
	{
		typedef uint16 NotificationType;
	};

	struct UNSUBSCRIBE
	{
		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};
	};
};

struct LOGBOOK_BYID;

struct LOGBOOK_BYID_LOGID;

struct LOGBOOK_BYID_LOGID_DATA
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9223, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9223;

	struct GET
	{
		typedef whiteboard::ByteStream Response_HTTP_CODE_CONTINUE_Type;
		typedef whiteboard::ByteStream Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			struct LOGID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/byId/{LogId}/Data */
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

			/** Gets LOGID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOGID::ConstReferenceType getLogId() const
			{
				return mrParameterList[Parameters::LOGID::Index].convertTo<Parameters::LOGID::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};

struct LOGBOOK_BYID_LOGID_DESCRIPTORS
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 9224, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 9224;

	struct GET
	{
		typedef whiteboard::ByteStream Response_HTTP_CODE_CONTINUE_Type;
		typedef whiteboard::ByteStream Response_HTTP_CODE_OK_Type;

		struct Parameters
		{
			struct LOGID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef int32 Type;
				typedef Type ConstReferenceType;
			};

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Logbook/byId/{LogId}/Descriptors */
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

			/** Gets LOGID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOGID::ConstReferenceType getLogId() const
			{
				return mrParameterList[Parameters::LOGID::Index].convertTo<Parameters::LOGID::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};
	};
};


} // namespace LOCAL

} // namespace WB_RES