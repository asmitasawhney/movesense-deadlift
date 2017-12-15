#pragma once
/***********************************************************************
* THIS FILE HAS BEEN GENERATED BY WBRES TOOL. DO NOT TRY TO CHANGE IT. *
***********************************************************************/
// Copyright (c) Suunto Oy 2014 - 2017. All rights reserved.

#include "whiteboard/Identifiers.h"
#include "whiteboard/ParameterList.h"
#include "whiteboard/Result.h"
#include "whiteboard/ResourceClient.h"

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

#define WB_RESOURCE_VALUE(whiteboardId, localResourceId, executionContextId) \
    static_cast<whiteboard::ResourceId::Value>( \
        (static_cast<uint32>(localResourceId) << 16) | \
        (static_cast<uint32>(whiteboardId) << 8) | \
        (static_cast<uint32>(executionContextId) << 4) | \
        (static_cast<uint32>(whiteboard::ID_INVALID_RESOURCE_INSTANCE)))

#define WB_CALLER_CONTEXT whiteboard::ID_INVALID_EXECUTION_CONTEXT


#include "../wb-resources/resources.h"

#define WB_EXEC_CTX_APPLICATION                  static_cast<whiteboard::ExecutionContextId>(0)
#define WB_EXEC_CTX_MEAS                         static_cast<whiteboard::ExecutionContextId>(1)

namespace WB_RES {

struct ActiveProcessorValues
{
	static const whiteboard::LocalDataTypeId DATA_TYPE_ID = 2048;

	enum Type
	{
		LP = 1U,
		AP = 2U
	};
};
typedef whiteboard::TypedEnum<ActiveProcessorValues, ActiveProcessorValues::Type, uint8> ActiveProcessor;

namespace LOCAL
{

struct ROOT;

struct DEV;

struct DEV_SYSTEM;

struct DEV_SYSTEM_DISPLAY;

struct DEV_SYSTEM_DISPLAY_RECORD
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2048, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2048;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_NOT_ACCEPTABLE> HTTP_CODE_NOT_ACCEPTABLE;

		struct Parameters
		{
			struct START
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef bool Type;
				typedef Type ConstReferenceType;
			};

			typedef START Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/Display/Record */
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

			/** Gets START parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::START::ConstReferenceType getStart() const
			{
				return mrParameterList[Parameters::START::Index].convertTo<Parameters::START::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::START::ConstReferenceType)
		{
		}
	};
};

struct DEV_SYSTEM_DISPLAY_SAVE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2049, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2049;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			struct FILENAME
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef const char* Type;
				typedef Type ConstReferenceType;
			};

			typedef FILENAME Parameter1;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 1;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/Display/Save */
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

			/** Gets FILENAME parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::FILENAME::ConstReferenceType getFilename() const
			{
				return mrParameterList[Parameters::FILENAME::Index].convertTo<Parameters::FILENAME::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::FILENAME::ConstReferenceType)
		{
		}
	};
};

struct DEV_SYSTEM_LPAPPVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2050, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2050;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_LPDALIVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2051, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2051;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_LPFILESYSTEMVERSION
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2052, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2052;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const char*, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_LPUPDATEPROGRESS
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2053, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2053;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<uint8, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};

	struct SUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<uint8, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};

	struct EVENT
	{
		typedef uint8 NotificationType;
		typedef NotificationType ConstReferenceNotificationType;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			const whiteboard::Api::OptionalParameter<ConstReferenceNotificationType>&)
		{
		}
	};

	struct UNSUBSCRIBE
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_MEMORY;

struct DEV_SYSTEM_MEMORY_FREE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2054, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2054;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_MEMORY_ISSUFFICIENT
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2055, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2055;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<bool, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_NO_CONTENT> HTTP_CODE_NO_CONTENT;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_MEMORY_LOWESTFREE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2056, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2056;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_MEMORY_SIZE
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2057, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2057;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_MEMORY_USED
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2058, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2058;

	struct GET
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;

		struct Parameters
		{
			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 0;
		};

		/** Compile time type checking */
		inline static void typeCheck()
		{
		}
	};
};

struct DEV_SYSTEM_UPDATELPAPP
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2059, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2059;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_BAD_REQUEST> HTTP_CODE_BAD_REQUEST;

		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			typedef LOCALRESOURCEID Parameter1;

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef BINARYSIZE Parameter2;

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef MAXCHUNKSIZE Parameter3;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpApp */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::LOCALRESOURCEID::ConstReferenceType,
			Parameters::BINARYSIZE::ConstReferenceType,
			const whiteboard::Api::OptionalParameter<Parameters::MAXCHUNKSIZE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};
};

struct DEV_SYSTEM_UPDATELPDALI
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2060, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2060;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_BAD_REQUEST> HTTP_CODE_BAD_REQUEST;

		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			typedef LOCALRESOURCEID Parameter1;

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef BINARYSIZE Parameter2;

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef MAXCHUNKSIZE Parameter3;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpDali */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::LOCALRESOURCEID::ConstReferenceType,
			Parameters::BINARYSIZE::ConstReferenceType,
			const whiteboard::Api::OptionalParameter<Parameters::MAXCHUNKSIZE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};
};

struct DEV_SYSTEM_UPDATELPFILESYSTEM
{
	static const whiteboard::ExecutionContextId EXECUTION_CONTEXT = WB_EXEC_CTX_MEAS;
	static const whiteboard::ResourceId::Value ID = WB_RESOURCE_VALUE(0, 2061, EXECUTION_CONTEXT);
	static const whiteboard::LocalResourceId LID = 2061;

	struct PUT
	{
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_OK> HTTP_CODE_OK;
		typedef whiteboard::StronglyTypedResult<const whiteboard::NoType&, whiteboard::HTTP_CODE_BAD_REQUEST> HTTP_CODE_BAD_REQUEST;

		struct Parameters
		{
			struct LOCALRESOURCEID
			{
				static const whiteboard::ParameterIndex Index = 0;

				typedef uint16 Type;
				typedef Type ConstReferenceType;
			};

			typedef LOCALRESOURCEID Parameter1;

			struct BINARYSIZE
			{
				static const whiteboard::ParameterIndex Index = 1;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef BINARYSIZE Parameter2;

			struct MAXCHUNKSIZE
			{
				static const whiteboard::ParameterIndex Index = 2;

				typedef uint32 Type;
				typedef Type ConstReferenceType;
			};

			typedef MAXCHUNKSIZE Parameter3;

			static const whiteboard::ParameterIndex NUMBER_OF_PARAMETERS = 3;
		};

		/** Reference wrapper for strongly typed parameter list for /Dev/System/UpdateLpFilesystem */
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

			/** Gets LOCALRESOURCEID parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::LOCALRESOURCEID::ConstReferenceType getLocalResourceId() const
			{
				return mrParameterList[Parameters::LOCALRESOURCEID::Index].convertTo<Parameters::LOCALRESOURCEID::ConstReferenceType>();
			}

			/** Gets BINARYSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::BINARYSIZE::ConstReferenceType getBinarySize() const
			{
				return mrParameterList[Parameters::BINARYSIZE::Index].convertTo<Parameters::BINARYSIZE::ConstReferenceType>();
			}

			/** Checks whether optional parameter MAXCHUNKSIZE has a value
			*
			* @return A value indicating whether the parameter has a value
			*/
			inline bool hasMaxChunkSize() const
			{
				if (mrParameterList.getNumberOfParameters() <= Parameters::MAXCHUNKSIZE::Index)
				{
					return false;
				}

				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].getType() != whiteboard::WB_TYPE_NONE;
			}

			/** Gets MAXCHUNKSIZE parameter value
			*
			* @return Current parameter value
			*/
			inline Parameters::MAXCHUNKSIZE::ConstReferenceType getMaxChunkSize() const
			{
				return mrParameterList[Parameters::MAXCHUNKSIZE::Index].convertTo<Parameters::MAXCHUNKSIZE::ConstReferenceType>();
			}

		private:
			/** Reference to actual parameter list */
			const whiteboard::ParameterList& mrParameterList;
		};

		/** Compile time type checking */
		inline static void typeCheck(
			Parameters::LOCALRESOURCEID::ConstReferenceType,
			Parameters::BINARYSIZE::ConstReferenceType,
			const whiteboard::Api::OptionalParameter<Parameters::MAXCHUNKSIZE::ConstReferenceType>& = whiteboard::NoType::NoValue)
		{
		}
	};
};


} // namespace LOCAL

} // namespace WB_RES
