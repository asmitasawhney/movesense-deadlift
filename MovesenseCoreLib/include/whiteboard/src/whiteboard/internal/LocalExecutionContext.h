// Copyright (c) Suunto Oy 2014. All rights reserved.
#pragma once

#include "whiteboard/WbVersion.h"
#include "whiteboard/metadata/MetadataStructures.h"
#include "whiteboard/internal/IExecutionContext.h"
#include "whiteboard/internal/EventProcessor.h"
#include "whiteboard/internal/protocol/IMessageDispatcher.h"
#include "whiteboard/internal/Whiteboard.h"

namespace whiteboard
{
// Forward declarations
class ParameterList;
class Value;
class DpcFunctor;
class WhiteboardCommunication;

namespace services
{
class MetadataProvider;
}

/** Execution context that abstracts message passing */
class LocalExecutionContext : public IExecutionContext, private IMessageDispatcher::IDispatcher
{
private:
    /** Prevent use of default constructor */
    LocalExecutionContext() DELETED;
    /** Prevent use of default copy constructor */
    LocalExecutionContext(const LocalExecutionContext&) DELETED;
    /** Prevent use of assignment constructor */
    LocalExecutionContext& operator=(const LocalExecutionContext&) DELETED;

public:
    /** get reference to the EventProcessor instance */
    inline EventProcessor& eventProcessor() { return mProcessor; }

    /** Initializes a new instance of ExecutionContext class
    *
    * @param id Id of the execution context
    * @param rDescriptor Execution context descriptor
    * @param rWhiteboard Associated whiteboard instance. Used only for unittest mockups.
    */
    LocalExecutionContext(
        const whiteboard::ExecutionContextId id,
        const metadata::ExecutionContextInfo& rDescriptor,
        Whiteboard& rWhiteboard = *Whiteboard::getInstance());

    /** Destructor */
    virtual ~LocalExecutionContext() {}

    /**
    *	Gets ID of the execution context
    *
    *	@return ID of the execution context
    */
    ExecutionContextId getId() const OVERRIDE FINAL { return mId; }

    /** Gets the execution context name. */
    const char* getName() const OVERRIDE FINAL { return mrDescriptor.name; }

    /**
    * Initializes the execution context
    *
    * @return A value indicating whether execution context was successfully initialized
    */
    virtual bool init();

    /**
    * Terminates the execution context
    */
    void terminate();

    /**
    *  Returns the handle to the created thread.
    */
    WbThreadHandle getThreadHandle();

WB_PUBLIC_IN_UNITTESTS(private):
public:
    /**
    * Handles given message (from IExecutionContext).
    * @param rEvent Event that should be handled
    */
    virtual void handle(const WbEvent& rEvent) OVERRIDE;

    /**
    * Processing of states between events(from IExecutionContext).
    *
    * @param eventsPending - true if there is events pending processing for the execution context.
    *
    * @return Options that guide further event prosessing.
    */
    metadata::EventProcessorOptions eventStateProcess(bool eventsPending) OVERRIDE;

private:
    /** Whiteboard accesses these functions directly */
    friend class ExecutionContext;
    friend class services::MetadataProvider;
    friend class Whiteboard;
    friend class WhiteboardCommunication;

    /** Get the settings used to initialize this execution context. */
    const metadata::ExecutionContextSettings& getMetadata() const { return mProcessor.getMetadata(); }

    /**
    *	Sends resource changed notification for to client's execution context
    *
    *   @note This function can be overriden in child classes in order to filter update notifications
    *         on execution context level. Note also that this is executed in caller's context. Overriding
    *         implementation should not have any blocking calls. Otherwise there is a high risk of deadlocks.
    *
    *	@param clientId ID of the client that should receive this message
    *	@param resourceId ID of the associated resource
    *	@param rValue Current value of the resource
    *	@param rParameters, where the type of the notification (update, insert, delete) is the first parameter
    *   @param dontBlockIfQueueFull Optional - If false, and queue full, blocks until space in the queue to dispatch, else returns failure.
    *	@return Result of the operation
    */
    virtual Result dispatchClientOnNotify(
        ClientId clientId, ResourceId resourceId, const Value& rValue, const ParameterList& rParameters, bool dontBlockIfQueueFull = false);

    /**
    *	Sends results of GET, PUT, SUBSCRIBE or UNSUBSCRIBE request to client's execution context
    *
    *	@param rRequest Request information
    *	@param resultCode Result code of the request
    *	@param rResultData Result data of the request
    *   @param dontBlockIfQueueFull Optional - If false, and queue full, blocks until space in the queue to dispatch, else returns failure.
    *	@return Result of the operation
    */
    Result dispatchClientResult(
        const Request& rRequest,
        Result resultCode,
        const Value& rResultData,
        bool dontBlockIfQueueFull = false);

    /**
    *	Sends GET, PUT, POST, DELETE, SUBSCRIBE or UNSUBSCRIBE request to provider's execution context.
    *
    *	@param rRequest Request information
    *	@param rParameters List of parameters for the request
    *   @param dontBlockIfQueueFull Optional - If false, and queue full, blocks until space in the queue to dispatch, else returns failure.
    *	@return Result of the operation
    */
    Result dispatchProviderRequest(
        const Request& rRequest,
        const ParameterList& rParameters,
        bool dontBlockIfQueueFull = false);

    /**
    *	Sends TIMER MESSAGE to provider's execution context.
    *
    *	@param providerId ID of the provider
    *	@param timerID ID of the timer
    */
    void dispatchProviderTimerMsg(LocalProviderId providerId, TimerId timerId);

    /**
    *	Sends TIMER MESSAGE to client's execution context.
    *
    *	@param providerId ID of the provider
    *	@param timerID ID of the timer
    */
    void dispatchClientTimerMsg(LocalProviderId clientId, TimerId timerId);

    /**
    *   Sends remote whiteboard disconnected status notification to the execution context.
    *
    *   @param whiteboardId [in] Local Id of the remote whiteboard that has been disconnected.
    */
    void dispatchOnRemoteWhiteboardDisconnectedNotification(WhiteboardId whiteboardId);

    /**
    *   Sends local client removal event to the execution context.
    *
    *   @param clientId [in] Id of the client that has been removed.
    */
    void dispatchLocalClientRemove(LocalClientId clientId);

    /**
    *   Sends local client notification about resource unavailability
    *
    *   @param clientId [in] Id of the client to notify
    *   @param resourceId [in] Id of the resource that is unavailable
    */
    void dispatchClientOnLocalResourceUnavailable(LocalClientId clientId, ResourceId resourceId);

private:
    /** Gets buffer allocator instance
    *
    * @return Buffer allocator instance
    */
    IBufferAllocator& getBufferAllocator() OVERRIDE FINAL;

    /** Dispatches a data message
    *
    * @param messageType The type of the message.
    * @param pBuffer Buffer that contains the message that should be dispatched
    * @param whiteboardIdInLocalScope ID of the whiteboard in this local context
    * @param protocol The protocol version
    * @param dontBlockIfQueueFull Optional - If false, and queue full, blocks until space in the queue to dispatch, else returns failure.
    */
    Result dispatch(const MessageType messageType,
                    Buffer* pBuffer,
                    WhiteboardId whiteboardIdInLocalScope,
                    ProtocolVersion protocol,
                    bool dontBlockIfQueueFull) OVERRIDE FINAL;

    /**
    *	Checks whether the given thread is the event processor thread
    *
    *	@param threadHandle thread handle in question
    *	@return true if it is the same thread
    *			false if not
    */
    bool isThisThread(WbThreadHandle thread) const { return mProcessor.isThisThread(thread); }

protected:
    /** Whiteboard instance */
    Whiteboard& mrWhiteboard;

private:
    /** The event processor. */
    EventProcessor mProcessor;

    /** Execution context ID */
    const whiteboard::ExecutionContextId mId;

    /** Execution context descriptor */
    const metadata::ExecutionContextInfo& mrDescriptor;
};

} // namespace whitespace