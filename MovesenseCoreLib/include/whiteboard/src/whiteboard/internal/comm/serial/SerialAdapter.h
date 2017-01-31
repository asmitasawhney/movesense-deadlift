#pragma once
/******************************************************************************
    Copyright (c) Suunto Oy 2015.
    All rights reserved.
******************************************************************************/

#include "whiteboard/CommAdapter.h"

namespace whiteboard
{

/** Class that implements Serial wrapper for whiteboard */
class SerialAdapter FINAL : public CommAdapter
{
private:
    /** Prevents use of the constructor
     *
     * @param port Port that should be used
     */
    SerialAdapter(WbSerialPortHandle portHandle);

public:
    /** Destructor */
    virtual ~SerialAdapter();

    /** Creates new Serial adapter
     *
     * @param portName Name of the serial port to use
     * @param baudRate Communication baud rate
     * @param dataBits Number of data bits
     * @param parity Communication parity mode
     * @param stopBits Number of stop bits
     * @param flowControl Communication flow control setting
     * @return New SerialAdapter instance or NULL if adapter could not be created
     */
    static SerialAdapter* create(const char* portName,
                                 uint32 baudRate,
                                 WbSerialDataBits dataBits,
                                 WbSerialParity parity,
                                 WbSerialStopBits stopBits,
                                 WbSerialFlowControl flowControl);

    /**
    * Gets name of communication adapter
    *
    * @return Name of the adapter
    */
    const char* getName() const OVERRIDE { return "Serial"; }

    /** Adds new route to the routing table that uses this adapter
    *
    * @param address Address of the remote end
    * @param serialNumber Serial number of the remote end
    * @param buddy A value indicating whether this should be the buddy route
    * @param rRouteHandle On output contains handle to routing table entry that was just added
    * @return Result of the operation
    */
    Result addRoute(const char* address, const char* serialNumber, bool buddy, RoutingTableEntryHandle& rRouteHandle) OVERRIDE;

    /** Enables the adapter
    *
    * @return Result of the operation
    */
    Result enableAdapter() OVERRIDE;

    /** Disables the adapter
    *
    * @return Result of the operation
    */
    Result disableAdapter() OVERRIDE;

    /** Connects the adapter to destination
    *
    * @param rDestination Address of the destination
    * @return Result of the operation
    */
    Result connect(const Address& rDestination) FINAL OVERRIDE;

    /** Cancels an on going connection attempt
    *
    * @param rDestination Address of the destination
    * @param reason Reason for the cancellation
    * @return Result of the operation
    */
    Result cancelConnect(const Address& rDestination, Result reason) OVERRIDE FINAL;

    /** Disconnects the adapter from destination
    *
    * @param rDestination Address of the destination
    * @return Result of the operation
    */
    Result disconnect(const Address& rDestination) FINAL OVERRIDE;

    /** Gets buffer allocator that should be used to allocate and deallocate
    * adapter buffers
    *
    * @return Buffer allocator instance
    */
    IBufferAllocator& getAllocator() const FINAL OVERRIDE;

    /** Sends a message using the communication adapter to specified destination
    *
    * @param rDestination Message destination
    * @param pBuffer pBuffer Buffer that contains the message
    * @return Result of the operation
    */
    Result send(const Address& rDestination, Buffer* pBuffer) FINAL OVERRIDE;

    /** Cancels an on going send request
    *
    * @param pBufferP Buffer that contains the message that should be canceled
    * @param reason Reason for the cancelation
    * @return Result of the operation
    */
    Result cancelSend(Buffer* pBuffer, Result reason) FINAL OVERRIDE;

    /** Formats the adapter address to human readable form
    *
    * @param rAddress Address that should be formatted
    * @param bufferSize Maximum number of charactes in the buffer including zero terminator
    * @param addressBuffer Buffer where address should be written
    * @return Result of the operation
    */
    Result formatAddress(const Address& rAddress, size_t bufferSize, char* addressBuffer) FINAL OVERRIDE;

private:
    /** Thread loop */
    void threadFunc();

    /** Entry point of receiver thread
    *
    * @param pUserData Pointer to execution context instance
    */
    static void threadRunnerCB(void* pUserData);

private:
    /** Port that is associated with this adapter */
    WbSerialPortHandle mPortHandle;

    /** Handle of the thread */
    WbThreadHandle mThreadHandle;

    /** Handle of a semaphore that is used to block receive thread
     * when adapter has been disabled */
    WbSemaphoreHandle mReceiveRunningSemaphore;
};

} // namespace whiteboard
