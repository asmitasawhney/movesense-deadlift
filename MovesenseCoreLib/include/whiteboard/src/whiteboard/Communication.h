#pragma once
/******************************************************************************
    Copyright (c) Suunto Oy 2015.
    All rights reserved.
******************************************************************************/

#include "whiteboard/integration/port.h"
#include "whiteboard/ICommunicationInterface.h"

namespace whiteboard
{

/** Facade for whiteboard communication initialization */
class WB_API Communication
{
public:
    /** Creates a new Serial adapter
     *
     * @param portName Name of the serial port to use
     * @param baudRate Communication baud rate
     * @param dataBits Number of data bits
     * @param parity Communication parity mode
     * @param stopBits Number of stop bits
     * @param flowControl Communication flow control setting
     * @return New SerialAdapter instance or NULL if adapter could not be created
     */
    static ICommunicationInterface* createSerialInterface(
        const char* portName,
        uint32 baudRate,
        WbSerialDataBits dataBits,
        WbSerialParity parity,
        WbSerialStopBits stopBits,
        WbSerialFlowControl flowControl);

    /** Free the memory allocated by the serial interface. */
    static void freeSerialInterface(ICommunicationInterface* pInterface);

    /** Creates a new ble adapter
    * @return New BleAdapter instance or NULL if adapter could not be created
    */
    static ICommunicationInterface* createBleInterface();

    /** Free the memory allocated by the BLE interface. */
    static void freeBleInterface(ICommunicationInterface* pInterface);

    /** Default UDP port */
    static const WbUdpPort DEFAULT_UDP_PORT = 7808;

    /** Creates new UDP adapter
     *
     * @param port UDP port to use
     * @return New UdpIpAdapter instance or NULL if adapter could not be created
     */
    static ICommunicationInterface* createUdpIpInterface(WbUdpPort port = DEFAULT_UDP_PORT);

    /** Free the memory allocated by the UDP interface. */
    static void freeUdpIpInterface(ICommunicationInterface* pInterface);

    /** Creates a new UDP adapter and binds it to specific network address
     *
     * @param rInterfaceAddress Address of the network interface to use
     * @param port UDP port to use
     * @return New UdpIpAdapter instance or NULL if adapter could not be created
     */
    static ICommunicationInterface* createUdpIpInterface(const WbIPv4Address& rInterfaceAddress, WbUdpPort port);

    /** Default port */
    static const WbTcpPort DEFAULT_TCP_PORT = 7809;

    /** Creates new TCP server.
     *
     * @param port TCP port to use
     * @return New TcpIpServer instance or NULL if adapter could not be created
     */
    static ICommunicationInterface* createTcpIpServerInterface(WbTcpPort port = DEFAULT_TCP_PORT);

    /** Free the memory allocated by the TCP server. */
    static void freeTcpIpServerInterface(ICommunicationInterface* pInterface);

    /** Creates a new TCP server and binds it to specific network address
     *
     * @param rInterfaceAddress Address of the network interface to use
     * @param port TCP port to use
     * @return New TcpIpServer instance or NULL if adapter could not be created
     */
    static ICommunicationInterface* createTcpIpServerInterface(const WbIPv4Address& rInterfaceAddress, WbTcpPort port);

    /** Creates new TCP client interface
     *
     * @return New TCP client interface instance or NULL if interface could not be created
     */
    static ICommunicationInterface* createTcpIpClientInterface();

    /** Free the memory allocated by the TCP client interface. */
    static void freeTcpIpClientInterface(ICommunicationInterface* pInterface);
};

} // namespace whiteboard