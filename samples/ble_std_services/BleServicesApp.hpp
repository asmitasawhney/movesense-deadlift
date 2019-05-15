#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>
#include <whiteboard/DpcFunctor.h>

#include "BleController.hpp"

class BleServicesApp FINAL : private wb::ResourceClient, public wb::LaunchableModule
{
    // Handle callbacks for HRS enabled/disabled
    // Note: This pattern is bound to change at some point
    class CustomBleController : public BleController
    {
    public:
        CustomBleController() {}

    protected:
        void OnHrsNotificationEnabled() { BleServicesApp::setNotificationRequest(true); }

        void OnHrsNotificationDisabled() { BleServicesApp::setNotificationRequest(false); }
    };
    // Allocate a global so that this supercedes the built in one
    CustomBleController bleCtrl;

public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;

    /** Sets the HR service request active or inactive
    *   @param state True if service activation requested, false if cancelled.
    */
    static void setNotificationRequest(bool state);

    BleServicesApp();
    ~BleServicesApp();

protected:
    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;

    /** @see whiteboard::ResourceClient::onNotify */
    virtual void onNotify(wb::ResourceId resourceId,
                          const wb::Value& value,
                          const wb::ParameterList& parameters);

    /** @see whiteboard::ResourceClient::onPutResult */
    virtual void onPutResult(wb::RequestId requestId,
                             wb::ResourceId resourceId,
                             wb::Result resultCode,
                             const wb::Value& result) OVERRIDE;

    /** @see whiteboard::ResourceClient::onTimer */
    virtual void onTimer(wb::TimerId timerId) OVERRIDE;

    /** Prepare to shutdown and set timer */
    void startShutdownTimer();

    /** Cancel shutdown and the timer if active */
    void stopShutdownTimer();

    /**
    *	DPS callback - events handler for HRS BLE events
    */
    void dpcHandler();

private:
    void setBleAdvPacket();

    wb::DpcFunctor mDpc;
    wb::TimerId mTimer;
    struct InternalStates
    {
        uint16_t HrsEnabled : 1;
        uint16_t HrsEnableReq : 1;
        uint16_t PeerConnected : 1;
    };
    InternalStates mStates;
    uint32_t mCounter;

    static BleServicesApp* spInstance;
};
