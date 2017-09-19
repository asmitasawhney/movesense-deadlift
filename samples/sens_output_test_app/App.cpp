#include "SensorOutputService.h"
#include "movesense.h"


MOVESENSE_APPLICATION_STACKSIZE(1024)

MOVESENSE_PROVIDERS_BEGIN(1)
MOVESENSE_PROVIDER_DEF(SensorOutputService)
MOVESENSE_PROVIDERS_END(1)

MOVESENSE_FEATURES_BEGIN()
SERIAL_COMMUNICATION(false)
BLE_COMMUNICATION(true)
MOVESENSE_FEATURES_END()