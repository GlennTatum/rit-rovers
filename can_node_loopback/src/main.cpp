#include <Arduino.h>
#include <ACAN_T4.h>

static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

/**
 * Each message handler must be of typedef
 * typedef void (*ACANCallBackRoutine) (const CANMessage & inMessage) ;
 * 
 * Example: message handler for message A
 */
void handleMessageA(const CANMessage & inMessage) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
}

const ACANPrimaryFilter primaryfilters[] = {
  ACANPrimaryFilter(kData ,kStandard, 0x542, handleMessageA)
};

void setup() {

  Serial.begin(9600);

  ACAN_T4_Settings settings(125 *1000);
  
  settings.mLoopBackMode = true;
  settings.mSelfReceptionMode = true;

  /**
   * Make sure to include the primaryFilters and the size of the list to be able to use the dispatchReceivedMessage method on a bus recieve
   */
  uint32_t ok;
  if((ok = ACAN_T4::can1.begin(settings, primaryfilters, 1)) != 0) {
    Serial.println("Error CAN1 on Initialization");
  }

}

bool tryCANSend(CANMessage message) {
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
    return ok;
}

void loop () {
  CANMessage message ; // Send a blank CAN message onto the bus with id 0x542
  message.id = 0x542;

  // Send every 2000 ms
  if (gSendDate <= millis ()) {
    bool ok;
    if(!(ok = tryCANSend(message))) {
      Serial.println("Failed transmit during operation");
    }

  }
  // On a recieve dispatch the recieved message that associates to a ACANPrimaryFilterHandler
  ACAN_T4::can1.dispatchReceivedMessage();
}