
/*

  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

// 3rd party libraries
#include <Streaming.h>

// CBUS device-specific library
#include <CBUSACAN2040.h>

// static pointer to object
CBUSACAN2040 *acan2040p;

// static callback function
static void cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
  acan2040p->notify_cb(cd, notify, msg);
}

//
/// constructor and destructor
//

CBUSACAN2040::CBUSACAN2040() {
  initMembers();
}

CBUSACAN2040::CBUSACAN2040(CBUSConfig *the_config) : CBUSbase(the_config) {
  initMembers();
}

void CBUSACAN2040::initMembers(void) {
  _num_rx_buffers = rx_qsize;
  _num_tx_buffers = tx_qsize;
  eventhandler = NULL;
  eventhandlerex = NULL;
  framehandler = NULL;
  _gpio_tx = 0;
  _gpio_rx = 0;

  acan2040p = this;
}

CBUSACAN2040::~CBUSACAN2040() {
}

//
/// initialise the CAN controller and buffers, and attach the ISR
/// default poll arg is set to false, so as not to break existing code
//

bool CBUSACAN2040::begin(bool poll, SPIClassRP2040& spi) {

  (void)(spi);      // not used
  (void)poll;       // not used

  // allocate tx and tx buffers
  queue_init(&tx_queue, sizeof(struct can2040_msg), _num_tx_buffers);
  queue_init(&rx_queue, sizeof(struct can2040_msg), _num_rx_buffers);

  acan2040 = new ACAN2040(0, _gpio_tx, _gpio_rx, CANBITRATE, F_CPU, cb);
  acan2040->begin();
  return true;
}

//
/// callback
//

//
/// attempt to send any buffered messages in the tx buffer
/// check for one or more messages in the receive buffer
//

bool CBUSACAN2040::available(void) {

  CANFrame cf;
  struct can2040_msg tx_msg;

  /// attempt to drain down the tx buffer
  /// do this here as it's the only function that is called sufficiently frequently

  while (acan2040->ok_to_send() && queue_try_remove(&tx_queue, &tx_msg)) {
    acan2040->send_message(&tx_msg);
  }

  /// check for new received messages

  return (!queue_is_empty(&rx_queue));
}

//
/// must call available() first to ensure a message is available in the buffer
//

CANFrame CBUSACAN2040::getNextMessage(void) {

  CANFrame cf;
  struct can2040_msg rx_msg;

  if (queue_try_remove(&rx_queue, &rx_msg)) {

    cf.id = rx_msg.id;
    cf.len = rx_msg.dlc;

    for (byte i = 0; i < rx_msg.dlc; i++) {
      cf.data[i] = rx_msg.data[i];
    }

    cf.rtr = (rx_msg.id & CAN2040_ID_RTR);
    cf.ext = (rx_msg.id & CAN2040_ID_EFF);

    ++_numMsgsRcvd;
  }

  return cf;
}

//
/// callback
//

void CBUSACAN2040::notify_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *amsg) {

  (void)(cd);

  switch (notify) {
  case CAN2040_NOTIFY_RX:
    // DEBUG_SERIAL.printf("acan2040 cb: message received\n");
    queue_try_add(&rx_queue, amsg);
    break;

  case CAN2040_NOTIFY_TX:
    // DEBUG_SERIAL.printf("acan2040 cb: message sent ok\n");
    break;
  case CAN2040_NOTIFY_ERROR:
    // DEBUG_SERIAL.printf("acan2040 cb: an error occurred\n");
    break;
  default:
    // DEBUG_SERIAL.printf("acan2040 cb: unknown event type\n");
    break;
  }

  return;
}

//
/// send a CBUS message
//

bool CBUSACAN2040::sendMessage(CANFrame *msg, bool rtr, bool ext, byte priority) {

  // caller must populate the message data
  // this method will create the correct frame header (CAN ID and priority bits)
  // rtr and ext default to false unless arguments are supplied - see method definition in .h
  // priority defaults to 1011 low/medium

  bool ok;

  // format message and send

  msg->rtr = rtr;
  msg->ext = ext;
  makeHeader(msg, priority);                      // default priority unless user overrides
  ok = sendMessageNoUpdate(msg);                  // send the CAN message

  // call user transmit handler
  if (transmithandler != nullptr) {
    (void)(*transmithandler)(msg);
  }

  return ok;
}

//
/// send a CAN message with no further changes to header and priority
//

bool CBUSACAN2040::sendMessageNoUpdate(CANFrame *msg) {

  bool ok;
  struct can2040_msg tx_msg;

  /// send message if can2040 can accept it, otherwise buffer it in the tx queue

  tx_msg.id = msg->id;
  tx_msg.dlc = msg->len;

  if (msg->rtr)
    tx_msg.id |= 0x40000000;

  if (msg->ext)
    tx_msg.id |= 0x80000000;

  for (uint8_t i = 0; i < msg->len && i < 8; i++) {
    tx_msg.data[i] = msg->data[i];
  }

  if ((ok = acan2040->ok_to_send())) {
    ok = acan2040->send_message(&tx_msg);
  } else {
    ok = queue_try_add(&tx_queue, &tx_msg);
  }

  return ok;
}

//
/// display the CAN bus status instrumentation
//

void CBUSACAN2040::printStatus(void) {
  // DEBUG_SERIAL.printf("not implemented");
  return;
}

//
/// reset the CAN transceiver
//

void CBUSACAN2040::reset(void) {
  queue_free(&rx_queue);
  queue_free(&tx_queue);
  delete acan2040;
  begin();
}

//
/// set the CS and interrupt pins - option to override defaults
/// used as CANL and CANH in this library
//

void CBUSACAN2040::setPins(byte gpio_tx, byte gpio_rx) {
  _gpio_tx = gpio_tx;
  _gpio_rx = gpio_rx;
}

//
/// set the number of CAN frame receive buffers
/// this can be tuned according to bus load and available memory
//

void CBUSACAN2040::setNumBuffers(unsigned int num_rx_buffers, unsigned int num_tx_buffers) {
  _num_rx_buffers = num_rx_buffers;
  _num_tx_buffers = num_tx_buffers;
}

