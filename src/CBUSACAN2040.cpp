
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

  // allocate tx and tx buffers -- tx is currently unused
  rx_buffer = new circular_buffer(_num_rx_buffers);
  tx_buffer = new circular_buffer(_num_tx_buffers);

  acan2040 = new ACAN2040(0, _gpio_tx, _gpio_rx, CANBITRATE, F_CPU, cb);
  acan2040->begin();
  return true;
}

//
/// callback
//

//
/// check for one or more messages in the receive buffer
//

bool CBUSACAN2040::available(void) {
  return rx_buffer->available();
}

//
/// must call available() first to ensure a message is available in the buffer
//

CANFrame CBUSACAN2040::getNextMessage(void) {

  CANFrame cf;

  ++_numMsgsRcvd;
  memcpy((CANFrame *)&cf, rx_buffer->get(), sizeof(CANFrame));
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

    _msg.id = amsg->id;
    _msg.len = amsg->dlc;

    for (uint8_t i = 0; i < _msg.len && i < 8; i++) {
      _msg.data[i] = amsg->data[i];
    }

    _msg.rtr = amsg->id & CAN2040_ID_RTR;
    _msg.ext = amsg->id & CAN2040_ID_EFF;

    rx_buffer->put(&_msg);
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

  if ((ok = acan2040->ok_to_send())) {
    tx_msg.id = msg->id;
    tx_msg.dlc = msg->len;

    if (msg->rtr)
      tx_msg.id |= 0x40000000;

    if (msg->ext)
      tx_msg.id |= 0x80000000;

    for (uint8_t i = 0; i < msg->len && i < 8; i++) {
      tx_msg.data[i] = msg->data[i];
    }

    ok = acan2040->send_message(&tx_msg);
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
  delete rx_buffer;
  delete tx_buffer;
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

void CBUSACAN2040::setNumBuffers(byte num_rx_buffers, byte num_tx_buffers) {
  _num_rx_buffers = num_rx_buffers;
  _num_tx_buffers = num_tx_buffers;
}


///
/// a circular buffer class
///

/// constructor and destructor

circular_buffer::circular_buffer(byte num_items) {

  _head = 0;
  _tail = 0;
  _hwm = 0;
  _capacity = num_items;
  _size = 0;
  _puts = 0;
  _gets = 0;
  _overflows = 0;
  _full = false;
  _buffer = (buffer_entry_t *)malloc(num_items * sizeof(buffer_entry_t));
}

circular_buffer::~circular_buffer() {
  free(_buffer);
}

/// if buffer has one or more stored items

bool circular_buffer::available(void) {

  return (_size > 0);
}

/// store an item to the buffer - overwrite oldest item if buffer is full
/// only called from an interrupt context so we don't need to worry about subsequent interrupts

void circular_buffer::put(const CANFrame *item) {

  memcpy((CANFrame*)&_buffer[_head]._item, (const CANFrame *)item, sizeof(CANFrame));
  _buffer[_head]._item_insert_time = micros();

  // if the buffer is full, this put will overwrite the oldest item

  if (_full) {
    _tail = (_tail + 1) % _capacity;
    ++_overflows;
  }

  _head = (_head + 1) % _capacity;
  _full = _head == _tail;
  _size = size();
  _hwm = (_size > _hwm) ? _size : _hwm;
  ++_puts;

  return;
}

/// retrieve the next item from the buffer

CANFrame *circular_buffer::get(void) {

  CANFrame *p = nullptr;

  // should always call ::available first to avoid returning null pointer

  // protect against changes to the buffer by suspending interrupts

  if (_size > 0) {
    p = &_buffer[_tail]._item;
    _full = false;
    _tail = (_tail + 1) % _capacity;
    _size = size();
    ++_gets;
  }

  return p;
}

/// get the insert time of the current buffer tail item
/// must be called before the item is removed by ::get

unsigned long circular_buffer::insert_time(void) {

  return (_buffer[_tail]._item_insert_time);
}

/// peek at the next item in the buffer without removing it

CANFrame *circular_buffer::peek(void) {

  // should always call ::available first to avoid this

  if (_size == 0) {
    return nullptr;
  }

  return (&_buffer[_tail]._item);
}

/// clear all items

void circular_buffer::clear(void) {

  _head = 0;
  _tail = 0;
  _full = false;
  _size = 0;

  return;
}

/// return high water mark

byte circular_buffer::hwm(void) {

  return _hwm;
}

/// return full indicator

bool circular_buffer::full(void) {

  return _full;
}

/// recalculate number of items in the buffer

byte circular_buffer::size(void) {

  byte size = _capacity;

  if (!_full) {
    if (_head >= _tail) {
      size = _head - _tail;
    } else {
      size = _capacity + _head - _tail;
    }
  }

  _size = size;
  return _size;
}

/// return empty indicator

bool circular_buffer::empty(void) {

  return (!_full && (_head == _tail));
}

/// return number of free slots

byte circular_buffer::free_slots(void) {

  return (_capacity - _size);
}

/// number of puts

unsigned int circular_buffer::puts(void) {

  return _puts;
}

/// number of gets

unsigned int circular_buffer::gets(void) {

  return _gets;
}

/// number of overflows

unsigned int circular_buffer::overflows(void) {

  return _overflows;
}
