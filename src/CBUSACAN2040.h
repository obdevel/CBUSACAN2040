
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

#pragma once

// header files

#include <CBUS.h>               // abstract base class
#include <ACAN2040.h>           // header for CAN driver

// constants

static const byte tx_qsize = 16;
static const byte rx_qsize = 64;
static const byte txpin = 1;
static const byte rx_pin = 2;
static const uint32_t CANBITRATE = 125000UL;                // 125Kb/s - fixed for CBUS

/// class definitions

//
/// an implementation of the abstract base CBUS class
/// using the ACAN2040 wrapper of the PIO CAN controller
//

class CBUSACAN2040 : public CBUSbase {

public:

  CBUSACAN2040();
  CBUSACAN2040(CBUSConfig *the_config);
  ~CBUSACAN2040();

  // these methods are declared virtual in the base class and must be implemented by the derived class
  bool begin(bool poll = false, SPIClassRP2040& spi = SPI);    // note default args
  bool available(void);
  CANFrame getNextMessage(void);
  bool sendMessage(CANFrame * msg, bool rtr = false, bool ext = false, byte priority = DEFAULT_PRIORITY);   // note default arguments
  bool sendMessageNoUpdate(CANFrame *msg);
  void reset(void);

  // these methods are specific to this implementation
  // they are not declared or implemented by the base CBUS class
  void setNumBuffers(unsigned int num_rx_buffers, unsigned int _num_tx_buffers = 2);
  void setPins(byte tx_pin, byte rx_pin);
  void printStatus(void);
  void notify_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *amsg);

  ACAN2040 *acan2040;
  queue_t *tx_queue, *rx_queue;

private:
  void initMembers(void);
  byte _gpio_tx, _gpio_rx;
  unsigned int _num_tx_buffers, _num_rx_buffers;
};

