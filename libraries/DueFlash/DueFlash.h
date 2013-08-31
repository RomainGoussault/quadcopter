////////////////////////////////////////////////////////////////////////////
//
//  This file is part of DueFlash
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef DUEFLASH_H
#define DUEFLASH_H

#include <Arduino.h>
#include "flash_efc.h"

//  DueFlash supports saving of non-volatile data for Arduino Due sketches.
//
//  All writes are in flash block 1 and are always multiples of 32 bits. They
//  can start on any 32 bit boundary withing flash block 1.
//
//  Note: uploading new software will erase all flash so data written to flash
//  using this library will not survive a new software upload.
//

//  FLASH_DEBUG can be enabled to get debugging information displayed.

#define FLASH_DEBUG

#ifdef FLASH_DEBUG
#define _FLASH_DEBUG(x) Serial.print(x);
#else
#define _FLASH_DEBUG(x)
#endif

//  DueFlash is the main class for flash functions

class DueFlash
{
public:
  
  // Constructor does basic initialization

  DueFlash();
  
  // write() writes the specified amount of data into flash.
  // flashStart is the address in memory where the write should start
  // data is a pointer to the data to be written (multiple of 32 bits)
  // dataLength is length of data in units of 32 bits

  boolean write(uint32_t *flashStart, uint32_t *data, uint32_t dataLength);
};

#endif // DUEFLASH_H
