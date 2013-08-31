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

#include "DueFlash.h"

DueFlash::DueFlash()
{
  uint32_t retCode;

  /* Initialize flash: 6 wait states for flash writing. */

  retCode = flash_init(FLASH_ACCESS_MODE_128, 6);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG("Flash init failed\n");
  }
}

boolean DueFlash::write(uint32_t *flashStart, uint32_t *data, uint32_t dataLength)
{
  uint32_t retCode;
  uint32_t byteLength = dataLength * sizeof(uint32_t);
  
  if ((uint32_t)flashStart < IFLASH1_ADDR) {
    _FLASH_DEBUG("Flash write address too low\n");
    return false;
  }

  if ((uint32_t)flashStart >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    _FLASH_DEBUG("Flash write address too high\n");
    return false;
  }

  if (((uint32_t)flashStart & 3) != 0) {
      _FLASH_DEBUG("Flash start address must be on four byte boundary\n");
      return false;
  }

  // Unlock page

  retCode = flash_unlock((uint32_t)flashStart, (uint32_t)flashStart + byteLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG("Failed to unlock flash for write\n");
    return false;
  }

  // write data

  retCode = flash_write((uint32_t)flashStart, data, byteLength, 1);

  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG("Flash write failed\n");
    return false;
  }

  // Lock page
 
  retCode = flash_lock((uint32_t)flashStart, (uint32_t)flashStart + byteLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    _FLASH_DEBUG("Failed to lock flash page\n");
    return false;
  }
  return true;
}
