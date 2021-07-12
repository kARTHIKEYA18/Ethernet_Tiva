/*
 * Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * - 10 Apr. 2015
 * Added support for Arduino Ethernet Shield 2
 * by Arduino.org team
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include <stdio.h>
#include <string.h>


#include "Ethernet_Tiva/src/utility/w5500.h"
//#if defined(W5500_ETHERNET_SHIELD)

// W5500 controller instance
W5500Class w5500;

// SPI details
//SPISettings wiznet_SPI_settings(8000000, MSBFIRST, SPI_MODE0);
//uint8_t SPI_CS;
void
InitConsol(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);


    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, 16000000);

    uint32_t ui32SysClock;
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                           SYSCTL_OSC_MAIN |
                                           SYSCTL_USE_OSC), 25000000);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,  SSI_MODE_MASTER, 1000000, 8);
}

void W5500Class::init(uint8_t ss_pin)
{
    InitConsol();
    SSIEnable(SSI0_BASE);
//  SPI_CS = ss_pin;
//
//  delay(1000);
//  initSS();
//  SPI.begin();
  w5500.swReset();
  for (int i=0; i<MAX_SOCK_NUM; i++) {
    uint8_t cntl_byte = (0x0C + (i<<5));
    write( 0x1E, cntl_byte, 2); //0x1E - Sn_RXBUF_SIZE
    write( 0x1F, cntl_byte, 2); //0x1F - Sn_TXBUF_SIZE
  }
}

uint16_t W5500Class::getTXFreeSize(SOCKET s)
{
    uint16_t val=0, val1=0;
    do {
        val1 = readSnTX_FSR(s);
        if (val1 != 0)
            val = readSnTX_FSR(s);
    } 
    while (val != val1);
    return val;
}

uint16_t W5500Class::getRXReceivedSize(SOCKET s)
{
    uint16_t val=0,val1=0;
    do {
        val1 = readSnRX_RSR(s);
        if (val1 != 0)
            val = readSnRX_RSR(s);
    } 
    while (val != val1);
    return val;
}

void W5500Class::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);

}

void W5500Class::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
    uint16_t ptr = readSnTX_WR(s);
    uint8_t cntl_byte = (0x14+(s<<5));
    ptr += data_offset;
    write(ptr, cntl_byte, data, len);
    ptr += len;
    writeSnTX_WR(s, ptr);
}

void W5500Class::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
    uint16_t ptr;
    ptr = readSnRX_RD(s);

    read_data(s, ptr, data, len);
    if (!peek)
    {
        ptr += len;
        writeSnRX_RD(s, ptr);
    }
}

void W5500Class::read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
{
    uint8_t cntl_byte = (0x18+(s<<5));
    read((uint16_t)src , cntl_byte, (uint8_t *)dst, len);
}

uint8_t W5500Class::write(uint16_t _addr, uint8_t _cb, uint8_t _data)
{
    SSIEnable(SSI0_BASE);
  SSIDataPut(SSI0_BASE,_addr >>8);
  SSIDataPut(SSI0_BASE,_addr & 0xFF);
  SSIDataPut(SSI0_BASE,_cb);
  SSIDataPut(SSI0_BASE,_data);
  SSIDisable(SSI0_BASE);
//    SPI.beginTransaction(wiznet_SPI_settings);
//    setSS();
//    SPI.transfer(_addr >> 8);
//    SPI.transfer(_addr & 0xFF);
//    SPI.transfer(_cb);
//    SPI.transfer(_data);
//    resetSS();
//    SPI.endTransaction();

    return 1;
}

uint16_t W5500Class::write(uint16_t _addr, uint8_t _cb, const uint8_t *_buf, uint16_t _len)
{
    uint32_t _buff= *_buf;
    SSIEnable(SSI0_BASE);
    SSIDataPut(SSI0_BASE,_addr >>8);
    SSIDataPut(SSI0_BASE,_addr & 0xFF);
    SSIDataPut(SSI0_BASE,_cb);
    SSIDataPut(SSI0_BASE,_buff);
    SSIDisable(SSI0_BASE);
//    SPI.beginTransaction(wiznet_SPI_settings);
//    setSS();
//    SPI.transfer(_addr >> 8);
//    SPI.transfer(_addr & 0xFF);
//    SPI.transfer(_cb);
//    for (uint16_t i=0; i<_len; i++){
//        SPI.transfer(_buf[i]);
//    }
//    resetSS();
//    SPI.endTransaction();

    return _len;
}

uint8_t W5500Class::read(uint16_t _addr, uint8_t _cb)
{
    SSIEnable(SSI0_BASE);
    SSIDataPut(SSI0_BASE,_addr >>8);
    SSIDataPut(SSI0_BASE,_addr & 0xFF);
    SSIDataPut(SSI0_BASE,_cb);
    uint32_t _data;
    SSIDataGet(SSI0_BASE, &_data);
    _data &= 0x00FF;
    SSIDisable(SSI0_BASE);
//    SPI.beginTransaction(wiznet_SPI_settings);
//    setSS();
//    SPI.transfer(_addr >> 8);
//    SPI.transfer(_addr & 0xFF);
//    SPI.transfer(_cb);
//    uint8_t _data = SPI.transfer(0);
//    resetSS();
//    SPI.endTransaction();

    return _data;
}

uint16_t W5500Class::read(uint16_t _addr, uint8_t _cb, uint8_t *_buf, uint16_t _len)
{ 
    uint32_t _buff;
    _buff=*_buf;
    SSIEnable(SSI0_BASE);
    SSIDataPut(SSI0_BASE,_addr >>8);
    SSIDataPut(SSI0_BASE,_addr & 0xFF);
    SSIDataPut(SSI0_BASE,_cb);
   SSIDataGet(SSI0_BASE, &_buff);
   SSIDisable(SSI0_BASE);
   return _len;

//    SPI.beginTransaction(wiznet_SPI_settings);
//    setSS();
//    SPI.transfer(_addr >> 8);
//    SPI.transfer(_addr & 0xFF);
//    SPI.transfer(_cb);
//    for (uint16_t i=0; i<_len; i++){
//        _buf[i] = SPI.transfer(0);
//    }
//    resetSS();
//    SPI.endTransaction();
}

void W5500Class::execCmdSn(SOCKET s, SockCMD _cmd) {
    // Send command to socket
    writeSnCR(s, _cmd);
    // Wait for command to complete
    while (readSnCR(s))
    ;
}


uint8_t W5500Class::readVersion(void)
{
    SSIEnable(SSI0_BASE);
    SSIDataPut( SSI0_BASE,0x00 );
    SSIDataPut( SSI0_BASE,0x39 );
    SSIDataPut( SSI0_BASE,0x01);
    uint32_t _data;
    SSIDataGet(SSI0_BASE,&_data);
    SSIDisable(SSI0_BASE);
//    SPI.beginTransaction(wiznet_SPI_settings);
//    setSS();
//    SPI.transfer( 0x00 );
//    SPI.transfer( 0x39 );
//    SPI.transfer( 0x01);
//    uint8_t _data = SPI.transfer(0);
//    resetSS();
//    SPI.endTransaction();

    return _data;
}


//#endif
