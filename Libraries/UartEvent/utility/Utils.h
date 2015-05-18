/*
 ||
 || @file 	    Utils.h
 || @version 	6.1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || |
 || #
 ||
 || @license
 || | Copyright (c) 2015 Colin Duffy
 || | This library is free software; you can redistribute it and/or
 || | modify it under the terms of the GNU Lesser General Public
 || | License as published by the Free Software Foundation; version
 || | 2.1 of the License.
 || |
 || | This library is distributed in the hope that it will be useful,
 || | but WITHOUT ANY WARRANTY; without even the implied warranty of
 || | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 || | Lesser General Public License for more details.
 || |
 || | You should have received a copy of the GNU Lesser General Public
 || | License along with this library; if not, write to the Free Software
 || | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 || #
 ||
 */

#ifndef Utils_h
#define Utils_h

#define BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define BITBAND_U32(reg, bit) (*(uint32_t *)BITBAND_ADDR((reg), (bit)))

#define UART_DMA_ENABLE     UART_C5_TDMAS | UART_C5_RDMAS
#define UART_DMA_DISABLE    0
#define IRQ_PRIORITY        64  // 0 = highest priority, 255 = lowest

#define UART_C5_TDMAS       (uint8_t)0x80
#define UART_C5_RDMAS       (uint8_t)0x20

#define C2_ENABLE           UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TIE
#define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE		C2_ENABLE

#define likely(x)           __builtin_expect(!!(x), 1)
#define unlikely(x)         __builtin_expect(!!(x), 0)

typedef struct __attribute__((packed)) {
    volatile int        term_rx_character;
    volatile char       *term_rx_string;
    volatile uintptr_t  *currentptr_rx;
    volatile int16_t    priority;
} event_t;
#endif
