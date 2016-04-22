// Sub8 Actuator Board Software
// Designed for Actuator Boards R3 - 2016
// Ported to mikroC PRO for ARM and Modified by Daniel Dugger
// Originally Written by Khaled Hassan

void uart_setup() {
     // UART1 (Clock is divided by 2 before entering UART, baud needs to be multiplied by 2 (9600 * 2 = 19200))
     // Serial Settings - Baud Rate: 9600, Data Bits: 8, Parity: None, Stop Bits: 1, Flow Control: None
     UART1_Init_Advanced(19200, _UART_8_BIT_DATA, _UART_NOPARITY, _UART_ONE_STOPBIT, &_GPIO_MODULE_USART1_PA9_10);
     
     // UART1_TX_EN (25-B12)
     GPIO_Digital_Output(&GPIOB_BASE, _GPIO_PINMASK_12);
}
void gpio_setup() {
     // Valves
     // V1 (14-A4), V2 (15-A5), V3 (16-A6), V4 (17-A7), V9 (29-A8)
     GPIO_Digital_Output(&GPIOA_BASE, _GPIO_PINMASK_4 | _GPIO_PINMASK_5 | _GPIO_PINMASK_6 | _GPIO_PINMASK_7 | _GPIO_PINMASK_8);
     // V5 (19-B1), V6 (20-B2), V7 (28-B15), V8 (26-B13), V10 (27-B14), V11 (22-B11), V12 (21-B10)
     GPIO_Digital_Output(&GPIOB_BASE, _GPIO_PINMASK_1 | _GPIO_PINMASK_2 | _GPIO_PINMASK_15 | _GPIO_PINMASK_13 | _GPIO_PINMASK_14 | _GPIO_PINMASK_11 | _GPIO_PINMASK_10);
     
     // Switches
     // S1 (10-A0), S2 (11-A1), S3 (12-A2), S4 (13-A3)
     GPIO_Digital_Input(&GPIOA_BASE, _GPIO_PINMASK_0 | _GPIO_PINMASK_1 | _GPIO_PINMASK_2 | _GPIO_PINMASK_3);
     // S5 (18-B0), S6 (41-B5), S7 (40-B4), S8 (39-B3), S9 (46-B9), S10 (45-B8), S11 (43-B7), S12 (42-B6)
     GPIO_Digital_Input(&GPIOB_BASE, _GPIO_PINMASK_0 | _GPIO_PINMASK_5 | _GPIO_PINMASK_4 | _GPIO_PINMASK_3 | _GPIO_PINMASK_9 | _GPIO_PINMASK_8 | _GPIO_PINMASK_7 | _GPIO_PINMASK_6);
}
unsigned short rs485_read() {
     while (1) {
          if (UART1_Data_Ready() == 1) {
                return UART1_Read();
          }
     }
}
unsigned short read() {
     unsigned short b1 = rs485_read();
     while (1) {
           unsigned short b2 = rs485_read();
           if (b2 == (b1 ^ 0xFF)) {
                 return b1;
           }
           b1 = b2;
     }
}
void rs485_write(unsigned short cmd) {
     GPIOB_ODR.B12 = 1;
     UART1_Write(cmd);
     while (UART1_Tx_Idle() == 0) {
           GPIOB_ODR.B12 = 1;
     }
     GPIOB_ODR.B12 = 0;
     UART1_Read();
}
void write(unsigned short cmd) {
     rs485_write(cmd); // Send byte
     rs485_write(cmd ^ 0xFF); // Send checksum
}
void open_valve(unsigned short cmd) {
    if (cmd == 0x21) { // 0x21 Valve 1 OPEN (0xDE)
        GPIOA_ODR.B4 = 1;
    } else if (cmd == 0x22) { // 0x22 Valve 2 OPEN (0xDD)
        GPIOA_ODR.B5 = 1;
    } else if (cmd == 0x23) { // 0x23 Valve 3 OPEN (0xDC)
        GPIOA_ODR.B6 = 1;
    } else if (cmd == 0x24) { // 0x24 Valve 4 OPEN (0xDB)
        GPIOA_ODR.B7 = 1;
    } else if (cmd == 0x25) { // 0x25 Valve 5 OPEN (0xDA)
        GPIOB_ODR.B1 = 1;
    } else if (cmd == 0x26) { // 0x26 Valve 6 OPEN (0xD9)
        GPIOB_ODR.B2 = 1;
    } else if (cmd == 0x27) { // 0x27 Valve 7 OPEN (0xD8)
        GPIOB_ODR.B15 = 1;
    } else if (cmd == 0x28) { // 0x28 Valve 8 OPEN (0xD7)
        GPIOB_ODR.B13 = 1;
    } else if (cmd == 0x29) { // 0x29 Valve 9 OPEN (0xD6)
        GPIOA_ODR.B8 = 1;
    } else if (cmd == 0x2A) { // 0x2A Valve 10 OPEN (0xD5)
        GPIOB_ODR.B14 = 1;
    } else if (cmd == 0x2B) { // 0x2B Valve 11 OPEN (0xD4)
        GPIOB_ODR.B11 = 1;
    } else if (cmd == 0x2C) { // 0x2C Valve 12 OPEN (0xD3)
        GPIOB_ODR.B10 = 1;
    } else {
        // something went REALLY wrong
    }
}
void close_valve(unsigned short cmd) {
    if (cmd == 0x31) { // 0x31 Valve 1 CLOSE (0xCE)
        GPIOA_ODR.B4 = 0;
    } else if (cmd == 0x32) { // 0x32 Valve 2 CLOSE (0xCD)
        GPIOA_ODR.B5 = 0;
    } else if (cmd == 0x33) { // 0x33 Valve 3 CLOSE (0xCC)
        GPIOA_ODR.B6 = 0;
    } else if (cmd == 0x34) { // 0x34 Valve 4 CLOSE (0xCB)
        GPIOA_ODR.B7 = 0;
    } else if (cmd == 0x35) { // 0x35 Valve 5 CLOSE (0xCA)
        GPIOB_ODR.B1 = 0;
    } else if (cmd == 0x36) { // 0x36 Valve 6 CLOSE (0xC9)
        GPIOB_ODR.B2 = 0;
    } else if (cmd == 0x37) { // 0x37 Valve 7 CLOSE (0xC8)
        GPIOB_ODR.B15 = 0;
    } else if (cmd == 0x38) { // 0x38 Valve 8 CLOSE (0xC7)
        GPIOB_ODR.B13 = 0;
    } else if (cmd == 0x39) { // 0x39 Valve 9 CLOSE (0xC6)
        GPIOA_ODR.B8 = 0;
    } else if (cmd == 0x3A) { // 0x3A Valve 10 CLOSE (0xC5)
        GPIOB_ODR.B14 = 0;
    } else if (cmd == 0x3B) { // 0x3B Valve 11 CLOSE (0xC4)
        GPIOB_ODR.B11 = 0;
    } else if (cmd == 0x3C) { // 0x3C Valve 12 CLOSE (0xC3)
        GPIOB_ODR.B10 = 0;
    } else {
        // something went REALLY wrong
    }
}
unsigned short read_switch(unsigned short cmd)
{
    if (cmd == 0x41) { // 0x41 Switch 1 (0xBE)
        if (!GPIOA_IDR.B0) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x42) { // 0x42 Switch 2 (0xBD)
        if (!GPIOA_IDR.B1){
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x43) { // 0x43 Switch 3 (0xBC)
        if (!GPIOA_IDR.B2) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x44) { // 0x44 Switch 4 (0xBB)
        if (!GPIOA_IDR.B3) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x45) { // 0x45 Switch 5 (0xBA)
        if (!GPIOB_IDR.B0) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x46) { // 0x46 Switch 6 (0xB9)
        if (!GPIOB_IDR.B5) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x47) { // 0x47 Switch 7 (0xB8)
        if (!GPIOB_IDR.B4) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x48) { // 0x48 Switch 8 (0xB7)
        if (!GPIOB_IDR.B3) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x49) { // 0x49 Switch 9 (0xB6)
        if (!GPIOB_IDR.B9) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x4A) { // 0x4A Switch 10 (0xB5)
        if (!GPIOB_IDR.B8) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x4B) { // 0x4B Switch 11 (0xB4)
        if (!GPIOB_IDR.B7) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else if (cmd == 0x4C) { // 0x4C Switch 12 (0xB3)
        if (!GPIOB_IDR.B6) {
            return 0x01;
        } else {
            return 0x00;
        }
    } else {
        return 0xEE; // something went REALLY wrong
    }
}
void main() {
     uart_setup();
     gpio_setup();
     while (1) {
          unsigned short cmd = read();
          if (cmd == 0x10) { // ping (0xEF)
                write(0x11);
          } else if ((cmd >= 0x21) && (cmd <= 0x2C)) { // open valve (allow air flow)
                open_valve(cmd);
                write(0x01);
          } else if ((cmd >= 0x31) && (cmd <= 0x3C)) { // close valve (prevent air flow)
                close_valve(cmd);
                write(0x00);
          } else if ((cmd >= 0x41) && (cmd <= 0x4C)) { // read switch
                write(read_switch(cmd));
          } else { // unknown command
                write(0xEE);
          }
     }
}