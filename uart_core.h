#ifndef _UART_CORE_H_INCLIDED
#define _UART_CORE_H_INCLUDED

#include "io_rw.h"
// transmit/receive data via MMIO uart core.
// display (print) number and string on serial console

#include "io_map.h" 
// to use SYS_CLK_FREQ

class UartCore {

        // first implemnt the registers map
        // 4 register, each 32 bit, devided into sections each serves specefic purpose
        enum 
        {  
            //r
            RD_DATA_REG = 0, // very first reg, includes all the reading processes (tx_full[9]-rx_full[8]-r_data[7:0](rx))
            //w
            DVSR_REG = 1, // baud rate divisor reg (dvsr[10:0])
            //w
            WRR_DATA_REG = 2, // writting (tx) data reg [7:0] 
            //w
            RM_RD_DATA_REG = 3 // reg to dump data into so the fifo regs could be emptied
        };

        // masking the first register as it is being devided into diffrent sections
        enum
        { 
            // bit 9 of the rd_data_reg this bit indicates wether the transmitter buffer is full or not
            TX_FULL_FIELD = 0x00000200,
            // bit 10 of the rd_data_reg this bit indicates wether the reciever buffer is empty or not
            RX_EMPTY_FIELD = 0x00000100,
            // // bit 7:0 received bits (rx bits)
            RX_DATA_FIELD = 0x000000ff
        };

    private:

        uint32_t base_addr;
        int baud_rate;
        void disp_str(const char *str);



    public:

        // first method is as always the condt and destruct.
        UartCore(uint32_t core_base_addr);
        UartCore();


        //******[baud rate method]******// 
        // **** REG 2 ***//
        void set_baud_rate(int baud);


        //******[UART reciever FiFo empty check method]******//
        // 1 = empty, 0 = otherwise
        // **** REG 0 ***//
        int rx_fifo_empty();


        //******[UART transmitter FiFo full check method]******//
        // 1 = full, 0 = otherwise.
        // **** REG 0 ***//
        int tx_fifo_full();


        //******[function to write the bytes to be transmitted into the transmission buffer]******//
        // **** REG 2 ***//
        void tx_byte(uint8_t byte);


        //******[function to read the received bytes from the receiver buffer]******//
        // **** REG 0 ***//
        void rx_byte(uint8_t byte);


        // below are funtions to help the end user (application dev to easily display chars on the terminal)
        // they are 7 overloaded functions each serves a diffrent task


        //*********** char and string display functions ***********//

        void disp (char ch); // display char on the serial moniter 
        // ch : is the char to be displayed

        void disp (const char* str); // display string on the serial moniter
        // str : a pointer to the string to be displayed

        //***********************************************//


        //*********** Integer display functions ***********//

        void disp (int n, int base, int len); // display an integer on the terminal console
        // n : integer to be displayed
        // base : 2/8/10/16 ->  for binary/octal/decimal/hex format
        // len : number of digits to be displayed (lenght)
        // padding blank spaces are added if printed digits smaller than len;
        // if len=0, # digits determined automatically without blanks

        void disp (int n, int base); // display an integer on the terminal console
        // n : integer to be displayed
        // base : 2/8/10/16 ->  for binary/octal/decimal/hex format
        // ( auto ) -  # digits determined automatically without blanks (len = 0)

        void disp (int n); // display an integer on the terminal console
        // n : integer to be displayed
        // ( auto ) -  base 10 is used
        // ( auto ) -  # digits determined automatically without blanks (len = 0)

        //***********************************************//


        //***** Floating-points display functions ******//

        void disp (double f, int digit); // to display a floating point number on the serial terminal
        // f : floating point number to be displayed
        // digit : number of digits in fraction portion to be displayed
        // ( auto ) - base 10 is used auto
        // ( auto ) -  lenght is auto

        void disp (double f); // to display a floating point number on the serial terminal
        // f : floating point number to be displayed
        // ( auto ) - 3 digits in fraction portion to be displayed
        // ( auto ) - base 10 is used auto
        // ( auto ) - lenght is auto

        //***********************************************//
};

#endif
