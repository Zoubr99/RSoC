// this is the header file for the write and read 


// base adress of slot n = bridge_base_address + n * 32 * 4
// therfore slot 3 = 0xc0000180

// starting at:

/**********************************************/
// int sw;                                     /
// sw = * (volatile uint32_t *) (0xc0000180);  /  typecasting
/**********************************************/

//first (*) is a derefrencing operator which gives
// the value stored in the adress the pointer is pointing to , (volatile uint32_t *) is basically a pointer


// now rather than using the adress of the slot it would be better to use 

/***********************************************************/
// #define sw_base      0xc0000180                          /
// #define data_reg     0                                   /
// sw = * (volatile uint32_t *) (sw_base + 4 * data_reg)    /
/***********************************************************/

// now for readability and useability it is better to encapsulate the previous code into a macro

/***************************************************************************************************/
// #define io_read(base_addr, offset)   ( *(volatile uint32_t *) ( (base_addr) + 4 * (offset) ) )   / 
   
// #define sw_base      0xc0000180                                                                  /                              /
// #define data_reg     0                                                                           /

// the prev code becomes :                                                                          /

// sw = io_read(sw_base, offset)                                                                    /
/***************************************************************************************************/

// another macro needed for calculating the base address of any given slot
// using the formula above -> base adress of slot n = bridge_base_address + n * 32 * 4


/***************************************************************************************************/
//                                                                                                  /
// #define get_slot_addr(mmio_base, n_slot)     ( (uint32_t) ((mmio_base) + (n_slot) * 32 * 4) )    /
//                                                                                                  /
/***************************************************************************************************/

// a similar macro as the one used for read will be created for the write function

/*******************************************************************************************************************/
// #define io_write(base_addr, offset, data)   ( *(volatile uint32_t *) ( (base_addr) + 4 * (offset) )  = (data) )  / 
//                                                                                                                  /
// *(0xc0000100) = data becomes :                                                                                   /
//                                                                                                                  /
// #include io_map.h                                                                                                /
// #define led_base     get_slot_address(BRIDGE_BASE, S2_LED)                                                       /                                                    
// #define data_reg     0                                                                                           / 
// io_write(led_base, data_reg, data)                                                                               /
/*******************************************************************************************************************/

// now summarizing all the notes into a c header file

#ifndef _IO_RW_H_INCLUDED
#define _IO_RW_H_INCLUDED

#include <inttypes.h> // to use unitN_t type

#ifdef __cplusplus
extern "C"{
#endif

// read macro
#define io_read(base_addr, offset)          ( *(volatile uint32_t *) ( (base_addr) + ((offset)<< 2) ) )

//write macro
#define io_write(base_addr, offset, data)   ( *(volatile uint32_t *) ( (base_addr) + ((offset)<< 2) )  = (data) )

//slot addr macro
#define get_slot_addr(mmio_base, n_slot)    ( (uint32_t) ((mmio_base) + ((n_slot) << 7) ) )

#ifdef __cplusplus
} // extern "C"
#endif



#endif