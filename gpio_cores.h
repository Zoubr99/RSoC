#ifndef _GPIO_H_INCLUDED
#define _GPIO_H_INCLUDED

// header file for gpio class defenition

// first GpoCore

class GpoCore 
{

    // register offset
    enum{
        DATA_REG = 0

    };

    private:

        uint32_t base_addr;
        uint32_t wr_data;

    
    public:
    // Constructor and Destructure
    GpoCore(uint32_t core_base_addr);
    ~GpoCore();

    // function overloading
    void write(uint32_t data);
    void write(int bit_value, int bit_pos);

};


// now the GpiCore

class GpiCore
{
    // register offset
    enum{

        DATA_REG = 0
    };



    private:
        uint32_t base_addr;

    public:
        // Constructor and Destructure
        GpiCore(uint32_t core_base_adde);
        ~GpiCore();

        //methods
        uint32_t read(); // read a 32 bit word
        int read(int bit_pos);




};

#endif