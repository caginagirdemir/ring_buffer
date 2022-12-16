
// mask values for uart interrupt register (mask, status and acknowledge)
#define UART_INT_TX 0x08 //transmit interrupt
#define UART_INT_RX 0x04 //receive interrupt

// mask values for uart registers (status and control)
#define DATA_READY 0x10 //data received in receive register
#define XMT_FIFO_FULL 0x40 //transmit fifo (hardware) is full
#define XMT_FIFO_EMPTY 0x80 //transmit fifo (hardware) is empty
#define UART_BUF_SIZE 1024 //size of fifo buffer (software)

struct interrupt_registers //uart interrupt hardware registers
{
    unsigned char *intr_mask; // interrupt mask
    unsigned char *intr_status; // source of interrupts
    unsigned char *intr_ack; // interrupt acknowledge
};

struct uart_registers //uart hardware registers
{
    unsigned int *control; // serial configuration reg
    unsigned int *status; // status
    unsigned char *tx_data; // transmit data (with hardware fifo!)
    unsigned char *rx_data; // rcv data (with hardware fifo!)
};

struct uart_registers *uart_registers_ptr; //pointer to uart hardware registers
struct interrupt_registers *interrupt_registers_ptr;//pointer to interrupt hardware registers
unsigned char uart_tx[UART_BUF_SIZE]; //transmit fifo buffer
unsigned char uart_rx[UART_BUF_SIZE]; //receive fifo buffer


//fifo control structure
struct _arm_uart_buf {
    unsigned char *beg; //begin of fifo buffer
    unsigned char *end; //end of fifo buffer
    unsigned char *read_ptr; //pointer to oldest item in fifo
    unsigned char *write_ptr; //pointer to begin of unused buffer in fifo
} arm_uart_bufs[2]; //two fifos for transmit and receive data

#define TX_FIFO 0 //first fifo is for transmit data
#define RX_FIFO 1 //second fifo is for receive data

//application interface function:
//initialisation of transmit and receive fifo control pointers and hardware pointers
void init_arm_uart_bufs(void)
{
    int i;
    struct _arm_uart_buf * sptr;
    uart_registers_ptr->control = (unsigned int *)0x80030000;
    uart_registers_ptr->status = (unsigned int *)0x80030002;
    uart_registers_ptr->tx_data = (unsigned char *)0x80030004;
    uart_registers_ptr->rx_data = (unsigned char *)0x80030006;
    interrupt_registers_ptr->intr_mask = (unsigned char *)0x800a002c;
    interrupt_registers_ptr->intr_status = (unsigned char *)0x800a0030;
    interrupt_registers_ptr->intr_ack = (unsigned char *)0x800a0034;
    for ( i = 0; i < 2; i++)
    {
        sptr = &arm_uart_bufs[i];
        switch (i)
        {
            case 0:
            sptr->beg = sptr->write_ptr = sptr->read_ptr = uart_tx;
            sptr->end = uart_tx + UART_BUF_SIZE;
            break;
            case 1:
            sptr->beg = sptr->write_ptr = sptr->read_ptr = uart_rx;
            sptr->end = uart_rx + UART_BUF_SIZE;
            break;
            }
        }
}

//application interface function for sending len characters from buf
int uart_send(unsigned char * buf, int len)
{
    unsigned char * txptr;
while ( len --) //put len characters in fifo
{
    txptr = next_ptr(TX_FIFO); //get pointer to next write item in
    transmit fifo (no overrun check!)
    *txptr= *buf++; //write item to transmit fifo
}
*uart_registers_ptr->control |= XMT_FIFO_EMPTY; //enable/generate transmit empty
intrerrupt
*interrupt_registers_ptr->intr_mask |= UART_INT_TX; //enable transmit interrupt->interrupt
calls uart_irq_interrupt()
return(0);
}

//application interface function to read item from receive fifo (in polling mode!)
//and subroutine to read item from transmit fifo
int bufRead(unsigned char * buf, int index)
{
    //is receive fifo empty?
    if ( arm_uart_bufs[index].read_ptr == arm_uart_bufs[index].write_ptr)
    {
        //fifo is empty
        *buf = *arm_uart_bufs[index].read_ptr; //read item (dummy!)
        *arm_uart_bufs[index].read_ptr = 0x0; //reset item
        return(0); //return and signal fifo empty
    }
    //fifo is not empty
    *buf = *arm_uart_bufs[index].read_ptr; //read item
    *arm_uart_bufs[index].read_ptr = 0x0; //reset item
    //is read pointer at the end?
    if( arm_uart_bufs[index].read_ptr == arm_uart_bufs[index].end) //wrap around read pointer to begin
        arm_uart_bufs[index].read_ptr = arm_uart_bufs[index].beg;
    else //increment read pointer to next item
        arm_uart_bufs[index].read_ptr++;
    return(1);
}

//interrupt service routine: will be called by receive or transmit interrupt
void uart_irq_interrupt(unsigned long vector)
{
    unsigned char status;
    status = *interrupt_registers_ptr->intr_status; //read interrupt status register
    if ( status & UART_INT_RX ) //receive interrupt happened?
    {
        *interrupt_registers_ptr->intr_ack |= UART_INT_RX; //acknowledge receive interrupt
        uart_rx(); //receive subroutine
    }
    if (status & UART_INT_TX ) //transmit interrupt happened?
    {
        *interrupt_registers_ptr->intr_ack |= UART_INT_TX; //acknowledge transmit interrupt
        uart_tx (); //transmit subroutine
    }
}

//transmit subroutine of interrupt servive
void uart_tx(void)
{
    int ret;
    unsigned char c;
    ret= 1;
    if (*uart_registers_ptr->status & XMT_FIFO_EMPTY) //is transmit buffer empty
    {
        //as long as transmit fifo in hardware is not full and software fifo not empty
        while (!(*uart_registers_ptr->status & XMT_FIFO_FULL) && (ret) )
        {
            ret = bufRead(&c, TX_FIFO); //read char from software fifo
            if (ret) //not empty?
            *uart_registers_ptr->tx_data= c; //copy item to transmit register
        }
        //software transmit fifo empty?
        if (ret == 0 )
            *interrupt_registers_ptr->intr_mask &= ~UART_INT_TX; //disable transmit interrupt
    }
}

//receive subroutine of interrupt servive
void uart_rx(void)
{
    unsigned char *rx_ptr;
    //as long as hardware receive buffer is filled
    while (*uart_registers_ptr->status & DATA_READY )
    {
    //get pointer to next write item in receive fifo (no overrun check!)
        rx_ptr = next_ptr(RX_FIFO);
        *rx_ptr = *uart_registers_ptr->rx_data; //write item to receive fifo
    }
}

//subroutine to get next receive or transmit fifo write pointer
unsigned char * next_ptr(int index)
{
    unsigned char * rtp;
    rtp = arm_uart_bufs[index].write_ptr; // write pointer for return
    //is write poiter at the end?
    if (arm_uart_bufs[index].write_ptr == arm_uart_bufs[index].end)
        arm_uart_bufs[index].write_ptr = arm_uart_bufs[index].beg; //wrap around write pointer to begin
    else
        arm_uart_bufs[index].write_ptr++; //increment write pointer
    return(rtp);
}