/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * RP2350 Bare-Metal UART.
 * 
 * BRIEF:
 * Minimal bare-metal UART on the RP2350. This bypasses
 * SDK abstractions and demonstrates register-level control in assembler.
 * Clocks the external crystal oscillator (XOSC) at 14.5MHz.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 2, 2025
 * UPDATE DATE: November 2, 2025
 */

.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

/**
 * Memory addresses and constants.
 */
.equ STACK_TOP,                   0x20082000               
.equ STACK_LIMIT,                 0x2007a000             
.equ XOSC_BASE,                   0x40048000          
.equ XOSC_CTRL,                   XOSC_BASE + 0x00       
.equ XOSC_STATUS,                 XOSC_BASE + 0x04       
.equ XOSC_STARTUP,                XOSC_BASE + 0x0c        
.equ PPB_BASE,                    0xe0000000               
.equ CPACR,                       PPB_BASE + 0x0ed88       
.equ CLOCKS_BASE,                 0x40010000              
.equ CLK_PERI_CTRL,               CLOCKS_BASE + 0x48       
.equ RESETS_BASE,                 0x40020000               
.equ RESETS_RESET,                RESETS_BASE + 0x0        
.equ RESETS_RESET_CLEAR,          RESETS_BASE + 0x3000     
.equ RESETS_RESET_DONE,           RESETS_BASE + 0x8        
.equ IO_BANK0_BASE,               0x40028000               
.equ IO_BANK0_GPIO16_CTRL_OFFSET, 0x84                   
.equ PADS_BANK0_BASE,             0x40038000               
.equ PADS_BANK0_GPIO16_OFFSET,    0x44                    
.equ UART0_BASE,                  0x40070000

/**
 * Initialize the .vectors section. The .vectors section contains vector
 * table and Reset_Handler.
 */
.section .vectors, "ax"                          // vector table section
.align 2                                         // align to 4-byte boundary

/**
 * Vector table section.
 */
.global _vectors                                 // export _vectors symbol
_vectors:
  .word STACK_TOP                                // initial stack pointer
  .word Reset_Handler + 1                        // reset handler (Thumb bit set)

/**
 * @brief   Reset handler for RP2350.
 *
 * @details Entry point after reset. Performs:
 *          - Stack initialization
 *          - Coprocessor enable
 *          - GPIO16 pad/function configuration
 *          - Branches to main() which contains the blink loop
 *
 * @param   None
 * @retval  None
 */
.global Reset_Handler                            // export Reset_Handler symbol
.type Reset_Handler, %function                        
.type Reset_Handler, %function                        
Reset_Handler:
  bl    Init_Stack                               // initialize MSP/PSP and limits
  bl    Init_XOSC                                // initialize external crystal oscillator
  bl    Enable_XOSC_Peri_Clock                   // enable XOSC peripheral clock
  bl    Init_Subsystem                           // initialize subsystems
  bl    UART_Release_Reset                       // ensure UART0 out of reset
  bl    UART_Init                                // initialize UART0 (pins, baud, enable)
  bl    Enable_Coprocessor                       // enable CP0 coprocessor
  b     main                                     // branch to main loop
.size Reset_Handler, . - Reset_Handler

/**
 * @brief   Initialize stack pointers.
 *
 * @details Sets Main and Process Stack Pointers (MSP/PSP) and their limits.
 *
 * @param   None
 * @retval  None
 */
.type Init_Stack, %function
Init_Stack:
  ldr   r0, =STACK_TOP                           // load stack top
  msr   PSP, r0                                  // set PSP
  ldr   r0, =STACK_LIMIT                         // load stack limit
  msr   MSPLIM, r0                               // set MSP limit
  msr   PSPLIM, r0                               // set PSP limit
  ldr   r0, =STACK_TOP                           // reload stack top
  msr   MSP, r0                                  // set MSP
  bx    lr                                       // return

/**
 * @brief   Init XOSC and wait until it is ready.
 *
 * @details Configures and initializes the external crystal oscillator (XOSC).
 *          Waits for the XOSC to become stable before returning.
 *
 * @param   None
 * @retval  None
 */
.type Init_XOSC, %function
Init_XOSC:
  ldr   r0, =XOSC_STARTUP                        // load XOSC_STARTUP address
  ldr   r1, =0x00c4                              // set delay 50,000 cycles
  str   r1, [r0]                                 // store value into XOSC_STARTUP
  ldr   r0, =XOSC_CTRL                           // load XOSC_CTRL address
  ldr   r1, =0x00FABAA0                          // set 1_15MHz, freq range, actual 14.5MHz
  str   r1, [r0]                                 // store value into XOSC_CTRL
.Init_XOSC_Wait:
  ldr   r0, =XOSC_STATUS                         // load XOSC_STATUS address
  ldr   r1, [r0]                                 // read XOSC_STATUS value
  tst   r1, #(1<<31)                             // test STABLE bit
  beq   .Init_XOSC_Wait                          // wait until stable bit is set
  bx    lr                                       // return

/**
 * @brief   Enable XOSC peripheral clock.
 *
 * @details Sets the peripheral clock to use XOSC as its AUXSRC.
 *
 * @param   None
 * @retval  None
 */
.type Enable_XOSC_Peri_Clock, %function
Enable_XOSC_Peri_Clock:
  ldr   r0, =CLK_PERI_CTRL                       // load CLK_PERI_CTRL address
  ldr   r1, [r0]                                 // read CLK_PERI_CTRL value
  orr   r1, r1, #(1<<11)                         // set ENABLE bit
  orr   r1, r1, #(4<<5)                          // set AUXSRC: XOSC_CLKSRC bit
  str   r1, [r0]                                 // store value into CLK_PERI_CTRL
  bx    lr                                       // return

/**
 * @brief   Init subsystem.
 *
 * @details Initiates the various subsystems by clearing their reset bits.
 *
 * @param   None
 * @retval  None
 */
.type Init_Subsystem, %function
Init_Subsystem:
.GPIO_Subsystem_Reset:
  ldr   r0, =RESETS_RESET                        // load RESETS->RESET address
  ldr   r1, [r0]                                 // read RESETS->RESET value
  bic   r1, r1, #(1<<6)                          // clear IO_BANK0 bit
  str   r1, [r0]                                 // store value into RESETS->RESET address
.GPIO_Subsystem_Reset_Wait:
  ldr   r0, =RESETS_RESET_DONE                   // load RESETS->RESET_DONE address
  ldr   r1, [r0]                                 // read RESETS->RESET_DONE value
  tst   r1, #(1<<6)                              // test IO_BANK0 reset done
  beq   .GPIO_Subsystem_Reset_Wait               // wait until done
  bx    lr                                       // return

/**
 * @brief   Enable coprocessor access.
 *
 * @details Grants full access to coprocessor 0 via CPACR.
 *
 * @param   None
 * @retval  None
 */
.type Enable_Coprocessor , %function
Enable_Coprocessor:
  ldr   r0, =CPACR                               // load CPACR address
  ldr   r1, [r0]                                 // read CPACR value
  orr   r1, r1, #(1<<1)                          // set CP0: Ctrl access priv coproc 0 bit
  orr   r1, r1, #(1<<0)                          // set CP0: Ctrl access priv coproc 0 bit
  str   r1, [r0]                                 // store value into CPACR
  dsb                                            // data sync barrier
  isb                                            // instruction sync barrier
  bx    lr                                       // return

/**
 * @brief   Release UART0 from reset and wait until it is ready.
 *
 * @details Clears the UART0 reset bit in the Reset controller (RESETS->RESET)
 *          and polls the corresponding bit in RESETS->RESET_DONE until the
 *          UART0 block is no longer in reset. This ensures UART registers are
 *          accessible before configuring the peripheral.
 *
 * @param   None
 * @retval  None
 */
.type UART_Release_Reset, %function
UART_Release_Reset:
  ldr   r0, =RESETS_RESET                        // load RESETS->RESET address
  ldr   r1, [r0]                                 // read RESETS->RESET value
  bic   r1, r1, #(1<<26)                         // clear UART0 reset bit
  str   r1, [r0]                                 // write value back to RESETS->RESET
.UART_Release_Reset_Wait:
  ldr   r0, =RESETS_RESET_DONE                   // load RESETS->RESET_DONE address
  ldr   r1, [r0]                                 // read RESETS->RESET_DONE value
  tst   r1, #(1<<26)                             // test UART0 reset-done bit
  beq   .UART_Release_Reset_Wait                 // loop until UART0 is out of reset
  bx    lr                                       // return

/**
 * @brief   Initialize UART0 (pins, baud divisors, line control and enable).
 *
 * @details Configures IO_BANK0 pins 0 (TX) and 1 (RX) to the UART function
 *          and programs the corresponding pad controls in PADS_BANK0. It
 *          programs the integer and fractional baud divisors (UARTIBRD and
 *          UARTFBRD), configures UARTLCR_H for 8-bit transfers and FIFOs,
 *          and enables the UART (UARTCR: UARTEN + TXE + RXE).
 *          The routine assumes the UART0 base is available at the
 *          `UART0_BASE` symbol. The selected divisors (IBRD=6, FBRD=33) are
 *          chosen to match the expected peripheral clock; if your UART
 *          peripheral clock differs, adjust these values accordingly.
 *
 * @param   None
 * @retval  None
 */
.type UART_Init, %function
UART_Init:
  ldr   r0, =IO_BANK0_BASE                       // load IO_BANK0 base
  ldr   r1, =2                                   // FUNCSEL = 2 -> select UART function
  str   r1, [r0, #4]                             // write FUNCSEL to GPIO0_CTRL (pin0 -> TX)
  str   r1, [r0, #0x0c]                          // write FUNCSEL to GPIO1_CTRL (pin1 -> RX)
  ldr   r0, =PADS_BANK0_BASE                     // load PADS_BANK0 base
  add   r0, r0, #0x04                            // compute PAD[0] address (PADS + 0x04)
  ldr   r1, =0x04                                // pad config value for TX (pull/func recommended)
  str   r1, [r0]                                 // write PAD0 config (TX pad)
  ldr   r0, =PADS_BANK0_BASE                     // load PADS_BANK0 base again
  add   r0, r0, #0x08                            // compute PAD[1] address (PADS + 0x08)
  ldr   r1, =0x40                                // pad config value for RX (pulldown/IE as needed)
  str   r1, [r0]                                 // write PAD1 config (RX pad)
  ldr   r0, =UART0_BASE                            // load UART0 base address
  ldr   r1, =0                                   // prepare 0 to disable UARTCR
  str   r1, [r0, #0x30]                          // UARTCR = 0 (disable UART while configuring)
  ldr   r1, =6                                   // integer baud divisor (IBRD = 6)
  str   r1, [r0, #0x24]                          // UARTIBRD = 6 (integer baud divisor)
  ldr   r1, =33                                  // fractional baud divisor (FBRD = 33)
  str   r1, [r0, #0x28]                          // UARTFBRD = 33 (fractional baud divisor)
  ldr   r1, =112                                 // UARTLCR_H = 0x70 (FIFO enable + 8-bit)
  str   r1, [r0, #0x2c]                          // UARTLCR_H = 0x70 (FIFO enable + 8-bit)
  ldr   r1, =3                                   // RXE/TXE mask (will be shifted into bits 8..9)
  lsl   r1, r1, #8                               // shift RXE/TXE into bit positions 8..9
  orr   r1, r1, #1                               // set UARTEN bit (bit 0)
  str   r1, [r0, #0x30]                          // UARTCR = enable (UARTEN + TXE + RXE)
  bx    lr                                       // return

/**
 * Initialize the .text section. 
 * The .text section contains executable code.
 */
.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

/**
 * @brief   Main application entry point.
 *
 * @details Implements the infinite blink loop.
 *
 * @param   None
 * @retval  None
 */
.global main                                     // export main
.type main, %function                            // mark as function
main:
.Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.Loop:
  bl    UART0_In                                 // call UART0_In
  bl    UART0_Out                                // call UART0_Out
  b     .Loop                                    // loop forever
.Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return to caller

/**
 * @brief   UART0 transmit (blocking).
 *
 * @details Waits for TX FIFO to be not full, then writes the lowest 8 bits of r0 to UART0.
 *          Data to send must be in r0 on entry.
 *
 * @param   r0: byte to transmit (lower 8 bits used)
 * @retval  None
 */
.type UART0_Out, %function
UART0_Out:
.UART0_Out_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.UART0_Out_loop:
  ldr   r4, =UART0_BASE                          // base address for uart0 registers
  ldr   r5, [r4, #0x18]                          // read UART0 flag register UARTFR into r5
  ldr   r6, =32                                  // mask for bit 5, TX FIFO full (TXFF)
  ands  r5, r5, r6                               // isolate TXFF bit and set flags
  bne   .UART0_Out_loop                          // if TX FIFO is full, loop
  ldr   r6, =0xff                                // mask for the 8 lowest bits
  ands  r0, r0, r6                               // mask off upper bits of r0, keep lower 8 bits
  str   r0, [r4, #0]                             // write data to UARTDR
.UART0_Out_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return

/**
 * @brief   UART0 receive (blocking).
 *
 * @details Waits for RX FIFO to be not empty, then reads a byte from UART0 into r0.
 *
 * @param   None
 * @retval  r0: received byte (lower 8 bits valid)
 */
.type UART0_In, %function
UART0_In:
.UART0_In_Push_Registers:
  push  {r4-r12, lr}                              // push registers r4-r12, lr to the stack
.UART0_In_loop:
  ldr   r4, =UART0_BASE                           // base address for uart0 registers (use r4 per convention)
  ldr   r5, [r4, #0x18]                           // read UART0 flag register UARTFR into r5
  ldr   r6, =16                                   // mask for bit 4, RX FIFO empty RXFE
  ands  r5, r5, r6                                // isolate RXFE bit and set flags
  bne   .UART0_In_loop                            // if RX FIFO is empty, loop
  ldr   r0, [r4, #0]                              // load data from UARTDR into r0 (return value)
.UART0_In_Pop_Registers:
  pop   {r4-r12, lr}                              // pop registers r4-r12, lr from the stack
  bx    lr                                        // return

/**
 * @brief   Configure GPIO.
 *
 * @details Configures a GPIO pin's pad control and function select.
 *
 * @param   r0 - PAD_OFFSET
 * @param   r1 - CTRL_OFFSET
 * @param   r2 - GPIO
 * @retval  None
 */
.type GPIO_Config, %function
GPIO_Config:
.GPIO_Config_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Config_Modify_Pad:
  ldr   r4, =PADS_BANK0_BASE                     // load PADS_BANK0_BASE address
  add   r4, r4, r0                               // PADS_BANK0_BASE + PAD_OFFSET
  ldr   r5, [r4]                                 // read PAD_OFFSET value
  bic   r5, r5, #(1<<7)                          // clear OD bit
  orr   r5, r5, #(1<<6)                          // set IE bit
  bic   r5, r5, #(1<<8)                          // clear ISO bit
  str   r5, [r4]                                 // store value into PAD_OFFSET
.GPIO_Config_Modify_CTRL:
  ldr   r4, =IO_BANK0_BASE                       // load IO_BANK0 base
  add   r4, r4, r1                               // IO_BANK0_BASE + CTRL_OFFSET
  ldr   r5, [r4]                                 // read CTRL_OFFSET value
  bic   r5, r5, #0x1f                            // clear FUNCSEL
  orr   r5, r5, #0x05                            // set FUNCSEL 0x05->SIO_0
  str   r5, [r4]                                 // store value into CTRL_OFFSET
.GPIO_Config_Enable_OE:
  ldr   r4, =1                                   // enable output
  mcrr  p0, #4, r2, r4, c4                       // gpioc_bit_oe_put(GPIO, 1)
.GPIO_Config_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr to the stack
  bx    lr                                       // return

/**
 * @brief   GPIO set.
 *
 * @details Drives GPIO output high via coprocessor.
 *
 * @param   r0 - GPIO
 * @retval  None
 */
.type GPIO_Set, %function
GPIO_Set:
.GPIO_Set_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Set_Execute:
  ldr   r4, =1                                   // enable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 1)
.GPIO_Set_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return

/**
 * @brief   GPIO clear.
 *
 * @details Drives GPIO output high via coprocessor.
 *
 * @param   r0 - GPIO
 * @retval  None
 */
.type GPIO_Clear, %function
GPIO_Clear:
.GPIO_Clear_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Clear_Execute:
  ldr   r4, =0                                   // disable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 1)
.GPIO_Clear_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return

/**
 * @brief   Delay_MS.
 *
 * @details Delays for r0 milliseconds. Conversion: loop_count = ms * 3600
 *          based on a 14.5MHz clock.
 *
 * @param   r0 - milliseconds
 * @retval  None
 */
.type Delay_MS, %function
Delay_MS:
.Delay_MS_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.Delay_MS_Check:
  cmp   r0, #0                                   // if MS is not valid, return
  ble   .Delay_MS_Done                           // branch if less or equal to 0 
.Delay_MS_Setup:
  ldr   r4, =3600                                // loops per MS based on 14.5MHz clock
  mul   r5, r0, r4                               // MS * 3600
.Delay_MS_Loop:
  subs  r5, r5, #1                               // decrement counter
  bne   .Delay_MS_Loop                           // branch until zero
.Delay_MS_Done:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return

/**
 * Test data and constants.
 * The .rodata section is used for constants and static data.
 */
.section .rodata                                 // read-only data section

/**
 * Initialized global data.
 * The .data section is used for initialized global or static variables.
 */
.section .data                                   // data section

/**
 * Uninitialized global data.
 * The .bss section is used for uninitialized global or static variables.
 */
.section .bss                                    // BSS section

