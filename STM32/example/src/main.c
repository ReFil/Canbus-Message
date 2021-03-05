// ********************* Includes *********************

#include "config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"
#include "board.h"
#include "drivers/clock.h"
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

#define CAN CAN1

bool has_external_debug_serial = false;


#define CTRLS_USB

#ifdef CTRLS_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

// ********************* serial debugging *********************

#ifdef CTRLS_USB

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_GAS_INPUT  0x366
#define CAN_GAS_OUTPUT 0x365U
#define CAN_GAS_SIZE 3

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// set state
bool enabled;
uint8_t setspeed;

#define MAX_TIMEOUT 10U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0xD5;  // standard crc8

void CAN1_RX0_IRQ_Handler(void) {
  //can receive interrupt handler
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_GAS_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      enabled = ((dat[0] >> 7) & 1U) != 0U;
      setspeed = dat[1];
      if (crc_checksum(dat, CAN_GAS_SIZE - 1, crc_poly) == dat[2]) {
        #ifdef DEBUG
          puts("enable detected");
          puth(setspeed);
          puts("\n");
        #endif
        if (enabled) {
        }
        else {
            state = NO_FAULT;
        }
        // clear the timeout
        timeout = 0;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}


volatile bool btns[4]; // order set, cancel, speed up, speed down
volatile bool oldbtns[4];

int led_value = 0;

void update_eon(void) {
  //can send finction
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" ");
    puth(state);
    puts(" ");
    puth(set_btn);
    puts("\n");
  #endif

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[2];
    dat[0] = (btns[0] >> 0 | btns[1] >> 1 | btns[2] >> 2 | btns[3] >> 3) & 0xFFU;
    dat[1] = ((state & 0xFU) << 4) & 0xFFU;
    dat[2] = crc_checksum(dat, CAN_GAS_SIZE - 1, crc_poly);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16);
    CAN->sTxMailBox[0].TDTR = 4;  // len of packet is 3
    CAN->sTxMailBox[0].TIR = (CAN_GAS_OUTPUT << 21) | 1U;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}


void TIM3_IRQ_Handler(void) {
  //timer interrupt for periodic can messages for example
}


// ***************************** main code *****************************

void loop(void) {
  // read/write
 // do things
 // look in /include/drivers to see all the things you can do


  // reset watchdog timer

  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)

  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();

  init();

#ifdef CTRLS_USB
  // enable USB
  usb_init();
#endif

//f4 board doesn't have ADC pins exposed
 // adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732 
  // Timer interrupt
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);


  update_eon();
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    loop();
  }

  return 0;
}
