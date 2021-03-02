// ********************* Includes *********************
#include "stm32f4xx.h"

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

//#define ADC
#define ENCODER

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

volatile uint8_t encoderCount = 2;

void TIM3_IRQ_Handler(void) {
  volatile static uint8_t ABs = 0;
  ABs = (ABs << 2) & 0x0f; //left 2 bits now contain the previous AB key read-out;
  ABs |= (get_gpio_input(GPIOA, 8) << 1) | get_gpio_input(GPIOA, 9);
  encoderCount = 2;
  switch (ABs)
  {
    case 0x0d:
      encoderCount = 3;
      break;
    case 0x0e:
      encoderCount = 1;
      break;
  }
}


// ***************************** main code *****************************

void loop(void) {
  // read/write

#ifdef ADC
  uint32_t value;
  value = adc_get(ADCCHAN_ACCEL0);
  puth(value);
  puts("\n");
  if(value < 1) {
    btns[2] = 1;
    btns[3] = 0;
  }
  else if(value < 2 && value > 1) {
    btns[2] = 1;
    btns[3] = 0;
  }
  else if(value < 3 && value > 2) {
    btns[2] = 1;
    btns[3] = 0;
  }
#endif
#ifdef ENCODER
  switch (encoderCount) {
    case 1:
      btns[2] = 1;
      btns[3] = 0;
    case 2:
      btns[2] = 0;
      btns[3] = 0;
    case 3:
      btns[2] = 0;
      btns[3] = 1;
  }
  btns[0] = get_gpio_input(GPIOA, 8);
  btns[1] = get_gpio_input(GPIOA, 9);
#else
  btns[0] = get_gpio_input(GPIOA, 8);
  btns[1] = get_gpio_input(GPIOA, 9);
  btns[2] = get_gpio_input(GPIOA, 10);
  btns[3] = get_gpio_input(GPIOC, 10);
#endif

  if(btns[0] && enabled && !oldbtns[0]){ //if set button pressed but system is already enabled
    btns[0] = 0;
    btns[1] = 1; //cancel instead
  }
  if(btns[0] != oldbtns[0] || btns[1] != oldbtns[1] || btns[2] != oldbtns[2] || btns[3] != oldbtns[3]) {//if button values have changed
    update_eon(); //send new button values to eon
  }

  oldbtns[0] = btns[0];
  oldbtns[1] = btns[1];
  oldbtns[2] = btns[2];
  oldbtns[3] = btns[3];

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
  #ifdef ENCODER
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)
  #endif

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();

  init();

#ifdef CTRLS_USB
  // enable USB
  usb_init();
#endif

#ifdef ADC
  adc_init();
#endif

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
#ifdef ENCODER
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);
#endif
  btns[0] = 0;
  btns[1] = 0;
  btns[2] = 0;
  btns[3] = 0;
  
  update_eon();
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main CTRLS loop
  while (1) {
    loop();
  }

  return 0;
}
