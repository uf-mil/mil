#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f3/gpio.h>
#include <libopencm3/stm32/f3/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/usbd.h>
#include <stdlib.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

#include <arm_bootloader/uniqueid.h>
#include <uf_subbus_protocol/protocol.h>
#include <uf_subbus_protocol/usb.h>

#include <stdlib.h>
#include <string.h>

#include <navigator_emergency_control/protocol.h>

//#include "i2c.h"
//#include "spi.h"
//#include "pwm.h"

using namespace navigator_emergency_control;

float data[4] = { 0 };     // Received UD LR data from controller
bool buttons[20] = { 0 };  // Received button press data

class Handler
{
  class GotMessageFunctor
  {
    Handler &handler;

  public:
    GotMessageFunctor(Handler &handler) : handler(handler)
    {
    }
    void operator()(const Command &command)
    {
      handler.handle_message(command);
    }
  };

  GotMessageFunctor gmf;
  uf_subbus_protocol::SimpleReceiver<Command, GotMessageFunctor> receiver;
  uf_subbus_protocol::SimpleSender<Response, uf_subbus_protocol::ISink> sender;
  arm_bootloader::Dest dest;

public:
  Handler(uf_subbus_protocol::ISink &sink, arm_bootloader::Dest dest)
    : gmf(*this), receiver(gmf), sender(sink), dest(dest)
  {
  }

  void handleByte(uint8_t byte)
  {
    receiver.handleRawByte(byte);
  }

  void handle_message(const Command &msg)
  {
    if (msg.dest != dest)
      return;

    Response resp;
    memset(&resp, 0, sizeof(resp));
    resp.id = msg.id;

    switch (msg.command)
    {
      case CommandID::Reset:
      {
        // action happens later, after response is sent
      }
      break;

      case CommandID::GetStatus:
      {
        resp.resp.GetStatus.magic = GetStatusResponse::MAGIC_VALUE;
      }
      break;

      case CommandID::GetIMUData:
      {
        // These are unused
        // i2c_read_imu(resp.resp.GetIMUData);
        // spi_read_gyro(resp.resp.GetIMUData.angular_velocity);
      }
      break;

      case CommandID::SetPWM:
      {
        // Retooling this command to publish data to joy_emergency, + additional control functions
        // pwm_set_length(0, msg.args.SetPWM.length[0]);
        // pwm_set_length(1, msg.args.SetPWM.length[1]);
        memcpy(resp.resp.SetPWM.joy, data, sizeof(resp.resp.SetPWM.joy));
        memcpy(resp.resp.SetPWM.buttons, buttons, sizeof(resp.resp.SetPWM.buttons));
      }
      break;

      default:
      {
        return;  // send nothing back if command is invalid
      }
      break;
    }

    if (resp.id)
    {
      sender.write_object(resp);
    }

    switch (msg.command)
    {
      case CommandID::Reset:
      {
        scb_reset_system();
      }
      break;

      default:
      {
      }
      break;
    }
  }
};

static Handler *handlerp = NULL;

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  (void)usbd_dev;

  char buf[64];
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

  if (!handlerp)
    return;

  for (int i = 0; i < len; i++)
  {
    handlerp->handleByte(buf[i]);
  }
}

void usart_setup()
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART1);

  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2);
  gpio_clear(GPIOB, GPIO0 | GPIO1 | GPIO2);

  // nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);	//This crashes the processor somehow?
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  // usart_enable_rx_interrupt(USART1);	//Can't setup interrupts, enabling irq crashes uP?

  usart_enable(USART1);
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                                 USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                                 uf_subbus_protocol::usb::cdcacm_control_request);
}

int main()
{
  rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_48MHZ]);
  rcc_usb_prescale_1();
  rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_USBRST);
  rcc_peripheral_reset(&RCC_AHBRSTR, RCC_AHBRSTR_IOPARST);
  rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_USBRST);
  rcc_peripheral_clear_reset(&RCC_AHBRSTR, RCC_AHBRSTR_IOPARST);
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USBEN);
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_IOPAEN);

  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11 | GPIO12);
  gpio_clear(GPIOA, GPIO11 | GPIO12);
  for (int i = 0; i < 0x800000; i++)
    __asm__ volatile("nop");

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF14, GPIO11 | GPIO12);

  // i2c_setup();
  // spi_setup();
  usart_setup();  // Setup wireless usart communication with controller

  arm_bootloader::Dest dest = arm_bootloader::get_unique_dest();

  char serial[10] = { 0 };
  {
    uint32_t x = dest;
    char hex[] = "0123456789abcdef";
    for (int i = 0; i < 8; i++)
    {
      serial[7 - i] = hex[x & 0xF];
      x = x >> 4;
    }
  }

  /* Buffer to be used for control requests. */
  uint8_t usbd_control_buffer[128];
  char const *usb_strings[] = {
    "uf-mil", "xbee", serial,
  };
  usbd_device *usbd_dev =
      usbd_init(&stm32f103_usb_driver, &uf_subbus_protocol::usb::dev, &uf_subbus_protocol::usb::config, usb_strings, 3,
                usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

  uf_subbus_protocol::usb::Sink sink(usbd_dev);

  Handler handler(sink, dest);
  handlerp = &handler;

  char buffer[100] = { 0 };  // Message buffer, header is A55A, endpoint is F55F
  int buff_cnt = 0;
  char numbuf[8] = { 0 };  // Buffer to temporarily store string to be converted to int
  int temp;
  int timer = 0;    // Timer for received character timeout
  int err_cnt = 0;  // Counter for number of corrupted packets

  while (1)
  {
    usbd_poll(usbd_dev);

    // Check if a character was received
    if ((USART_ISR(USART1) & USART_ISR_RXNE) != 0)
    {
      // Retrieve data
      buffer[buff_cnt] = usart_recv(USART1);
      buff_cnt++;
      timer = 0;  // Zero the timer if data received

      if ((buffer[buff_cnt - 1] == 'F') && (buffer[buff_cnt - 2] == '5') && (buffer[buff_cnt - 3] == '5') &&
          (buffer[buff_cnt - 4] == 'F'))  // End of message
      {
        int i = 0;

        // Begin inelegant parsing of disorganized messages
        for (; i < buff_cnt; i++)
        {
          // Up/Down Check
          // Sends as "(UD: -0000)", w/ or w/o sign
          if ((buffer[i] == 'D') && (buffer[i + 1] == ':'))
          {
            // Store 5 chars for 4 possible digits and 1 sign
            numbuf[0] = buffer[i + 3];
            numbuf[1] = buffer[i + 4];
            numbuf[2] = buffer[i + 5];
            numbuf[3] = buffer[i + 6];
            numbuf[4] = buffer[i + 7];
            // Find garbage chars, a CRC check would be better
            for (temp = 0; temp < 5; temp++)
            {
              // Not a digit or a sign char
              if (!((numbuf[temp] >= '0') && (numbuf[temp] <= '9')) && (numbuf[temp] != '-') && (numbuf[temp] != ')'))
              {
                temp = 6;
                break;
              }
            }
            if (temp > 5)
            {
              break;
            }
            temp = atoi(numbuf);
            if ((temp > (-2049)) && (temp < (2049)))
            {
              // Store UD here
              data[0] = (((float)temp) / 2048);
              err_cnt = 0;
            }
            memset(numbuf, 0, 8);
            break;
          }
        }

        for (; i < buff_cnt; i++)
        {
          // Left/Right Check
          // Sends as "(LR: -0000)"
          if ((buffer[i] == 'R') && (buffer[i + 1] == ':'))
          {
            numbuf[0] = buffer[i + 3];
            numbuf[1] = buffer[i + 4];
            numbuf[2] = buffer[i + 5];
            numbuf[3] = buffer[i + 6];
            numbuf[4] = buffer[i + 7];
            // Find garbage chars, a CRC check would be better
            for (temp = 0; temp < 5; temp++)
            {
              if (!((numbuf[temp] >= '0') && (numbuf[temp] <= '9')) && (numbuf[temp] != '-') && (numbuf[temp] != ')'))
              {
                temp = 6;
                break;
              }
            }
            if (temp > 5)
            {
              break;
            }
            temp = atoi(numbuf);
            if ((temp > (-2049)) && (temp < (2049)))
            {
              // Store LR here
              data[1] = (-1) * (((float)temp) / 2048);
              err_cnt = 0;
            }
            memset(numbuf, 0, 8);
            break;
          }
        }

        for (; i < buff_cnt; i++)
        {
          // Torque Check
          // Sends as "(TQ: -0000)"
          if ((buffer[i] == 'Q') && (buffer[i + 1] == ':'))
          {
            numbuf[0] = buffer[i + 3];
            numbuf[1] = buffer[i + 4];
            numbuf[2] = buffer[i + 5];
            numbuf[3] = buffer[i + 6];
            numbuf[4] = buffer[i + 7];
            // Find garbage chars, a CRC check would be better
            for (temp = 0; temp < 5; temp++)
            {
              if (!((numbuf[temp] >= '0') && (numbuf[temp] <= '9')) && (numbuf[temp] != '-') && (numbuf[temp] != ')'))
              {
                temp = 6;
                break;
              }
            }
            if (temp > 5)
            {
              break;
            }
            temp = atoi(numbuf);
            if ((temp > (-2049)) && (temp < (2049)))
            {
              // Store Torque LR here
              data[3] = (float)temp;
              data[3] = (-1) * (((float)temp) / 2048);
              err_cnt = 0;
            }
            memset(numbuf, 0, 8);
            break;
          }
        }

        for (; i < buff_cnt; i++)
        {
          // Button Check
          // Sends as "BT: nn nn", two integers compared w/ each other
          if ((buffer[i] == 'T') && (buffer[i + 1] == ':'))
          {
            numbuf[0] = buffer[i + 3];
            numbuf[1] = buffer[i + 4];
            temp = atoi(numbuf);
            memset(numbuf, 0, 8);

            numbuf[0] = buffer[i + 6];
            numbuf[1] = buffer[i + 7];
            buttons[17] = 1;
            // If the two don't match, data corrupted
            if (temp != atoi(numbuf))
            {
              break;
            }

            // Buttons map to assignments in navigator_joystick.py
            buttons[2] = ((temp & 0b0000001) == 0b0000001);   // X
            buttons[3] = ((temp & 0b0000010) == 0b0000010);   // Y
            buttons[0] = ((temp & 0b0000100) == 0b0000100);   // A
            buttons[1] = ((temp & 0b0001000) == 0b0001000);   // B
            buttons[11] = ((temp & 0b0010000) == 0b0010000);  // Dpad L
            buttons[12] = ((temp & 0b0100000) == 0b0100000);  // Dpad R
            buttons[7] = ((temp & 0b1000000) == 0b1000000);   // Start

            memset(numbuf, 0, 8);
            break;
          }
        }
        memset(buffer, 0, 100);
        buff_cnt = 0;
      }

      // If we don't receive F55F endpoint, transmission corrupted
      else if (buff_cnt > 99)
      {
        memset(buffer, 0, 100);
        buff_cnt = 0;
        err_cnt++;
        if (err_cnt > 100)  // Zero in case controller sends bad data continuously
        {
          memset(data, 0.0, sizeof(data));
          memset(buttons, 0, sizeof(buttons));
        }
      }

      uint32_t flags = USART_ISR(USART1_BASE);
      USART_ICR(USART1_BASE) = flags;  // clear set flags
    }
    else
    {
      timer++;  // Increment timer if no data is received
      if (timer > 800000)
      {
        memset(data, 0.0, sizeof(data));      // Zero if no data is received after
        memset(buttons, 0, sizeof(buttons));  // arbitrary short time to prevent
        timer = 0;                            // motion if controller disconnects
      }
    }
  }
}
