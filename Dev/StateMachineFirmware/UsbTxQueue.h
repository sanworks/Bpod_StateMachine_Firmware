/*
  UsbTxQueue: a transmit queue in front of a Teensy 4 USB serial port.

  The Teensy 4 core sends what is written to USB serial in one of four transfers,
  each queued 75 us after the last write to it. handler() writes the events of each
  cycle as they occur, so events in consecutive cycles take one transfer each. Once
  all four wait for the host, the next write() waits for the host to take one, inside
  the timer interrupt: the handler misses its 100 us ticks, and the state machine,
  its outputs and its clock stop for as long as the host takes, now and then a few ms.

  This queue keeps the bytes in order in RAM and passes them to the core only when
  it can take them without waiting. Every write, and handler() once per cycle, sends
  what is queued as far as transfers are free. Only if the queue itself fills (the
  host has stopped reading) does a write wait, as before.

  Part of the Bpod state machine firmware; GNU General Public License v3.
*/
#ifndef USB_TX_QUEUE_H
#define USB_TX_QUEUE_H

#include <Arduino.h>

class UsbTxQueue : public Stream {
public:
  explicit UsbTxQueue(usb_serial_class &port) : port(port) {}

  // ArCOM's blocking reads spin on available(): keep sending meanwhile, in case
  // the host waits for a reply still in the queue
  int available() override {
    pump();
    return port.available();
  }
  int read() override { return port.read(); }
  int peek() override { return port.peek(); }

  size_t write(uint8_t b) override { return write(&b, 1); }

  size_t write(const uint8_t *data, size_t size) override {
    if (size > room()) {
      // The host has stopped reading for a while: wait for it, in order. If the
      // other context is sending (loop() was interrupted mid-drain), there is no
      // safe way to wait for it, and these bytes overtake the queued ones.
      if (claim()) {
        drain();
        pumping = false;
      }
      return port.write(data, size);
    }
    __disable_irq(); // loop() and handler() both write
    for (size_t i = 0; i < size; i++) {
      buffer[head] = data[i];
      head = (head + 1) % SIZE;
    }
    __enable_irq();
    pump();
    return size;
  }

  // Sends a partly filled transfer now (without waiting), after what is queued
  void flush() override {
    if (!claim()) return;
    send();
    if (head == tail) port.flush();
    pumping = false;
  }

  // Send what is queued as far as the core takes it without waiting. Only one
  // context sends at a time: handler() skips this while loop() is sending.
  void pump() {
    if (!claim()) return;
    send();
    pumping = false;
  }

private:
  static const uint32_t SIZE = 32768;
  static const uint32_t TX_BYTES = 2048; // TX_SIZE in the core's usb_serial.c

  usb_serial_class &port;
  uint8_t buffer[SIZE];
  volatile uint32_t head = 0; // next byte queued
  volatile uint32_t tail = 0; // next byte sent
  volatile bool pumping = false;

  size_t room() const { return SIZE - 1 - ((head + SIZE - tail) % SIZE); }

  bool claim() {
    __disable_irq();
    bool ok = !pumping;
    pumping = true;
    __enable_irq();
    return ok;
  }

  // The size of the queued run from tail that is contiguous in buffer, at most n
  uint32_t run(uint32_t n) const {
    uint32_t h = head, t = tail;
    uint32_t len = (h >= t) ? h - t : SIZE - t;
    return len < n ? len : n;
  }

  void send() {
    // The core's transfers complete in order, so a free transfer besides the one
    // being filled means that one and the next are free too: up to one transfer's
    // worth goes out without waiting.
    for (uint32_t n; (n = run(TX_BYTES)) > 0;) {
      if (usb_serial_write_buffer_free() < (int)TX_BYTES) break;
      port.write(buffer + tail, n);
      tail = (tail + n) % SIZE;
    }
  }

  void drain() {
    for (uint32_t n; (n = run(SIZE)) > 0;) {
      port.write(buffer + tail, n);
      tail = (tail + n) % SIZE;
    }
  }
};

#endif
