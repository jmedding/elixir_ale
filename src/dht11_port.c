/*
 *  dht11_port.c:
 *  Simple test program to test the wiringPi functions
 *  DHT11 test
 */

 /*
 *  Copyright 2014 Frank Hunleth
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * GPIO port implementation.
 *
 * This code has been heavily modified from Elixier_Ale
 * Copyright (C) 2014 Frank Hunleth
 * and Erlang/ALE, Copyright (C) 2013 Erlang Solutions Ltd.
 * See http://opensource.erlang-solutions.com/erlang_ale/.
 *
 * DHT11 Single Wire protocal implementation
 *
 * The DHT11 portion has been adapted from
 *    http://www.uugear.com/portfolio/dht11-humidity-temperature-sensor-module/
 * wheree you can find the original c code and additional information
 * on the single wire protocal that may be interesting
 */
 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <err.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "erlcmd.h"

#define DEBUG
#ifdef DEBUG
#define debug(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\r\n"); } while(0)
#else
#define debug(...)
#endif

#define HIGH 1
#define LOW 0
#define MAXTIMINGS  85
#define DHT_ERROR_TIMEOUT 2

#define DHT_MAXCOUNT 200
#define DHT_PULSES 41
// Make sure array is initialized to start at zero.
int pulseCounts[DHT_PULSES*2] = {0};


//#define DHTPIN    7
int dht11_dat[5] = { 0, 0, 0, 0, 0 };

/*
 * GPIO handling definitions and prototypes
 */
enum gpio_state {
    GPIO_OUTPUT,
    GPIO_INPUT,
    GPIO_INPUT_WITH_INTERRUPTS
};

struct gpio {
    enum gpio_state state;
    int fd;
    int pin_number;
};

struct dht11Result {
    float humidity;
    float tempC;
    float tempF;
};
 
/**
 * @brief write a string to a sysfs file
 * @return returns 0 on failure, >0 on success
 */
int dht11_sysfs_write_file(const char *pathname, const char *value)
{
    int fd = open(pathname, O_WRONLY);
    if (fd < 0) {
        debug("Error opening %s", pathname);
        return 0;
    }

    size_t count = strlen(value);
    ssize_t written = write(fd, value, count);
    close(fd);

    if (written < 0 || (size_t) written != count) {
        debug("Error writing '%s' to %s", value, pathname);
        return 0;
    }

    return written;
}

 /**
 * @brief Set pin with the value "0" or "1"
 *
 * @param pin           The pin structure
 * @param       value         Value to set (0 or 1)
 *
 * @return  1 for success, -1 for failure
 */
int dht11_write(struct gpio *pin, unsigned int val)
{
    if (pin->state != GPIO_OUTPUT)
        return -1;

    char buf = val ? '1' : '0';
    ssize_t amount_written = pwrite(pin->fd, &buf, sizeof(buf), 0);
    if (amount_written < (ssize_t) sizeof(buf))
        err(EXIT_FAILURE, "pwrite");

    return 1;
}

/**
* @brief  Read the value of the pin
*
* @param  pin            The GPIO pin
*
* @return   The pin value if success, -1 for failure
*/
int dht11_read(struct gpio *pin)
{
    char buf;
    ssize_t amount_read = pread(pin->fd, &buf, sizeof(buf), 0);
    if (amount_read < (ssize_t) sizeof(buf))
        err(EXIT_FAILURE, "pread");

    return buf == '1' ? 1 : 0;
}

// GPIO functions

/**
 * @brief Open and configure a GPIO
 *
 * @param pin           The pin structure
 * @param pin_number    The GPIO pin
 * @param   dir           Direction of pin (input or output)
 *
 * @return  1 for success, -1 for failure
 */
int dht11_gpio_init(struct gpio *pin, unsigned int pin_number, enum gpio_state dir)
{
    /* Initialize the pin structure. */
    pin->state = dir;
    pin->fd = -1;
    pin->pin_number = pin_number;

    /* Construct the gpio control file paths */
    char direction_path[64];
    sprintf(direction_path, "/sys/class/gpio/gpio%d/direction", pin_number);

    char value_path[64];
    sprintf(value_path, "/sys/class/gpio/gpio%d/value", pin_number);

    /* Check if the gpio has been exported already. */
    if (access(value_path, F_OK) == -1) {
        /* Nope. Export it. */
        char pinstr[64];
        sprintf(pinstr, "%d", pin_number);
        if (!dht11_sysfs_write_file("/sys/class/gpio/export", pinstr))
            return -1;
    }

    /* The direction file may not exist if the pin only works one way.
       It is ok if the direction file doesn't exist, but if it does
       exist, we must be able to write it.
    */
    if (access(direction_path, F_OK) != -1) {
      const char *dir_string = (dir == GPIO_OUTPUT ? "out" : "in");
        /* Writing the direction fails on a Raspberry Pi in what looks
           like a race condition with exporting the GPIO. Poll until it
           works as a workaround. */
        int retries = 1000; /* Allow 1000 * 1 ms = 1 second max for retries */
        while (!dht11_sysfs_write_file(direction_path, dir_string) &&
               retries > 0) {
            usleep(1000);
            retries--;
        }
        if (retries == 0) {
            debug("Could not access pin direction path during init");
            return -1;
        }
    } 

    pin->pin_number = pin_number;

    /* Open the value file for quick access later */
    pin->fd = open(value_path, pin->state == GPIO_OUTPUT ? O_RDWR : O_RDONLY);
    if (pin->fd < 0)
        return -1;

    return 1;
}

/**
 * Called after poll() returns when the GPIO sysfs file indicates
 * a status change.
 *
 * @param pin which pin to check
 */
void dht11_gpio_process(struct gpio *pin)
{
    int value = dht11_read(pin);

    char resp[256];
    int resp_index = sizeof(uint16_t); // Space for payload size
    resp[resp_index++] = 'n'; // Notification
    ei_encode_version(resp, &resp_index);
    ei_encode_tuple_header(resp, &resp_index, 2);
    ei_encode_atom(resp, &resp_index, "gpio_interrupt");
    ei_encode_atom(resp, &resp_index, value ? "rising" : "falling");
    erlcmd_send(resp, resp_index);
}
 
 
//struct dht11Result dht11_sense(struct gpio *pin)
int dht11_sense(struct gpio *pin)
{
  uint8_t j   = 0;
  int i;
  float f; /* fahrenheit */
  struct dht11Result result;
  enum gpio_state dir = GPIO_OUTPUT;
  char meas [MAXTIMINGS][20];
  //int err;  // TODO: hanlde func errors  
  dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;
 
  /* pull pin down for 18 milliseconds */
  //pinMode( DHTPIN, OUTPUT );
  //digitalWrite( DHTPIN, LOW );
  debug("sense start");
  if (dht11_gpio_init(pin, pin->pin_number, dir) < 0) 
    errx(EXIT_FAILURE, "Error initializing_1 GPIO as OUTPUT");
  dht11_write(pin, LOW);
  usleep( 18 * 1000 );

  /* then pull it up for 20-40 microseconds */
  //digitalWrite( DHTPIN, HIGH );
  dht11_write(pin, HIGH);
  usleep( 20 ); // not sure that we need to wait, might miss first pullup from dht11

  /* prepare to read the pin */
  //pinMode( DHTPIN, INPUT );
  dir = GPIO_INPUT;
  if (dht11_gpio_init(pin, pin->pin_number, dir) < 0) 
    errx(EXIT_FAILURE, "Error initializing_2 GPIO as INPUT");

  // Short wait for DHT11 to pull pin HIGH
  usleep(1);

  // Wait for DHT to pull pin low.
  uint32_t count = 0;
  while (dht11_read(pin)) {
    if (++count >= DHT_MAXCOUNT) {
      // Timeout waiting for response.
      //set_default_priority();
      debug("TimedOut while waiting for DHT to pull pin low");
      return DHT_ERROR_TIMEOUT;
    }
  }
 
  // Concept is to meausre the number of loops for each state change
  // and then compare to see if they are 1's or 0's

  // Record pulse widths for the expected result bits.
  for (i=0; i < DHT_PULSES*2; i+=2) {
    // Count how long pin is low and store in pulseCounts[i]
    while (!dht11_read(pin)) {
      if (++pulseCounts[i] >= DHT_MAXCOUNT) {
        // Timeout waiting for response.
        //set_default_priority();
        debug("Timed out while waiting for pin to go high on cycle %d", i);
        return DHT_ERROR_TIMEOUT;
      }
    }
    // Count how long pin is high and store in pulseCounts[i+1]
    while (dht11_read(pin)) {
      if (++pulseCounts[i+1] >= DHT_MAXCOUNT) {
        // Timeout waiting for response.
        //set_default_priority();
        int k;
        for ( k = 0; k <= i; k+=2) {
          debug("Cycle %d: %d, %d", k, pulseCounts[k], pulseCounts[k+1]);
        }
        debug("Timed out while waiting for pin to go low on cycle %d", i);
        return DHT_ERROR_TIMEOUT;
      }
    }
  }


  // Compute the average low pulse width to use as a 50 microsecond reference threshold.
  // Ignore the first two readings because they are a constant 80 microsecond pulse.
  uint32_t threshold = 0;
  for (i=2; i < DHT_PULSES*2; i+=2) {
    threshold += pulseCounts[i];
  }
  threshold /= DHT_PULSES-1;

  // Interpret each high pulse as a 0 or 1 by comparing it to the 50us reference.
  // If the count is less than 50us it must be a ~28us 0 pulse, and if it's higher
  // then it must be a ~70us 1 pulse.
  uint8_t data[5] = {0};
  for (i=3; i < DHT_PULSES*2; i+=2) {
    int index = (i-3)/16;
    data[index] <<= 1;
    if (pulseCounts[i] >= threshold) {
      // One bit for long pulse.
      data[index] |= 1;
    }
    // Else zero bit for short pulse.
  }

  debug("Data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3], data[4]);

  debug("sense polling finished");
  // Reset dht11 pin to high, to wait for next start signal.
  dir = GPIO_OUTPUT;
  if (dht11_gpio_init(pin, pin->pin_number, dir) < 0) 
    errx(EXIT_FAILURE, "Error initializing_3 GPIO as OUTPUT");
  dht11_write( pin, HIGH);
  /*
   * check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
   * print it out if data is good
   */
  if ( (j >= 40) &&
       (data[4] == ( (data[0] + data[1] + data[2] + data[3]) & 0xFF) ) )
  {
    f = data[2] * 9. / 5. + 32;
    debug( "Humidity = %d.%d %% Temperature = %d.%d *C (%.1f *F)\n",
      data[0], data[1], data[2], data[3], f );
    return 10;
  }else {
    debug( "Data not good, skip\n" );
    debug( "Humidity = %d.%d %% Temperature = %d.%d *C\n",
      data[0], data[1], data[2], data[3]);
    return -1;
  }
}
 
void dht11_handle_request(const char *req, void *cookie)
{
    // cookie contains a gpio struct
    struct gpio *pin = (struct gpio *) cookie;

    // Commands are of the form {Command, Arguments}:
    // { atom(), term() }
    int req_index = sizeof(uint16_t);
    if (ei_decode_version(req, &req_index, NULL) < 0)
        errx(EXIT_FAILURE, "Message version issue?");

    int arity;
    if (ei_decode_tuple_header(req, &req_index, &arity) < 0 ||
            arity != 2)
        errx(EXIT_FAILURE, "expecting {cmd, args} tuple");

    char cmd[MAXATOMLEN];
    if (ei_decode_atom(req, &req_index, cmd) < 0)
        errx(EXIT_FAILURE, "expecting command atom");

    char resp[256];
    int resp_index = sizeof(uint16_t); // Space for payload size
    resp[resp_index++] = 'r'; // Response
    ei_encode_version(resp, &resp_index);
    if (strcmp(cmd, "sense") == 0) {
        debug("sense");
        // shoud make a senseResult tuple for reading...
        // but use void for now
        int value = dht11_sense(pin);
        if (value !=-1) 
            ei_encode_long(resp, &resp_index, value);
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "dht11_sense_failed");
        }
    } else if (strcmp(cmd, "read") == 0) {
        debug("read");
        int value = dht11_read(pin);
        if (value !=-1)
            ei_encode_long(resp, &resp_index, value);
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "gpio_read_failed");
        }
    } else if (strcmp(cmd, "write") == 0) {
        long value;
        if (ei_decode_long(req, &req_index, &value) < 0)
            errx(EXIT_FAILURE, "write: didn't get value to write");
        debug("write %ld", value);
        if (dht11_write(pin, value))
            ei_encode_atom(resp, &resp_index, "ok");
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "gpio_write_failed");
        }
    } else
        errx(EXIT_FAILURE, "unknown command: %s", cmd);

    debug("sending response: %d bytes", resp_index);
    erlcmd_send(resp, resp_index);
}


int dht11_main(int argc, char *argv[])
{
    if (argc != 3)
        errx(EXIT_FAILURE, "%s dht11 <pin#>", argv[0]);

    int pin_number = strtol(argv[2], NULL, 0);
    enum gpio_state initial_state = GPIO_OUTPUT;
    struct gpio pin;
    if (dht11_gpio_init(&pin, pin_number, initial_state) < 0)
        errx(EXIT_FAILURE, "Error initializing GPIO %d as %s", pin_number, argv[3]);

    // Set Rpi GPIO pin pullup (here or after triggering the communication with DHT11?)
    if (dht11_write(&pin, HIGH))
            debug("GPIO set High after init");
        else {
            errx(EXIT_FAILURE, "Error setting GPIO %d as high after init", pin_number);
        }
    struct erlcmd handler;
    erlcmd_init(&handler, dht11_handle_request, &pin);

    /* For DHT11, since we don't need interupts, we can get rid of the second fdset
    *  But we should also ensure that the second arg in the poll func is '1', as this
    *  defines how many fdsed structs to poll over (1 or 2)
    */

    for (;;) {
        struct pollfd fdset[2];

        fdset[0].fd = STDIN_FILENO;
        fdset[0].events = POLLIN;
        fdset[0].revents = 0;

        fdset[1].fd = pin.fd;
        fdset[1].events = POLLPRI;
        fdset[1].revents = 0;

        /* Always fill out the fdset structure, but only have poll() monitor
     * the sysfs file if interrupts are enabled.
     */
        int rc = poll(fdset, pin.state == GPIO_INPUT_WITH_INTERRUPTS ? 2 : 1, -1);
        if (rc < 0) {
            // Retry if EINTR
            if (errno == EINTR)
                continue;

            err(EXIT_FAILURE, "poll");
        }

        // This path should be called on pin.write/read from elixir
        // Eventually, the dht11_handle_request method will be called and
        // the results will be sent back to elixir
        if (fdset[0].revents & (POLLIN | POLLHUP))
            erlcmd_process(&handler);

        // This path is called when an interupt is triggered
        if (fdset[1].revents & POLLPRI)
            dht11_gpio_process(&pin);
    }

    return 0;
}