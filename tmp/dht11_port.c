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
//#include <stdint.h>

#include <err.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "erlcmd.h"

//#define DEBUG
#ifdef DEBUG
#define debug(...) do { fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\r\n"); } while(0)
#else
#define debug(...)
#endif

#define HIGH 1
#define LOW 0
#define MAXTIMINGS  85
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
}
 
dht11Result read_dht11(struct gpio *pin)
{
  uint8_t laststate = HIGH;
  uint8_t counter   = 0;
  uint8_t j   = 0, i;
  float f; /* fahrenheit */
  struct dht11Result result;
  enum gpio_state dir = GPIO_OUTPUT;
 
  dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;
 
  /* pull pin down for 18 milliseconds */
  //pinMode( DHTPIN, OUTPUT );
  //digitalWrite( DHTPIN, LOW );
  gpio_init(pin, pin->pin_number, dir);
  gpio_write(pin, LOW;
  delay( 18 );

  /* then pull it up for 40 microseconds */
  //digitalWrite( DHTPIN, HIGH );
  gpio_write(pin, HIGH);
  delayMicroseconds( 40 );

  /* prepare to read the pin */
  //pinMode( DHTPIN, INPUT );
  dir = GPIO_INPUT;
  gpio_init(pin, pin->pin_number, dir);
 
  /* detect change and read data */
  for ( i = 0; i < MAXTIMINGS; i++ )
  {
    counter = 0;
    //while ( digitalRead( DHTPIN ) == laststate )
    while ( gpio_read( pin ) == laststate)
    {
      counter++;
      delayMicroseconds( 1 );
      if ( counter == 255 )
      {
        break;
      }
    }
    //laststate = digitalRead( DHTPIN );
    laststate = gpio_read(pin);
 
    if ( counter == 255 )
      break;
 
    /* ignore first 3 transitions */
    if ( (i >= 4) && (i % 2 == 0) )
    {
      /* shove each bit into the storage bytes */
      dht11_dat[j / 8] <<= 1;
      if ( counter > 16 )
        dht11_dat[j / 8] |= 1;
      j++;
    }
  }
 

  /*
   * check we read 40 bits (8bit x 5 ) + verify checksum in the last byte
   * print it out if data is good
   */
  if ( (j >= 40) &&
       (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
  {
    f = dht11_dat[2] * 9. / 5. + 32;
    printf( "Humidity = %d.%d %% Temperature = %d.%d *C (%.1f *F)\n",
      dht11_dat[0], dht11_dat[1], dht11_dat[2], dht11_dat[3], f );
  }else  {
    printf( "Data not good, skip\n" );
  }
}

 /**
 * @brief Set pin with the value "0" or "1"
 *
 * @param pin           The pin structure
 * @param       value         Value to set (0 or 1)
 *
 * @return  1 for success, -1 for failure
 */
int gpio_write(struct gpio *pin, unsigned int val)
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
int gpio_read(struct gpio *pin)
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
int gpio_init(struct gpio *pin, unsigned int pin_number, enum gpio_state dir)
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
        if (!sysfs_write_file("/sys/class/gpio/export", pinstr))
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
        while (!sysfs_write_file(direction_path, dir_string) &&
               retries > 0) {
            usleep(1000);
            retries--;
        }
        if (retries == 0)
            return -1;
    }

    pin->pin_number = pin_number;

    /* Open the value file for quick access later */
    pin->fd = open(value_path, pin->state == GPIO_OUTPUT ? O_RDWR : O_RDONLY);
    if (pin->fd < 0)
        return -1;

    return 1;
}
 
//int dht11_main( void )
//{
//  printf( "Raspberry Pi wiringPi DHT11 Temperature test program\n" );
//
//  while ( 1 )
//  {
//    read_dht11_dat();
//    delay( 1000 ); /* wait 1sec to refresh */
//  }
// 
//  return(0);
//}
//

void gpio_handle_request(const char *req, void *cookie)
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
        string result = dht11_read(pin);
        if (value !=-1) //encode tuple???
            ei_encode_long(resp, &resp_index, value);
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "gpio_read_failed");
        }
    } else if (strcmp(cmd, "read") == 0) {
        debug("read");
        int value = gpio_read(pin);
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
        debug("write %d", value);
        if (gpio_write(pin, value))
            ei_encode_atom(resp, &resp_index, "ok");
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "gpio_write_failed");
        }
    } else if (strcmp(cmd, "set_int") == 0) {
        char mode[32];
        if (ei_decode_atom(req, &req_index, mode) < 0)
            errx(EXIT_FAILURE, "set_int: didn't get value");
        debug("write %s", mode);

        if (gpio_set_int(pin, mode))
            ei_encode_atom(resp, &resp_index, "ok");
        else {
            ei_encode_tuple_header(resp, &resp_index, 2);
            ei_encode_atom(resp, &resp_index, "error");
            ei_encode_atom(resp, &resp_index, "gpio_set_int_failed");
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
    if (gpio_init(&pin, pin_number, initial_state) < 0)
        errx(EXIT_FAILURE, "Error initializing GPIO %d as %s", pin_number, argv[3]);

    // TODO - set Rpi GPIO pin pullup (here or after triggering the communication with DHT11?)
    struct erlcmd handler;
    erlcmd_init(&handler, gpio_handle_request, &pin);

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
        // Eventually, the gpio_handle_request method will be called and
        // the results will be sent back to elixir
        if (fdset[0].revents & (POLLIN | POLLHUP))
            erlcmd_process(&handler);

        // This path is called when an interupt is triggered
        if (fdset[1].revents & POLLPRI)
            gpio_process(&pin);
    }

    return 0;
}