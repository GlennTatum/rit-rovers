// Uses POSIX functions to send and receive data from a
// Simple Motor Controller G2.
// NOTE: The Simple Motor Controller's input mode must be set to Serial/USB.
// NOTE: You must change the 'const char * device' line below.
 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
 
// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    return -1;
  }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result)
  {
    perror("tcflush failed");  // just a warning, not a fatal error
  }
 
  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result)
  {
    perror("tcgetattr failed");
    close(fd);
    return -1;
  }
 
  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 
  // Set up timeouts: Calls to read() will return as soon as there is
  // at least one byte available or when 100 ms has passed.
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 0;
 
  // This code only supports certain standard baud rates. Supporting
  // non-standard baud rates should be possible but takes more work.
  switch (baud_rate)
  {
  case 4800:   cfsetospeed(&options, B4800);   break;
  case 9600:   cfsetospeed(&options, B9600);   break;
  case 19200:  cfsetospeed(&options, B19200);  break;
  case 38400:  cfsetospeed(&options, B38400);  break;
  case 115200: cfsetospeed(&options, B115200); break;
  default:
    fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
      baud_rate);
    cfsetospeed(&options, B9600);
    break;
  }
  cfsetispeed(&options, cfgetospeed(&options));
 
  result = tcsetattr(fd, TCSANOW, &options);
  if (result)
  {
    perror("tcsetattr failed");
    close(fd);
    return -1;
  }
 
  return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int fd, const uint8_t * buffer, size_t size)
{
  ssize_t result = write(fd, buffer, size);
  if (result != (ssize_t)size)
  {
    perror("failed to write to port");
    return -1;
  }
  return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
ssize_t read_port(int fd, uint8_t * buffer, size_t size)
{
  size_t received = 0;
  while (received < size)
  {
    ssize_t r = read(fd, buffer + received, size - received);
    if (r < 0)
    {
      perror("failed to read from port");
      return -1;
    }
    if (r == 0)
    {
      // Timeout
      break;
    }
    received += r;
  }
  return received;
}
 
// Reads a variable from the SMC.
// Returns 0 on success or -1 on failure.
int smc_get_variable(int fd, uint8_t variable_id, uint16_t * value)
{
  uint8_t command[] = { 0xA1, variable_id };
  int result = write_port(fd, command, sizeof(command));
  if (result) { return -1; }
  uint8_t response[2];
  ssize_t received = read_port(fd, response, sizeof(response));
  if (received < 0) { return -1; }
  if (received != 2)
  {
    fprintf(stderr, "read timeout: expected 2 bytes, got %zu\n", received);
    return -1;
  }
  *value = response[0] + 256 * response[1];
  return 0;
}
 
// Gets the target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_get_target_speed(int fd, int16_t * value)
{
  return smc_get_variable(fd, 20, (uint16_t *)value);
}
 
// Gets a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns 0 on success, -1 on failure.
int smc_get_error_status(int fd, uint16_t * value)
{
  return smc_get_variable(fd, 0, value);
}
 
// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 on success, -1 on failure.
int smc_exit_safe_start(int fd)
{
  const uint8_t command = 0x83;
  return write_port(fd, &command, 1);
}
 
// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_set_target_speed(int fd, int speed)
{
  uint8_t command[3];
 
  if (speed < 0)
  {
    command[0] = 0x86; // Motor Reverse
    speed = -speed;
  }
  else
  {
    command[0] = 0x85; // Motor Forward
  }
  command[1] = speed & 0x1F;
  command[2] = speed >> 5 & 0x7F;
 
  return write_port(fd, command, sizeof(command));
}
 
int main()
{
  // Choose the serial port name.
  // Linux USB example:  "/dev/ttyACM0"  (see also: /dev/serial/by-id)
  // macOS USB example:  "/dev/cu.usbmodem001234562"
  // Cygwin example:     "/dev/ttyS7"
  const char * device = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_High-Power_Simple_Motor_Controller_G2_18v25_34FF-7306-4D55-3433-0844-2543-if00";
 
  // Choose the baud rate (bits per second).  This does not matter if you are
  // connecting to the SMC over USB.  If you are connecting via the TX and RX
  // lines, this should match the baud rate in the SMC G2's serial settings.
  uint32_t baud_rate = 9600;
 
  int fd = open_serial_port(device, baud_rate);
  if (fd < 0) { return 1; }
 
  int result = smc_exit_safe_start(fd);
  if (result) { return 1; }
 
  uint16_t error_status;
  result = smc_get_error_status(fd, &error_status);
  if (result) { return 1; }
  printf("Error status: 0x%04x\n", error_status);
 
  int16_t target_speed;
  result = smc_get_target_speed(fd, &target_speed);
  if (result) { return 1; }
  printf("Target speed is %d.\n", target_speed);
 
  int16_t new_speed = (target_speed <= 0) ? 3200 : -3200;
  printf("Setting target speed to %d.\n", new_speed);
  result = smc_set_target_speed(fd, new_speed);
  if (result) { return 1; }
 
  close(fd);
  return 0;
}