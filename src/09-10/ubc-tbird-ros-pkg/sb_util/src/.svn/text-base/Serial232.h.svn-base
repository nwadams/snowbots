/**  @file serial232.h
 * Implements communication over a serial port.
**/


#ifndef FURIOUS_DRIVER_SERIAL232_H
#define FURIOUS_DRIVER_SERIAL232_H

namespace sb
{
  class Serial232
  {
    public:
    /**  Constructor.  Takes the port number and the baud rate. */
    Serial232(const std::string name, int baud);

    /**  Destructor.  Resets then closes port of file descriptor fd. */
    ~Serial232();

    /**  Reads up to n chars and puts it into buf. */
    int read(char *buf, int n);

    /**  Writes the string pointed to by buf, up to n chars.  Returns the 
     * number of bytes written. */
    int write(const char *buf, int n);

    private:
    int fd;
    struct termios oldtermios;
  }; /* end class Serial232 */

} /* end namespace sb */

#endif // FURIOUS_DRIVER_SERIAL232_H
