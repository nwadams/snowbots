#define MAX_NUM_PORTS 28


typedef struct termios termios_t;


int Get_Serial_Port_Descriptor(int index);
void Set_Serial_Port_Descriptor(int index, int fd);

int Serial_Open (int port_no, int baud, termios_t *oldtermios);
int Serial_Close (int fd, struct termios *oldtermios);
int Serial_Bytes_Waiting (int fd);
int Serial_Read (int fd, char *buf, int n);
void Serial_Write (int fd, char *buf, int n, int *counter);
