#include <stido.h>
#include <iostream>
// ...


struct header {
  unsigned char message_type;
  unsigned short length_in_byte;
  unsigned char header_checksum;
};

struct message {
  unsigned char[255] body;
}

unsigned char start_byte = 0x7F;
header MHeader;
message Message;

int main(void)
{
  while(true)
  {
    usleep(1000);
  }
  return 0;
}
