#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

#pragma pack(push, 1)
struct packed_data
{
  uint8_t first;
  uint16_t second;
  uint8_t third;
  uint8_t fourth;
};
#pragma pack(pop)

struct standard_data
{
  uint8_t first;
  uint16_t second;
  uint8_t third;
  uint8_t fourth;
};

int main ()
{
  struct packed_data packed_data;
  struct standard_data standard_data;

  printf("packed_data_offsets:\n\tfirst %ld\n", offsetof(struct packed_data, first));
  printf("\tsecond %ld\n", offsetof(struct packed_data, second));
  printf("\tthird %ld\n", offsetof(struct packed_data, third));
  printf("\tfourth %ld\n", offsetof(struct packed_data, fourth));

  printf ("\n\n");

  printf("standard_data_offsets:\n\tfirst %ld\n", offsetof(struct standard_data, first));
  printf("\tsecond %ld\n", offsetof(struct standard_data, second));
  printf("\tthird %ld\n", offsetof(struct standard_data, third));
  printf("\tfourth %ld\n", offsetof(struct standard_data, fourth));


  return 0;
}