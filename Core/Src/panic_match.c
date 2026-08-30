#include "panic_match.h"

#include <string.h>

static char ascii_lower(char value)
{
  if (value >= 'A' && value <= 'Z')
  {
    return (char)(value - 'A' + 'a');
  }
  return value;
}

bool fc_panic_message_contains(const char *message, size_t message_size,
                               const char *needle)
{
  if (message == NULL || message_size == 0U || needle == NULL)
  {
    return false;
  }

  const size_t needle_size = strlen(needle);
  if (needle_size == 0U || message_size < needle_size)
  {
    return false;
  }

  for (size_t offset = 0; offset + needle_size <= message_size; ++offset)
  {
    size_t index = 0;
    while (index < needle_size &&
           ascii_lower(message[offset + index]) == ascii_lower(needle[index]))
    {
      ++index;
    }
    if (index == needle_size)
    {
      return true;
    }
  }
  return false;
}
