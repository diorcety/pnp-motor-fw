/*
 * Copyright (c) 2020 Siddharth Chandrasekaran <siddharth@embedjournal.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "circbuf.h"

int __circ_gbuf_pop(circ_gbuf_t *circ_buf, void *elem, int read_only)
{
  int total;
  char *tail;

  total = circ_buf->push_count - circ_buf->pop_count;
  if (total < 0)
    total += (2 * circ_buf->size);

  if (total == 0)
    return -1; // Empty

  tail = (char *)circ_buf->buffer + ((circ_buf->pop_count % circ_buf->size)
                                     * circ_buf->element_size);

  if (elem)
    memcpy(elem, tail, circ_buf->element_size);

  if (!read_only)
  {
#ifdef CRICBUF_CLEAN_ON_POP
    memset(tail, 0, circ_buf->element_size);
#endif
    circ_buf->pop_count++;
    if (circ_buf->pop_count >= (2*circ_buf->size))
      circ_buf->pop_count = 0;
  }
  return 0;
}

int __circ_gbuf_push(circ_gbuf_t *circ_buf, void *elem)
{
  int total;
  char *head;

  total = circ_buf->push_count - circ_buf->pop_count;
  if (total < 0)
    total += (2 * circ_buf->size);

  if (total >=  circ_buf->size)
    return -1; // Full

  head = (char *)circ_buf->buffer + ( (circ_buf->push_count % circ_buf->size)
                                      * circ_buf->element_size );
  memcpy(head, elem, circ_buf->element_size);
  circ_buf->push_count++;
  if (circ_buf->push_count >= (2*circ_buf->size))
    circ_buf->push_count = 0;
  return 0;
}

int __circ_gbuf_free_space(circ_gbuf_t *circ_buf)
{
  return circ_buf->size - __circ_gbuf_used_space(circ_buf);
}

int __circ_gbuf_used_space(circ_gbuf_t *circ_buf)
{
  int total;

  total = circ_buf->push_count - circ_buf->pop_count;
  if (total < 0)
    total += (2 * circ_buf->size);

  return total;
}