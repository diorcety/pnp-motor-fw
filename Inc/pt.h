/*
MIT License

Copyright (c) 2016 Serge Zaitsev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef PT_H
#define PT_H

#include <stddef.h>

/* Protothread status values */
#define PT_STATUS_IDLE 0
#define PT_STATUS_BLOCKED 1
#define PT_STATUS_YIELDED 2
#define PT_STATUS_FINISHED 3

/* Helper macros to generate unique labels */
#define _pt_line3(name, line) _pt_##name##line
#define _pt_line2(name, line) _pt_line3(name, line)
#define _pt_line(name) _pt_line2(name, __LINE__)

#if PT_USE_SETJMP
/*
 * Local continuation that uses setjmp()/longjmp() to keep thread state.
 *
 * Pros: local variables are preserved, works with all control structures.
 * Cons: slow, may erase global register variables.
 */
#include <setjmp.h>
struct pt
{
  jmp_buf env;
  int isset;
  int status;
  int exitcode;
};
#define pt_init()                                                              \
  { .isset = 0, .status = PT_STATUS_IDLE, .exitcode = 0 }
#define pt_reset(pt)                                                           \
  { (pt)->label = 0; (pt)->status = PT_STATUS_IDLE; (pt)->exitcode = 0;  }
#define pt_begin(pt)                                                           \
  do {                                                                         \
    if ((pt)->isset) {                                                         \
      longjmp((pt)->env, 0);                                                   \
    }                                                                          \
  } while (0)
#define pt_label(pt, stat)                                                     \
  do {                                                                         \
    (pt)->isset = 1;                                                           \
    (pt)->status = (stat);                                                     \
    setjmp((pt)->env);                                                         \
  } while (0)
#define pt_end(pt) pt_label(pt, PT_STATUS_FINISHED)
#elif PT_USE_GOTO
/*
 * Local continuation based on goto label references.
 *
 * Pros: works with all control sturctures.
 * Cons: requires GCC or Clang, doesn't preserve local variables.
 */
struct pt
{
  void *label;
  int status;
};
#define pt_init()                                                              \
  { .label = NULL, .status = PT_STATUS_IDLE, .exitcode = 0 }
#define pt_reset(pt)                                                           \
  { (pt)->label = 0; (pt)->status = PT_STATUS_IDLE; (pt)->exitcode = 0; }
#define pt_begin(pt)                                                           \
  do {                                                                         \
    if ((pt)->label != NULL) {                                                 \
      goto *(pt)->label;                                                       \
    }                                                                          \
  } while (0)

#define pt_label(pt, stat)                                                     \
  do {                                                                         \
    (pt)->status = (stat);                                                     \
    _pt_line(label) : (pt)->label = &&_pt_line(label);                         \
  } while (0)
#define pt_end(pt) pt_label(pt, PT_STATUS_FINISHED)
#else
/*
 * Local continuation based on switch/case and line numbers.
 *
 * Pros: works with any C99 compiler, most simple implementation.
 * Cons: doesn't allow more than one label per line, doesn't preserve local
 * variables.
 */
struct pt
{
  int label;
  int status;
  int exitcode;
};
#define pt_init()                                                              \
  { .label = 0, .status = PT_STATUS_IDLE, .exitcode = 0 }
#define pt_reset(pt)                                                           \
  { (pt)->label = 0; (pt)->status = PT_STATUS_IDLE; (pt)->exitcode = 0; }
#define pt_begin(pt)                                                           \
  switch ((pt)->label) {                                                       \
  case 0:
#define pt_label(pt, stat)                                                     \
  do {                                                                         \
    (pt)->label = __LINE__;                                                    \
    (pt)->status = (stat);                                                     \
    /* FALLTHRU */                                                             \
  case __LINE__:;                                                              \
  } while (0)
#define pt_end(pt)                                                             \
  pt_label(pt, PT_STATUS_FINISHED);                                            \
  }
#endif

/*
 * Core protothreads API
 */
#define pt_status(pt) (pt)->status

#define pt_exitcode(pt) (pt)->exitcode

#define pt_wait(pt, cond)                                                      \
  do {                                                                         \
    pt_label(pt, PT_STATUS_BLOCKED);                                           \
    if (!(cond)) {                                                             \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define pt_yield(pt)                                                           \
  do {                                                                         \
    pt_label(pt, PT_STATUS_YIELDED);                                           \
    if (pt_status(pt) == PT_STATUS_YIELDED) {                                  \
      (pt)->status = PT_STATUS_BLOCKED;                                        \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define pt_exit(pt, ec)                                                        \
  do {                                                                         \
    pt_label(pt, PT_STATUS_FINISHED);                                          \
    (pt)->exitcode = ec;                                                       \
    return;                                                                    \
  } while (0)

#define pt_loop(pt, cond)                                                      \
  for (int _intr = 0; _intr == 0;)                                             \
    if (1) {                                                                   \
      pt_label(pt, PT_STATUS_BLOCKED);                                         \
      if (!(cond)) {                                                           \
        break;                                                                 \
      }                                                                        \
      goto _pt_line(body);                                                     \
    } else                                                                     \
      while (1)                                                                \
        if (1) {                                                               \
          _intr = 1;                                                           \
          break;                                                               \
        } else                                                                 \
          while (1)                                                            \
            if (1) {                                                           \
              return;                                                          \
            } else                                                             \
  _pt_line(body) :

/*
 * Queues
 */
#define pt_queue(T, size)                                                      \
  struct {                                                                     \
    T buf[size];                                                               \
    unsigned int r;                                                            \
    unsigned int w;                                                            \
  }
#define pt_queue_init()                                                        \
  { .r = 0, .w = 0 }
#define pt_queue_len(q) (sizeof((q)->buf) / sizeof((q)->buf[0]))
#define pt_queue_cap(q) ((q)->w - (q)->r)
#define pt_queue_empty(q) ((q)->w == (q)->r)
#define pt_queue_full(q) (pt_queue_cap(q) == pt_queue_len(q))
#define pt_queue_reset(q) ((q)->w = (q)->r = 0)

#define pt_queue_push(q, el)                                                   \
  (!pt_queue_full(q) && ((q)->buf[(q)->w++ % pt_queue_len(q)] = (el), 1))
#define pt_queue_peek(q)                                                       \
  (pt_queue_empty(q) ? NULL : &(q)->buf[(q)->r % pt_queue_len(q)])
#define pt_queue_pop(q)                                                        \
  (pt_queue_empty(q) ? NULL : &(q)->buf[(q)->r++ % pt_queue_len(q)])
#define pt_queue_remove(q)                                                     \
  (pt_queue_empty(q) ? 0 : (q)->w--)

/*
 * Wrapper for system calls and other functions that return -1 and set errno
 */
#define pt_sys(pt, call)                                                       \
  pt_wait(pt,                                                                  \
          (errno = 0) ||                                                       \
              !(((call) == -1) && (errno == EAGAIN || errno == EWOULDBLOCK ||  \
                                   errno == EINPROGRESS || errno == EINTR)))

#endif /* PT_H */
