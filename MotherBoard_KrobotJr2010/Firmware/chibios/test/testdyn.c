/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "test.h"

/**
 * @page test_dynamic Dynamic APIs test
 *
 * <h2>Description</h2>
 * This module implements the test sequence for the dynamic thread creation
 * APIs.
 *
 * <h2>Objective</h2>
 * Objective of the test module is to cover 100% of the dynamic APIs code.
 *
 * <h2>Preconditions</h2>
 * The module requires the following kernel options:
 * - @p CH_USE_DYNAMIC
 * - @p CH_USE_HEAP
 * - @p CH_USE_MEMPOOLS
 * .
 * In case some of the required options are not enabled then some or all tests
 * may be skipped.
 *
 * <h2>Test Cases</h2>
 * - @subpage test_dynamic_001
 * - @subpage test_dynamic_002
 * .
 * @file testdyn.c
 * @brief Dynamic thread APIs test source file
 * @file testdyn.h
 * @brief Dynamic thread APIs test header file
 */

#if CH_USE_DYNAMIC

/**
 * @page test_dynamic_001 Threads creation from Memory Heap
 *
 * <h2>Description</h2>
 * Two threads are started by allocating the memory from the Memory Heap then
 * the remaining heap space is arbitrarily allocated and a third tread startup
 * is attempted.<br>
 * The test expects the first two threads to successfully start and the last
 * one to fail.
 */

static msg_t thread(void *p) {

  test_emit_token(*(char *)p);
  return 0;
}

#if CH_USE_HEAP

static MemoryHeap heap1;

static char *dyn1_gettest(void) {

  return "Dynamic APIs, threads creation from heap";
}

static void dyn1_setup(void) {

  chHeapInit(&heap1, test.buffer, sizeof(union test_buffers));
}

static void dyn1_execute(void) {
  size_t n, sz;
  void *p1;
  tprio_t prio = chThdGetPriority();

  (void)chHeapStatus(&heap1, &sz);
  /* Starting threads from the heap. */
  threads[0] = chThdCreateFromHeap(&heap1, THD_WA_SIZE(THREADS_STACK_SIZE),
                                   prio-1, thread, "A");
  threads[1] = chThdCreateFromHeap(&heap1, THD_WA_SIZE(THREADS_STACK_SIZE),
                                   prio-2, thread, "B");
  /* Allocating the whole heap in order to make the thread creation fail.*/
  (void)chHeapStatus(&heap1, &n);
  p1 = chHeapAlloc(&heap1, n);
  threads[2] = chThdCreateFromHeap(&heap1, THD_WA_SIZE(THREADS_STACK_SIZE),
                                   prio-3, thread, "C");
  chHeapFree(p1);

  test_assert(1, (threads[0] != NULL) &&
                 (threads[1] != NULL) &&
                 (threads[2] == NULL) &&
                 (threads[3] == NULL) &&
                 (threads[4] == NULL),
                 "thread creation failed");

  /* Claiming the memory from terminated threads. */
  test_wait_threads();
  test_assert_sequence(2, "AB");

  /* Heap status checked again.*/
  test_assert(3, chHeapStatus(&heap1, &n) == 1, "heap fragmented");
  test_assert(4, n == sz, "heap size changed");
}

const struct testcase testdyn1 = {
  dyn1_gettest,
  dyn1_setup,
  NULL,
  dyn1_execute
};
#endif /* CH_USE_HEAP */

#if CH_USE_MEMPOOLS
/**
 * @page test_dynamic_002 Threads creation from Memory Pool
 *
 * <h2>Description</h2>
 * Five thread creation are attempted from a pool containing only four
 * elements.<br>
 * The test expects the first four threads to successfully start and the last
 * one to fail.
 */

static MemoryPool mp1;

static char *dyn2_gettest(void) {

  return "Dynamic APIs, threads creation from memory pool";
}

static void dyn2_setup(void) {

  chPoolInit(&mp1, THD_WA_SIZE(THREADS_STACK_SIZE), NULL);
}

static void dyn2_execute(void) {
  int i;
  tprio_t prio = chThdGetPriority();

  /* Adding the WAs to the pool. */
  for (i = 0; i < 4; i++)
    chPoolFree(&mp1, wa[i]);

  /* Starting threads from the memory pool. */
  threads[0] = chThdCreateFromMemoryPool(&mp1, prio-1, thread, "A");
  threads[1] = chThdCreateFromMemoryPool(&mp1, prio-2, thread, "B");
  threads[2] = chThdCreateFromMemoryPool(&mp1, prio-3, thread, "C");
  threads[3] = chThdCreateFromMemoryPool(&mp1, prio-4, thread, "D");
  threads[4] = chThdCreateFromMemoryPool(&mp1, prio-5, thread, "E");

  test_assert(1, (threads[0] != NULL) &&
                 (threads[1] != NULL) &&
                 (threads[2] != NULL) &&
                 (threads[3] != NULL) &&
                 (threads[4] == NULL),
                 "thread creation failed");

  /* Claiming the memory from terminated threads. */
  test_wait_threads();
  test_assert_sequence(2, "ABCD");

  /* Now the pool must be full again. */
  for (i = 0; i < 4; i++)
    test_assert(3, chPoolAlloc(&mp1) != NULL, "pool list empty");
  test_assert(4, chPoolAlloc(&mp1) == NULL, "pool list not empty");
}

const struct testcase testdyn2 = {
  dyn2_gettest,
  dyn2_setup,
  NULL,
  dyn2_execute
};
#endif /* CH_USE_MEMPOOLS */

#endif /* CH_USE_DYNAMIC */

/*
 * Test sequence for dynamic APIs pattern.
 */
const struct testcase * const patterndyn[] = {
#if CH_USE_DYNAMIC
#if CH_USE_HEAP
  &testdyn1,
#endif
#if CH_USE_MEMPOOLS
  &testdyn2,
#endif
#endif
  NULL
};
