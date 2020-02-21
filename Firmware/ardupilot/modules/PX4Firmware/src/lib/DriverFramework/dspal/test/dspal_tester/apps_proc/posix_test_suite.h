/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef POSIX_TEST_SUITE_H
#define POSIX_TEST_SUITE_H

#include "AEEStdDef.h"

/**
 * @brief Runs all the posix tests.
 *
 * @par
 * Tests that are run (in order)
 * 1) dspal_tester_test_clockid
 * 2) dspal_tester_test_sigevent
 * 3) dspal_tester_test_time
 * 4) dspal_tester_test_timer_realtime_sig_none
 * 5) dspal_tester_test_timer_monotonic_sig_none
 * 6) dspal_tester_test_timer_process_cputime_sig_none
 * 7) dspal_tester_test_timer_thread_cputime_sig_none
 * 8) dspal_tester_test_time_return_value
 * 9) dspal_tester_test_time_param
 * 10) dspal_tester_test_usleep
 * 11) dspal_tester_test_clock_getres
 * 12) dspal_tester_test_clock_gettime
 * 13) dspal_tester_test_clock_settime
 * 14) dspal_tester_test_one_shot_timer_cb
 * 15) dspal_tester_test_periodic_timer_cb
 * 16) dspal_tester_test_periodic_timer_signal_cb
 * 17) dspal_tester_test_periodic_timer_sigwait
 * 18) dspal_tester_test_pthread_attr_init
 * 19) dspal_tester_test_pthread_create
 * 20) dspal_tester_test_pthread_cancel
 * 21) dspal_tester_test_pthread_self
 * 22) dspal_tester_test_pthread_exit
 * 23) dspal_tester_test_pthread_kill
 * 24) dspal_tester_test_pthread_mutex_lock
 * 25) dspal_tester_test_pthread_mutex_lock_thread
 * 26) dspal_tester_test_pthread_stack
 * 27) dspal_tester_test_pthread_heap
 * 28) dspal_tester_test_usleep
 * 29) dspal_tester_test_semaphore_wait
 *
 * @return
 * TEST_PASS ------ All tests passed
 * TEST_FAIL ------ One or more tests failed
*/
int run_posix_test_suite();

#endif // POSIX_TEST_SUITE_H
