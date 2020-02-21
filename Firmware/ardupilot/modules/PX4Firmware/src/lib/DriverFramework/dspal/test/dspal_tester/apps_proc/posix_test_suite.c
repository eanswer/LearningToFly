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

#include <stdio.h>

#include "test_utils.h"
#include "dspal_tester.h"

int run_posix_test_suite(void)
{
	int test_results = TEST_PASS;

	LOG_INFO("testing time.h");

	test_results |= display_test_results(dspal_tester_test_clockid(), "clockid values exist");
	test_results |= display_test_results(dspal_tester_test_sigevent(), "sigevent values exist");
	test_results |= display_test_results(dspal_tester_test_time(), "time returns good value");
	test_results |= display_test_results(dspal_tester_test_timer_realtime_sig_none(), "timer realtime");
	test_results |= display_test_results(dspal_tester_test_timer_monotonic_sig_none(), "timer monotonic");
	test_results |= display_test_results(dspal_tester_test_timer_process_cputime_sig_none(), "timer process cputime");
	test_results |= display_test_results(dspal_tester_test_timer_thread_cputime_sig_none(), "timer thread cputime");
	test_results |= display_test_results(dspal_tester_test_time_return_value(), "time return value");
	test_results |= display_test_results(dspal_tester_test_time_param(), "time parameter");
	test_results |= display_test_results(dspal_tester_test_usleep(), "usleep for two seconds");
	test_results |= display_test_results(dspal_tester_test_clock_getres(), "clock_getres");
	test_results |= display_test_results(dspal_tester_test_clock_gettime(), "clock_gettime");
	test_results |= display_test_results(dspal_tester_test_clock_settime(), "clock_settime");
	test_results |= display_test_results(dspal_tester_test_one_shot_timer_cb(), "one shot timer cb");
	test_results |= display_test_results(dspal_tester_test_periodic_timer_cb(), "periodic timer cb");
	test_results |= display_test_results(dspal_tester_test_periodic_timer_signal_cb(), "periodic timer signal cb");
	test_results |= display_test_results(dspal_tester_test_periodic_timer_sigwait(), "periodic timer sigwait");

	LOG_INFO("testing pthread.h");

	test_results |= display_test_results(dspal_tester_test_pthread_attr_init(), "pthread attr init");
	test_results |= display_test_results(dspal_tester_test_pthread_create(), "pthread create");
	test_results |= display_test_results(dspal_tester_test_pthread_cancel(), "pthread cancel");
	test_results |= display_test_results(dspal_tester_test_pthread_self(), "pthread self");
	test_results |= display_test_results(dspal_tester_test_pthread_exit(), "pthread exit");
	test_results |= display_test_results(dspal_tester_test_pthread_kill(), "pthread kill");
	test_results |= display_test_results(dspal_tester_test_pthread_cond_timedwait(), "pthread condition timed wait");
	test_results |= display_test_results(dspal_tester_test_pthread_mutex_lock(), "pthread mutex lock");
	test_results |= display_test_results(dspal_tester_test_pthread_mutex_lock_thread(), "thread mutex lock thread");
	test_results |= display_test_results(dspal_tester_test_pthread_stack(), "thread large allocation on stack");
	test_results |= display_test_results(dspal_tester_test_pthread_heap(), "thread large allocation on heap");
	test_results |= display_test_results(dspal_tester_test_usleep(), "usleep for two seconds");

	LOG_INFO("testing semaphore.h");

	test_results |= display_test_results(dspal_tester_test_semaphore_wait(), "semaphore wait");

	LOG_INFO("testing C++");

	test_results |= display_test_results(dspal_tester_test_cxx_heap(), "test C++ heap");
	test_results |= display_test_results(dspal_tester_test_cxx_static(), "test C++ static initialization");

	LOG_INFO("tests complete");

	return test_results;
}
