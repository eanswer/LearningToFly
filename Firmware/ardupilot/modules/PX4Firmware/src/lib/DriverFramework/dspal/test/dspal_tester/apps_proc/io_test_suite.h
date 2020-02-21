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

#ifndef IO_TEST_SUITE_H_
#define IO_TEST_SUITE_H_


/**
 * @brief Runs all the io tests.
 *
 * @par
 * Tests that are run (in order)
 * 1) dspal_tester_spi_test
 * 2) dspal_tester_serial_test
 * 3) dspal_tester_i2c_test
 * 4) dspal_tester_test_gpio_open_close
 * 5) dspal_tester_test_gpio_ioctl_io
 * 6) dspal_tester_test_gpio_read_write
 * 7) dspal_tester_test_gpio_int
 * 8) dspal_tester_test_posix_file_open
 * 9) dspal_tester_test_posix_file_close
 * 10) dspal_tester_test_posix_file_read
 * 11) dspal_tester_test_posix_file_write
 * 12) dspal_tester_test_posix_file_ioctl
 *
 * @return
 * TEST_PASS ------ All tests passed
 * TEST_FAIL ------ One or more tests failed
*/
int run_io_test_suite(void);

#endif /* IO_TEST_SUITE_H_ */
