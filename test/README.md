# aiousb-linux test directory.

These tests are created when developing new features or testing specific issues. There is no effort made to keep them bug free, and in rare cases could cause damage if run with the wrong board.

Please look in the samples to see if there is one that covers the feature you're wanting to exercise before attempting to use anything in this directory.

## Debugging
If CMAKE_BUILD_TYPE is Debug the library will be built with `-O0 -g` to enable debugging. The kernel module cannot be built as debug, but `AIO_DEBUG=1` can be used in the cmake configuration to turn on output. This will impact driver performance and is not recommended for production.

### Attaching C++ debugger to Python process
python3 aiousb_poc.py debug
debugger start "attach to libaiousb.so"
select process. Breakpoints will get hit when python calls into aiousb

### Watch notes
* *(char(*)[32])config_buff //shows contents of string
