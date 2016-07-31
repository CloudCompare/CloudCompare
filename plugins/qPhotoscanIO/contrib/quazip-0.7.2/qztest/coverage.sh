#!/bin/sh
lcov --capture -b ../quazip -d ../quazip/.obj --output-file cov.info
genhtml --demangle-cpp cov.info --output-directory cov
