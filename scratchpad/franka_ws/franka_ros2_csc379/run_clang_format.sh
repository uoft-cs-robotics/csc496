#!/bin/bash

find . -regextype egrep -regex ".*\.(c|cpp|h|hpp)$" -not -path '*/install/*' \
  -not -path '*/build/*' -not -path '*/log/*' -not -path '*/deps/*' -not -path '*/arduino/*' | xargs clang-format-11 -i
