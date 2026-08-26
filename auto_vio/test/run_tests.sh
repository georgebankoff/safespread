#!/bin/sh
# Build and run every host test. Run from this directory.
set -e
out=$(mktemp -d)
for t in nav_test parse_test turn_test route_test circle_fit_test steering_test line_test throttle_test; do
  g++ -std=c++17 -I. -o "$out/$t" "$t.cpp"
  "$out/$t"
done
echo "--- all host tests passed ---"
