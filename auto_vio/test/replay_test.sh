#!/bin/sh
set -eu

out_dir=${1:-"$(mktemp -d)"}
binary="$out_dir/replay"
fixture=../replay/fixtures/straight_pass.csv

g++ -std=c++17 -I.. -o "$binary" ../replay/replay.cpp

set +e
summary=$("$binary" "$fixture")
status=$?
set -e
[ "$status" -eq 2 ] || {
  echo "expected stale fixture to reject acceptance with status 2, got $status" >&2
  exit 1
}
expected='accepted=11 rejected=1 max_cross_track_ft=0.030 p95_cross_track_ft=0.030 max_pose_age_ms=500 steering_saturation_pct=0.0 fault=0 final_route_index=10 acceptance=rejected'
[ "$summary" = "$expected" ] || {
  echo "unexpected replay summary" >&2
  echo "want: $expected" >&2
  echo " got: $summary" >&2
  exit 1
}

{
  sed -n '1,11p' "$fixture"
  sed -n '13p' "$fixture" | sed 's/^1500,/1000,/'
} >"$out_dir/clean.csv"
"$binary" "$out_dir/clean.csv" >"$out_dir/clean_summary"
grep -F 'accepted=11 rejected=0' "$out_dir/clean_summary" >/dev/null
grep -F 'max_pose_age_ms=100' "$out_dir/clean_summary" >/dev/null
grep -F 'fault=0 final_route_index=10 acceptance=accepted' "$out_dir/clean_summary" >/dev/null

assert_rejected() {
  fixture_path=$1
  expected_error=$2
  if "$binary" "$fixture_path" >"$out_dir/stdout" 2>"$out_dir/stderr"; then
    echo "expected replay rejection for $fixture_path" >&2
    exit 1
  fi
  grep -F "$expected_error" "$out_dir/stderr" >/dev/null || {
    echo "missing error '$expected_error'" >&2
    sed -n '1,20p' "$out_dir/stderr" >&2
    exit 1
  }
}

printf '%s\n' 'wrong,header' >"$out_dir/missing_header.csv"
assert_rejected "$out_dir/missing_header.csv" 'line 1: missing required CSV header'

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture" | sed 's/,0.03,0.0,/,nan,0.0,/'
} >"$out_dir/nonfinite.csv"
assert_rejected "$out_dir/nonfinite.csv" 'line 2: non-finite x_ft'

{
  sed -n '1,2p' "$fixture"
  sed -n '3p' "$fixture" | sed 's/,42,/,43,/'
} >"$out_dir/wrong_epoch.csv"
assert_rejected "$out_dir/wrong_epoch.csv" 'line 3: epoch changed from 42 to 43'

{
  sed -n '1p' "$fixture"
  printf '%s\n' '100,2,42,0.02,0.1,0.3,1.00,0.0,true,1,0.02'
} >"$out_dir/malformed.csv"
assert_rejected "$out_dir/malformed.csv" 'line 2: malformed row'

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture" | sed 's/,0.03,0.4,1704,/,0.20,0.4,1704,/'
} >"$out_dir/out_of_budget.csv"
if "$binary" "$out_dir/out_of_budget.csv" >"$out_dir/acceptance"; then
  echo 'expected out-of-budget replay to reject acceptance' >&2
  exit 1
fi
grep -F 'acceptance=rejected' "$out_dir/acceptance" >/dev/null

{
  sed -n '1p' "$fixture"
  printf '%s\n' '50,,,,,,,,,,,,,,'
  printf '%s\n' '75,1,42,0.03,0.0,0.4,1.00,0.1,true,,,,,,'
  sed -n '2p' "$fixture"
  printf '%s\n' '125,,,,,,,,,,,,,,'
} >"$out_dir/mixed_healthy_export.csv"
"$binary" "$out_dir/mixed_healthy_export.csv" >"$out_dir/mixed_summary"
grep -F 'accepted=1 rejected=0' "$out_dir/mixed_summary" >/dev/null
grep -F 'acceptance=accepted' "$out_dir/mixed_summary" >/dev/null

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture" | sed 's/,true,/,false,/'
} >"$out_dir/tracking_fault.csv"
if "$binary" "$out_dir/tracking_fault.csv" >"$out_dir/tracking_summary"; then
  echo 'expected degraded tracking to reject acceptance' >&2
  exit 1
fi
grep -F 'accepted=0 rejected=1' "$out_dir/tracking_summary" >/dev/null
grep -F 'fault=9' "$out_dir/tracking_summary" >/dev/null

{
  sed -n '1,2p' "$fixture"
  printf '%s\n' '100,,,,,,,,,,,,,,pose transport failed'
} >"$out_dir/phone_fault.csv"
if "$binary" "$out_dir/phone_fault.csv" >"$out_dir/phone_fault_summary"; then
  echo 'expected phone-origin fault to reject acceptance' >&2
  exit 1
fi
grep -F 'fault=1' "$out_dir/phone_fault_summary" >/dev/null

{
  sed -n '1,2p' "$fixture"
  printf '%s\n' '100,2,42,,,,1.0,,,1,0.02,0.3,1700,1600,9'
} >"$out_dir/fault_buffer_export.csv"
if "$binary" "$out_dir/fault_buffer_export.csv" >"$out_dir/fault_buffer_summary" 2>"$out_dir/fault_buffer_stderr"; then
  echo 'expected fault-buffer record to reject acceptance' >&2
  exit 1
fi
[ ! -s "$out_dir/fault_buffer_stderr" ] || {
  echo 'fault-buffer export was treated as malformed' >&2
  sed -n '1,20p' "$out_dir/fault_buffer_stderr" >&2
  exit 1
}
grep -F 'fault=9' "$out_dir/fault_buffer_summary" >/dev/null

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture"
  sed -n '3p' "$fixture" | sed 's/,1,0.02,/,81,0.02,/'
} >"$out_dir/route_jump.csv"
if "$binary" "$out_dir/route_jump.csv" >"$out_dir/route_jump_summary"; then
  echo 'expected impossible route-index jump to reject acceptance' >&2
  exit 1
fi
grep -F 'fault=10' "$out_dir/route_jump_summary" >/dev/null

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture" | sed 's/,0,0.03,/,2,0.03,/'
  sed -n '3p' "$fixture" | sed 's/,1,0.02,/,1,0.02,/'
} >"$out_dir/route_regression.csv"
if "$binary" "$out_dir/route_regression.csv" >"$out_dir/route_regression_summary"; then
  echo 'expected route-index regression to reject acceptance' >&2
  exit 1
fi
grep -F 'fault=10' "$out_dir/route_regression_summary" >/dev/null

{
  sed -n '1p' "$fixture"
  sed -n '2p' "$fixture" | sed 's/,1704,/,800,/'
  sed -n '3p' "$fixture"
} >"$out_dir/steering_saturation.csv"
if "$binary" "$out_dir/steering_saturation.csv" >"$out_dir/saturation_summary"; then
  echo 'expected excessive steering saturation to reject acceptance' >&2
  exit 1
fi
grep -F 'steering_saturation_pct=50.0' "$out_dir/saturation_summary" >/dev/null
grep -F 'acceptance=rejected' "$out_dir/saturation_summary" >/dev/null

echo 'replay_test: all assertions passed'
