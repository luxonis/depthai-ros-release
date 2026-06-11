#!/bin/bash
set -e 

export DEPTHAI_PLATFORM=$1
export DEPTHAI_PROTOCOL=$2

echo "Running colcon tests in /ws"
echo "DEPTHAI_PLATFORM=$DEPTHAI_PLATFORM"
echo "DEPTHAI_PROTOCOL=$DEPTHAI_PROTOCOL"

cd /ws

colcon test --merge-install --ctest-args tests || true

colcon test-result --all --verbose
RESULT=$?

if [ $RESULT -eq 0 ]; then
  echo "All tests passed."
else
  echo "Some tests failed."
fi

exit $RESULT
