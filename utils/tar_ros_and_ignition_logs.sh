#!/bin/bash
pushd ~
find .ros  -name "*.log" | tar -cvzf ~/tb4desk/issues/ros_logs.tz -T -
find .ignition  -name "*.log" | tar -cvzf ~/tb4desk/issues/ignition_logs.tz -T -
popd

