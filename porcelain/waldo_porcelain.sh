#!/bin/bash

function waldo_connect {
  roslaunch waldo_moveit real.launch
}

function waldo_view {
  roslaunch waldo_moveit rviz.launch
}

function waldo {
  command=$1

  if [ "$command" = "connect" ]; then
    waldo_connect "${@:2}"
  fi

  if [ "$command" = "view" ]; then
    waldo_view "${@:2}"
  fi
}
