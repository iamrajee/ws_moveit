#!/bin/bash
rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r
rosdep update
