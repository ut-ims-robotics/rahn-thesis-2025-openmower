#!/bin/bash
echo "TRIGGERING EMERGENCY NOW"
rosservice call /ll/_service/emergency "emergency: 1"
