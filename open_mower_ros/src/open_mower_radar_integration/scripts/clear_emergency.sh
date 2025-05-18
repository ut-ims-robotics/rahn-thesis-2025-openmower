#!/bin/bash
echo "CLEARING EMERGENCY NOW"
rosservice call /ll/_service/emergency "emergency: 0"

