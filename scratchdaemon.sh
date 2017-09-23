#!/bin/bash
# copy this file to /usr/local/bin
exec >>/var/log/scratchdaemon.log 2>&1

set -x
if [[ $1 = inudev ]] ; then
    shift
    setsid "$0" "$@" </dev/null &
    exit
fi
if [[ $1 = kill ]] ; then
    shift
    kill $(ps --no-headers -o pid -C "${0##*/} $*")
    sleep 1
    kill $(ps --no-headers -o pid -C "scratchdaemon $*")
    exit
fi

lock=/var/run/scratchdaemon.$(printf "%s_" "$@")
exec 9>$lock
flock -n 9 || { echo "Already locked, exit" ; exit 1 ; }

while true ; do
    SECONDS=0
    /usr/local/bin/scratchdaemon "$@"
    [[ $SECONDS -ge 10 ]] || sleep $((10-SECONDS))
    echo "restarting..."
done
