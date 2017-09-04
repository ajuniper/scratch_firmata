#!/bin/bash
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

while true ; do
    /usr/local/bin/scratchdaemon "$@"
    echo "restarting..."
done
