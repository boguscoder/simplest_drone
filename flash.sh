#!/bin/sh
set -u

if command -v probe-rp-usb >/dev/null 2>&1; then
    probe-rp-usb reset >/dev/null 2>&1 || true
fi

i=0
while [ "$i" -lt 50 ]; do
    if picotool info >/dev/null 2>&1; then
        break
    fi
    i=$((i + 1))
    sleep 0.2
done

exec picotool load -u -v -x -t elf "$1"
