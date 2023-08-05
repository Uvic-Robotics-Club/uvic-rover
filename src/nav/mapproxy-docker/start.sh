#!/bin/bash

if [ ! -f /mapproxy/mapproxy.yaml ]; then
    cp /mapproxy.yaml /mapproxy/mapproxy.yaml
fi

cd /mapproxy
mapproxy-util serve-develop -b 0.0.0.0:8080 mapproxy.yaml
