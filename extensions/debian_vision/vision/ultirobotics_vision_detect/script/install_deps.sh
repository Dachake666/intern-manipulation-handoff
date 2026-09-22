#!/bin/sh -e

mkdir -p deps
vcs import --workers 3 deps < deps.yaml
