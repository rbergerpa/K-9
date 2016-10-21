#!/bin/bash

cd `dirname $0`

gunicorn -w 4 -b 0.0.0.0:80 drive:app
