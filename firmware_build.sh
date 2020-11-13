#!/bin/bash
source ../mbed_venv/bin/activate
if [ ${1} = "release" ]; then
	mbed compile -t GCC_ARM --profile release -N firmware --source . --build ./BUILD/RELEASE
elif [ ${1} = "debug" ]; then
	mbed compile -t GCC_ARM --profile debug -N firmware --source . --build ./BUILD/DEBUG
fi
deactivate
