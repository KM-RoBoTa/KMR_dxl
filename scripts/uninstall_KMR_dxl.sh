#!/bin/sh

# Check if sudo is used
if [ "$(id -u)" != 0 ]; then
  echo 'Sorry, you need to run this script with sudo'
  exit 1
fi

# Confirm action
read -sp "This will uninstall the KMR_dxl library. Are you sure? [y/n]" -n 1 -r
    echo    # (optional) move to a new line

    if [[ $REPLY =~ ^[Yy]$ ]];then
        echo "Uninstalling the library..."
        rm /usr/local/lib/libKMR_dxl.a
        rm -r /usr/local/include/KMR_dxl
    else
        echo "Exiting without doing anything"
        exit 0
    fi

echo "The library KMR_dxl has been successfully deleted. Exiting"
