#!/bin/bash
for hat in *.txtdef; do
  echo "making HAT EEPROM for: ${hat%.txtdef}"
  if [ ! -f "../${hat%.txtdef}.dtbo" ]; then
    echo "ERROR: missing overlay /${hat%.txtdef}.dtbo"
  fi

  ./eepmake $hat ${hat%.txtdef}.eep ../${hat%.txtdef}.dtbo
done
