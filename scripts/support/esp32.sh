#!/bin/bash


cmd_flash() {
	local esptool=${ESP_IDF_PATH}/components/esptool_py/esptool/esptool.py
	local elf_name=${O}/${KERNEL_ELF_NAME}

	echo "Converting ELF to BIN"
	${esptool} --chip esp32 elf2image ${elf_name}

	echo "Flashing"
	${esptool} --chip esp32 \
		--port /dev/ttyUSB0 \
		--baud 115200 \
		--before default_reset \
		--after hard_reset \
		write_flash \
		-u \
		--flash_mode dio \
		--flash_freq 40m \
		--flash_size detect \
		0x1000 ${elf_name/.elf/.bin}
}

CMD="$1"; shift
case "${CMD}" in
   flash)
   	cmd_flash "$@"
   	;;
   *)
   	echo "${CMD} not supported"
   	exit 1
   	;;
esac
