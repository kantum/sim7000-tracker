set(spm_KERNEL_HEX_NAME zephyr.hex)
set(spm_ZEPHYR_BINARY_DIR /home/kantum/depots/swoopin-iot/build/spm/zephyr)
set(spm_KERNEL_ELF_NAME zephyr.elf)
list(APPEND spm_BUILD_BYPRODUCTS /home/kantum/depots/swoopin-iot/build/spm/zephyr/zephyr.hex)
list(APPEND spm_BUILD_BYPRODUCTS /home/kantum/depots/swoopin-iot/build/spm/zephyr/zephyr.elf)
list(APPEND spm_BUILD_BYPRODUCTS
    /home/kantum/depots/swoopin-iot/build/spm/libspmsecureentries.a)
