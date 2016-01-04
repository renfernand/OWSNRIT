################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
kernel/openos/%.o: ../kernel/openos/%.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"c:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-gcc.exe" -c -I"C:/OWSNRIT8/openwsn/inc" -I"C:/OWSNRIT8/openwsn/openstack" -I"C:/OWSNRIT8/openwsn/openstack/cross-layers" -I"C:/OWSNRIT8/openwsn/openstack/02a-MAClow" -I"C:/OWSNRIT8/openwsn/openstack/02b-MAChigh" -I"C:/OWSNRIT8/openwsn/openstack/03a-IPHC" -I"C:/OWSNRIT8/openwsn/openstack/03b-IPv6" -I"C:/OWSNRIT8/openwsn/openstack/04-TRAN" -I"C:/OWSNRIT8/openwsn/openapps" -I"C:/OWSNRIT8/openwsn/openapps/tohlone" -I"C:/OWSNRIT8/openwsn/openapps/c6t" -I"C:/OWSNRIT8/openwsn/openapps/cexample" -I"C:/OWSNRIT8/openwsn/openapps/cinfo" -I"C:/OWSNRIT8/openwsn/openapps/cleds" -I"C:/OWSNRIT8/openwsn/openapps/osens" -I"C:/OWSNRIT8/openwsn/openapps/cwellknown" -I"C:/OWSNRIT8/openwsn/openapps/techo" -I"C:/OWSNRIT8/openwsn/openapps/cstorm" -I"C:/OWSNRIT8/openwsn/openapps/uecho" -I"C:/OWSNRIT8/openwsn/projects/common/03oos_openwsn" -I"C:/OWSNRIT8/openwsn/projects/cc2538em/03oos_openwsn" -I"C:/OWSNRIT8/openwsn/bsp/boards/cc2538em" -I"C:/OWSNRIT8/openwsn/bsp/boards/cc2538em/inc" -I"C:/OWSNRIT8/openwsn/bsp/boards/cc2538em/source" -I"C:/OWSNRIT8/openwsn/bsp/boards/cc2538em/osens_itf" -I"C:/OWSNRIT8/openwsn/bsp/boards" -I"C:/OWSNRIT8/openwsn/drivers" -I"C:/OWSNRIT8/openwsn/drivers/common" -I"C:/OWSNRIT8/openwsn/kernel/openos" -I"C:/OWSNRIT8/openwsn/kernel" -I"c:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/arm-none-eabi/include" -g -gstrict-dwarf -Wall -mthumb -mcpu=cortex-m3 -g3 -O0 -Wstrict-prototypes -Wall -mlittle-endian -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


