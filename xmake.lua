add_rules("mode.debug","mode.release")

toolchain("MCUXpressoIDE")
	set_kind("standalone")
	if is_host("windows") then
		set_sdkdir("D:/IDE/nxp/MCUXpressoIDE_11.5.1_7266/ide/tools")
	else
		set_sdkdir("/opt/mcuxpresso-ide/ide/tools")
	end
toolchain_end()

target("LPCXFlashDriverLib")
	set_kind("static")
	set_languages("c99")
	set_toolchains("MCUXpressoIDE")

	add_defines("__REDLIB__", "NDEBUG", "__CODE_RED", "__CORE_M0", "__GENERIC_M0", "SECTOR_HASHING")

	add_cflags("-Os", "-Wall", "-fmessage-length=0",
		"-fno-builtin", "-ffunction-sections", "-fdata-sections",
		"-mcpu=cortex-m0", "-mthumb", "-fstack-usage",
		"-specs=D:/IDE/nxp/MCUXpressoIDE_11.5.1_7266/ide/tools/arm-none-eabi/lib/redlib.specs", {force=true})

	add_includedirs("LPCXFlashDriverLib/inc")
	add_files("LPCXFlashDriverLib/src/*.c")

target("MIMXRT1050_W25Q256.axf")
	set_kind("binary")
	set_languages("c99")
	set_toolchains("MCUXpressoIDE")

	after_build(function (target)
		-- local s = format("%s/%s.cfx", path.directory(target:targetfile()), path.basename(target:name()))
		-- print(s)
		local s = format("iMXRT1050_QSPI/builds/%s.cfx", path.basename(target:name()))
		os.cp(target:targetfile(), s)
	end)

	after_clean(function (target)
		-- local s = format("%s/%s.cfx", path.directory(target:targetfile()), path.basename(target:name()))
		-- os.rm(s)
		os.rm("build/iMXRT1050_QSPI.map")
	end)

	add_deps("LPCXFlashDriverLib")

	add_defines("__REDLIB__", "__MCUXPRESSO", "__USE_CMSIS", "NDEBUG",
		"CPU_MIMXRT1052DVL6B_cm7", "CPU_MIMXRT1052DVL6B")

	add_cflags("-Os", "-Wall", "-fmessage-length=0", "-fno-builtin",
		"-ffunction-sections", "-fdata-sections", "-fsingle-precision-constant",
		"-mcpu=cortex-m7", "-mthumb", "-fstack-usage",
		"-specs=D:/IDE/nxp/MCUXpressoIDE_11.5.1_7266/ide/tools/arm-none-eabi/lib/redlib.specs", {force=true})

	add_asflags("-c", "-x", "assembler-with-cpp", "-g3", "-mcpu=cortex-m7",
		"-mthumb", "-specs=D:/IDE/nxp/MCUXpressoIDE_11.5.1_7266/ide/tools/arm-none-eabi/lib/redlib.specs", {force=true})

	add_ldflags("-nostdlib", "-s", "-Xlinker -print-memory-usage", "-Xlinker --gc-sections",
		"-Xlinker -Map=build/iMXRT1050_QSPI.map", "-mcpu=cortex-m7",
		"-mthumb", "-T FlashDriver_32Kbuffer.ld", {force=true})
	
	add_linkdirs("iMXRT1050_QSPI/linkscripts")

	add_includedirs("iMXRT1050_QSPI/utilities", "iMXRT1050_QSPI/QSPIsource",
		"iMXRT1050_QSPI/drivers", "iMXRT1050_QSPI/CMSIS", "iMXRT1050_QSPI/board",
		"LPCXFlashDriverLib/inc", "D:/IDE/nxp/MCUXpressoIDE_11.5.1_7266/ide/tools/redlib/include")

	add_files("iMXRT1050_QSPI/board/*.c", "iMXRT1050_QSPI/drivers/*.c",
		"iMXRT1050_QSPI/QSPIsource/*.c", "iMXRT1050_QSPI/source/*.c",
		"iMXRT1050_QSPI/utilities/*.c", "iMXRT1050_QSPI/CMSIS/*.c",
		"iMXRT1050_QSPI/source/checkblank.s")
