SHELL=cmd

CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = Project_2_Required.o
PORTN=$(shell type COMPORT.inc)

Project_2_Required.elf: $(OBJ)
	$(CC) $(ARCH) -o Project_2_Required.elf Project_2_Required.o -mips16 -DXPRJ_default=default \
		-legacy-libc -Wl,-Map=Project_2_Required.map
	$(OBJCPY) Project_2_Required.elf
	@echo Success!
	
Project_2_Required.o: Project_2_Required.c
	$(CC) -g -x c -mips16 -Os -c $(ARCH) -MMD -o Project_2_Required.o Project_2_Required.c \
		-DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.d *.map 2>NUL

LoadFlash:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	pro32 -p Project_2_Required.hex

putty:
	@Taskkill /IM putty.exe /F 2>NUL | wait 500
	cmd /c start c:\putty\putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N -v

dummy: Project_2_Required.hex Project_2_Required.map

explorer:
	explorer .
