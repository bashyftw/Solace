# makefile configuration
NAME            = tri66
OBJECTS         = tri66.o spi.o can.o
CPU             = msp430x413

ASFLAGS         = -mmcu=${CPU} -x assembler-with-cpp -D_GNU_ASSEMBLER_ -c
CFLAGS          = -mmcu=${CPU} -O2 -Wall -g -I"." -I"c:/mspgcc/msp430/include"

#switch the compiler (for the internal make rules)
CC              = c:/mspgcc/bin/msp430-gcc
AS              = c:/mspgcc/bin/msp430-gcc

.PHONY: all FORCE clean download download-jtag download-bsl dist

#all should be the first target. it's built when make is run without args
all: ${NAME}.elf ${NAME}.a43 ${NAME}.lst

#additional rules for files
${NAME}.elf: ${OBJECTS} 
	${CC} -mmcu=${CPU} -o $@ ${OBJECTS}

${NAME}.a43: ${NAME}.elf
	c:/mspgcc/bin/msp430-objcopy -O ihex $^ $@

${NAME}.lst: ${NAME}.elf
	c:/mspgcc/bin/msp430-objdump -dSt $^ >$@

clean:
	c:/MinGW/msys/1.0/bin/rm -f ${NAME}.elf ${NAME}.a43 ${NAME}.lst ${OBJECTS}

#automatic collection of dependencies in the source files.
#it's only updated the first time, after that it must be done maually
#with "make depend"
#the dependecies are included from a separate file:
-include dependencies.in
#target to update the file, it's removed first
depend: rmdepend dependencies.in
#remove the file
rmdepend:
	c:/MinGW/msys/1.0/bin/rm -f dependencies.in
#build the file that contains the dependencies. no deps in this rule.
#if there were deps it would be rebuilt every chnage, which is unneded:
dependencies.in:
	$(CC) -MM ${CFLAGS} $(patsubst %.o,%.c,$(OBJECTS)) >$@

