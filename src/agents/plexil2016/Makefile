
PLEXILC=plexilc
#PLEXIL_EXEC=gdb -ex run --args plexil-universalExec
PLEXIL_EXEC=plexil-universalExec
INTERFACE_CONFIG=external.xml
PLAN_PLX=plan.plx

PLE=$(wildcard *.ple)

all: $(patsubst %.ple,%.plx,$(PLE))

%.plx: %.ple
	$(PLEXILC) $<

%.epx: %.ple
	java -classpath /usr/share/java/PlexilCompiler.jar:/usr/share/java/antlr3-runtime.jar:/usr/share/java/nanoxml.jar:/usr/share/java/saxon/saxon.jar plexil.Compiler -e $<

.PHONY: all clean run
clean:
	rm -f $(patsubst %.ple,%.plx,$(PLE))

run: all
	$(PLEXIL_EXEC) -c $(INTERFACE_CONFIG) -p $(PLAN_PLX)

