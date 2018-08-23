
PLEXILC=plexilc
PLEXIL_EXEC=plexil-universalExec
INTERFACE_CONFIG=external.xml
PLAN_PLX=plan.plx

PLE=$(wildcard *.ple)

all: $(patsubst %.ple,%.plx,$(PLE))

%.plx: %.ple
	plexilc $<

.PHONY: all clean run
clean:
	rm -f $(patsubst %.ple,%.plx,$(PLE))

run: all
	$(PLEXIL_EXEC) -c $(INTERFACE_CONFIG) -p $(PLAN_PLX)

