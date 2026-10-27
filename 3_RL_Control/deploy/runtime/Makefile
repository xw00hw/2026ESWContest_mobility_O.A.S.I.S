# Pi 5 (Cortex-A76):  make            x86 test build:  make ARCH=-march=native
MODEL ?= ../model
BUILD ?= build
ARCH  ?= -mcpu=cortex-a76
CFLAGS = -O3 $(ARCH) -ffast-math -fno-math-errno -std=c11 -Wall -Wextra -I$(MODEL)

all: $(BUILD)/liboasis_actor.so $(BUILD)/actor_bench

$(BUILD)/liboasis_actor.so: oasis_rt/actor_core.c $(MODEL)/actor_weights.h
	mkdir -p $(BUILD) && $(CC) $(CFLAGS) -shared -fPIC $< -o $@ -lm

$(BUILD)/actor_bench: oasis_rt/actor_bench.c oasis_rt/actor_core.c $(MODEL)/actor_weights.h
	mkdir -p $(BUILD) && $(CC) $(CFLAGS) oasis_rt/actor_bench.c oasis_rt/actor_core.c -o $@ -lm

clean:
	rm -rf $(BUILD)
