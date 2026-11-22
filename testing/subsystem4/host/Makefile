################################################################################
# Subsystem 4 host unit tests (macOS / Linux, clang or gcc)
#
#   make          build and run
#   make clean
#
# Compiles the target's own imu_logic.c and imu_fixmath.c unchanged, with
# the host model in hump_model.c feeding them.
################################################################################

CC      ?= clang
SRCDIR  := ../../../app_program/subsystem4
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Wshadow -Wconversion \
           -Wno-sign-conversion -Wno-float-conversion -I$(SRCDIR) -I.
LDLIBS  := -lm

TARGET  := s4_host_tests
SRCS    := test_main.c test_fixmath.c test_logic.c hump_model.c \
           $(SRCDIR)/imu_logic.c $(SRCDIR)/imu_fixmath.c

.PHONY: run clean

run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) $(wildcard *.h) $(wildcard $(SRCDIR)/imu_*.h)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

clean:
	rm -f $(TARGET)
