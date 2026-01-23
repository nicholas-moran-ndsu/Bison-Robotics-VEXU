################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

# ===== X-Drive build profiles =====
.PHONY: xdrive4 xdrive8 xdrive12 xdrive4_upload xdrive8_blue_upload xdrive8_red_upload xdrive12_upload

# Build 4-motor config (no upload)
xdrive4:
	@echo "=== Building XDRIVE4 (4 motors) ==="
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE4 ' quick

# Build 8-motor config (no upload)
xdrive8:
	@echo "=== Building XDRIVE8 (8 motors) ==="
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE8 -Wall -Wextra -Wunused ' quick

# Build 12-motor config (no upload)
xdrive12:
	@echo "=== Building XDRIVE12 (12 motors) ==="
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE12 ' quick

# Build + upload helpers (pick slots you prefer)
xdrive4_upload:
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE4 ' quick
	@pros upload --slot 4
# Build + upload 8-motor blue alliance
xdrive8_blue_upload:
	@$(MAKE) clean
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE8 -DALLIANCE_BLUE ' quick
	@pros upload --slot 1
# Build + upload 8-motor red alliance
xdrive8_red_upload:
	@$(MAKE) clean
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE8 -DALLIANCE_RED ' quick
	@pros upload --slot 2
# Build + upload 12-motor config
xdrive12_upload:
	@$(MAKE) EXTRA_CXXFLAGS+=' -DXDRIVE12 ' quick
	@pros upload --slot 6
