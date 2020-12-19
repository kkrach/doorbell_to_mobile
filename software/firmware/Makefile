#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := doorbell_firmware

-include $(IDF_PATH)/make/project.mk

CONFIG_SERVER_PORT="8070"
include sdkconfig

webserver:
	cd build && python -m SimpleHTTPServer $(CONFIG_SERVER_PORT)
