meta:
	ADDON_NAME = ofxRPlidar
	ADDON_DESCRIPTION = RPlidar SDK wrapper for openframeworks
	ADDON_AUTHOR = @Eotel
	ADDON_TAGS = "RPlidar" "LiDAR" "Scanner"
	ADDON_URL = https://github.com/Eotel/ofxRPlidar

common:
	ADDON_INCLUDES_EXCLUDE = libs/rplidar/src/hal%
	ADDON_INCLUDES_EXCLUDE += libs/rplidar/src/arch%

linux64:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/macOS%

linux:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/macOS%

linuxarmv6l:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/macOS%

linuxarmv7l:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/macOS%

linuxaarch64:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/macOS%

osx:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/win32%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/linux%
	#ADDON_DEFINES += _MACOS
	ADDON_CPPFLAGS += -D_MACOS

vs:
	ADDON_SOURCES_EXCLUDE = libs/rplidar/src/arch/macOS%
	ADDON_SOURCES_EXCLUDE += libs/rplidar/src/arch/linux%
	#ADDON_DEFINES += _WIN32
	ADDON_CPPFLAGS += -D_WIN32
