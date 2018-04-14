set(VITA3K_ORG_NAME "Vita3K")
set(VITA3K_APP_NAME "Vita3K")
set(VITA3K_APP_VERSION "v0.1")

execute_process(
	COMMAND git rev-list HEAD --count
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_COUNT
	OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
	COMMAND git rev-parse --short HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_HASH
	OUTPUT_STRIP_TRAILING_WHITESPACE)
	

set (CMAKE_PREFIX_PATH "$ENV{QTDIR}")
MESSAGE(STATUS "AYYYYY" "${CMAKE_PREFIX_PATH}")
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/src/emulator/cmake_modules")
include(cotire)

if(GIT_HASH STREQUAL "")
	set(VITA3K_GIT_REV "Development version")
else()
	set(VITA3K_GIT_REV "${GIT_COUNT}-${GIT_HASH}")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT VITA3K_SRC_DIR)
	set(VITA3K_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
	message("-- Initializing VITA3K_SRC_DIR=${VITA3K_SRC_DIR}")
else()
	message("-- Using Custom VITA3K_SRC_DIR=${VITA3K_SRC_DIR}")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
set(RES_FILES "")
include(CheckCXXCompilerFlag)

# Qt section
find_package(Qt5 5.10 COMPONENTS Widgets Qml)
if(WIN32)
	find_package(Qt5 5.10 COMPONENTS WinExtras REQUIRED)
	set(VITA3K_QT_LIBS Qt5::Widgets Qt5::WinExtras Qt5::Qml)
else()
	find_package(Qt5 5.10 COMPONENTS DBus Gui)
	if(Qt5DBus_FOUND)
		set(VITA3K_QT_LIBS Qt5::Widgets Qt5::DBus Qt5::Qml)
		add_definitions(-DHAVE_QTDBUS)
	else()
		set(VITA3K_QT_LIBS Qt5::Widgets Qt5::Qml)
	endif()
	include_directories(${Qt5Gui_PRIVATE_INCLUDE_DIRS})
endif()

# Let's make sure we have Qt before we continue
if(NOT Qt5Widgets_FOUND)
	if(Qt5Widgets_VERSION VERSION_LESS 5.10.0)
		message("Minimum supported Qt5 version is 5.10.0! You have version ${Qt5Widgets_VERSION} installed, please upgrade!")
		if("${CMAKE_SYSTEM}" MATCHES "Linux")
			message(FATAL_ERROR "Most distros do not provide an up-to-date version of Qt.
If you're on Ubuntu or Linux Mint, there are PPAs you can use to install an up-to-date qt5 version.
        https://launchpad.net/~beineri/+archive/ubuntu/opt-qt-5.10.1-xenial
        https://launchpad.net/~beineri/+archive/ubuntu/opt-qt-5.10.1-trusty
just make sure to run
    source /opt/qt510/bin/qt510-env.sh
before re-running cmake")
		elseif(WIN32)
			message(FATAL_ERROR "You can download the latest version of Qt5 here: https://www.qt.io/download-open-source/")
		else()
			message(FATAL_ERROR "Look online for instructions on installing an up-to-date Qt5 on ${CMAKE_SYSTEM}.")
		endif()
	endif()

	message("CMake was unable to find Qt5!")
	if(WIN32)
		message(FATAL_ERROR "Make sure the QTDIR env variable has been set properly. (for example C:\\Qt\\5.10.1\\msvc2017_64\\)")
	elseif("${CMAKE_SYSTEM}" MATCHES "Linux")
		message(FATAL_ERROR "Make sure to install your distro's qt5 package!")
	else()
		message(FATAL_ERROR "You need to have Qt5 installed, look online for instructions on installing Qt5 on ${CMAKE_SYSTEM}.")
	endif()
endif()


set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

    add_definitions(-DWIN32_LEAN_AND_MEAN)
	
# To make UI files on cmake 3.7 or less work
if(CMAKE_VERION VERSION_LESS 3.8)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

if(Qt5_POSITION_INDEPENDENT_CODE)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	if(UNIX)
		# Cotire needs this set for some reason
		set(CMAKE_CXX_COMPILE_OPTIONS_PIE -fPIC)
	endif()
endif()


add_subdirectory(audio)
add_subdirectory(cpu)
add_subdirectory(crypto)
add_subdirectory(ctrl)
add_subdirectory(disasm)
add_subdirectory(glutil)
add_subdirectory(gxm)
add_subdirectory(gui)
add_subdirectory(net)
add_subdirectory(host)
add_subdirectory(io)
add_subdirectory(kernel)
add_subdirectory(mem)
add_subdirectory(module)
add_subdirectory(modules)
add_subdirectory(nids)
add_subdirectory(util)
add_subdirectory(threads)

add_executable(
emulator
WIN32
MACOSX_BUNDLE
load_self.cpp
load_self.h
main.cpp
relocation.cpp
relocation.h
sfo.cpp
sfo.h
vpk.cpp
vpk.h
)

target_link_libraries(emulator PUBLIC ${VITA3K_QT_LIBS})
target_link_libraries(emulator PRIVATE cpu elfio glutil host modules nids vita-toolchain util)
MESSAGE(STATUS "VITA3L_QT_LIBS: " "${VITA3K_QT_LIBS}")

set_target_properties(emulator PROPERTIES OUTPUT_NAME Vita3K
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
cotire(emulator)

	
if(APPLE)
	add_custom_command(
		OUTPUT Vita3K.icns
		COMMAND mkdir -p Vita3K.iconset
		COMMAND sips -z 16 16     "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_16x16.png
		COMMAND sips -z 32 32     "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_16x16@2x.png
		COMMAND sips -z 32 32     "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_32x32.png
		COMMAND sips -z 64 64     "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_32x32@2x.png
		COMMAND sips -z 128 128   "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_128x128.png
		COMMAND sips -z 256 256   "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_128x128@2x.png
		COMMAND sips -z 256 256   "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_256x256.png
		COMMAND sips -z 512 512   "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_256x256@2x.png
		COMMAND sips -z 512 512   "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" --out Vita3K.iconset/icon_512x512.png
		COMMAND cp "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png" Vita3K.iconset/icon_512x512@2x.png
		COMMAND iconutil -c icns Vita3K.iconset
		COMMAND rm -r Vita3K.iconset
		DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/Vita3K.png")
	set_source_files_properties(Vita3K.icns PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

	add_custom_command(
		TARGET emulator
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/shaders" "$<TARGET_FILE_DIR:emulator>/../Resources/shaders"
		COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/src/external/sdl/macos/SDL2.framework" "$<TARGET_FILE_DIR:emulator>/../Frameworks/SDL2.framework")
	set_target_properties(emulator PROPERTIES LINK_FLAGS "-rpath @executable_path/../Frameworks/")
	target_sources(emulator PRIVATE Vita3K.icns)
	set(MACOSX_BUNDLE_ICON_FILE Vita3K.icns)
elseif(WIN32)
	target_sources(emulator PRIVATE resource.h Vita3K.ico Vita3K.rc)
	add_custom_command(
		TARGET emulator
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/shaders" "$<TARGET_FILE_DIR:emulator>/shaders"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/src/external/sdl/windows/lib/x64/SDL2.dll" "$<TARGET_FILE_DIR:emulator>"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/src/external/unicorn/windows/unicorn.dll" "$<TARGET_FILE_DIR:emulator>"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/src/external/unicorn/windows/libgcc_s_seh-1.dll" "$<TARGET_FILE_DIR:emulator>"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PROJECT_SOURCE_DIR}/src/external/unicorn/windows/libwinpthread-1.dll" "$<TARGET_FILE_DIR:emulator>")
endif()

# Ignore autogenerated moc_* files if present
set(EXCLUDE_FILES ${EXCLUDE_FILES} "moc_")
set(EXCLUDE_FILES ${EXCLUDE_FILES} "vita3k_automoc")
set(EXCLUDE_FILES ${EXCLUDE_FILES} "qrc_resources.cpp")

foreach (TMP_PATH ${VITA3K_SRC})
	foreach (EXCLUDE_PATH ${EXCLUDE_FILES})
		string(FIND ${TMP_PATH} ${EXCLUDE_PATH} EXCLUDE_FILE_FOUND)
		if(NOT ${EXCLUDE_FILE_FOUND} EQUAL -1)
			list(REMOVE_ITEM VITA3K_SRC ${TMP_PATH})
		endif ()
	endforeach(EXCLUDE_PATH)
endforeach(TMP_PATH)

# Remove the Qt moc files as part of clean, they are compiled when generating automoc
file(GLOB_RECURSE TMP_MOC "${VITA3K_SRC_DIR}/moc_*.cpp" "${VITA3K_SRC_DIR}/vita3k_automoc.cpp" "${VITA3K_SRC_DIR}/qrc_resources.cpp")
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${TMP_MOC}")

if(WIN32)
	add_executable(vita3k ${VITA3K_SRC} ${RES_FILES} resources.qrc)
else()
	add_executable(vita3k ${VITA3K_SRC} ${RES_FILES} resources.qrc)
endif()
