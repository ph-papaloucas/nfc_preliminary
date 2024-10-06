## 
##Create source code library 
file(GLOB source_files "aero/src/*.cpp")
file(GLOB header_files "aero/include/*.h")
add_library(aero STATIC ${source_files} ${header_files} ${externals_source_files} ${externals_header_files})
target_include_directories(aero PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/aero/include)


##
##Create adjoint library which depends on aero library
add_subdirectory(adjoint)
##
##Create interface library nfc
file(GLOB nfc_source_files "nfc/*.cpp")
file(GLOB nfc_header_files "nfc/*.h")
add_library(nfc STATIC ${nfc_source_files} ${nfc_header_files})
target_include_directories(nfc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/nfc)