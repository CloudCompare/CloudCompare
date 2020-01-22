cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_STANDARD_QCHAISCRIPTING "Check to install ChaiScripting plugin" OFF )

if ( PLUGIN_STANDARD_QCHAISCRIPTING )
	# Name the plugin
    project( QCHAISCRIPTING )
    add_definitions(-DCHAISCRIPT_NO_THREADS) #avoiding a crash that happens when destroying chai with threads enabled
    
    include_directories( extern/ChaiScript/include)
    #include_directories( src )
	#add_subdirectory( src )
    #add_subdirectory( ui )
	if(MSVC)
  add_definitions(/W4 /w14545 /w34242 /w34254 /w34287 /w44263 /w44265 /w44296 /w44311 /w44826 /we4289 /w14546 /w14547 /w14549 /w14555 /w14619 /w14905 /w14906 /w14928)

  if (MSVC_VERSION STREQUAL "1800")
    # VS2013 doesn't have magic statics
    add_definitions(/w44640)
  else()
    # enum warnings are too noisy on MSVC2013
    add_definitions(/w34062)
  endif()

  add_definitions(/bigobj)
  # Note on MSVC compiler flags.
  # The code base selective disables warnings as necessary when the compiler is complaining too much
  # about something that is perfectly valid, or there is simply no technical way around it
  # This particular warning, C4503 is in regards to the decorated names that MSVC generates internally.
  # The error did not come up until the move to C++11, but the compiler doesn't give enough information
  # to determine where the error is coming from, and the internet provides no real information for
  # how to workaround or fix the error. So I'm disabling it globally.
  add_definitions(/wd4503)
    include( ../../CMakePluginTpl.cmake )
    #add_definitions(/W4 /w14545 /w34242 /w34254 /w34287 /w44263 /w44265 /w44296 /w44311 /w44826 /we4289 /w14546 /w14547 /w14549 /w14555 /w14619 /w14905 /w14906 /w14928)
    #add_definitions(/wd4503)
    #add_definitions(/std:c++14)
    #add_definitions(/bigobj /permissive-)
    set(CHAI_VERSION 5.8.6)
    add_library(chaiscript_stdlib-${CHAI_VERSION} MODULE extern/ChaiScript/src/chaiscript_stdlib.cpp)
    
endif()
endif()
