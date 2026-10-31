#pragma once

#include <iostream>

#ifdef FFD_DEBUG_ENABLED
    #define FFD_DEBUG(msg) do { std::cerr << "[FFD DEBUG] " << msg << std::endl; std::cerr.flush(); } while(0)
#else
    #define FFD_DEBUG(msg) do {} while(0)
#endif
