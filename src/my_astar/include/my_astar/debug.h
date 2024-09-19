#pragma once

#ifdef DEBUG
    #define DEBUG_PRINT(s) std::cout << "\033[32m" << s << "\033[0m" << std::endl
#else
    #define DEBUG_PRINT(s)
#endif