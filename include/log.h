#ifndef DPCT_LOG_H
#define DPCT_LOG_H

#ifdef DEBUG_LOG
#define DEBUG_MSG(str) do { std::cout << "Debug : " << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

#define LOG_MSG(str) do { std::cout << "Log: " str << std::endl; } while( false )

// int main()
// {
//     DEBUG_MSG("Hello" << ' ' << "World!" << 1 );
//     return 0;
// }

#endif // DPCT_LOG_H
