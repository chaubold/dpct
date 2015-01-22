#ifndef DPCT_LOG_H
#define DPCT_LOG_H

#ifdef DEBUG_LOG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

// int main()
// {
//     DEBUG_MSG("Hello" << ' ' << "World!" << 1 );
//     return 0;
// }

#endif // DPCT_LOG_H
