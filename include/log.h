#ifndef DPCT_LOG_H
#define DPCT_LOG_H

#include <sstream>
#include <stdexcept>

#ifdef DEBUG_LOG
#define DEBUG_MSG(str) do { std::cout << __FILE__ << ":" << __LINE__ << ": Debug : " << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

#define LOG_MSG(str) do { std::cout << "Log: " << str << std::endl; } while( false )

#define THROW_RUNTIME_ERROR(x) do { std::stringstream s; s << __FILE__ << ":" << __LINE__ << ": " << x; throw std::runtime_error(s.str());} while( false )
// int main()
// {
//     DEBUG_MSG("Hello" << ' ' << "World!" << 1 );
//     return 0;
// }

#endif // DPCT_LOG_H
