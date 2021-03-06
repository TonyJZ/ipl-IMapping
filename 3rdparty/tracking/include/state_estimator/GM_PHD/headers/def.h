#ifndef DEF_H
#define DEF_H
#include <stdexcept>
#include <stdio.h>

#ifdef __GNUC__
#define THROW_ERR(msg) __extension__({char buff[256]; sprintf(buff,"%s:%d %s",__FILE__,__LINE__,msg); throw std::logic_error(buff);})
#elif _MSC_VER
#define THROW_ERR(msg) throw std::logic_error(msg)
#endif // _WIN32



#ifndef MIN
#define MIN(a, b)  (((a) > (b)) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (a))
#endif

#endif // DEF_H
