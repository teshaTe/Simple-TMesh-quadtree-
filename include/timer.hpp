#ifndef H_SIMPLE_TIMER
#define H_SIMPLE_TIMER

#include <chrono>
#include <ctime>
#include <string>
#include <iostream>

namespace prof {

class timer
{
public:
    timer():m_iter{0}{}

    void End( std::string var )
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::microseconds>( end - m_start );
        std::cout.precision(6);
        std::cout << var << ", iteration ["<< m_iter <<"] : " << time_span.count()/1000000.0f << " [Sec];" << std::endl;

        if(m_iter > 1)
            printf("\x1b[A");
    }

    inline void Start() { m_start = std::chrono::high_resolution_clock::now(); m_iter++; }
    inline void resetIter() { std::cout <<"\nlast iter: " << m_iter << std::endl; m_iter = 0; }

    ~timer() = default;

private:
    std::chrono::high_resolution_clock::time_point m_start;
    int m_iter;
};

}


#endif //#define H_SIMPLE_TIMER
