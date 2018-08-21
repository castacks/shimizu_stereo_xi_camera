
#ifndef __INCLUDE_PROFILER_PROFILER_HPP__
#define __INCLUDE_PROFILER_PROFILER_HPP__

#include <fstream>
#include <iostream>
#include <string>
#include <map>

#include <boost/date_time/posix_time/posix_time.hpp>

class Profiler 
{
public:
    typedef boost::posix_time::ptime time_t;
    typedef boost::posix_time::time_duration td_t;
    typedef unsigned long int_t;
    typedef double real_t;

public:
    Profiler();
    ~Profiler();

    void set_name(const std::string& name);
    std::string& get_name(void);

    int_t get_count(void);
    int_t get_total_time(void);

    void in(void);
    void out(void);

protected:
    std::string mName;
    int_t mCount;
    int_t mTotalTimeMuS; // Micro-seconds.

    time_t mTimeIn;
    time_t mTimeOut;
    td_t mTimeDuration;

    bool mIsCounting;
};

extern std::map<const char*, Profiler> gProfilers;

void save_profile_info(const std::string& fn, std::map<const char*, Profiler>& m);

#define USE_PROFILER

#ifdef USE_PROFILER

#define PROFILER_IN(name) \
    if ( gProfilers.end() == gProfilers.find(name) ) \
    {\
        gProfilers[name] = Profiler();\
        gProfilers[name].set_name(name);\
    }\
    gProfilers[name].in();\

#define PROFILER_OUT(name) \
    gProfilers[name].out();

#define PROFILER_SAVE(fn, m) \
    save_profile_info(fn, m);

#else

#define PROFILER_IN(name) 
#define PROFILER_OUT(name) 
#define PROFILER_SAVE(fn, m) 

#endif

#endif // __INCLUDE_PROFILER_PROFILER_HPP__
