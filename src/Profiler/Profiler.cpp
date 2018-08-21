
#include "Profiler/Profiler.hpp"

std::map<const char*, Profiler> gProfilers;

void save_profile_info(const std::string& fn, std::map<const char*, Profiler>& m)
{
    // Open file for output.
    std::ofstream ofs;
    ofs.open(fn.c_str());

    std::map<const char*, Profiler>::iterator iter;

    for ( iter = m.begin(); iter != m.end(); ++iter )
    {
        ofs << iter->second.get_name() << ", " 
            << iter->second.get_count() << ", " 
            << iter->second.get_total_time() << std::endl;
    }

    ofs.close();
}

Profiler::Profiler()
: mName("NO_NAME"), mCount(0), mTotalTimeMuS(0), mIsCounting(false)
{

}

Profiler::~Profiler()
{

}

void Profiler::set_name(const std::string& name)
{
    mName = name;
}

std::string& Profiler::get_name(void)
{
    return mName;
}

void Profiler::in(void)
{
    if ( false == mIsCounting )
    {
        mTimeIn = boost::posix_time::microsec_clock::local_time();
        mIsCounting = true;
    }
    else
    {
        std::cout << mName << " is counting!" << std::endl;
    }
}

void Profiler::out(void)
{
    if ( true == mIsCounting )
    {
        mTimeOut = boost::posix_time::microsec_clock::local_time();
        mTimeDuration = mTimeOut - mTimeIn;

        mTotalTimeMuS += mTimeDuration.total_microseconds();
        mCount++;

        mIsCounting = false;
    }
    else
    {
        std::cout << mName << " is not counting!" << std::endl;
    }
}

Profiler::int_t Profiler::get_count(void)
{
    return mCount;
}

Profiler::int_t Profiler::get_total_time(void)
{
    return mTotalTimeMuS;
}
