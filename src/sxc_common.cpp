
#include <cmath>
#include <iostream>

#include "sxc_common.hpp"

// using namespace sxc;

namespace sxc
{

xf dBToGain(xf dB)
{
    return std::pow( 10.0, dB / 10.0 );
}

xf GainToDB(xf G)
{
    if ( G <= 0.0 )
    {
        std::cout << "Gain value shold be positive. Return 1.0." << std::endl;
        return 1.0;
    }

    return 10.0 * std::log10( G );
}

}
