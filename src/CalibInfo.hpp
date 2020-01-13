#ifndef _CALIB_INFO_HPP_
#define _CALIB_INFO_HPP_

namespace spartan
{

struct CalibInfo
{
    double K_LEFT[9], K_RIGHT[9],
        D_LEFT[5], D_RIGHT[5],
        ROT_LEFT[9], ROT_RIGHT[9],
        T_LEFT[3], T_RIGHT[3];
    CalibInfo() :
        D_LEFT{0,0,0,0,0},
        D_RIGHT{0,0,0,0,0},
        ROT_LEFT{1,0,0,0,1,0,0,0,1},
        ROT_RIGHT{1,0,0,0,1,0,0,0,1},
        T_LEFT{0,0,0},
        T_RIGHT{0,0,0}
    {
    }
};

} // namespace spartan

#endif // _CALIB_INFO_HPP_
