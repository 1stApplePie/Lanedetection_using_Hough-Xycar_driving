// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include <cmath>
#include <algorithm>
#include "LaneKeepingSystem/PIDController.hpp"

namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    // int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
    PREC castError = static_cast<PREC>(errorFromMid);

    float straight_mPropertionalGain;

    mDifferentialGainError = castError - mProportionalGainError;
    mProportionalGainError = castError; 
    mIntegralGainError += castError;
    if (std::abs(castError) < 66)
    {   
        straight_mPropertionalGain = mProportionalGain / 2;
        //return straight_mPropertionalGain * mProportionalGainError + mDifferentialGain * mDifferentialGainError;
        return straight_mPropertionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError;
    }

    mIntegralGainError = std::max(mIntegralGainError,static_cast<PREC>(500.0f));

    return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar