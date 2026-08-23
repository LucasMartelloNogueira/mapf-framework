#include "mapf/core/result.hpp"

namespace mapf {

    Result::Result(
        bool success,
        int sumOfCosts,
        int makespan,
        double injustice,
        double durationSeconds
    ) :
        success(success),
        sumOfCosts(sumOfCosts),
        makespan(makespan),
        injustice(injustice),
        durationSeconds(durationSeconds) {}

}
