#include "mapf/core/result.hpp"

namespace mapf {

    Result::Result(
        bool success,
        int sumOfCosts,
        int makespan,
        double injustice,
        double durationSeconds,
        std::size_t numInitialConflicts,
        std::size_t numResolvedConflicts,
        std::size_t numUnresolvedConflicts
    ) :
        success(success),
        sumOfCosts(sumOfCosts),
        makespan(makespan),
        injustice(injustice),
        durationSeconds(durationSeconds),
        numInitialConflicts(numInitialConflicts),
        numResolvedConflicts(numResolvedConflicts),
        numUnresolvedConflicts(numUnresolvedConflicts) {}

}
