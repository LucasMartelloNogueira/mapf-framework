#pragma once

namespace mapf {

    class Result {
        public:
            bool success = false;
            int sumOfCosts = 0;
            int makespan = 0;
            double injustice = 0.0;
            double durationSeconds = 0.0;

            Result() = default;
            Result(
                bool success,
                int sumOfCosts,
                int makespan,
                double injustice,
                double durationSeconds
            );
    };

}
