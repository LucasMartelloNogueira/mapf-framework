#include "mapf/solvers/local_path_repair_parallel_solver.hpp"

#include "local_path_repair_solver_common.hpp"

#include "mapf/pathfinding/a_star.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace mapf {

    namespace {
        class ThreadPool {
            private:
                std::vector<std::jthread> workers;
                std::queue<std::function<void()>> tasks;
                std::mutex tasksMutex;
                std::condition_variable ready;
                bool stopping = false;

            public:
                explicit ThreadPool(std::size_t workerCount) {
                    workers.reserve(workerCount);

                    for (std::size_t i = 0; i < workerCount; i++) {
                        workers.emplace_back([this]() {
                            while (true) {
                                std::function<void()> task;

                                {
                                    std::unique_lock lock(tasksMutex);
                                    ready.wait(lock, [this]() {
                                        return stopping || !tasks.empty();
                                    });

                                    if (stopping && tasks.empty()) {
                                        return;
                                    }

                                    task = std::move(tasks.front());
                                    tasks.pop();
                                }

                                task();
                            }
                        });
                    }
                }

                ThreadPool(const ThreadPool&) = delete;
                ThreadPool& operator=(const ThreadPool&) = delete;

                ~ThreadPool() {
                    {
                        std::lock_guard lock(tasksMutex);
                        stopping = true;
                    }

                    ready.notify_all();
                    workers.clear();
                }

                template<class Function>
                auto submit(Function&& function)
                    -> std::future<std::invoke_result_t<std::decay_t<Function>&>>
                {
                    using ReturnType = std::invoke_result_t<std::decay_t<Function>&>;
                    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
                        std::forward<Function>(function)
                    );
                    std::future<ReturnType> future = task->get_future();

                    {
                        std::lock_guard lock(tasksMutex);
                        if (stopping) {
                            throw std::runtime_error("Cannot submit work to a stopped thread pool.");
                        }

                        tasks.emplace([task]() {
                            (*task)();
                        });
                    }

                    ready.notify_one();
                    return future;
                }
        };
    }

    LocalPathRepairParallelSolver::LocalPathRepairParallelSolver(
        const Instance& instance,
        std::size_t numberOfThreads,
        bool continueIfFailed
    ) :
        instance(instance),
        numberOfThreads(numberOfThreads),
        continueIfFailed(continueIfFailed)
    {
        if (numberOfThreads == 0) {
            throw std::invalid_argument("The number of threads must be positive.");
        }
    }

    LocalPathRepairResult LocalPathRepairParallelSolver::solve() {
        const auto startedAt = std::chrono::steady_clock::now();
        const std::vector<Agent>& agents = instance.getAgents();
        std::vector<std::list<Cell*>> initialPaths(agents.size());

        if (!agents.empty()) {
            Grid& grid = const_cast<Grid&>(instance.getGrid());
            const std::size_t workerCount = std::min(numberOfThreads, agents.size());
            ThreadPool pool(workerCount);
            std::vector<std::future<std::list<Cell*>>> futures;
            futures.reserve(agents.size());

            for (const Agent& agent : agents) {
                futures.push_back(pool.submit([&grid, agent]() {
                    AStarSolver solver;
                    Cell* start = grid.getCellPtr(
                        agent.startPosition.x,
                        agent.startPosition.y
                    );
                    Cell* goal = grid.getCellPtr(
                        agent.goalPosition.x,
                        agent.goalPosition.y
                    );
                    return solver.solve(grid, start, goal);
                }));
            }

            for (std::size_t i = 0; i < futures.size(); i++) {
                initialPaths[i] = futures[i].get();
            }
        }

        return local_path_repair_detail::repairInitialPaths(
            instance,
            std::move(initialPaths),
            continueIfFailed,
            startedAt
        );
    }

}
