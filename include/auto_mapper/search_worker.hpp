// Copyright 2026 jcfurey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTO_MAPPER__SEARCH_WORKER_HPP_
#define AUTO_MAPPER__SEARCH_WORKER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "auto_mapper/frontier_search.hpp"

namespace auto_mapper
{

struct SearchJob
{
  uint64_t id{0};
  uint64_t map_revision{0};
  std::shared_ptr<const Grid> grid;
  Pose2 robot;
  SearchParams params;
  std::vector<BlacklistEntry> blacklist;
};

struct SearchCompletion
{
  SearchJob job;
  SearchResult result;
  std::string error;
};

// At most one active job, one replacement, and one result. The worker only
// touches immutable job inputs and its own reusable search buffers.
class SearchWorker
{
public:
  SearchWorker();
  ~SearchWorker();
  SearchWorker(const SearchWorker &) = delete;
  SearchWorker & operator=(const SearchWorker &) = delete;
  uint64_t submit(SearchJob job);
  void cancel();
  std::optional<SearchCompletion> poll();

private:
  void run();
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> generation_{0};
  std::mutex mutex_;
  std::condition_variable changed_;
  std::optional<SearchJob> pending_;
  std::optional<SearchCompletion> completed_;
  std::thread thread_;
};

}  // namespace auto_mapper

#endif  // AUTO_MAPPER__SEARCH_WORKER_HPP_
