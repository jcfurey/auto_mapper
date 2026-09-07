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

#include "auto_mapper/search_worker.hpp"

#include <exception>
#include <utility>

namespace auto_mapper
{

SearchWorker::SearchWorker()
: thread_(&SearchWorker::run, this)
{}

SearchWorker::~SearchWorker()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    ++generation_;
    pending_.reset();
  }
  changed_.notify_one();
  thread_.join();
}

uint64_t SearchWorker::submit(SearchJob job)
{
  std::lock_guard<std::mutex> lock(mutex_);
  job.id = ++generation_;
  const uint64_t id = job.id;
  pending_ = std::move(job);
  completed_.reset();
  changed_.notify_one();
  return id;
}

void SearchWorker::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++generation_;
  pending_.reset();
  completed_.reset();
}

std::optional<SearchCompletion> SearchWorker::poll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto completion = std::move(completed_);
  completed_.reset();
  return completion;
}

void SearchWorker::run()
{
  FrontierSearch search;
  while (true) {
    SearchCompletion completion;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait(lock, [this] {return stopping_ || pending_.has_value();});
      if (stopping_) {return;}
      completion.job = std::move(*pending_);
      pending_.reset();
    }
    const uint64_t id = completion.job.id;
    try {
      completion.result = search.search(
        completion.job.grid, completion.job.robot, completion.job.params, completion.job.blacklist,
        [this, id] {return stopping_ || generation_ != id;});
    } catch (const std::exception & error) {
      completion.error = error.what();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stopping_ && generation_ == id) {completed_ = std::move(completion);}
    }
  }
}

}  // namespace auto_mapper
