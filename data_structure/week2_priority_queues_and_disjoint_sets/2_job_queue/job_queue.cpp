#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

using std::vector;
using std::cin;
using std::cout;
using std::swap;

struct WORKER
{
  /* data */
  int index;
  long long next_free_time_ = 0;
};

class JobQueue {
 private:
  int num_workers_;
  vector<int> jobs_;

  vector<int> assigned_workers_;
  vector<long long> start_times_;
  vector<WORKER> workers_;

  void WriteResponse() const {
    for (int i = 0; i < jobs_.size(); ++i) {
      cout << assigned_workers_[i] << " " << start_times_[i] << "\n";
    }
  }

  void ReadData() {
    int m;
    cin >> num_workers_ >> m;
    jobs_.resize(m);
    for(int i = 0; i < m; ++i)
      cin >> jobs_[i];
  }

  void AssignJobs() {
    // TODO: replace this code with a faster algorithm.
    assigned_workers_.resize(jobs_.size());
    start_times_.resize(jobs_.size());
    workers_.resize(num_workers_);
    BuildHeap();
    for (int i = 0; i < jobs_.size(); ++i){
      WORKER avail_worker_ = GetAvailableWorker();
      assigned_workers_[i] = avail_worker_.index;
      start_times_[i] = avail_worker_.next_free_time_;
      ChangeAvailability(0, jobs_[i]);
    }
  }

  int Parent(int i){
    return floor(i/2.0);
  }

  int LeftChild(int i){
    return 2*i+1;
  }
  int RightChild(int i){
    return 2*i+2;
  }

  void SiftUp(int i){
    while ((i > 0) && ((workers_[i].next_free_time_ == workers_[Parent(i)].next_free_time_ && workers_[i].index < workers_[Parent(i)].index)|| workers_[i].next_free_time_ < workers_[Parent(i)].next_free_time_ )){
      swap(workers_[i], workers_[Parent(i)]);
      i = Parent(i);
    }

  }

  void SiftDown(int i){
    int minIndex = i;
    int left = LeftChild(i);
    if (left < workers_.size() && ((workers_[left].next_free_time_ == workers_[minIndex].next_free_time_ && workers_[left].index < workers_[minIndex].index) || workers_[left].next_free_time_ < workers_[minIndex].next_free_time_)){
      minIndex = left;
    }
    int right = RightChild(i);
    if (right < workers_.size() && ((workers_[right].next_free_time_ == workers_[minIndex].next_free_time_ && workers_[right].index < workers_[minIndex].index) || workers_[right].next_free_time_ < workers_[minIndex].next_free_time_)){
      minIndex = right;
    }
    if(minIndex != i){
      swap(workers_[i], workers_[minIndex]);
      SiftDown(minIndex);
    }
  }

  void BuildHeap(){
    int n = workers_.size();
    // initialize the workers vector
    for (int i = 0; i < n; ++i){
      workers_[i] = {i, 0};
    }
    // build a binary min-heap to model a scheduler
    for (int i = floor((i-1)/2); i >= 0; --i){
      SiftDown(i);
    }
  }

  void ChangeAvailability(int i, int job_duration_){
    long long prev_start_time_ = workers_[i].next_free_time_;
    workers_[i].next_free_time_ += job_duration_;
    if (prev_start_time_ < workers_[i].next_free_time_){
      SiftDown(i);
    }
  }

  WORKER GetAvailableWorker(){
    return workers_[0];
  }
  


 public:
  void Solve() {
    ReadData();
    AssignJobs();
    WriteResponse();
  }
};

int main() {
  std::ios_base::sync_with_stdio(false);
  JobQueue job_queue;
  job_queue.Solve();
  return 0;
}
