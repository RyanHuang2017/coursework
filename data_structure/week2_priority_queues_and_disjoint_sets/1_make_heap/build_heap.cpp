#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

using std::vector;
using std::cin;
using std::cout;
using std::swap;
using std::pair;
using std::make_pair;

class HeapBuilder {
 private:
  vector<int> data_;
  vector< pair<int, int> > swaps_;

  void WriteResponse() const {
    cout << swaps_.size() << "\n";
    for (int i = 0; i < swaps_.size(); ++i) {
      cout << swaps_[i].first << " " << swaps_[i].second << "\n";
    }
  }

  void ReadData() {
    int n;
    cin >> n;
    data_.resize(n);
    for(int i = 0; i < n; ++i)
      cin >> data_[i];
  }

  void GenerateSwaps() {
    swaps_.clear();
    int n = data_.size();
    for (int i = floor((n-1)/2.0); i >=0; i--){
      SiftDown(i);
    } 
  }

  int Parent(int i){
    return floor((i-1)/2.0);
  }

  int LeftChild(int i){
    return 2*i+1;
  }

  int RightChild(int i){
    return 2*i+2;
  }

  // sift down for binary min-heap
  void SiftDown(int i){
    int minIndex = i;
    int left = LeftChild(i);
    if ((left < data_.size()) && (data_[left] < data_[minIndex])){
      minIndex = left;
    }
    int right = RightChild(i);
    if ((right < data_.size()) && (data_[right] < data_[minIndex])){
      minIndex = right;
    }
    if (minIndex != i){
      swap(data_[i], data_[minIndex]);
      swaps_.push_back(make_pair(i, minIndex));
      SiftDown(minIndex);
    }
  }


 public:
  void Solve() {
    ReadData();
    GenerateSwaps();
    WriteResponse();
  }
};

int main() {
  std::ios_base::sync_with_stdio(false);
  HeapBuilder heap_builder;
  heap_builder.Solve();
  return 0;
}
