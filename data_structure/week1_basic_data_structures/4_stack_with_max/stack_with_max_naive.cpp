#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <algorithm>

using std::cin;
using std::string;
using std::vector;
using std::cout;
using std::max_element;

class StackWithMax {
    vector<int> stack;
    int maxElem;

  public:

    void Push(int value) {
        if (stack.empty()){
            maxElem = value;
            stack.push_back(value);
            return;
        }
        if (value > maxElem)
        {
            // 2 * value - maxElem is always
            // larger than the previous maxElem
            stack.push_back(2*value - maxElem);
            maxElem = value;
        }
        else
        {
            stack.push_back(value);
        }
    }

    void Pop() {
        assert(stack.size());
        int top_ = stack.back();
        stack.pop_back();
        if (top_ > maxElem){
            maxElem = 2 * maxElem - top_;
        }
    }

    int Max() const {
        assert(stack.size());
        // return *max_element(stack.begin(), stack.end());
        return maxElem;
    }
};

int main() {
    int num_queries = 0;
    cin >> num_queries;

    string query;
    string value;

    StackWithMax stack;

    for (int i = 0; i < num_queries; ++i) {
        cin >> query;
        if (query == "push") {
            cin >> value;
            stack.Push(std::stoi(value));
        }
        else if (query == "pop") {
            stack.Pop();
        }
        else if (query == "max") {
            cout << stack.Max() << "\n";
        }
        else {
            assert(0);
        }
    }
    return 0;
}